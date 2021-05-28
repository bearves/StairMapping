#include "PclProcessor.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <termios.h>
#include "PreProcessor.h"

namespace stair_mapping
{
    PclProcessor::PclProcessor(ros::NodeHandle& node)
    {
        preprocess_pub_ = node.advertise<sensor_msgs::PointCloud2>("preprocessed_points", 1);
        submap_pub_ = node.advertise<sensor_msgs::PointCloud2>("submap_points", 1);
        global_map_opt_pub_ = node.advertise<sensor_msgs::PointCloud2>("global_map_opt_points", 1);
        global_map_raw_pub_ = node.advertise<sensor_msgs::PointCloud2>("global_map_raw_points", 1);
        odom_sub_ = node.subscribe("/qz_state_publisher/robot_odom", 1, &PclProcessor::odomMsgCallback, this);
        pcl_sub_ = node.subscribe("transformed_points", 1, &PclProcessor::pclMsgCallback, this);
        //height_map_pub_ = node.advertise<sensor_msgs::PointCloud2>("height_map_pcl", 1);
        //cost_map_pub_ = node.advertise<nav_msgs::OccupancyGrid>("terrain_cost_map", 1);
    }

    void PclProcessor::pclMsgCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        using namespace Eigen;
        PointCloudT::Ptr p_in_cloud(new PointCloudT);
        PointCloudT::Ptr p_pre_cloud(new PointCloudT);

        ROS_INFO("Realsence msg received");

        sensor_msgs::PointCloud2 pre_out_cloud2;
        sensor_msgs::PointCloud2 submap_out_cloud2;

        pcl::fromROSMsg(*msg, *p_in_cloud);

        preProcess(p_in_cloud, p_pre_cloud);

        pcl::toROSMsg(*p_pre_cloud, pre_out_cloud2);
        pre_out_cloud2.header.frame_id = "base_world";
        pre_out_cloud2.header.stamp = ros::Time::now();
        preprocess_pub_.publish(pre_out_cloud2);

        PointCloudT::Ptr p_submap_cloud(new PointCloudT);
        submapMatch(p_pre_cloud, p_submap_cloud);

        pcl::toROSMsg(*p_submap_cloud, submap_out_cloud2);
        submap_out_cloud2.header.frame_id = "base_world";
        submap_out_cloud2.header.stamp = ros::Time::now();
        submap_pub_.publish(submap_out_cloud2);

    }

    void PclProcessor::odomMsgCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        Eigen::Matrix4d pose_mat = getPoseMatrix(*msg);
        odom_msg_mtx_.lock();
        current_odom_mat_ = pose_mat;
        odom_msg_mtx_.unlock();
    }

    Eigen::Matrix4d PclProcessor::getPoseMatrix(const nav_msgs::Odometry &odom)
    {
        using namespace Eigen;

        // TODO: this is a bug of the state publisher since
        // the position data is wrongly published in the twist field
        //auto pos = odom.pose.pose.position;
        auto pos = odom.twist.twist.linear;
        auto ori = odom.pose.pose.orientation;
        Translation3d t(pos.x, pos.y, pos.z);
        Quaterniond q(ori.w, ori.x, ori.y, ori.z);
        // since we have corrected rotation using IMU, we ignore the rotation in odom
        Quaterniond q_corrected(1, 0, 0, 0);
        Affine3d pose = t*q_corrected;

        return pose.matrix();
    }

    void PclProcessor::preProcess(const PointCloudT::Ptr &p_in_cloud, PointCloudT::Ptr &p_out_cloud)
    {
        // crop
        PointCloudT::Ptr p_cloud_cr(new PointCloudT);
        PreProcessor::crop(p_in_cloud, p_cloud_cr, Eigen::Vector3f(0, -0.5, -2), Eigen::Vector3f(2.3, 0.5, 3));

        // downsampling
        PointCloudT::Ptr p_cloud_ds(new PointCloudT);
        Eigen::Vector2i sizes = PreProcessor::downSample(p_cloud_cr, p_cloud_ds, 0.015);
        ROS_INFO("After downsample size: %d -> %d", sizes[0], sizes[1]);

        p_out_cloud = p_cloud_ds;
    }

    void PclProcessor::submapMatch(const PointCloudT::Ptr &p_in_cloud, PointCloudT::Ptr &p_out_cloud)
    {
        using namespace Eigen;

        int submap_store_cap = 1;

        odom_msg_mtx_.lock();
        Matrix4d t_frame_odom = current_odom_mat_;
        odom_msg_mtx_.unlock();

        // FrontEnd
        // scan-to-submap matcher
        // if no submap exists
        if (global_map_.submapCount() == 0)
        {
            // init new submap with current pcl, set its transform to I
            ROS_INFO("Init new global map");
            SubMap::Ptr p_sm(new SubMap(submap_store_cap));
            p_sm->init();
            global_map_.addNewSubmap(
                p_sm, 
                Matrix4d::Identity(), // m2m laser match
                Matrix4d::Identity(), // m2m odom
                t_frame_odom,          // imu
                100*InfoMatrix::Identity() // information of the submap
            );
        }

        SubMap::Ptr last_sm = global_map_.getLastSubMap();
        SubMap::Ptr current_sm;

        if (last_sm == nullptr) return;

        // match current frame to the last submap
        double score = 1e8;
        double SUCCESS_SCORE = 2;

        Matrix4d t_guess = last_sm->getRelativeTfGuess(t_frame_odom);
        Matrix4d t_frame_to_last_map = t_guess;
        InfoMatrix info_mat;

        // reject frames when robot is not moving at all
        Vector3d translation_guess = Affine3d(t_guess).translation();
        if (!last_sm->isEmpty() && translation_guess.norm() < 0.015)
        {
            ROS_WARN("Frame too close: %f", translation_guess.norm());
            return;
        }

        try
        {
            score = last_sm->match(*p_in_cloud, t_guess, t_frame_to_last_map, info_mat);
        }
        catch (std::runtime_error &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }

        // if match succeeds, add current frame to current submap or create new submap, otherwise drop it
        if (score < SUCCESS_SCORE)
        {
            // if a new submap is needed
            if (last_sm->hasEnoughFrame())
            {
                // init new submap with current pcl
                ROS_INFO("Create new submap");

                SubMap::Ptr p_sm(new SubMap(submap_store_cap));
                p_sm->init();
                p_sm->addFrame(*p_in_cloud, Matrix4d::Identity(), t_frame_odom);
                // match to last submap if exists, record transform Ti
                global_map_.addNewSubmap(
                    p_sm, 
                    t_frame_to_last_map, // m2m scan match
                    t_guess,             // m2m odom
                    t_frame_odom,        // global imu
                    info_mat             // infomation matrix of the laser match
                );
                current_sm = p_sm;
            }
            else
            {
                // add current pcl to current submap
                last_sm->addFrame(*p_in_cloud, t_frame_to_last_map, t_frame_odom);
                current_sm = last_sm;
            }
        }

        if (current_sm == nullptr) return;

        p_out_cloud = current_sm->getSubmapPoints();
    }

    void PclProcessor::startMapServer()
    {
        th_ = std::thread([this](){
            ROS_INFO("Map server started");
            ros::Rate map_publish_rate(5);
            while(ros::ok())
            {
                buildMap();
                publishMap();
                publishMapTf();
                map_publish_rate.sleep();
            }
        });
    }

    void PclProcessor::buildMap()
    {
        global_map_.runGlobalPoseOptimizer();
        // concat all submaps together 
        auto submap_cnt = global_map_.updateGlobalMapPoints();
        ROS_INFO("Submap count: %ld", submap_cnt);
    }

    void PclProcessor::publishMap()
    {
        sensor_msgs::PointCloud2 opt_pc2;
        PointCloudT::Ptr p_global_opt_ds(new PointCloudT);
        auto p_global_opt_pc = global_map_.getGlobalMapOptPoints();
        PreProcessor::downSample(p_global_opt_pc, p_global_opt_ds, 0.01);
        pcl::toROSMsg(*p_global_opt_ds, opt_pc2);
        opt_pc2.header.frame_id = "map";
        opt_pc2.header.stamp = ros::Time::now();
        global_map_opt_pub_.publish(opt_pc2);

        sensor_msgs::PointCloud2 raw_pc2;
        PointCloudT::Ptr p_global_raw_ds(new PointCloudT);
        auto p_global_raw_pc = global_map_.getGlobalMapRawPoints();
        PreProcessor::downSample(p_global_raw_pc, p_global_raw_ds, 0.01);
        pcl::toROSMsg(*p_global_raw_ds, raw_pc2);
        raw_pc2.header.frame_id = "map";
        raw_pc2.header.stamp = ros::Time::now();
        global_map_raw_pub_.publish(raw_pc2);
    }

    void PclProcessor::publishMapTf()
    {
        geometry_msgs::TransformStamped transformStamped;

        //transformStamped.header.seq = msg->header.seq;
        Eigen::Affine3d last_tf(global_map_.getLastSubMapOptTf());
        transformStamped = tf2::eigenToTransform(last_tf);
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "base_world";

        //ROS_INFO("Robot tf: %f %f %f", -p / M_PI * 180, r / M_PI * 180, y / M_PI * 180);
        br_.sendTransform(transformStamped);
    }
}