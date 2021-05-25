#include "PclProcessor.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <termios.h>
#include "PreProcessor.h"
#include "HeightMap.h"

namespace stair_mapping
{
    PclProcessor::PclProcessor(ros::NodeHandle& node)
    {
        preprocess_pub_ = node.advertise<sensor_msgs::PointCloud2>("preprocessed_points", 1);
        submap_pub_ = node.advertise<sensor_msgs::PointCloud2>("submap_points", 1);
        global_map_opt_pub_ = node.advertise<sensor_msgs::PointCloud2>("global_map_opt_points", 1);
        global_map_raw_pub_ = node.advertise<sensor_msgs::PointCloud2>("global_map_raw_points", 1);
        //height_map_pub_ = node.advertise<sensor_msgs::PointCloud2>("height_map_pcl", 1);
        //cost_map_pub_ = node.advertise<nav_msgs::OccupancyGrid>("terrain_cost_map", 1);
        odom_sub_ = node.subscribe("/qz_state_publisher/robot_odom", 1, &PclProcessor::odomMsgCallback, this);
        pcl_sub_ = node.subscribe("transformed_points", 1, &PclProcessor::pclMsgCallback, this);
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
        Eigen::Vector2i sizes = PreProcessor::downSample(p_cloud_cr, p_cloud_ds, 0.02);
        ROS_INFO("After downsample size: %d -> %d", sizes[0], sizes[1]);

        p_out_cloud = p_cloud_ds;
    }

    void PclProcessor::submapMatch(const PointCloudT::Ptr &p_in_cloud, PointCloudT::Ptr &p_out_cloud)
    {
        using namespace Eigen;

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
            SubMap::Ptr p_sm(new SubMap);
            p_sm->init();
            global_map_.addNewSubmap(
                p_sm, 
                Matrix4d::Identity(), // m2m laser match
                Matrix4d::Identity(), // m2m odom
                t_frame_odom          // imu
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

        // reject frames when robot is not moving at all
        Vector3d translation_guess = Affine3d(t_guess).translation();
        if (!last_sm->isEmpty() && translation_guess.norm() < 0.01)
        {
            ROS_WARN("Frame too close: %f", translation_guess.norm());
            return;
        }

        try
        {
            score = last_sm->match(*p_in_cloud, t_guess, t_frame_to_last_map);
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

                SubMap::Ptr p_sm(new SubMap);
                p_sm->init();
                p_sm->addFrame(*p_in_cloud, Matrix4d::Identity(), t_frame_odom);
                // match to last submap if exists, record transform Ti
                global_map_.addNewSubmap(
                    p_sm, 
                    t_frame_to_last_map, // m2m scan match
                    t_guess,             // m2m odom
                    t_frame_odom         // global imu
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
        sensor_msgs::PointCloud2 global_map_opt_out_cloud2;
        const PointCloudT::Ptr p_global_opt_points = global_map_.getGlobalMapOptPoints();
        pcl::toROSMsg(*p_global_opt_points, global_map_opt_out_cloud2);
        global_map_opt_out_cloud2.header.frame_id = "map";
        global_map_opt_out_cloud2.header.stamp = ros::Time::now();
        global_map_opt_pub_.publish(global_map_opt_out_cloud2);

        sensor_msgs::PointCloud2 global_map_raw_out_cloud2;
        const PointCloudT::Ptr p_global_raw_points = global_map_.getGlobalMapRawPoints();
        pcl::toROSMsg(*p_global_raw_points, global_map_raw_out_cloud2);
        global_map_raw_out_cloud2.header.frame_id = "map";
        global_map_raw_out_cloud2.header.stamp = ros::Time::now();
        global_map_raw_pub_.publish(global_map_raw_out_cloud2);
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

    void PclProcessor::doProcess(const PointCloudT::Ptr &p_in_cloud, PointCloudT::Ptr &p_out_cloud, 
                       Eigen::MatrixXd& height_map, Eigen::MatrixXd& cost_map)
    {
        // crop
        PointCloudT::Ptr p_cloud_cr(new PointCloudT);
        PreProcessor::crop(p_in_cloud, p_cloud_cr, Eigen::Vector3f(0, -0.5, -0.8), Eigen::Vector3f(2.3, 0.5, 0.2));

        // downsampling
        PointCloudT::Ptr p_cloud_ds(new PointCloudT);
        Eigen::Vector2i sizes = PreProcessor::downSample(p_cloud_cr, p_cloud_ds, 0.04);
        ROS_INFO("After downsample size: %d -> %d", sizes[0], sizes[1]);

        // convert to height map
        PointCloudT::Ptr height_filled_pcl(new PointCloudT);
        PointCloudT::Ptr cost_pcl(new PointCloudT);

        HeightMap::generateHeightMap(p_cloud_ds, height_map);
        HeightMap::fillSmallHoles(height_map);
        HeightMap::generateCostMap(height_map, cost_map);

        HeightMap::getPclFromHeightMap(height_map, height_filled_pcl);
        HeightMap::getPclFromHeightMap(cost_map, cost_pcl);

        p_out_cloud = height_filled_pcl;
    }

    void PclProcessor::generateCostGrid(Eigen::MatrixXd &cost_map, nav_msgs::OccupancyGrid &cost_grid)
    {
        cost_grid.header.frame_id = "base_world";
        cost_grid.header.stamp = ros::Time::now();
        cost_grid.info.height = HeightMap::MAP_ROWS;
        cost_grid.info.width = HeightMap::MAP_COLS;
        cost_grid.info.resolution = HeightMap::GRID_SIZE;
        cost_grid.info.origin.orientation.x = 0;
        cost_grid.info.origin.orientation.y = 0;
        cost_grid.info.origin.orientation.z = 0;
        cost_grid.info.origin.orientation.w = 1;
        cost_grid.info.origin.position.x = HeightMap::MAP_X0;
        cost_grid.info.origin.position.y = -HeightMap::MAP_WIDTH/2;
        cost_grid.info.origin.position.z = -2;
        cost_grid.data.resize(HeightMap::MAP_COLS * HeightMap::MAP_ROWS);

        double cost_threshold = 0.1;

        for (int i = 0; i < HeightMap::MAP_ROWS; i++)
        {
            for (int j = 0; j < HeightMap::MAP_COLS; j++)
            {
                if (cost_map(i, j) < HeightMap::INVALID_VALUE+0.1)
                {
                    cost_grid.data[i * HeightMap::MAP_COLS + j] = -1;
                }
                else if (cost_map(i, j) < cost_threshold)
                {
                    cost_grid.data[i * HeightMap::MAP_COLS + j] = cost_map(i,j)/cost_threshold * 100;
                }
                else
                {
                    cost_grid.data[i * HeightMap::MAP_COLS + j] = 100;
                }
            }
        }
    }
}