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
        global_height_map_pub_ = node.advertise<sensor_msgs::PointCloud2>("global_height_map", 1);
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

        terrain_mapper_.preprocess(p_in_cloud, p_pre_cloud);

        pcl::toROSMsg(*p_pre_cloud, pre_out_cloud2);
        pre_out_cloud2.header.frame_id = "base_world";
        pre_out_cloud2.header.stamp = ros::Time::now();
        preprocess_pub_.publish(pre_out_cloud2);

        PointCloudT::Ptr p_submap_cloud(new PointCloudT);

        odom_msg_mtx_.lock();
        Matrix4d t_frame_odom = current_odom_mat_;
        odom_msg_mtx_.unlock();
        terrain_mapper_.matchSubmap(p_pre_cloud, p_submap_cloud, t_frame_odom);

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


    void PclProcessor::startMapServer()
    {
        th_ = std::thread([this](){
            ROS_INFO("Map server started");
            ros::Rate map_publish_rate(5);
            while(ros::ok())
            {
                terrain_mapper_.buildGlobalMap();
                publishMap();
                publishMapTf();
                map_publish_rate.sleep();
            }
        });
    }

    void PclProcessor::publishMap()
    {
        sensor_msgs::PointCloud2 opt_pc2;
        auto p_global_opt_pc = terrain_mapper_.getGlobalMapOptPoints();
        pcl::toROSMsg(*p_global_opt_pc, opt_pc2);
        opt_pc2.header.frame_id = "map";
        opt_pc2.header.stamp = ros::Time::now();
        global_map_opt_pub_.publish(opt_pc2);

        sensor_msgs::PointCloud2 raw_pc2;
        auto p_global_raw_pc = terrain_mapper_.getGlobalMapRawPoints();
        pcl::toROSMsg(*p_global_raw_pc, raw_pc2);
        raw_pc2.header.frame_id = "map";
        raw_pc2.header.stamp = ros::Time::now();
        global_map_raw_pub_.publish(raw_pc2);

        sensor_msgs::PointCloud2 elevation_pc2;
        auto p_elevation_pc = terrain_mapper_.getElevationGridPoints();
        pcl::toROSMsg(*p_elevation_pc, elevation_pc2);
        elevation_pc2.header.frame_id = "map";
        elevation_pc2.header.stamp = ros::Time::now();
        global_height_map_pub_.publish(elevation_pc2);
    }

    void PclProcessor::publishMapTf()
    {
        geometry_msgs::TransformStamped transformStamped;

        //transformStamped.header.seq = msg->header.seq;
        Eigen::Affine3d last_tf(terrain_mapper_.getLastSubMapOptTf());
        transformStamped = tf2::eigenToTransform(last_tf);
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "base_world";

        //ROS_INFO("Robot tf: %f %f %f", -p / M_PI * 180, r / M_PI * 180, y / M_PI * 180);
        br_.sendTransform(transformStamped);
    }

}