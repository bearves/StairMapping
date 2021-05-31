#pragma once

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <thread>
#include "TerrainMapper.h"
#include <mutex>

namespace stair_mapping 
{
    class PclProcessor
    {
    public:
        PclProcessor(ros::NodeHandle& node);
        void startMapServer();
    private:
        ros::Subscriber pcl_sub_;
        ros::Subscriber odom_sub_;

        ros::Publisher preprocess_pub_;
        ros::Publisher submap_pub_;
        ros::Publisher global_map_opt_pub_;
        ros::Publisher global_map_raw_pub_;
        ros::Publisher global_height_map_pub_;
        ros::Publisher cost_map_pub_;
        tf2_ros::TransformBroadcaster br_;

        std::thread th_;        
        std::mutex odom_msg_mtx_;

        Eigen::Matrix4d current_odom_mat_;
        TerrainMapper terrain_mapper_;

        void pclMsgCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
        void odomMsgCallback(const nav_msgs::OdometryConstPtr &msg);

        Eigen::Matrix4d getPoseMatrix(const nav_msgs::Odometry &odom);

        void publishCorrectedTf(const ros::Time& stamp, const Eigen::Matrix4d& original_robot_tf);
        void publishMap();
    };
}