#pragma once

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Dense>
#include <thread>
#include <mutex>
#include "mini_bridge/GaitPhase.h"
#include "mini_bridge/RobotTipState.h"
#include "Terrain3dMapper.h"
#include "RobotKinetics.h"

namespace stair_mapping 
{
    class Terrain3dMapperNode 
    {
    public:
        Terrain3dMapperNode(ros::NodeHandle& node);
        void startMapServer();
    private:
        ros::Subscriber pcl_sub_;
        ros::Subscriber odom_sub_;
        ros::Subscriber tip_state_sub_;
        ros::Subscriber gait_phase_sub_;

        ros::Publisher preprocess_pub_;
        ros::Publisher submap_pub_;
        ros::Publisher global_map_opt_pub_;
        ros::Publisher global_map_raw_pub_;
        ros::Publisher corrected_odom_pub_;
        ros::Publisher tip_points_pub_;

        tf2_ros::TransformBroadcaster br_;

        std::thread th_;        
        std::mutex odom_msg_mtx_;

        Eigen::Matrix4d current_odom_mat_;
        Terrain3dMapper terrain_mapper_;
        RobotKinetics robot_kin_;

        void pclMsgCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
        void odomMsgCallback(const nav_msgs::OdometryConstPtr &msg);
        void tipStateCallback(const mini_bridge::RobotTipStateConstPtr &msg);
        void gaitPhaseCallback(const mini_bridge::GaitPhaseConstPtr &msg);

        Eigen::Matrix4d getPoseMatrix(const nav_msgs::Odometry &odom);

        void publishCorrectedTf(const ros::Time& stamp, const Eigen::Matrix4d& original_robot_tf);
        void publishMap();
    };
}