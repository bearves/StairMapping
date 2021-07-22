#pragma once

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Dense>
#include <thread>
#include <mutex>
#include "mini_bridge/GaitPhase.h"
#include "mini_bridge/RobotTipState.h"
#include "mini_bridge/GaitPhaseV2.h"
#include "mini_bridge/RobotTipStateV2.h"
#include "Terrain3dMapper.h"
#include "RobotTf.h"

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
        ros::Subscriber imu_sub_;

        ros::Publisher imu_transformed_pub_;
        ros::Publisher preprocess_pub_;
        ros::Publisher submap_pub_;
        ros::Publisher global_map_opt_pub_;
        ros::Publisher global_map_raw_pub_;
        ros::Publisher gnd_patch_pub_;
        ros::Publisher corrected_odom_pub_;
        ros::Publisher tip_points_pub_;

        bool display_process_details_{false};
        int message_version_{2};

        tf2_ros::TransformBroadcaster br_;

        std::thread th_;        
        std::mutex odom_msg_mtx_;
        std::mutex imu_msg_mtx_;
        std::mutex tip_msg_mtx_;

        Eigen::Matrix4d current_odom_mat_; // ^wT_b_i
        Eigen::Matrix4d current_tf_of_baselink_wrt_world_;  // ^wT_b_i (rotation part)
        Eigen::Matrix4d current_tf_of_camera_wrt_baselink_; // ^bT_c
        bool is_imu_transform_ok_{false};

        Terrain3dMapper terrain_mapper_;
        RobotKinetics robot_kin_;
        ImuCalibrator imu_calibrator_;

        void pclMsgCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
        void odomMsgCallback(const nav_msgs::OdometryConstPtr &msg);
        void imuMsgCallback(const sensor_msgs::ImuConstPtr &msg);
        void tipStateCallback(const mini_bridge::RobotTipStateConstPtr &msg);
        void gaitPhaseCallback(const mini_bridge::GaitPhaseConstPtr &msg);
        void tipStateV2Callback(const mini_bridge::RobotTipStateV2ConstPtr &msg);
        void gaitPhaseV2Callback(const mini_bridge::GaitPhaseV2ConstPtr &msg);

        Eigen::Matrix4d getPoseMatrix(const nav_msgs::Odometry &odom);

        void publishCorrectedTf(const ros::Time& stamp, const Eigen::Matrix4d& original_robot_tf);
        void publishMap();
    };
}