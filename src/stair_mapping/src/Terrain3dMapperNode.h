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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
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
        typedef sensor_msgs::Image Img;
        typedef sensor_msgs::CameraInfo CamInfo;
        typedef message_filters::TimeSynchronizer<Img, Img, CamInfo> RGBDSync;

        ros::Subscriber odom_sub_;
        ros::Subscriber tip_state_sub_;
        ros::Subscriber gait_phase_sub_;
        ros::Subscriber imu_sub_;
        message_filters::Subscriber<sensor_msgs::Image> color_img_sub_;
        message_filters::Subscriber<sensor_msgs::Image> depth_img_sub_;
        message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub_;
        std::shared_ptr<RGBDSync> p_sync_rgbd_;

        ros::Publisher rgbd_converted_pub_;
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

        void pclFrontend(
            const PtCldPtr& p_in_cloud, 
            const open3d::geometry::RGBDImage& rgbd_img, 
            const open3d::camera::PinholeCameraIntrinsic& intrinsic,
            const ros::Time& stamp);

        void odomMsgCallback(const nav_msgs::OdometryConstPtr &msg);
        void imuMsgCallback(const sensor_msgs::ImuConstPtr &msg);
        void tipStateCallback(const mini_bridge::RobotTipStateConstPtr &msg);
        void gaitPhaseCallback(const mini_bridge::GaitPhaseConstPtr &msg);
        void tipStateV2Callback(const mini_bridge::RobotTipStateV2ConstPtr &msg);
        void gaitPhaseV2Callback(const mini_bridge::GaitPhaseV2ConstPtr &msg);
        void rgbdImgMsgCallback(
            const Img::ConstPtr &color_msg,
            const Img::ConstPtr &depth_msg,
            const CamInfo::ConstPtr &caminfo_msg);

        Eigen::Matrix4d getPoseMatrix(const nav_msgs::Odometry &odom);

        void publishCorrectedTf(const ros::Time& stamp, const Eigen::Matrix4d& original_robot_tf);
        void publishMap();
    };
}