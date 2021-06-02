#pragma once

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <mini_bridge/RobotTipState.h>

namespace stair_mapping
{

class RobotTfBroadcastNode
{
public:
    RobotTfBroadcastNode(ros::NodeHandle& node);


private:
    ros::Subscriber pcl_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber tip_state_sub_;

    ros::Publisher pcl_pub_;
    ros::Publisher tip_points_pub_;
    tf2_ros::TransformBroadcaster br;

    double y_start_ = 0;
    bool is_first_msg_ = true;

    Eigen::Matrix<double, 3, 6> hip_pos_;
    Eigen::Matrix<double, 3, 3> hip_cs_[6];

    void imuCallback(const sensor_msgs::ImuConstPtr &msg);
    void pclDataCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void tipStateCallback(const mini_bridge::RobotTipStateConstPtr &msg);
};
    
} // namespace stair_mapping
