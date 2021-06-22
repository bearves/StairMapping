#pragma once

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>

namespace stair_mapping
{

class RobotTfBroadcastNode
{
public:
    RobotTfBroadcastNode(ros::NodeHandle& node);


private:
    ros::Subscriber pcl_sub_;
    ros::Subscriber imu_sub_;

    ros::Publisher pcl_pub_;
    tf2_ros::TransformBroadcaster br;

    double y_start_ = 0;
    bool is_first_msg_ = true;

    Eigen::Vector3d imu_calibrate_;
    double cali_r, cali_p, cali_y;

    void imuCallback(const sensor_msgs::ImuConstPtr &msg);
    void pclDataCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
};
    
} // namespace stair_mapping
