#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <eigen3/Eigen/Dense>
#include "pcl_ros/transforms.h"
#include "pcl/common/transforms.h"
#include "pcl/conversions.h"
#include "pcl_ros/point_cloud.h"

ros::Subscriber pcl_sub;
ros::Publisher pcl_pub;

void poseCallback(const sensor_msgs::ImuConstPtr &msg)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    //transformStamped.header.seq = msg->header.seq;
    transformStamped.header.stamp = msg->header.stamp;
    transformStamped.header.frame_id = "base_world";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    tf2::Quaternion q, q_corrected;
    q.setX(msg->orientation.x);
    q.setY(msg->orientation.y);
    q.setZ(msg->orientation.z);
    q.setW(msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);
    y = 0;
    q_corrected.setRPY(-p, r, y);
    //q_corrected.setRPY(r, p, y);

    transformStamped.transform.rotation.x = q_corrected.x();
    transformStamped.transform.rotation.y = q_corrected.y();
    transformStamped.transform.rotation.z = q_corrected.z();
    transformStamped.transform.rotation.w = q_corrected.w();

    //ROS_INFO("Robot tf: %f %f %f", -p / M_PI * 180, r / M_PI * 180, y / M_PI * 180);

    br.sendTransform(transformStamped);
}

void pclDataCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    static tf2_ros::Buffer buffer;
    static tf2_ros::TransformListener lsner(buffer);

    sensor_msgs::PointCloud2 transformed_pcl2;

    try
    {
        //pcl_ros::transformPointCloud("base_world", *msg, transformed_pcl2, buffer);
        ros::Duration deltaT(0.8);
        auto t = buffer.lookupTransform("base_world", msg->header.frame_id, ros::Time(0));
        Eigen::Matrix4f tm;
        pcl_ros::transformAsMatrix(t.transform, tm);
        pcl_ros::transformPointCloud(tm, *msg, transformed_pcl2);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }
    transformed_pcl2.header.frame_id = "base_world";
    transformed_pcl2.header.stamp = msg->header.stamp;
    pcl_pub.publish(transformed_pcl2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_tf_broadcaster");
    ROS_INFO("Robot tf broadcaster");
    ros::NodeHandle node;

    ros::Subscriber imu_sub = node.subscribe("/qz_state_publisher/robot_imu", 10, &poseCallback);
    pcl_pub = node.advertise<sensor_msgs::PointCloud2>("transformed_points", 1);
    pcl_sub = node.subscribe("/camera/depth/color/points", 1, &pclDataCallback);

    ros::spin();
    return 0;
};