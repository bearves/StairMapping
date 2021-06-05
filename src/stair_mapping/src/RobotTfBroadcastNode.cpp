#include "RobotTfBroadcastNode.h"

#include <eigen3/Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace stair_mapping
{
    RobotTfBroadcastNode::RobotTfBroadcastNode(ros::NodeHandle& node)
    {
        y_start_ = 0;
        is_first_msg_ = true;

        node.param("imu_pose_calibrate_r", cali_r, 0.0);
        node.param("imu_pose_calibrate_p", cali_p, 0.0);
        node.param("imu_pose_calibrate_y", cali_y, 0.0);
        imu_calibrate_ << cali_r, cali_p, cali_y;

        ROS_INFO("IMU Pose calibration (RPY): %lf, %lf, %lf", cali_r, cali_p, cali_y);

        pcl_pub_ = node.advertise<sensor_msgs::PointCloud2>("transformed_points", 1);
        imu_sub_ = node.subscribe("/qz_state_publisher/robot_imu", 10, &RobotTfBroadcastNode::imuCallback, this);
        pcl_sub_ = node.subscribe("/camera/depth/color/points", 1, &RobotTfBroadcastNode::pclDataCallback, this);
    }

    void RobotTfBroadcastNode::imuCallback(const sensor_msgs::ImuConstPtr &msg)
    {
        using namespace Eigen;
        geometry_msgs::TransformStamped transformStamped;

        //transformStamped.header.seq = msg->header.seq;
        transformStamped.header.stamp = msg->header.stamp;
        transformStamped.header.frame_id = "base_world";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = 0;
        transformStamped.transform.translation.y = 0;
        transformStamped.transform.translation.z = 0;
        Quaterniond q_imu, q_body, q_corrected;
        q_imu.w() = msg->orientation.w;
        q_imu.x() = msg->orientation.x;
        q_imu.y() = msg->orientation.y;
        q_imu.z() = msg->orientation.z;

        // rotate from imu's cs to the body's cs
        AngleAxisd rot_z(AngleAxisd(-M_PI / 2, Vector3d::UnitZ()));
        // add calibrations of imu pose 
        AngleAxisd rot_cy(AngleAxisd(imu_calibrate_[2], Vector3d::UnitZ()));
        AngleAxisd rot_cp(AngleAxisd(imu_calibrate_[1], Vector3d::UnitY()));
        AngleAxisd rot_cr(AngleAxisd(imu_calibrate_[0], Vector3d::UnitX()));
        q_body = q_imu * rot_z * rot_cy * rot_cp * rot_cr;

        //y = 0;
        if (is_first_msg_)
        {
            auto euler = q_body.toRotationMatrix().eulerAngles(2, 1, 0);
            y_start_ = euler[0]; // init yaw angle
            ROS_INFO("Start yaw angle: %lf", y_start_);
            is_first_msg_ = false;
        }
        // remove the yaw start rotation
        AngleAxisd rot_z0(AngleAxisd(-y_start_, Vector3d::UnitZ()));
        q_corrected = rot_z0 * q_body;

        transformStamped.transform.rotation.x = q_corrected.x();
        transformStamped.transform.rotation.y = q_corrected.y();
        transformStamped.transform.rotation.z = q_corrected.z();
        transformStamped.transform.rotation.w = q_corrected.w();

        //ROS_INFO("Robot tf: %f %f %f", -p / M_PI * 180, r / M_PI * 180, y / M_PI * 180);

        br.sendTransform(transformStamped);
    }

    void RobotTfBroadcastNode::pclDataCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        static tf2_ros::Buffer buffer;
        static tf2_ros::TransformListener lsner(buffer);

        sensor_msgs::PointCloud2 transformed_pcl2;

        ros::Duration timeout(0.3);
        ros::Duration deltaT(0.8);
        auto t_align = msg->header.stamp + deltaT;
        geometry_msgs::TransformStamped transform;
        try
        {
            //if (buffer.canTransform("base_world", msg->header.frame_id, t_align, timeout))
            if (0)
            {
                transform = buffer.lookupTransform("base_world", msg->header.frame_id, t_align);
            }
            else
            {
                transform = buffer.lookupTransform("base_world", msg->header.frame_id, ros::Time(0));
            }
            Eigen::Matrix4f tm;
            pcl_ros::transformAsMatrix(transform.transform, tm);
            pcl_ros::transformPointCloud(tm, *msg, transformed_pcl2);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }
        transformed_pcl2.header.frame_id = "base_world";
        transformed_pcl2.header.stamp = msg->header.stamp;
        pcl_pub_.publish(transformed_pcl2);
    }

} // namespace stair_mapping
