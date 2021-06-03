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

        hip_pos_ << 
            -0.340, -0.340, 0.000, 0.340, 0.340, 0.000,
            -0.050,  0.050, 0.175, 0.050,-0.050,-0.175,
             0.000,  0.000, 0.000, 0.000, 0.000, 0.000; 
    
        touch_prob_.setZero();
        
        for (int i = 0; i < 6; i++)
        {
            hip_cs_[i] = Eigen::Matrix3d::Identity();
        }
        node.param("imu_pose_calibrate_r", cali_r, 0.0);
        node.param("imu_pose_calibrate_p", cali_p, 0.0);
        node.param("imu_pose_calibrate_y", cali_y, 0.0);
        imu_calibrate_ << cali_r, cali_p, cali_y;

        ROS_INFO("IMU Pose calibration (RPY): %lf, %lf, %lf", cali_r, cali_p, cali_y);

        pcl_pub_ = node.advertise<sensor_msgs::PointCloud2>("transformed_points", 1);
        tip_points_pub_ = node.advertise<sensor_msgs::PointCloud2>("tip_points", 1);
        imu_sub_ = node.subscribe("/qz_state_publisher/robot_imu", 10, &RobotTfBroadcastNode::imuCallback, this);
        pcl_sub_ = node.subscribe("/camera/depth/color/points", 1, &RobotTfBroadcastNode::pclDataCallback, this);
        tip_state_sub_ = node.subscribe("/qz_state_publisher/robot_tip_state", 10, &RobotTfBroadcastNode::tipStateCallback, this);
        gait_phase_sub_ = node.subscribe("/qz_state_publisher/robot_gait_phase", 10, &RobotTfBroadcastNode::gaitPhaseCallback, this);
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

    void RobotTfBroadcastNode::gaitPhaseCallback(const mini_bridge::GaitPhaseConstPtr &msg)
    {
        using namespace Eigen;
        for(int i = 0; i < 6; i++)
        {
            touch_prob_[i] = msg->touch_possibility[i];
        }
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
    

    void RobotTfBroadcastNode::tipStateCallback(const mini_bridge::RobotTipStateConstPtr &msg)
    {
        pcl::PointCloud<pcl::PointXYZRGB> tip_points;
        tip_points.clear();
        for(int i = 0; i < 6; i++)
        {
            // get message, i.e. tip pos wrt hip
            Eigen::Vector3d tip_pos_wrt_hip(
                msg->tip_pos[i*3+0],
                msg->tip_pos[i*3+1],
                msg->tip_pos[i*3+2]
            );

            // calculate tip pos wrt body (base_link)
            Eigen::Vector3d tip_pos_wrt_body = hip_cs_[i] * tip_pos_wrt_hip + hip_pos_.col(i);
            pcl::PointXYZRGB point;
            point.getVector3fMap() = tip_pos_wrt_body.cast<float>();

            if (touch_prob_[i] > 0.9) // touch, green
            {
                point.r = 255; point.g = 0; point.b = 0;
            }
            else // not touch, red
            {
                point.r = 0; point.g = 255; point.b = 0;
            }

            tip_points.push_back(point);
        }
        sensor_msgs::PointCloud2 pc2;
        pcl::toROSMsg(tip_points, pc2);
        pc2.header.frame_id = "base_link";
        pc2.header.stamp = ros::Time::now();
        tip_points_pub_.publish(pc2);
    }
} // namespace stair_mapping
