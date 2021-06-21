#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <eigen3/Eigen/Dense>
#include "PointType.h"

namespace stair_mapping
{
    class RobotKinetics
    {
        public:
            RobotKinetics();

            void updateTouchState(const ros::Time& stamp, const boost::array<float, 6UL>& touch_prob);
            void updateTipPosition(const ros::Time& stamp, const boost::array<double, 18UL>& tip_pos);
            const PtCldPtr getTipPoints() const;
            Eigen::Matrix<double, 4, 6> getTipPosWithTouchState(Eigen::Matrix4d tf_from_base_link) const;

        private:
            Eigen::Matrix<double, 3, 6> hip_pos_;
            Eigen::Matrix<double, 3, 6> tip_pos_wrt_body_;
            Eigen::Matrix<double, 3, 3> hip_cs_[6];

            Eigen::Matrix<float, 1, 6> touch_prob_;

            PtCldPtr p_tip_pts_wrt_body_;
    };

    class ImuCalibrator
    {
    public:
        ImuCalibrator();
        void setParam(ros::NodeHandle& node);
        geometry_msgs::TransformStamped updateCalibratedImuTf(
            const geometry_msgs::Quaternion& imu_reading);

        Eigen::Matrix4d getCalibratedImuTfFromCamera();
        Eigen::Matrix4d getCalibratedImuTfFromBaseLink();

        inline bool isImuTransformReady() const { return is_imu_transform_ready_; }

    private:
        double yaw_start_ = 0;
        bool is_first_imu_msg_ = true;
        bool is_imu_transform_ready_ = false;
        Eigen::Vector3d imu_calibrate_;
        double cali_r, cali_p, cali_y;

        // transform from base_link to base_world
        Eigen::Quaterniond imu_tf_calibrated_;

        // Custom quaternion to RPY implementation. The implementation from EIGEN lib
        // has a wrong value range for common use in the direction-heading scenario
        Eigen::Vector3d quatToEulerAngle(Eigen::Quaterniond data);
    };
    
} // namespace stair_mapping
