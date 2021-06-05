#pragma once

#include <ros/ros.h>
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
            const PointCloudTC::Ptr getTipPoints() const;
            Eigen::Matrix<double, 4, 6> getTipPosWithTouchState() const;

        private:
            Eigen::Matrix<double, 3, 6> hip_pos_;
            Eigen::Matrix<double, 3, 6> tip_pos_wrt_body_;
            Eigen::Matrix<double, 3, 3> hip_cs_[6];

            Eigen::Matrix<float, 1, 6> touch_prob_;

            PointCloudTC::Ptr p_tip_pts_wrt_body_;
    };
    
} // namespace stair_mapping
