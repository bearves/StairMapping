#include "RobotKinetics.h"

namespace stair_mapping
{

    RobotKinetics::RobotKinetics()
        : p_tip_pts_wrt_body_(new PointCloudTC)
    {
        hip_pos_ << -0.340, -0.340, 0.000, 0.340,  0.340,  0.000,
                    -0.050,  0.050, 0.175, 0.050, -0.050, -0.175,
                     0.000,  0.000, 0.000, 0.000,  0.000,  0.000;

        for (int i = 0; i < 6; i++)
        {
            hip_cs_[i] = Eigen::Matrix3d::Identity();
        }

        touch_prob_.setZero();
    }

    void RobotKinetics::updateTouchState(const ros::Time &stamp, const boost::array<float, 6UL> &touch_prob)
    {
        for (int i = 0; i < 6; i++)
        {
            touch_prob_[i] = touch_prob[i];
        }
    }

    void RobotKinetics::updateTipPosition(const ros::Time &stamp, const boost::array<double, 18UL> &tip_pos)
    {
        p_tip_pts_wrt_body_->clear();

        for (int i = 0; i < 6; i++)
        {
            // get message, i.e. tip pos wrt hip
            Eigen::Vector3d tip_pos_wrt_hip(
                tip_pos[i * 3 + 0],
                tip_pos[i * 3 + 1],
                tip_pos[i * 3 + 2]);

            // calculate tip pos wrt body (base_link)
            tip_pos_wrt_body_.col(i) = hip_cs_[i] * tip_pos_wrt_hip + hip_pos_.col(i);
            PointTC point;
            point.getVector3fMap() = tip_pos_wrt_body_.col(i).cast<float>();

            if (touch_prob_[i] > 0.9) // touch, red
            {
                point.r = 255;
                point.g = 0;
                point.b = 0;
            }
            else // not touch, green
            {
                point.r = 0;
                point.g = 255;
                point.b = 0;
            }

            p_tip_pts_wrt_body_->push_back(point);
        }
    }

    const PointCloudTC::Ptr RobotKinetics::getTipPoints() const
    {
        return p_tip_pts_wrt_body_;
    }

    Eigen::Matrix<double, 4, 6> RobotKinetics::getTipPosWithTouchState() const
    {
        Eigen::Matrix<double, 4, 6> pos_with_touch;
        pos_with_touch.block<3, 6>(0, 0) = tip_pos_wrt_body_;
        pos_with_touch.block<1, 6>(3, 0) = touch_prob_.cast<double>();
        return pos_with_touch; 
    }

} // namespace stair_mapping
