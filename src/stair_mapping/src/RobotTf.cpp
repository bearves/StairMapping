#include "RobotTf.h"
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>

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

    Eigen::Matrix<double, 4, 6> RobotKinetics::getTipPosWithTouchState(Eigen::Matrix4d tf_from_base_link) const
    {
        // transform the tip points to the target frame from base_link 
        Eigen::Matrix<double, 4, 6> pos_with_touch;
        pos_with_touch.block<3, 6>(0, 0) = tip_pos_wrt_body_;
        pos_with_touch.block<1, 6>(3, 0).setOnes();
        pos_with_touch = tf_from_base_link * pos_with_touch;

        // put the touch states in the last line
        pos_with_touch.block<1, 6>(3, 0) = touch_prob_.cast<double>();
        return pos_with_touch; 
    }

    ImuCalibrator::ImuCalibrator()
    {
        yaw_start_ = 0;
        is_first_imu_msg_ = true;
        is_imu_transform_ready_ = false;
        imu_calibrate_.setZero();
    }

    void ImuCalibrator::setParam(ros::NodeHandle &node)
    {
        node.param("imu_pose_calibrate_r", cali_r, 0.0);
        node.param("imu_pose_calibrate_p", cali_p, 0.0);
        node.param("imu_pose_calibrate_y", cali_y, 0.0);
        imu_calibrate_ << cali_r, cali_p, cali_y;
    }

    geometry_msgs::TransformStamped ImuCalibrator::updateCalibratedImuTf(
            const geometry_msgs::Quaternion& imu_reading)
    {
        using namespace Eigen;

        geometry_msgs::TransformStamped imu_tsfm;

        imu_tsfm.transform.translation.x = 0;
        imu_tsfm.transform.translation.y = 0;
        imu_tsfm.transform.translation.z = 0;
        Quaterniond q_imu, q_body, q_corrected;
        q_imu.w() = imu_reading.w;
        q_imu.x() = imu_reading.x;
        q_imu.y() = imu_reading.y;
        q_imu.z() = imu_reading.z;

        // rotate from imu's cs to the body's cs
        AngleAxisd rot_z(AngleAxisd(-M_PI / 2, Vector3d::UnitZ()));
        // add calibrations of imu pose 
        AngleAxisd rot_cy(AngleAxisd(imu_calibrate_[2], Vector3d::UnitZ()));
        AngleAxisd rot_cp(AngleAxisd(imu_calibrate_[1], Vector3d::UnitY()));
        AngleAxisd rot_cr(AngleAxisd(imu_calibrate_[0], Vector3d::UnitX()));
        q_body = q_imu * rot_z * rot_cy * rot_cp * rot_cr;

        //y = 0;
        if (is_first_imu_msg_)
        {
            auto euler = q_body.toRotationMatrix().eulerAngles(2, 1, 0);
            yaw_start_ = euler[0]; // init yaw angle
            ROS_INFO("Start yaw angle: %lf", yaw_start_);
            is_first_imu_msg_ = false;
        }
        // remove the yaw start rotation
        AngleAxisd rot_z0(AngleAxisd(-yaw_start_, Vector3d::UnitZ()));
        q_corrected = rot_z0 * q_body;

        imu_tsfm.transform.rotation.x = q_corrected.x();
        imu_tsfm.transform.rotation.y = q_corrected.y();
        imu_tsfm.transform.rotation.z = q_corrected.z();
        imu_tsfm.transform.rotation.w = q_corrected.w();

        imu_tf_calibrated_ = q_corrected;

        return imu_tsfm;
    }

    Eigen::Matrix4d ImuCalibrator::getCalibratedImuTfFromBaseLink()
    {
        return Eigen::Affine3d(imu_tf_calibrated_).matrix();
    }

    Eigen::Matrix4d ImuCalibrator::getCalibratedImuTfFromCamera()
    {
        static tf2_ros::Buffer buffer;
        static tf2_ros::TransformListener lsner(buffer);
        geometry_msgs::TransformStamped tf_camera_to_body;
        Eigen::Matrix4d imu_tf_from_camera = Eigen::Matrix4d::Identity();
        try
        {
            tf_camera_to_body = buffer.lookupTransform("base_link", "camera_depth_optical_frame", ros::Time(0));
            Eigen::Matrix4f tm;
            pcl_ros::transformAsMatrix(tf_camera_to_body.transform, tm);

            imu_tf_from_camera = Eigen::Affine3d(imu_tf_calibrated_).matrix() * tm.cast<double>();
            is_imu_transform_ready_ = true;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }

        return imu_tf_from_camera;
    }

} // namespace stair_mapping
