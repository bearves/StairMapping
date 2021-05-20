#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/photo.hpp>
#include <eigen3/Eigen/Dense>

namespace stair_mapping 
{
    class ImageProcessor
    {
    public:
        ImageProcessor(ros::NodeHandle& node);
    private:
        std::shared_ptr<image_transport::ImageTransport> it_ptr_;
        image_transport::Subscriber source_sub_;
        image_transport::Publisher output_pub_;
        ros::Subscriber camera_info_sub_;
        sensor_msgs::CameraInfo camera_info_;
        void imageMsgCallback(const sensor_msgs::ImageConstPtr &msg);
        void cameraInfoMsgCallback(const sensor_msgs::CameraInfoConstPtr &msg);
        void doProcess(cv::Mat& in_image, cv::Mat& out_image);
        void transformFromImageToGround(
            cv::Point2f& point2D_on_image, cv::Vec3d& point3D_camera, cv::Vec3d& point3D_world);
        
        cv::Vec3d vecEigen2CV(Eigen::Vector3d p);
    };
}