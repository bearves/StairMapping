#include "ImageProcessor.h"
#include <sstream>

namespace stair_mapping
{
    ImageProcessor::ImageProcessor(ros::NodeHandle& node)
    {
        it_ptr_ = std::make_shared<image_transport::ImageTransport>(node);
        output_pub_ = it_ptr_->advertise("out_image", 1);
        camera_info_sub_ = node.subscribe("/camera/color/camera_info", 1, &ImageProcessor::cameraInfoMsgCallback, this);
        source_sub_ = it_ptr_->subscribe("/camera/color/image_raw", 1, &ImageProcessor::imageMsgCallback, this);
    }

    void ImageProcessor::cameraInfoMsgCallback(const sensor_msgs::CameraInfoConstPtr &msg)
    {
        camera_info_ = *msg;
        // deep copy of msg->D since it is a vector
        for(int i = 0; i < msg->D.size(); i++)
        {
            camera_info_.D[i] = msg->D[i];
        }
    }

    void ImageProcessor::imageMsgCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        // get raw image
        cv::Mat input_image, output_image;
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv_ptr->image.copyTo(input_image);

        // process
        doProcess(input_image, output_image);

        // publish processed image
        ros::Time time = ros::Time::now();
        cv_bridge::CvImage cvi;
        cvi.header.stamp = time;
        cvi.header.frame_id = "image";
        cvi.encoding = "bgr8";
        cvi.image = output_image;

        sensor_msgs::Image im;
        cvi.toImageMsg(im);
        output_pub_.publish(im);
        ROS_INFO("Converted Successfully!");
    }

    void ImageProcessor::doProcess(cv::Mat& in_image, cv::Mat& out_image)
    {
        cv::Mat canny, blur, denoise;
        in_image.copyTo(out_image);

        // blur for color extraction
        cv::GaussianBlur(in_image, denoise, cv::Size(5,5), 5);

        // extract orange channel
        cv::Mat hsv_img, mask, mask_result;
        cv::cvtColor(denoise, hsv_img, cv::COLOR_BGR2HSV);
        cv::inRange(hsv_img, cv::Vec3i(5, 40, 20), cv::Vec3i(30, 180, 255), mask);
        // remove small particals
        cv::morphologyEx(mask, mask, 
            cv::MorphTypes::MORPH_OPEN, 
            cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, cv::Size(11, 11)));
        cv::morphologyEx(mask, mask, 
            cv::MorphTypes::MORPH_CLOSE, 
            cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, cv::Size(5, 5)));

        // crop areas in which we are not insterested 
        cv::rectangle(mask, cv::Rect(0, 0, mask.cols, mask.rows*0.35), cv::Scalar(0), cv::FILLED);
        cv::rectangle(mask, cv::Rect(0, 0, mask.cols*0.2, mask.rows), cv::Scalar(0), cv::FILLED);
        cv::rectangle(mask, cv::Rect(mask.cols*0.8, 0, mask.cols*0.2, mask.rows), cv::Scalar(0), cv::FILLED);
        
        // find contours
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // remove small blobs
        std::vector<std::vector<cv::Point> > valid_contours;
        valid_contours.clear();
        for(int i = 0; i < contours.size(); i++) 
        {
            auto cont = contours[i];
            double area = cv::contourArea(cont);
            if (area < 600) 
                continue;
            valid_contours.push_back(cont);
        }
        if (valid_contours.size() > 3) 
            ROS_WARN("Unknown blobs may be found");

        // refine contours and find rotated rectangles
        std::vector<std::vector<cv::Point> > hulls;
        std::vector<std::vector<cv::Point2f> > rects;
        std::vector<cv::Point2f> centers;
        for(auto& cont : valid_contours) 
        {
            // convex hull
            std::vector<cv::Point> hull;
            cv::convexHull(cont, hull);
            hulls.push_back(hull);
            // min reactangle
            auto rect = cv::minAreaRect(hull);
            cv::Point2f vtx[4];
            rect.points(vtx);
            rects.push_back(std::vector<cv::Point2f>(std::begin(vtx), std::end(vtx)));
            // center of the contour
            auto m = cv::moments(hull);
            cv::Point contour_center(int(m.m10/m.m00), int(m.m01/m.m00));
            centers.push_back(contour_center);
        }
        cv::Mat blank = cv::Mat::zeros(in_image.size(), CV_8UC1);
        cv::drawContours(blank, hulls, -1, cv::Scalar(255), 2);
        ROS_INFO("Found %ld contours", hulls.size());

        cv::cvtColor(blank, out_image, cv::COLOR_GRAY2BGR);
        for (auto &rect_vtx : rects)
        {
            for (int j = 0; j < 4; j++)
            {
                cv::line(out_image, rect_vtx[j], rect_vtx[(j + 1) % 4], cv::Scalar(255,0,0), 2);
            }
        }

        cv::Vec3d point3d_cam, point3d_world;
        for (auto point : centers)
        {
            cv::circle(out_image, point, 1, cv::Scalar(0, 255, 0), 2);
            transformFromImageToGround(point, point3d_cam, point3d_world);

            char label_text[50];
            sprintf(label_text, "x: %.3f, y: %.3f", point3d_world[0], point3d_world[1]);
            
            // add label on image
            cv::putText(
                out_image, 
                label_text,
                cv::Point(point.x+3, point.y-3), 
                CV_FONT_HERSHEY_SIMPLEX, 
                1,  // scale
                cv::Scalar(0, 240, 240), // color
                2,  // thickness
                cv::LineTypes::LINE_AA);

        }
    }
    void ImageProcessor::transformFromImageToGround(
        cv::Point2f &point2D_on_image, 
        cv::Vec3d &point3D_camera, 
        cv::Vec3d &point3D_world)
    {
        double cx = camera_info_.K[2];
        double cy = camera_info_.K[5];
        double fx = camera_info_.K[0];
        double fy = camera_info_.K[4];
        double kx = (point2D_on_image.x - cx)/fx;
        double ky = (point2D_on_image.y - cy)/fy;

        Eigen::Vector3d nz(0, 0, 1);
        Eigen::Vector3d rwb(0, 0, 0.5);
        Eigen::Vector3d rbc(0.22, 0, 0.26);
        Eigen::Matrix3d Rwb = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d Rbc;
        Rbc << 0., -0.34202014, 0.93969262,
              -1.,          0.,         0.,
               0., -0.93969262, -0.34202014;

        Eigen::Matrix<double, 1, 3> r1 = nz.transpose() * Rwb * Rbc;
        double h = -nz.transpose() * (rwb + Rwb * rbc);

        Eigen::Matrix3d M;
        M << r1(0), r1(1), r1(2),
                 1,     0,   -kx,
                 0,     1,   -ky; 
        Eigen::Vector3d rhs(h, 0, 0);

        Eigen::Vector3d pc = M.colPivHouseholderQr().solve(rhs);
        Eigen::Vector3d pw = Rwb * (Rbc * pc + rbc) + rwb;

        point3D_camera = vecEigen2CV(pc);
        point3D_world = vecEigen2CV(pw);
    }

    cv::Vec3d ImageProcessor::vecEigen2CV(Eigen::Vector3d p)
    {
        return cv::Vec3d(p[0], p[1], p[2]);
    }
}