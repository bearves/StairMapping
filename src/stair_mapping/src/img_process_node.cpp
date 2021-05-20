#include <ros/ros.h>
#include "ImageProcessor.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_process");
    ros::NodeHandle node;
    stair_mapping::ImageProcessor processor(node);
    ROS_INFO("Start imag processing node");
    ros::spin();
    return 0;
}