#include <ros/ros.h>
#include "PclProcessor.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_data_process");
    ros::NodeHandle node;
    stair_mapping::PclProcessor pcl_processor(node);
    ROS_INFO("Start pcl data node");
    ros::spin();
    return 0;
}