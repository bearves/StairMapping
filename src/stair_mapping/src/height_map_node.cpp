#include <ros/ros.h>
#include "HeightMapNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "height_map_server");
    ROS_INFO("Height map server");
    ros::NodeHandle node;
    stair_mapping::HeightMapNode height_map_node(node);

    ros::MultiThreadedSpinner s(2); // Use 4 threads
    ros::spin(s);
    return 0;
};