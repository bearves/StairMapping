#include <ros/ros.h>
#include "Terrain3dMapperNode.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_data_process");
    ros::NodeHandle node;
    ROS_INFO("Start pcl data node");
    stair_mapping::Terrain3dMapperNode terrain_3d_map_node(node);
    terrain_3d_map_node.startMapServer();
    ros::MultiThreadedSpinner s(4); // Use 4 threads
    ros::spin(s);
    return 0;
}