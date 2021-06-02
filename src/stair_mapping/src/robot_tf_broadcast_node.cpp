#include <ros/ros.h>
#include "RobotTfBroadcastNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_tf_broadcaster");
    ROS_INFO("Robot tf broadcaster");
    ros::NodeHandle node;
    stair_mapping::RobotTfBroadcastNode rbt_tf_brdcst_node(node);
    ros::spin();
    return 0;
};