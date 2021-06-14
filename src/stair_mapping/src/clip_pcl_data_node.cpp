#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <termios.h>
#include <open3d/Open3D.h>
#include <open3d_conversions/open3d_conversions.h>
#include "PointType.h"

ros::Subscriber sub;
ros::Publisher pub;

void pclDataCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    using namespace stair_mapping;
    PtCld cloud;
    open3d_conversions::rosToOpen3d(msg, cloud);
    
    int c = getchar();
    char frame_file_name[200];
    sprintf(frame_file_name, "key_frame_%d.pcd", msg->header.seq);
    std::string file_name(frame_file_name);
    open3d::io::WritePointCloudToPCD(frame_file_name, cloud, 
        open3d::io::WritePointCloudOption(true));
    ROS_INFO("Pcd %s saved", file_name.c_str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_data_process");
    ros::NodeHandle node;
    ROS_INFO("Start pcl data node");
    pub = node.advertise<sensor_msgs::PointCloud2>("transformed_points", 1);
    sub = node.subscribe("/camera/depth/color/points", 1, &pclDataCallback);
    ros::spin();
    return 0;
}