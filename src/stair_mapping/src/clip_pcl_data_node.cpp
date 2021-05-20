#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <termios.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;
ros::Subscriber sub;
ros::Publisher pub;

void pclDataCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    PointCloudT cloud;
    pcl::fromROSMsg(*msg, cloud);
    
    int c = getchar();
    char frame_file_name[200];
    sprintf(frame_file_name, "key_frame_%d.pcd", msg->header.seq);
    std::string file_name(frame_file_name);
    pcl::io::savePCDFileASCII(file_name, cloud);
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