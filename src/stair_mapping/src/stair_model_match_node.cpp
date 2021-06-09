#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <termios.h>
#include "PointType.h"
#include "PreProcessor.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;
ros::Subscriber sub;
ros::Publisher pub;

void pclDataCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    PointCloudT::Ptr cloud(new PointCloudT);
    PointCloudT::Ptr cloud_ds(new PointCloudT);
    PointCloudN::Ptr cloud_normal(new PointCloudN);
    pcl::fromROSMsg(*msg, *cloud);

    pcl::console::TicToc timer;

    timer.tic();
    PreProcessor::downSample(cloud, cloud_ds, 0.03);
    
    pcl::search::KdTree<PointT>::Ptr p_tree(new pcl::search::KdTree<PointT>);
    pcl::NormalEstimationOMP<PointT, PointN> ne;
    ne.setInputCloud(cloud_ds);
    ne.setSearchMethod(p_tree);
    ne.setKSearch(10);
    ne.compute(*cloud_normal);
    ROS_INFO("Compute normal in %lf ms", timer.toc());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stair_model_match_node");
    ros::NodeHandle node;
    ROS_INFO("Start model match node");
    pub = node.advertise<sensor_msgs::PointCloud2>("feature_points", 1);
    sub = node.subscribe("transformed_points", 1, &pclDataCallback);
    ros::spin();
    return 0;
}