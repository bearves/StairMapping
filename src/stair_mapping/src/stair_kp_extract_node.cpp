#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <termios.h>
#include "PointType.h"
#include "PreProcessor.h"

typedef pcl::PointXYZI PointTI;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudTI;

ros::Subscriber sub;
ros::Publisher pub;
ros::Publisher pub2;

namespace pcl
{
    template <>
    struct SIFTKeypointFieldSelector<PointXYZ>
    {
        inline float
        operator()(const PointXYZ &p) const
        {
            return p.z;
        }
    };
}

void key_point_harris(PointCloudT::Ptr& cloud_in, PointCloudN::Ptr& cloud_normal_in, PointCloudTI::Ptr& cloud_out);
void key_point_iss3d(PointCloudT::Ptr& cloud_in, PointCloudN::Ptr& cloud_normal_in, PointCloudTI::Ptr& cloud_out);
void key_point_sift(PointCloudT::Ptr& cloud_in, PointCloudN::Ptr& cloud_normal_in, PointCloudTI::Ptr& cloud_out);

void pclDataCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    PointCloudT::Ptr cloud(new PointCloudT);
    PointCloudT::Ptr cloud_cr(new PointCloudT);
    PointCloudT::Ptr cloud_ds(new PointCloudT);
    PointCloudN::Ptr cloud_normal(new PointCloudN);
    pcl::fromROSMsg(*msg, *cloud);

    pcl::console::TicToc timer;

    timer.tic();
    PreProcessor::crop(cloud, cloud_cr, 
        Eigen::Vector3f(0, -0.5, -0.8),
        Eigen::Vector3f(3.0, 0.5, 2.0));
    PreProcessor::downSample(cloud_cr, cloud_ds, 0.03);

    pcl::search::KdTree<PointT>::Ptr p_tree(new pcl::search::KdTree<PointT>);
    pcl::NormalEstimationOMP<PointT, PointN> ne;
    ne.setInputCloud(cloud_ds);
    ne.setSearchMethod(p_tree);
    ne.setKSearch(10);
    ne.compute(*cloud_normal);

    PointCloudTI::Ptr keypoints(new PointCloudTI); 

    //key_point_sift(cloud_ds, cloud_normal, keypoints);
    key_point_harris(cloud_ds, cloud_normal, keypoints);
    //key_point_iss3d(cloud_ds, cloud_normal, keypoints);

    ROS_INFO("Compute kp in %lf ms", timer.toc());
    ROS_INFO_STREAM("Keypoints number: " << keypoints->size());

    sensor_msgs::PointCloud2 kp_pc2;
    pcl::toROSMsg(*keypoints, kp_pc2);
    kp_pc2.header.stamp = msg->header.stamp;
    kp_pc2.header.frame_id = msg->header.frame_id;

    sensor_msgs::PointCloud2 cl_pc2;
    pcl::toROSMsg(*cloud_ds, cl_pc2);
    cl_pc2.header.stamp = msg->header.stamp;
    cl_pc2.header.frame_id = msg->header.frame_id;

    pub.publish(kp_pc2);
    pub2.publish(cl_pc2);
}

void key_point_harris(PointCloudT::Ptr& cloud_in, PointCloudN::Ptr& cloud_normal_in, PointCloudTI::Ptr& cloud_out)
{
    pcl::HarrisKeypoint3D<PointT, PointTI, PointN> harris_detector;

    harris_detector.setNonMaxSupression(true);
    harris_detector.setNormals(cloud_normal_in);
    harris_detector.setRadiusSearch(0.05);
    harris_detector.setThreshold(0.00000001);
    harris_detector.setNumberOfThreads(6);
    harris_detector.setInputCloud(cloud_in);  
    harris_detector.compute(*cloud_out); 
}

void key_point_iss3d(PointCloudT::Ptr& cloud_in, PointCloudN::Ptr& cloud_normal_in, PointCloudTI::Ptr& cloud_out)
{
    pcl::ISSKeypoint3D<PointT, PointTI> iss_detector;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>); 

    double model_solution = 0.01; 
    iss_detector.setSearchMethod(tree);
    iss_detector.setNormals(cloud_normal_in);
    iss_detector.setSalientRadius(6 * model_solution);
    iss_detector.setNonMaxRadius(4 * model_solution);
    iss_detector.setThreshold21(0.975);
    iss_detector.setThreshold32(0.975);
    iss_detector.setMinNeighbors(5);
    iss_detector.setNumberOfThreads(6);
    iss_detector.setInputCloud(cloud_in);
    iss_detector.compute(*cloud_out);
}

void key_point_sift(PointCloudT::Ptr &cloud_in, PointCloudN::Ptr &cloud_normal_in, PointCloudTI::Ptr &cloud_out)
{
    pcl::SIFTKeypoint<PointT, PointTI> sift_detector; 
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);  

    const float min_scale = 0.01f;    //the standard deviation of the smallest scale in the scale space
    const int n_octaves = 10;           //the number of octaves (i.e. doublings os scale) to compute
    const int n_scales_per_octave = 2;  //the number of scales to compute within each octave
    const float min_contrast = 0.01f; //the minimum contrast required for detection
    sift_detector.setSearchMethod(tree);
    sift_detector.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift_detector.setMinimumContrast(min_contrast);
    sift_detector.setInputCloud(cloud_in);
    sift_detector.compute(*cloud_out);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stair_model_match_node");
    ros::NodeHandle node;
    ROS_INFO("Start model match node");
    pub = node.advertise<sensor_msgs::PointCloud2>("feature_points", 1);
    pub2 = node.advertise<sensor_msgs::PointCloud2>("processed_cloud", 1);
    sub = node.subscribe("transformed_points", 1, &pclDataCallback);
    ros::spin();
    return 0;
}