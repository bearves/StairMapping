#include "PreProcessor.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <Eigen/Dense>

Eigen::Vector2i PreProcessor::downSample(
    const PointCloudT::Ptr p_input_cloud, 
    PointCloudT::Ptr p_output_cloud, 
    double leaf_size,
    uint minimum_point_per_voxel)
{
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(p_input_cloud);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.setMinimumPointsNumberPerVoxel(minimum_point_per_voxel);
    vg.filter(*p_output_cloud);
    int original_size = p_input_cloud->width * p_input_cloud->height;
    int resultant_size = p_output_cloud->width * p_output_cloud->height;
    return Eigen::Vector2i(original_size, resultant_size);
}

void PreProcessor::crop(const PointCloudT::Ptr p_input_cloud, PointCloudT::Ptr p_output_cloud, Eigen::Vector3f min, Eigen::Vector3f max)
{
    pcl::PassThrough<PointT> ptx, pty, ptz;
    PointCloudT::Ptr p_after_x(new PointCloudT);
    PointCloudT::Ptr p_after_y(new PointCloudT);

    ptx.setInputCloud(p_input_cloud);
    ptx.setFilterFieldName ("x");
    ptx.setFilterLimits(min[0], max[0]);
    ptx.filter(*p_after_x);

    pty.setInputCloud(p_after_x);
    pty.setFilterFieldName ("y");
    pty.setFilterLimits(min[1], max[1]);
    pty.filter(*p_after_y);

    ptz.setInputCloud(p_after_y);
    ptz.setFilterFieldName ("z");
    ptz.setFilterLimits(min[2], max[2]);
    ptz.filter(*p_output_cloud);
}

void PreProcessor::crop(const PointCloudTN::Ptr p_input_cloud, PointCloudTN::Ptr p_output_cloud, Eigen::Vector3f min, Eigen::Vector3f max)
{
    pcl::PassThrough<PointTN> ptx, pty, ptz;
    PointCloudTN::Ptr p_after_x(new PointCloudTN);
    PointCloudTN::Ptr p_after_y(new PointCloudTN);

    ptx.setInputCloud(p_input_cloud);
    ptx.setFilterFieldName ("x");
    ptx.setFilterLimits(min[0], max[0]);
    ptx.filter(*p_after_x);

    pty.setInputCloud(p_after_x);
    pty.setFilterFieldName ("y");
    pty.setFilterLimits(min[1], max[1]);
    pty.filter(*p_after_y);

    ptz.setInputCloud(p_after_y);
    ptz.setFilterFieldName ("z");
    ptz.setFilterLimits(min[2], max[2]);
    ptz.filter(*p_output_cloud);
}

void PreProcessor::removeAlone(const PointCloudT::Ptr p_input_cloud, PointCloudT::Ptr p_output_cloud, int meanK, double thresh)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(p_input_cloud);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(thresh);
    sor.filter(*p_output_cloud);
}