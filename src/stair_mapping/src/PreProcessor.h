#pragma once

#include "PointType.h"
#include <Eigen/Dense>

class PreProcessor
{
public:
    static Eigen::Vector2i downSample(
        const PointCloudT::Ptr p_input_cloud,
        PointCloudT::Ptr p_output_cloud,
        double leaf_size,
        uint minimum_point_per_voxel = 1);

    static void crop(const PointCloudT::Ptr p_input_cloud, PointCloudT::Ptr p_output_cloud, Eigen::Vector3f min, Eigen::Vector3f max);
    static void crop(const PointCloudTN::Ptr p_input_cloud, PointCloudTN::Ptr p_output_cloud, Eigen::Vector3f min, Eigen::Vector3f max);
    static void removeAlone(const PointCloudT::Ptr p_input_cloud, PointCloudT::Ptr p_output_cloud, int meanK, double thresh);
};