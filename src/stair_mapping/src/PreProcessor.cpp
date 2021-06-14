#include "PreProcessor.h"
#include <Eigen/Dense>
#include <open3d/Open3D.h>
#include <ros/ros.h>

namespace stair_mapping
{
    Eigen::Vector2i PreProcessor::downSample(
        const PtCldPtr& p_input_cloud, PtCldPtr& p_output_cloud, double leaf_size)
    {
        p_output_cloud = p_input_cloud->VoxelDownSample(leaf_size);
        int original_size = p_input_cloud->points_.size();
        int resultant_size = p_output_cloud->points_.size();
        return Eigen::Vector2i(original_size, resultant_size);
    }

    void PreProcessor::crop(
        const PtCldPtr& p_input_cloud, PtCldPtr& p_output_cloud, 
        Eigen::Vector3d min, Eigen::Vector3d max)
    {
        using namespace open3d::geometry;
        AxisAlignedBoundingBox box(min, max);
        p_output_cloud = p_input_cloud->Crop(box);
    }

    void PreProcessor::removeAlone(
        const PtCldPtr& p_input_cloud, PtCldPtr& p_output_cloud, 
        int meanK, double thresh)
    {
        auto result = p_input_cloud->RemoveStatisticalOutliers(meanK, thresh);
        std::tie(p_output_cloud, std::ignore) = result;
    }

} // namespace stair_mapping
