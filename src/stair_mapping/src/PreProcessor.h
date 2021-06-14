#pragma once

#include "PointType.h"
#include <Eigen/Dense>

namespace stair_mapping
{
    class PreProcessor
    {
    public:
        static Eigen::Vector2i downSample(const PtCldPtr& p_input_cloud, PtCldPtr& p_output_cloud, double leaf_size);

        static void crop(const PtCldPtr& p_input_cloud, PtCldPtr& p_output_cloud, Eigen::Vector3d min, Eigen::Vector3d max);
        static void removeAlone(const PtCldPtr& p_input_cloud, PtCldPtr& p_output_cloud, int meanK, double thresh);
    };
}