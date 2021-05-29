#pragma once

#include "PointType.h"
#include "GlobalMap.h"

namespace stair_mapping
{
    class TerrainMapper
    {
    public:
        TerrainMapper();

        void preprocess(const PointCloudT::Ptr &p_in_cloud, PointCloudT::Ptr &p_out_cloud);

        void matchSubmap(
            const PointCloudT::Ptr &p_in_cloud,
            PointCloudT::Ptr &p_out_cloud,
            const Eigen::Matrix4d &t_frame_odom);

        void buildGlobalMap();

        Eigen::Matrix4d getLastSubMapRawTf();
        Eigen::Matrix4d getLastSubMapOptTf();

        const PointCloudT::Ptr getGlobalMapRawPoints();
        const PointCloudT::Ptr getGlobalMapOptPoints();

    private:
        GlobalMap global_map_;
    };

} // namespace stair_mapping
