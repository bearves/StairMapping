#pragma once

#include "PointType.h"
#include "GlobalMap.h"
#include "ElevationGrid.h"

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
        Eigen::Matrix4d getCorrectTf();

        const PointCloudT::Ptr getGlobalMapRawPoints();
        const PointCloudT::Ptr getGlobalMapOptPoints();
        const PointCloudT::Ptr getElevationGridPoints();

    private:
        GlobalMap global_map_;
        PointCloudT::Ptr global_raw_points_;
        PointCloudT::Ptr global_opt_points_;
        Eigen::Matrix4d correct_tf_;

        ElevationGrid ele_grid_; 
        PointCloudT::Ptr global_height_map_;
    };

} // namespace stair_mapping
