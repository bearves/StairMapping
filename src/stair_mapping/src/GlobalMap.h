#pragma once

#include <vector>
#include <mutex>
#include "SubMap.h"
#include "PoseGraph.h"

namespace stair_mapping
{
class GlobalMap
{
public:
    GlobalMap();

    std::size_t submapCount();

    SubMap::Ptr getLastSubMap();

    void addNewSubmap(
        SubMap::Ptr sm, 
        Eigen::Matrix4d tf_m2m_laser,
        Eigen::Matrix4d tf_m2m_odom,
        Eigen::Matrix4d tf_m2gm_imu,
        InfoMatrix info_laser
    );

    bool runGlobalPoseOptimizer();
    std::size_t updateGlobalMapPoints();

    Eigen::Matrix4d getLastSubMapRawTf();
    Eigen::Matrix4d getLastSubMapOptTf();
    Eigen::Matrix4d getCorrectTf();

    const PointCloudT::Ptr getGlobalMapRawPoints();
    const PointCloudT::Ptr getGlobalMapOptPoints();
    const PointCloudT::Ptr getGroundPatchPoints();

private:
    std::vector<SubMap::Ptr> submaps_;
    // The transform between submaps from scan match result
    std::vector<Eigen::Matrix4d> T_m2m_laser_;
    std::vector<InfoMatrix> info_m2m_laser_;
    // The transform between submaps from robot odom
    std::vector<Eigen::Matrix4d> T_m2m_odom_;
    // The transform(orientation) of the submap in 
    //  the global cs according to the robot imu
    std::vector<Eigen::Matrix4d> T_m2gm_imu_;

    // The accumuated transform of the submap in the global cs
    std::vector<Eigen::Matrix4d> T_m2gm_raw_;
    // The optimized transform of the submap in the global cs
    std::vector<Eigen::Matrix4d> T_m2gm_opt_;
    // The transform compensate of the submap in the global cs
    std::vector<Eigen::Matrix4d> T_m2gm_compensate_;

    PointCloudT::Ptr p_global_map_raw_points_;
    PointCloudT::Ptr p_global_map_opt_points_;
    PointCloudT::Ptr p_foothold_ground_patch_;

    PoseGraph pg_; 
    size_t last_submap_cnt_;

    std::mutex build_map_mutex_;

    void calculateFootholdGroundDistance(
        int submap_count,
        const PointCloudT::Ptr& ground,
        Eigen::Matrix<double, 4, 6>& last_tip_points,
        PointCloudT::Ptr& ground_patch
    );

};
} // namespace stair_mapping
