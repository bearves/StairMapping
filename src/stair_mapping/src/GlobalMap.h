#pragma once

#include <vector>
#include "SubMap.h"

namespace stair_mapping
{
class GlobalMap
{
public:
    GlobalMap();

    SubMap::Ptr getLastSubMap();
    Eigen::Matrix4d getLastSubMapTf();

    int submapCount();

    void addNewSubmap(SubMap::Ptr sm, Eigen::Matrix4d transform);
    
    void updateGlobalMapPoints();

    const PointCloudT::Ptr getGlobalMapPoints();

private:
    std::vector<SubMap::Ptr> submaps_;
    std::vector<Eigen::Matrix4d> T_m2m_;
    std::vector<Eigen::Matrix4d> T_m2gm_raw_;

    PointCloudT::Ptr p_global_map_points_;
};
} // namespace stair_mapping
