#pragma once

#include <vector>
#include "SubMap.h"

namespace stair_mapping
{
class GlobalMap
{
public:
    GlobalMap();
    int submapCount();
    SubMap::Ptr getLastMap();
    void addNewSubmap(SubMap::Ptr sm, Eigen::Matrix4d transform);

private:
    std::vector<SubMap::Ptr> submaps_;
    std::vector<Eigen::Matrix4d> T_m2m_;
};
} // namespace stair_mapping
