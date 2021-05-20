#include "GlobalMap.h"

namespace stair_mapping
{
    GlobalMap::GlobalMap()
    {

    }

    int GlobalMap::submapCount()
    {
        return submaps_.size();
    }

    void GlobalMap::addNewSubmap(SubMap::Ptr sm, Eigen::Matrix4d transform)
    {
        this->submaps_.push_back(sm);
        this->T_m2m_.push_back(transform);
    }

    SubMap::Ptr GlobalMap::getLastMap()
    {
        if (submaps_.size() == 0 )
            return nullptr;
        else
            return submaps_[submaps_.size() - 1];
    }
}