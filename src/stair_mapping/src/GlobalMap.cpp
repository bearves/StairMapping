#include "GlobalMap.h"

namespace stair_mapping
{
    GlobalMap::GlobalMap()
    : p_global_map_points_(new PointCloudT)
    {
    }

    int GlobalMap::submapCount()
    {
        return submaps_.size();
    }

    void GlobalMap::addNewSubmap(SubMap::Ptr sm, Eigen::Matrix4d transform)
    {
        // first map
        auto submap_cnt = submapCount();
        if (submap_cnt <= 0) 
        {
            T_m2gm_raw_.push_back(transform);
        }
        else
        {
            auto last_tf = T_m2gm_raw_[submap_cnt-1];
            T_m2gm_raw_.push_back(last_tf * transform);
        }
        submaps_.push_back(sm);
        T_m2m_.push_back(transform);
    }

    SubMap::Ptr GlobalMap::getLastSubMap()
    {
        if (submaps_.size() == 0 )
            return nullptr;
        else
            return submaps_[submaps_.size() - 1];
    }

    Eigen::Matrix4d GlobalMap::getLastSubMapTf()
    {
        if (submaps_.size() == 0 )
            return Eigen::Matrix4d::Identity();
        else
            return T_m2gm_raw_[T_m2gm_raw_.size() - 1];
    }

    void GlobalMap::updateGlobalMapPoints()
    {
        using namespace Eigen;

        PointCloudT transformed_frame;
        PointCloudT::Ptr p_all_points(new PointCloudT);

        auto submap_cnt = submapCount();
        for(int i = 0; i < submap_cnt; i++)
        {
            pcl::transformPointCloud(*(submaps_[i]->getSubmapPoints()), transformed_frame, T_m2gm_raw_[i]);
            p_all_points->operator+=( transformed_frame );
        }
        *p_global_map_points_ = *p_all_points;
    }

    const PointCloudT::Ptr GlobalMap::getGlobalMapPoints()
    {
        return p_global_map_points_;
    }
}