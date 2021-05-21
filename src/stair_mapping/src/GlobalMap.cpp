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

    void GlobalMap::updateGlobalMapPoints()
    {
        using namespace Eigen;

        PointCloudT transformed_frame;
        PointCloudT::Ptr p_all_points(new PointCloudT);

        // add all frame points to submap
        Matrix4d T_m2gm = Matrix4d::Identity();
        for(int i = 0; i < submapCount(); i++)
        {
            T_m2gm *= T_m2m_[i];
            pcl::transformPointCloud(*(submaps_[i]->getSubmapPoints()), transformed_frame, T_m2gm);
            p_all_points->operator+=( transformed_frame );
        }
        *p_global_map_points_ = *p_all_points;
    }

    const PointCloudT::Ptr GlobalMap::getGlobalMapPoints()
    {
        return p_global_map_points_;
    }
}