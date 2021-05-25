#include "GlobalMap.h"

namespace stair_mapping
{
    GlobalMap::GlobalMap()
    : p_global_map_points_(new PointCloudT),
      last_submap_cnt_(0)
    {
    }

    std::size_t GlobalMap::submapCount()
    {
        return submaps_.size();
    }

    void GlobalMap::addNewSubmap(SubMap::Ptr sm, Eigen::Matrix4d transform)
    {
        // lock since the change of submap number can affect the map building thread
        build_map_mutex_.lock();
        auto submap_cnt = submapCount();
        if (submap_cnt <= 0) 
        {
            // first map
            T_m2gm_raw_.push_back(transform);
        }
        else
        {
            auto last_tf = T_m2gm_raw_[submap_cnt-1];
            T_m2gm_raw_.push_back(last_tf * transform);
        }
        submaps_.push_back(sm);
        T_m2m_.push_back(transform);
        build_map_mutex_.unlock();
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
        Eigen::Matrix4d last_tf;

        build_map_mutex_.lock();
        if (submaps_.size() == 0 )
            last_tf = Eigen::Matrix4d::Identity();
        else
            last_tf = T_m2gm_raw_[T_m2gm_raw_.size() - 1];
        build_map_mutex_.unlock();

        return last_tf;
    }

    const PointCloudT::Ptr GlobalMap::getGlobalMapPoints()
    {
        return p_global_map_points_;
    }

    std::size_t GlobalMap::updateGlobalMapPoints()
    {
        using namespace Eigen;

        PointCloudT transformed_frame;
        PointCloudT::Ptr p_all_points(new PointCloudT);

        build_map_mutex_.lock();
        auto submap_cnt = submapCount();
        build_map_mutex_.unlock();

        // since only the old data are read and never changed
        // the map building process is thread safe
        for(int i = 0; i < submap_cnt; i++)
        {
            pcl::transformPointCloud(
                *(submaps_[i]->getSubmapPoints()), 
                transformed_frame, 
                T_m2gm_raw_[i]);
            p_all_points->operator+=( transformed_frame );
        }
        *p_global_map_points_ = *p_all_points;
        return submap_cnt;
    }

    bool GlobalMap::runGlobalPoseOptimizer()
    {
        pg_.reset();

        build_map_mutex_.lock();
        auto submap_cnt = submapCount();
        build_map_mutex_.unlock();

        // only optimize every 5 submap have been added
        if (!(submap_cnt > 0 && submap_cnt % 5 == 0)) 
            return false;
        
        // if we have run optimizer before at this submap cnt, skip optimizing
        if (submap_cnt == last_submap_cnt_)
            return false;

        for(int i = 0; i < submap_cnt; i++)
        {
            Vertex3d v(T_m2gm_raw_[i]);
            pg_.addVertex(v);
        }

        for(int i = 0; i < submap_cnt-1; i++)
        {
            Pose3d t_edge(T_m2m_[i+1]);
            pg_.addEdge(i, i+1, t_edge, InfoMatrix::Identity());
        }

        try
        {
            bool ret = pg_.solve();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }

        for(int i = 0; i < submap_cnt; i++)
        {

        }

        last_submap_cnt_ = submap_cnt;

        return true;
    }

}