#include "GlobalMap.h"
#include <pcl/octree/octree_pointcloud_density.h>

namespace stair_mapping
{
    GlobalMap::GlobalMap()
        : p_global_map_raw_points_(new PointCloudT),
          p_global_map_opt_points_(new PointCloudT),
          last_submap_cnt_(0)
    {
    }

    std::size_t GlobalMap::submapCount()
    {
        return submaps_.size();
    }

    void GlobalMap::addNewSubmap(
        SubMap::Ptr sm, 
        Eigen::Matrix4d tf_m2m_laser,
        Eigen::Matrix4d tf_m2m_odom,
        Eigen::Matrix4d tf_m2gm_imu,
        InfoMatrix info_laser)
    {
        // lock since the change of submap number can affect the map building thread
        build_map_mutex_.lock();
        auto submap_cnt = submapCount();
        if (submap_cnt <= 0) 
        {
            // first map
            T_m2gm_raw_.push_back(tf_m2m_laser);
            T_m2gm_opt_.push_back(tf_m2m_laser);
        }
        else
        {
            auto last_tf = T_m2gm_raw_[submap_cnt-1];
            T_m2gm_raw_.push_back(last_tf * tf_m2m_laser);
            auto last_opt_tf = T_m2gm_opt_[submap_cnt-1];
            T_m2gm_opt_.push_back(last_opt_tf * tf_m2m_laser);
        }
        submaps_.push_back(sm);
        T_m2m_laser_.push_back(tf_m2m_laser);
        info_m2m_laser_.push_back(info_laser);
        T_m2m_odom_.push_back(tf_m2m_odom);
        T_m2gm_imu_.push_back(tf_m2gm_imu);
        build_map_mutex_.unlock();
    }

    SubMap::Ptr GlobalMap::getLastSubMap()
    {
        if (submaps_.size() == 0 )
            return nullptr;
        else
            return submaps_[submaps_.size() - 1];
    }

    Eigen::Matrix4d GlobalMap::getLastSubMapRawTf()
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

    Eigen::Matrix4d GlobalMap::getLastSubMapOptTf()
    {
        Eigen::Matrix4d last_opt_tf;

        build_map_mutex_.lock();
        if (submaps_.size() == 0 )
            last_opt_tf = Eigen::Matrix4d::Identity();
        else
            last_opt_tf = T_m2gm_opt_[T_m2gm_opt_.size() - 1];
        build_map_mutex_.unlock();

        return last_opt_tf;
    }

    Eigen::Matrix4d GlobalMap::getCorrectTf()
    {
        Eigen::Matrix4d last_opt_tf;
        Eigen::Matrix4d last_robot_imu_tf;

        build_map_mutex_.lock();
        if (submaps_.size() == 0 )
        {
            last_opt_tf = Eigen::Matrix4d::Identity();
            last_robot_imu_tf = last_opt_tf;
        }
        else
        {
            last_opt_tf = T_m2gm_opt_[T_m2gm_opt_.size() - 1];
            last_robot_imu_tf = T_m2gm_imu_[T_m2gm_imu_.size() - 1];
        }
        build_map_mutex_.unlock();

        Eigen::Affine3d r(last_robot_imu_tf);

        return last_opt_tf * r.inverse().matrix();
    }

    const PointCloudT::Ptr GlobalMap::getGlobalMapRawPoints()
    {
        return p_global_map_raw_points_;
    }

    const PointCloudT::Ptr GlobalMap::getGlobalMapOptPoints()
    {
        return p_global_map_opt_points_;
    }


    std::size_t GlobalMap::updateGlobalMapPoints()
    {
        using namespace Eigen;

        PointCloudT transformed_opt_frame;
        PointCloudT transformed_raw_frame;
        PointCloudT::Ptr p_all_raw_points(new PointCloudT);
        PointCloudT::Ptr p_all_opt_points(new PointCloudT);

        build_map_mutex_.lock();
        auto submap_cnt = submapCount();
        build_map_mutex_.unlock();

        // since only the old data are read and never changed
        // the map building process is thread safe
        for(int i = 0; i < submap_cnt; i++)
        {
            pcl::transformPointCloud(
                *(submaps_[i]->getSubmapPoints()), 
                transformed_opt_frame, 
                T_m2gm_opt_[i]);
            p_all_opt_points->operator+=( transformed_opt_frame );

            pcl::transformPointCloud(
                *(submaps_[i]->getSubmapPoints()), 
                transformed_raw_frame, 
                T_m2gm_raw_[i]);
            p_all_raw_points->operator+=( transformed_raw_frame );
        }

        *p_global_map_opt_points_ = *p_all_opt_points;
        *p_global_map_raw_points_ = *p_all_raw_points;

        return submap_cnt;
    }

    bool GlobalMap::runGlobalPoseOptimizer()
    {
        pg_.reset();

        build_map_mutex_.lock();
        auto submap_cnt = submapCount();
        build_map_mutex_.unlock();

        // only optimize every 3 submap have been added
        if (!(submap_cnt > 0 && submap_cnt % 2 == 0)) 
            return false;
        
        // if we have run optimizer before at this submap cnt, skip optimizing
        if (submap_cnt == last_submap_cnt_)
            return false;

        // add vertices
        for(int i = 0; i < submap_cnt; i++)
        {
            Vertex3d v(T_m2gm_raw_[i]);
            pg_.addVertex(v);
        }
        // edge of submap-to-submap scan match constraints
        for(int i = 0; i < submap_cnt-1; i++)
        {
            Pose3d t_edge(T_m2m_laser_[i+1]);
            pg_.addEdge(EDGE_TYPE::TRANSFORM, i, i+1, t_edge, info_m2m_laser_[i+1]);
        }
        // edge of submap-to-submap odom constraints
        for(int i = 0; i < submap_cnt-1; i++)
        {
            InfoMatrix ifm;
            ifm.setZero();
            // only weight translations
            ifm.diagonal() << 10, 10, 10, 1e-16, 1e-16, 1e-16;
            Pose3d t_edge(T_m2m_odom_[i+1]);
            pg_.addEdge(EDGE_TYPE::TRANSLATION, i, i+1, t_edge, ifm);
        }
        // edge of orientation imu constraints
        if (submap_cnt > 0)
        {
            Pose3d t_edge(T_m2gm_imu_[submap_cnt - 1]);
            InfoMatrix ifm;
            ifm.setZero();
            // only weight orientations
            ifm.diagonal() << 1e-16, 1e-16, 1e-16, 4, 4, 1e-16;
            pg_.addEdge(EDGE_TYPE::ABS_ROTATION, 0, submap_cnt-1, t_edge, ifm);
        }

        // solve
        try
        {
            bool ret = pg_.solve();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            return false;
        }

        // copy out results
        for(int i = 0; i < submap_cnt; i++)
        {
            const Vertex3d v = pg_.getVertices()->at(i);
            T_m2gm_opt_[i] = v.toMat4d();
        }

        last_submap_cnt_ = submap_cnt;

        return true;
    }

}