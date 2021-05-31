#include "TerrainMapper.h"

namespace stair_mapping
{
    TerrainMapper::TerrainMapper()
        : global_opt_points_(new PointCloudT),
          global_raw_points_(new PointCloudT),
          ele_grid_(5.0, 10.0, 0.02, -10),
          global_height_map_(new PointCloudT)
    {
    }

    void TerrainMapper::preprocess(const PointCloudT::Ptr &p_in_cloud, PointCloudT::Ptr &p_out_cloud)
    {
        // crop
        PointCloudT::Ptr p_cloud_cr(new PointCloudT);
        PreProcessor::crop(p_in_cloud, p_cloud_cr, Eigen::Vector3f(0, -0.5, -2), Eigen::Vector3f(2.0, 0.5, 3));

        // downsampling
        PointCloudT::Ptr p_cloud_ds(new PointCloudT);
        Eigen::Vector2i sizes = PreProcessor::downSample(p_cloud_cr, p_cloud_ds, 0.02);
        ROS_INFO("After downsample size: %d -> %d", sizes[0], sizes[1]);

        p_out_cloud = p_cloud_ds;
    }

    void TerrainMapper::matchSubmap(
        const PointCloudT::Ptr &p_in_cloud, 
        PointCloudT::Ptr &p_out_cloud, 
        const Eigen::Matrix4d& t_frame_odom)
    {
        using namespace Eigen;

        int submap_store_cap = 2;

        // FrontEnd
        // scan-to-submap matcher
        // if no submap exists
        if (global_map_.submapCount() == 0)
        {
            // init new submap with current pcl, set its transform to I
            ROS_INFO("Init new global map");
            SubMap::Ptr p_sm(new SubMap(submap_store_cap));
            p_sm->init();
            global_map_.addNewSubmap(
                p_sm, 
                Matrix4d::Identity(), // m2m laser match
                Matrix4d::Identity(), // m2m odom
                t_frame_odom,          // imu
                100*InfoMatrix::Identity() // information of the submap
            );
        }

        SubMap::Ptr last_sm = global_map_.getLastSubMap();
        SubMap::Ptr current_sm;

        if (last_sm == nullptr) return;

        // match current frame to the last submap
        double score = 1e8;
        double SUCCESS_SCORE = 0.1;

        Matrix4d t_guess = last_sm->getRelativeTfGuess(t_frame_odom);
        Matrix4d t_frame_to_last_map = t_guess;
        InfoMatrix info_mat;

        // reject frames when robot is not moving at all
        Vector3d translation_guess = Affine3d(t_guess).translation();
        if (!last_sm->isEmpty() && translation_guess.norm() < 0.015)
        {
            ROS_WARN("Frame too close: %f", translation_guess.norm());
            return;
        }

        try
        {
            score = last_sm->match(p_in_cloud, t_guess, t_frame_to_last_map, info_mat);
        }
        catch (std::runtime_error &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }

        // if match succeeds, add current frame to current submap or create new submap, otherwise drop it
        if (score < SUCCESS_SCORE)
        {
            // if a new submap is needed
            if (last_sm->hasEnoughFrame())
            {
                // init new submap with current pcl
                ROS_INFO("Create new submap");

                SubMap::Ptr p_sm(new SubMap(submap_store_cap));
                p_sm->init();
                p_sm->addFrame(*p_in_cloud, Matrix4d::Identity(), t_frame_odom);
                // match to last submap if exists, record transform Ti
                global_map_.addNewSubmap(
                    p_sm, 
                    t_frame_to_last_map, // m2m scan match
                    t_guess,             // m2m odom
                    t_frame_odom,        // global imu
                    info_mat             // infomation matrix of the laser match
                );
                current_sm = p_sm;
            }
            else
            {
                // add current pcl to current submap
                last_sm->addFrame(*p_in_cloud, t_frame_to_last_map, t_frame_odom);
                current_sm = last_sm;
            }
        }

        if (current_sm == nullptr) return;

        p_out_cloud = current_sm->getSubmapPoints();
    }
    void TerrainMapper::buildGlobalMap()
    {
        global_map_.runGlobalPoseOptimizer();
        // concat all submaps together 
        auto submap_cnt = global_map_.updateGlobalMapPoints();
        ROS_INFO("Submap count: %ld", submap_cnt);

        const PointCloudT::Ptr raw_pc = global_map_.getGlobalMapRawPoints();
        PreProcessor::downSample(raw_pc, global_raw_points_, 0.02, 1);

        const PointCloudT::Ptr opt_pc = global_map_.getGlobalMapOptPoints();
        PreProcessor::downSample(opt_pc, global_opt_points_, 0.02, 2);

        ele_grid_.update(global_opt_points_);
    }

    Eigen::Matrix4d TerrainMapper::getLastSubMapRawTf()
    {
        return global_map_.getLastSubMapRawTf();
    }

    Eigen::Matrix4d TerrainMapper::getLastSubMapOptTf()
    {
        return global_map_.getLastSubMapOptTf();
    }

    const PointCloudT::Ptr TerrainMapper::getGlobalMapRawPoints()
    {
        return global_raw_points_;
    }

    const PointCloudT::Ptr TerrainMapper::getGlobalMapOptPoints()
    {
        return global_opt_points_;
    }

    const PointCloudT::Ptr TerrainMapper::getElevationGridPoints()
    {
        ele_grid_.getPclFromHeightMap(global_height_map_);
        return global_height_map_;
    }

} // namespace stair_mapping
