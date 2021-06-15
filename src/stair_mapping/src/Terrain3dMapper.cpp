#include "Terrain3dMapper.h"

namespace stair_mapping
{
    Terrain3dMapper::Terrain3dMapper()
        : global_opt_points_(new PtCld),
          global_raw_points_(new PtCld),
          ground_patch_points_(new PtCld),
          correct_tf_(Eigen::Matrix4d::Identity())
    {
    }

    void Terrain3dMapper::preprocess(const PtCldPtr &p_in_cloud, PtCldPtr &p_out_cloud)
    {
        // crop
        PtCldPtr p_cloud_cr = std::make_shared<PtCld>();
        PreProcessor::crop(p_in_cloud, p_cloud_cr, Eigen::Vector3d(0, -0.5, -2), Eigen::Vector3d(3.0, 0.5, 3));

        // ROS_INFO("Cropped pcd size: %ld", p_cloud_cr->points_.size());
        // downsampling
        PtCldPtr p_cloud_ds = std::make_shared<PtCld>();
        Eigen::Vector2i sizes = PreProcessor::downSample(p_cloud_cr, p_cloud_ds, 0.03);
        ROS_INFO("After downsample size: %d -> %d", sizes[0], sizes[1]);

        p_out_cloud = p_cloud_ds;
    }

    // FrontEnd
    // scan-to-submap matcher
    void Terrain3dMapper::matchSubmap(
        const PtCldPtr &p_in_cloud, 
        PtCldPtr &p_out_cloud, 
        const Eigen::Matrix4d& t_frame_odom,
        const Eigen::Matrix<double, 4, 6>& tip_states)
    {
        using namespace Eigen;

        int submap_store_cap = 2;

        // reject empty cloud
        if (p_in_cloud->IsEmpty()) return;

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
        double score = -1e8;
        double SUCCESS_SCORE = 0.6;

        Matrix4d t_guess = last_sm->getRelativeTfGuess(t_frame_odom);
        Matrix4d t_frame_to_last_map = t_guess;
        InfoMatrix info_mat;

        // reject frames when robot is not moving at all
        Vector3d translation_guess = Affine3d(t_guess).translation();
        if (!last_sm->isEmpty() && translation_guess.norm() < 0.01)
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
        if (score > SUCCESS_SCORE)
        {
            // if a new submap is needed
            if (last_sm->hasEnoughFrame())
            {
                // init new submap with current pcl
                ROS_INFO("Create new submap");

                SubMap::Ptr p_sm(new SubMap(submap_store_cap));
                p_sm->init();
                p_sm->addFrame(*p_in_cloud, Matrix4d::Identity(), t_frame_odom, tip_states);
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
                last_sm->addFrame(*p_in_cloud, t_frame_to_last_map, t_frame_odom, tip_states);
                current_sm = last_sm;
            }
        }

        if (current_sm == nullptr) return;

        p_out_cloud = current_sm->getSubmapPoints();
    }

    void Terrain3dMapper::buildGlobalMap(bool display_raw_result)
    {
        // optimize global map, update submap tf matrices
        global_map_.runGlobalPoseOptimizer();
        // update global points, update compensation matrices
        auto submap_cnt = global_map_.updateGlobalMapPoints(display_raw_result);
        ROS_INFO("Submap count: %ld", submap_cnt);

        correct_tf_ = global_map_.getCorrectTf();

        const PtCldPtr opt_pc = global_map_.getGlobalMapOptPoints();
        PreProcessor::downSample(opt_pc, global_opt_points_, 0.02);

        ground_patch_points_ = global_map_.getGroundPatchPoints();

        if (display_raw_result)
        {
            const PtCldPtr raw_pc = global_map_.getGlobalMapRawPoints();
            PreProcessor::downSample(raw_pc, global_raw_points_, 0.02);
        }
    }

    Eigen::Matrix4d Terrain3dMapper::getLastSubMapRawTf()
    {
        return global_map_.getLastSubMapRawTf();
    }

    Eigen::Matrix4d Terrain3dMapper::getLastSubMapOptTf()
    {
        return global_map_.getLastSubMapOptTf();
    }

    Eigen::Matrix4d Terrain3dMapper::getCorrectTf()
    {
        return correct_tf_;
    }

    const PtCldPtr Terrain3dMapper::getGlobalMapRawPoints()
    {
        return global_raw_points_;
    }

    const PtCldPtr Terrain3dMapper::getGroundPatchPoints()
    {
        return ground_patch_points_;
    }

    const PtCldPtr Terrain3dMapper::getGlobalMapOptPoints()
    {
        return global_opt_points_;
    }

} // namespace stair_mapping
