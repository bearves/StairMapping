#include "SubMap.h"
#include <exception>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>

namespace stair_mapping
{
    SubMap::SubMap(int max_stored_pcl_count):
        max_stored_frame_count_(max_stored_pcl_count),
        current_count_(0),
        p_submap_points_(new PointCloudT)
    {
    }

    void SubMap::init()
    {
        current_count_ = 0;
        T_f2sm_.clear();
        frames_.clear();
        T_odom_.clear();
    }

    void SubMap::addFrame(PointCloudT frame, 
        Eigen::Matrix4d t_f2sm, 
        Eigen::Matrix4d t_frame_odom)
    {
        frames_.push_back(frame);
        T_f2sm_.push_back(t_f2sm);
        T_odom_.push_back(t_frame_odom);
        current_count_++;
        updateSubmapPoints();
    }

    bool SubMap::hasEnoughFrame()
    {
        return current_count_ >= max_stored_frame_count_;
    }

    void SubMap::updateSubmapPoints()
    {
        using namespace Eigen;

        PointCloudT transformed_frame;
        PointCloudT::Ptr p_all_points(new PointCloudT);

        // add all frame points to submap
        for(int i = 0; i < current_count_; i++)
        {
            pcl::transformPointCloud(frames_[i], transformed_frame, T_f2sm_[i]);
            p_all_points->operator+=( transformed_frame );
        }
        // downsample submap
        PointCloudT::Ptr p_submap_ds(new PointCloudT);
        Eigen::Vector2i sizes = PreProcessor::downSample(p_all_points, p_submap_ds, 0.02);
        *p_submap_points_ = *p_submap_ds;
    }

    double SubMap::match(PointCloudT frame, 
        const Eigen::Matrix4d& init_guess, 
        Eigen::Matrix4d& t_match_result)
    {
        int point_counts = frame.width * frame.height;
        
        if (point_counts < 300)
        {
            throw std::runtime_error("Too few points in this frame");
        }

        if (current_count_ <= 0)
        {
            t_match_result = Eigen::Matrix4d::Identity();
            return 0; // best score
        }
        else
        {
            t_match_result = init_guess;
            // TODO: ICP/NDT match here
            double score = matchIcp(frame.makeShared(), p_submap_points_, init_guess, t_match_result);
            std::cout << "Score:" << score << std::endl;
            std::cout << "Init guess:\n" << init_guess << std::endl;
            std::cout << "NDT registration result:\n" << t_match_result << std::endl;

            t_match_result = init_guess;
            return 0; // best score
        }
    }

    double SubMap::matchIcp(
        const PointCloudT::Ptr& input_cloud, 
        const PointCloudT::Ptr& target_cloud, 
        const Eigen::Matrix4d& init_guess, 
        Eigen::Matrix4d& transform_result)
    {
        pcl::IterativeClosestPoint<PointT, PointT> icp;

        return 0;
    }

    Eigen::Matrix4d SubMap::getRelativeTfGuess(const Eigen::Matrix4d& current_odom)
    {
        using namespace Eigen;
        // no frames in this submap yet
        if (current_count_ <= 0)
        {
            return Matrix4d::Identity();
        }
        // get the last frame's T_f2sm and T_odom of this submap
        Matrix4d T_f2sm_last = T_f2sm_[current_count_ - 1];
        Matrix4d odom_last = T_odom_[current_count_ - 1];
        Affine3d tm_last(odom_last);

        // get the transform of the odoms between last frame and current frame
        Matrix4d T_o2o = current_odom * tm_last.inverse().matrix() ;
        // get the transform from submap's origin to this frame
        return T_o2o * T_f2sm_last;
    }

    PointCloudT::Ptr SubMap::getSubmapPoints()
    {
        return this->p_submap_points_;
    }
}