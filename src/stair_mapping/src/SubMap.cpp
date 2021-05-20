#include "SubMap.h"
#include <exception>

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

    void SubMap::addFrame(PointCloudT frame, Eigen::Matrix4d t_f2sm, Eigen::Matrix4d t_frame_odom)
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
        Eigen::Vector2i sizes = PreProcessor::downSample(p_all_points, p_submap_ds, 0.01);
        *p_submap_points_ = *p_submap_ds;
    }

    double SubMap::match(PointCloudT frame, Eigen::Matrix4d init_guess, Eigen::Matrix4d& t_match_result)
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
            t_match_result = Eigen::Matrix4d::Identity();
            // TODO: ICP/NDT match here
            return 0; // best score
        }
    }

    PointCloudT::Ptr SubMap::getSubmapPoints()
    {
        return this->p_submap_points_;
    }
}