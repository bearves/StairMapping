#pragma once

#include <vector>
#include <Eigen/Dense>
#include "PointType.h"
#include "PreProcessor.h"
#include <pcl/common/transforms.h>

namespace stair_mapping
{

class SubMap
{
public:
    SubMap(int max_stored_pcl_count = 3);

    void init();
    void addFrame(PointCloudT frame, Eigen::Matrix4d t_f2sm);
    bool hasEnoughFrame();
    double match(PointCloudT frame, Eigen::Matrix4d& t_match_result);
    PointCloudT::Ptr getSubmapPoints();

    typedef std::shared_ptr<SubMap> Ptr;

private:
    int max_stored_frame_count_;
    int current_count_;
    std::vector<Eigen::Matrix4d> T_f2sm_;
    std::vector<PointCloudT> frames_;
    PointCloudT::Ptr p_submap_points_;

    void updateSubmapPoints();
};


}