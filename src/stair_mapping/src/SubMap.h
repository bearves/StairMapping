#pragma once

#include <vector>
#include <ros/ros.h>
#include <Eigen/Dense>
#include "PointType.h"
#include "PreProcessor.h"
#include <pcl/common/transforms.h>
#include "PoseGraph.h"

namespace stair_mapping
{

class SubMap
{
public:
    SubMap(int max_stored_pcl_count);

    void init();

    void addFrame(
        PointCloudT frame,
        Eigen::Matrix4d t_f2sm, 
        Eigen::Matrix4d t_frame_odom);

    bool hasEnoughFrame();
    bool isEmpty();

    double match( 
        const PointCloudT::Ptr& frame, 
        const Eigen::Matrix4d& init_guess, 
        Eigen::Matrix4d& t_match_result,
        InfoMatrix& info_match_result);

    const PointCloudT::Ptr getSubmapPoints();

    Eigen::Matrix4d getRelativeTfGuess(
        const Eigen::Matrix4d& current_odom);

    typedef std::shared_ptr<SubMap> Ptr;

private:
    int max_stored_frame_count_;
    int current_count_;
    std::vector<Eigen::Matrix4d> T_f2sm_;
    std::vector<Eigen::Matrix4d> T_odom_;
    std::vector<PointCloudT> frames_;
    PointCloudT::Ptr p_submap_points_;

    void updateSubmapPoints();

    double matchIcp(
        const PointCloudT::Ptr& input_cloud, 
        const PointCloudT::Ptr& target_cloud, 
        const Eigen::Matrix4d& init_guess, 
        Eigen::Matrix4d& transform_result,
        InfoMatrix& transform_info);
    
    void getNormal(
        const PointCloudT::Ptr& input_cloud,
        const PointCloudN::Ptr& normal_cloud
    );

    InfoMatrix computeInfomation(
        const PointCloudTN::Ptr& result_cloud,
        const PointCloudTN::Ptr& target_cloud
    );
};


}