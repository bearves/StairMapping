#pragma once

#include <vector>
#include <ros/ros.h>
#include <Eigen/Dense>
#include "PointType.h"
#include "PreProcessor.h"
#include "PoseGraph.h"

namespace stair_mapping
{

class SubMap
{
public:
    SubMap(int max_stored_pcl_count);

    void init();

    void addFrame(
        PtCld frame,
        open3d::geometry::RGBDImage rgbd_img,
        const open3d::camera::PinholeCameraIntrinsic intrinsic,
        const Eigen::Matrix4d& t_f2sm, 
        const Eigen::Matrix4d& t_frame_odom,
        const Eigen::Matrix4d& t_cam_wrt_base,
        const Eigen::Matrix<double, 4, 6>& tip_states);

    bool hasEnoughFrame();
    bool isEmpty();

    double match( 
        const PtCldPtr& frame, 
        const open3d::geometry::RGBDImage& rgbd_img,
        const Eigen::Matrix4d& init_guess, 
        const Eigen::Matrix4d& t_cam_wrt_base, 
        Eigen::Matrix4d& t_match_result,
        InfoMatrix& info_match_result);

    const PtCldPtr getSubmapPoints();
    const PtCldPtr getCroppedSubmapPoints();
    Eigen::Matrix4d getSubmapTfCamWrtBase();
    bool getFirstRGBDFrame(open3d::geometry::RGBDImage& image)
    {
        if (rgbd_imgs_.size() > 0)
        {
            image = rgbd_imgs_[0];
            return true;
        }
        else
        {
            return false;
        }
    };

    open3d::camera::PinholeCameraIntrinsic getIntrinsic()
    {
        return intrinsic_;
    }

    Eigen::Matrix<double, 4, 6> getLastTipPointsWithTransform(const Eigen::Matrix4d& tf);

    Eigen::Matrix4d getRelativeTfGuess(
        const Eigen::Matrix4d& current_odom);

    typedef std::shared_ptr<SubMap> Ptr;
    
    static bool PRINT_VERBOSE_INFO;

private:
    int max_stored_frame_count_;
    int current_count_;
    std::vector<Eigen::Matrix4d> T_f2sm_;
    std::vector<Eigen::Matrix4d> T_cam_wrt_base_;
    std::vector<Eigen::Matrix4d> T_odom_;
    std::vector<Eigen::Matrix<double, 4, 6> > tip_states_;
    std::vector<PtCld> frames_;
    std::vector<open3d::geometry::RGBDImage> rgbd_imgs_;

    open3d::camera::PinholeCameraIntrinsic intrinsic_;

    PtCldPtr p_submap_points_;
    PtCldPtr p_cropped_submap_points_;

    void updateSubmapPoints();

    double matchIcp(
        const PtCldPtr& input_cloud, 
        const PtCldPtr& target_cloud, 
        const open3d::geometry::RGBDImage& input_rgbd_img,
        const open3d::geometry::RGBDImage& target_rgbd_img,
        const Eigen::Matrix4d& init_guess, 
        const Eigen::Matrix4d& t_cam_wrt_base, 
        Eigen::Matrix4d& transform_result,
        InfoMatrix& transform_info);
    
    InfoMatrix computeInfomation(
        const PtCldPtr& result_cloud,
        const PtCldPtr& target_cloud
    );
};


}