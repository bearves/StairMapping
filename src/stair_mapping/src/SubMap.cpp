#include "SubMap.h"
#include <exception>
#include <open3d/Open3D.h>
#include "PoseGraph.h"

namespace stair_mapping
{
    bool SubMap::PRINT_VERBOSE_INFO = false;

    SubMap::SubMap(int max_stored_pcl_count):
        max_stored_frame_count_(max_stored_pcl_count),
        current_count_(0),
        p_submap_points_(new PtCld),
        p_cropped_submap_points_(new PtCld)
    {
    }

    void SubMap::init()
    {
        current_count_ = 0;
        T_f2sm_.clear();
        frames_.clear();
        rgbd_imgs_.clear();
        T_odom_.clear();
        T_cam_wrt_base_.clear();
    }

    void SubMap::addFrame(
        PtCld frame, 
        open3d::geometry::RGBDImage rgbd_img,
        const open3d::camera::PinholeCameraIntrinsic intrinsic,
        const Eigen::Matrix4d& t_f2sm, 
        const Eigen::Matrix4d& t_frame_odom,
        const Eigen::Matrix4d& t_cam_wrt_base,
        const Eigen::Matrix<double, 4, 6>& tip_states)
    {
        frames_.push_back(frame);
        rgbd_imgs_.push_back(rgbd_img);
        T_f2sm_.push_back(t_f2sm); // ^{b_{i-1}}T_{b_i}
        T_cam_wrt_base_.push_back(t_cam_wrt_base);
        T_odom_.push_back(t_frame_odom);
        tip_states_.push_back(tip_states);

        intrinsic_ = intrinsic;
        
        current_count_++;
        updateSubmapPoints();
    }

    bool SubMap::hasEnoughFrame()
    {
        return current_count_ >= max_stored_frame_count_;
    }

    bool SubMap::isEmpty()
    {
        return current_count_ <= 0;
    }

    void SubMap::updateSubmapPoints()
    {
        using namespace Eigen;

        PtCldPtr p_all_points = std::make_shared<PtCld>();

        // add all frame points to submap
        for(int i = 0; i < current_count_; i++)
        {
            // Transform existed frame points to the submap's origin wrt camera local coordinate,
            // In this way, the icp matching can be conducted between current submap points 
            // and the next key frame.
            auto transformed_frame = frames_[i];
            Matrix4d T_base_wrt_cam = Affine3d(T_cam_wrt_base_[i]).inverse().matrix();
            Matrix4d T_f2sm_wrt_cam = T_base_wrt_cam * T_f2sm_[i] * T_cam_wrt_base_[i];
            transformed_frame.Transform(T_f2sm_wrt_cam);
            p_all_points->operator+=( transformed_frame );
        }
        *p_submap_points_ = *p_all_points;

        // crop for global map building
        PreProcessor::crop(
            p_submap_points_,
            p_cropped_submap_points_,
            Vector3d(-0.5, -2, 0),
            Vector3d(0.5, 2, 1.5));
    }

    double SubMap::match(
        const PtCldPtr& frame, 
        const open3d::geometry::RGBDImage& rgbd_img,
        const Eigen::Matrix4d& init_guess, 
        const Eigen::Matrix4d& t_cam_wrt_base, 
        Eigen::Matrix4d& t_match_result,
        InfoMatrix& info_match_result)
    {
        int point_counts = frame->points_.size();
        
        if (point_counts < 300)
        {
            throw std::runtime_error("Too few points in this frame");
        }

        if (current_count_ <= 0)
        {
            t_match_result = Eigen::Matrix4d::Identity();
            info_match_result.setZero();
            info_match_result.diagonal() << 100, 100, 100, 20, 20, 20;
            return 1; // best score
        }
        else
        {
            t_match_result = init_guess;
            double score = matchIcp(
                frame, 
                p_submap_points_, 
                rgbd_img,
                rgbd_imgs_[0],
                init_guess, 
                t_cam_wrt_base,
                t_match_result,
                info_match_result);

            if (SubMap::PRINT_VERBOSE_INFO)
            {
                std::cout << "Score:" << score << std::endl;
                std::cout << "Init guess:\n"
                          << init_guess << std::endl;
                std::cout << "Registration result:\n"
                          << t_match_result << std::endl;
                std::cout << "Info matrix eig:\n"
                          << info_match_result.eigenvalues().real() << std::endl;
                std::cout << "Tranlate matrix eig:\n"
                          << info_match_result.topLeftCorner(3, 3).eigenvalues().real() << std::endl;
            }

            //t_match_result = init_guess;
            return score; // best score
        }
    }

    double SubMap::matchIcp(
        const PtCldPtr& input_cloud, 
        const PtCldPtr& target_cloud, 
        const open3d::geometry::RGBDImage& input_rgbd_img,
        const open3d::geometry::RGBDImage& target_rgbd_img,
        const Eigen::Matrix4d& init_guess, 
        const Eigen::Matrix4d& t_cam_wrt_base, 
        Eigen::Matrix4d& transform_result,
        InfoMatrix& transform_info)
    {
        using namespace Eigen;
        using namespace open3d::geometry;

        auto t_base_wrt_cam = Affine3d(t_cam_wrt_base).inverse().matrix();
        // the transform guess in the camera's local frame
        Matrix4d init_guess_cam = t_base_wrt_cam * init_guess * t_cam_wrt_base;

        PtCldPtr p_input_cloud_init = std::make_shared<PtCld>();
        PtCldPtr p_input_cloud_cr = std::make_shared<PtCld>();
        PtCldPtr p_target_cloud_cr = std::make_shared<PtCld>();

        // firstly transform the input cloud with init_guess to 
        // make the two clouds overlap as much as possible
        *p_input_cloud_init = *input_cloud;
        p_input_cloud_init->Transform(init_guess_cam);

        // since the stairs are very similar structures
        // crop on Z and X axis to avoid the stair-jumping mismatch
        // aka. match this step to the next
        auto max_input = p_input_cloud_init->GetMaxBound();
        auto min_input = p_input_cloud_init->GetMinBound();
        auto max_target = target_cloud->GetMaxBound();
        auto min_target = target_cloud->GetMinBound();

        double loose = 0.1;
        double z_crop_max = std::min(max_input.z(), max_target.z()) + loose;
        double z_crop_min = std::max(min_input.z(), min_target.z()) - loose;
        double y_crop_max = std::min(max_input.y(), max_target.y()) + loose;
        double y_crop_min = std::max(min_input.y(), min_target.y()) - loose;
        double x_crop_max = std::min(max_input.x(), max_target.x()) + loose;
        double x_crop_min = std::max(min_input.x(), min_target.x()) - loose;

        PreProcessor::crop(p_input_cloud_init, p_input_cloud_cr, 
            Eigen::Vector3d(x_crop_min, y_crop_min, z_crop_min), 
            Eigen::Vector3d(x_crop_max, y_crop_max, z_crop_max));
        PreProcessor::crop(target_cloud, p_target_cloud_cr, 
            Eigen::Vector3d(x_crop_min, y_crop_min, z_crop_min), 
            Eigen::Vector3d(x_crop_max, y_crop_max, z_crop_max));

        if (p_input_cloud_cr->IsEmpty() || p_target_cloud_cr->IsEmpty())
        {
            ROS_WARN("Empty input cloud when matching");
            transform_result = init_guess;
            return -1e8;
        }

        open3d::utility::Timer timer;
        timer.Start();

        // compute normal for each clouds
        p_input_cloud_cr->EstimateNormals(KDTreeSearchParamKNN(10));
        // recompute normals can reduce pt2pl icp time
        p_target_cloud_cr->EstimateNormals(KDTreeSearchParamKNN(10));

        timer.Stop();
        ROS_INFO("Normal estimation in %lf ms", timer.GetDuration());

        ////////////////////////////////////////////////////////////////////////////////////////////
        // Pointcloud ICP algorithm
        // Tensor ICP 
        // auto source = open3d::t::geometry::PointCloud::FromLegacyPointCloud(*p_input_cloud_cr);
        // auto target = open3d::t::geometry::PointCloud::FromLegacyPointCloud(*p_target_cloud_cr);
        // open3d::core::Tensor initTsr = open3d::core::Tensor::Eye(
        //     4, open3d::core::Dtype::Float32, (open3d::core::Device)("CUDA:0"));

        // auto result = open3d::t::pipelines::registration::RegistrationICP(
        //     source.To(open3d::core::Device("CUDA:0")), 
        //     target.To(open3d::core::Device("CUDA:0")), 
        //     0.08, initTsr, 
        //     open3d::t::pipelines::registration::TransformationEstimationPointToPlane(),
        //     open3d::t::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 20)
        // );
        // auto tsfm_icp = open3d::core::eigen_converter::TensorToEigenMatrixXd(result.transformation_);

        // ICP Legacy with normal
        // auto result = open3d::pipelines::registration::RegistrationICP(
        //     *p_input_cloud_cr, *p_target_cloud_cr,
        //     0.08, Eigen::Matrix4d::Identity(),
        //     open3d::pipelines::registration::TransformationEstimationPointToPlane(),
        //     open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 20)
        // );
        // auto tsfm_icp = result.transformation_;
        ///////////////////////////////////////////////////////////////////////////////////////////////
        // Legacy RGBD ICP algorithm
        auto result_rgbd = open3d::pipelines::odometry::ComputeRGBDOdometry(
            input_rgbd_img,
            target_rgbd_img,
            intrinsic_,
            init_guess_cam
        );
        bool is_success = false;
        Matrix4d tsfm_rgbd;
        std::tie(is_success, tsfm_rgbd, std::ignore) = result_rgbd;

        // Tensor RGBD ICP algorithm
        // convert to tensor rgbd image
        timer.Start();
        //open3d::t::geometry::RGBDImage input_rgbd_img_t(
        //    open3d::t::geometry::Image::FromLegacyImage(input_rgbd_img.color_).To(open3d::core::Device("CUDA:0")),
        //    open3d::t::geometry::Image::FromLegacyImage(input_rgbd_img.depth_).To(open3d::core::Device("CUDA:0"))
        //);
        //open3d::t::geometry::RGBDImage target_rgbd_img_t(
        //    open3d::t::geometry::Image::FromLegacyImage(target_rgbd_img.color_).To(open3d::core::Device("CUDA:0")),
        //    open3d::t::geometry::Image::FromLegacyImage(target_rgbd_img.depth_).To(open3d::core::Device("CUDA:0"))
        //);
        //// convert intrinsic to tensor
        //auto focal_length = intrinsic_.GetFocalLength();
        //auto principal_point = intrinsic_.GetPrincipalPoint();
        //open3d::core::Tensor intrinsic_t = open3d::core::Tensor::Init<double>(
        //    {{focal_length.first, 0, principal_point.first},
        //     {0, focal_length.second, principal_point.second},
        //     {0, 0, 1}}).To(open3d::core::Device("CUDA:0"));
        //// init transform
        //open3d::core::Tensor trans_t = 
        //    open3d::core::eigen_converter::EigenMatrixToTensor(init_guess_cam)
        //    .To(open3d::core::Device("CUDA:0"));

        //// Parameters
        //float depth_scale = 1.0f;
        //float depth_diff = 0.07f;
        //auto odom_method = open3d::t::pipelines::odometry::Method::Hybrid;
        //// Apply odometry
        //auto result_t = open3d::t::pipelines::odometry::RGBDOdometryMultiScale(
        //    input_rgbd_img_t, target_rgbd_img_t, intrinsic_t, trans_t, depth_scale, 3.0,
        //    std::vector<open3d::t::pipelines::odometry::OdometryConvergenceCriteria>{
        //        30, 20, 10},
        //    odom_method,
        //    open3d::t::pipelines::odometry::OdometryLossParams(depth_diff));
        
        //auto tsfm_rgbd = open3d::core::eigen_converter::TensorToEigenMatrixXd(result_t.transformation_);
        timer.Stop();

        // info mat computation: firstly transform to base coordinate
        p_input_cloud_cr->Transform(t_cam_wrt_base);
        p_target_cloud_cr->Transform(t_cam_wrt_base);
        transform_info = computeInfomation(p_input_cloud_cr, p_target_cloud_cr);

        //ROS_INFO("RGBD result: fitness: %lf rsme: %lf", result_t.fitness_, result_t.inlier_rmse_);
        //ROS_INFO("ICP result: fitness: %lf rsme: %lf", result.fitness_, result.inlier_rmse_);
        ROS_INFO("Applied ICP iteration(s) in %lf ms", timer.GetDuration());
        std::cout << "RGBD Odometry:\n" << tsfm_rgbd << "\n";
        //std::cout << "ICP Odometry:\n" << tsfm_icp * init_guess_cam << "\n";

        if (transform_info.hasNaN())
        {
            ROS_ERROR("INVALID INFO MAT");
            std::cout << "Info matrix 2:\n"
                      << transform_info << std::endl;
        }

        //if (result_t.fitness_ > 0.4)
        if (is_success)
        {
            //transform_result = t_cam_wrt_base * tsfm_icp * init_guess_cam * t_base_wrt_cam;
            transform_result = t_cam_wrt_base * tsfm_rgbd * t_base_wrt_cam;
        }
        else
        {
            ROS_WARN("\nICP has not converged.\n");
            transform_result = init_guess;
        }
        // check the error of the result and init guess
        // reject results that have super large errors
        Eigen::Affine3d af_res(transform_result);
        Eigen::Affine3d af_ini(init_guess);

        Eigen::Affine3d err = af_res * af_ini.inverse();
        double lin_err = err.translation().norm();
        double ang_err = Eigen::AngleAxisd(err.rotation()).angle();
        ROS_INFO("Match error from guess: %lf %lf", lin_err, ang_err);
        if (lin_err > 0.08 || 
            ang_err > 0.08)
        {
            ROS_ERROR("Large match error detected: %lf %lf", lin_err, ang_err);
            transform_result = init_guess;
        }

        //return result_t.fitness_;
        return is_success ? 1 : 0;
    }

    InfoMatrix SubMap::computeInfomation(
        const PtCldPtr& result_cloud,
        const PtCldPtr& target_cloud
    )
    {
        using namespace Eigen;
        size_t pt_counts = target_cloud->points_.size();
        InfoMatrix info_mat;
        info_mat.setZero();

        for(int i = 0; i < pt_counts; i++)
        {
            Vector3d p(target_cloud->points_[i]);
            Vector3d n(target_cloud->normals_[i]);
            Vector3d pxn = p.cross(n);
            auto nt = n.transpose();
            auto pxnt = pxn.transpose();

            info_mat.block<3,3>(0,0) += n * nt;
            info_mat.block<3,3>(0,3) += n * pxnt;
            info_mat.block<3,3>(3,0) += pxn * nt;
            info_mat.block<3,3>(3,3) += pxn * pxnt;
        }

        double sigma = 0.01; // meter
        info_mat /= (double)pt_counts * sigma * sigma;
        
        //EigenSolver<InfoMatrix> eig;
        //eig.compute(info_mat, true);

        return info_mat;
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
        Matrix4d T_o2o = tm_last.inverse().matrix() * current_odom;
        // get the transform from submap's origin to this frame
        return T_f2sm_last * T_o2o;
    }

    Eigen::Matrix<double, 4, 6> SubMap::getLastTipPointsWithTransform(const Eigen::Matrix4d& tf)
    {
        using namespace Eigen;
        // no frames in this submap yet
        if (current_count_ <= 0)
        {
            return Matrix<double, 4, 6>::Zero();
        }
        Matrix4d T_f2sm_last = T_f2sm_[current_count_ - 1];
        Matrix<double, 4, 6> last_tip_pos = tip_states_[current_count_ - 1];
        last_tip_pos.block<1, 6>(3, 0).setOnes();
        
        Matrix<double, 4, 6> transformed_tip_pos =  tf * T_f2sm_last * last_tip_pos;
        transformed_tip_pos.block<1, 6>(3, 0) = tip_states_[current_count_ - 1].block<1, 6>(3, 0);
        return transformed_tip_pos;
    }

    const PtCldPtr SubMap::getSubmapPoints()
    {
        return this->p_submap_points_;
    }

    Eigen::Matrix4d SubMap::getSubmapTfCamWrtBase()
    {
        return this->T_cam_wrt_base_.at(0);
    }

    const PtCldPtr SubMap::getCroppedSubmapPoints()
    {
        return this->p_cropped_submap_points_;
    }
}