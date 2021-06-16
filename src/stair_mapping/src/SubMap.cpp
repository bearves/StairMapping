#include "SubMap.h"
#include <exception>
#include <open3d/Open3D.h>
#include "PoseGraph.h"

namespace stair_mapping
{

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
        T_odom_.clear();
    }

    void SubMap::addFrame(PtCld frame, 
        Eigen::Matrix4d t_f2sm, 
        Eigen::Matrix4d t_frame_odom,
        const Eigen::Matrix<double, 4, 6>& tip_states)
    {
        frames_.push_back(frame);
        T_f2sm_.push_back(t_f2sm);
        T_odom_.push_back(t_frame_odom);
        tip_states_.push_back(tip_states);
        
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
            auto transformed_frame = frames_[i];
            transformed_frame.Transform(T_f2sm_[i]);
            p_all_points->operator+=( transformed_frame );
        }
        *p_submap_points_ = *p_all_points;

        // crop for global map building
        PreProcessor::crop(
            p_submap_points_,
            p_cropped_submap_points_,
            Vector3d(0, -0.5, -2),
            Vector3d(1.5, 0.5, 0.4));
    }

    double SubMap::match(
        const PtCldPtr& frame, 
        const Eigen::Matrix4d& init_guess, 
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
                init_guess, 
                t_match_result,
                info_match_result);

#if VERBOSE_INFO
            std::cout << "Score:" << score << std::endl;
            std::cout << "Init guess:\n" << init_guess << std::endl;
            std::cout << "Registration result:\n" << t_match_result << std::endl;
            std::cout << "Info matrix eig:\n" << info_match_result.eigenvalues().real() << std::endl;
#endif

            //t_match_result = init_guess;
            return score; // best score
        }
    }

    double SubMap::matchIcp(
        const PtCldPtr& input_cloud, 
        const PtCldPtr& target_cloud, 
        const Eigen::Matrix4d& init_guess, 
        Eigen::Matrix4d& transform_result,
        InfoMatrix& transform_info)
    {
        using namespace open3d::geometry;

        PtCldPtr p_input_cloud_init = std::make_shared<PtCld>();
        PtCldPtr p_input_cloud_cr = std::make_shared<PtCld>();
        PtCldPtr p_target_cloud_cr = std::make_shared<PtCld>();

        // firstly transform the input cloud with init_guess to 
        // make the two clouds overlap as much as possible
        *p_input_cloud_init = *input_cloud;
        p_input_cloud_init->Transform(init_guess);

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
        double x_crop_max = std::min(max_input.x(), max_target.x()) + loose;
        double x_crop_min = std::max(min_input.x(), min_target.x()) - loose;

        PreProcessor::crop(p_input_cloud_init, p_input_cloud_cr, 
            Eigen::Vector3d(x_crop_min, -0.4, z_crop_min), 
            Eigen::Vector3d(x_crop_max, 0.4, z_crop_max));
        PreProcessor::crop(target_cloud, p_target_cloud_cr, 
            Eigen::Vector3d(x_crop_min, -0.4, z_crop_min), 
            Eigen::Vector3d(x_crop_max, 0.4, z_crop_max));

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

        // ICP with normal
        auto result = open3d::pipelines::registration::RegistrationICP(
            *p_input_cloud_cr, *p_target_cloud_cr,
            0.08, Eigen::Matrix4d::Identity(),
            open3d::pipelines::registration::TransformationEstimationPointToPlane(),
            open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 20)
        );
        transform_info = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(
            *p_input_cloud_cr,
            *p_target_cloud_cr,
            0.08,
            result.transformation_
        );

        //transform_info = computeInfomation(icp_result_cloud, p_target_tn);

        timer.Stop();
        ROS_INFO("ICP result: fitness: %lf rsme: %lf", result.fitness_, result.inlier_rmse_);
        ROS_INFO("Applied ICP iteration(s) in %lf ms", timer.GetDuration());

        if (result.fitness_ > 0.6)
        {
            transform_result = result.transformation_ * init_guess;
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
        if (lin_err > 0.1 || 
            ang_err > 0.2)
        {
            ROS_ERROR("Large match error detected: %lf %lf", lin_err, ang_err);
            transform_result = init_guess;
        }

        return result.fitness_;
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
        Matrix4d T_o2o = current_odom * tm_last.inverse().matrix() ;
        // get the transform from submap's origin to this frame
        return T_o2o * T_f2sm_last;
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

    const PtCldPtr SubMap::getCroppedSubmapPoints()
    {
        return this->p_cropped_submap_points_;
    }
}