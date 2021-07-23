#include "SubMap.h"
#include <exception>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp6d.h>
#include <pcl/console/time.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/ply_io.h>
#include "PoseGraph.h"
#include <fstream>

namespace stair_mapping
{
    bool SubMap::PRINT_VERBOSE_INFO = false;

    SubMap::SubMap(int max_stored_pcl_count):
        max_stored_frame_count_(max_stored_pcl_count),
        current_count_(0),
        p_submap_points_(new PointCloudT),
        p_cropped_submap_points_(new PointCloudT)
    {
    }

    void SubMap::init()
    {
        current_count_ = 0;
        T_f2sm_.clear();
        frames_.clear();
        T_odom_.clear();
        T_cam_wrt_base_.clear();
    }

    void SubMap::addFrame(PointCloudT frame, 
        const Eigen::Matrix4d& t_f2sm, 
        const Eigen::Matrix4d& t_frame_odom,
        const Eigen::Matrix4d& t_cam_wrt_base,
        const Eigen::Matrix<double, 4, 6>& tip_states)
    {
        frames_.push_back(frame);
        T_f2sm_.push_back(t_f2sm); // ^{b_{i-1}}T_{b_i}
        T_cam_wrt_base_.push_back(t_cam_wrt_base);
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

        PointCloudT transformed_frame;
        PointCloudT::Ptr p_all_points(new PointCloudT);

        // add all frame points to submap
        for(int i = 0; i < current_count_; i++)
        {
            // Transform existed frame points to the submap's origin wrt camera local coordinate,
            // In this way, the icp matching can be conducted between current submap points 
            // and the next key frame.
            Matrix4d T_base_wrt_cam = Affine3d(T_cam_wrt_base_[i]).inverse().matrix();
            Matrix4d T_f2sm_wrt_cam = T_base_wrt_cam * T_f2sm_[i] * T_cam_wrt_base_[i];
            pcl::transformPointCloud(frames_[i], transformed_frame, T_f2sm_wrt_cam);
            p_all_points->operator+=( transformed_frame );
        }
        *p_submap_points_ = *p_all_points;

        // crop for global map building
        PreProcessor::crop(
            p_submap_points_,
            p_cropped_submap_points_,
            Vector3f(-0.5, -2, 0),
            Vector3f(0.5, 2, 1.5));
    }

    double SubMap::match(
        const PointCloudT::Ptr& frame, 
        const Eigen::Matrix4d& init_guess, 
        const Eigen::Matrix4d& t_cam_wrt_base, 
        Eigen::Matrix4d& t_match_result,
        InfoMatrix& info_match_result)
    {
        int point_counts = frame->width * frame->height;
        
        if (point_counts < 300)
        {
            throw std::runtime_error("Too few points in this frame");
        }

        if (current_count_ <= 0)
        {
            t_match_result = Eigen::Matrix4d::Identity();
            info_match_result.setZero();
            info_match_result.diagonal() << 100, 100, 100, 20, 20, 20;
            return 0; // best score
        }
        else
        {
            t_match_result = init_guess;
            double score = matchIcp(
                frame, 
                p_submap_points_, 
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
            // t_match_result = init_guess;
            return score; // best score
        }
    }

    double SubMap::matchIcp(
        const PointCloudT::Ptr& input_cloud, 
        const PointCloudT::Ptr& target_cloud, 
        const Eigen::Matrix4d& init_guess, 
        const Eigen::Matrix4d& t_cam_wrt_base, 
        Eigen::Matrix4d& transform_result,
        InfoMatrix& transform_info)
    {
        using namespace Eigen;

        Matrix4d t_base_wrt_cam = Affine3d(t_cam_wrt_base).inverse().matrix();
        // the transform guess in the camera's local frame
        Matrix4d init_guess_cam = t_base_wrt_cam * init_guess * t_cam_wrt_base;

        PointCloudT::Ptr p_input_cloud_init(new PointCloudT);
        PointCloudT::Ptr p_input_cloud_cr(new PointCloudT);
        PointCloudT::Ptr p_target_cloud_cr(new PointCloudT);
        PointCloudN::Ptr p_input_normal_cr(new PointCloudN);
        PointCloudN::Ptr p_target_normal_cr(new PointCloudN);
        PointCloudTN::Ptr p_input_tn(new PointCloudTN);
        PointCloudTN::Ptr p_target_tn(new PointCloudTN);
        PointCloudTN::Ptr p_target_tn_wrt_base(new PointCloudTN);

        // firstly transform the input cloud with init_guess to 
        // make the two clouds overlap as much as possible
        pcl::transformPointCloud(*input_cloud, *p_input_cloud_init, init_guess_cam);

        // since the stairs are very similar structures
        // crop on Z and X axis to avoid the stair-jumping mismatch
        // aka. match this step to the next
        PointT min_input, max_input, min_target, max_target;
        pcl::getMinMax3D(*p_input_cloud_init, min_input, max_input);
        pcl::getMinMax3D(*target_cloud, min_target, max_target);

        double loose = 0.1;
        double z_crop_max = std::min(max_input.z, max_target.z) + loose;
        double z_crop_min = std::max(min_input.z, min_target.z) - loose;
        double y_crop_max = std::min(max_input.y, max_target.y) + loose;
        double y_crop_min = std::max(min_input.y, min_target.y) - loose;
        double x_crop_max = std::min(max_input.x, max_target.x) + loose;
        double x_crop_min = std::max(min_input.x, min_target.x) - loose;

        PreProcessor::crop(p_input_cloud_init, p_input_cloud_cr, 
            Vector3f(x_crop_min, y_crop_min, z_crop_min), 
            Vector3f(x_crop_max, y_crop_max, z_crop_max));
        PreProcessor::crop(target_cloud, p_target_cloud_cr, 
            Vector3f(x_crop_min, y_crop_min, z_crop_min), 
            Vector3f(x_crop_max, y_crop_max, z_crop_max));

        if (p_input_cloud_cr->empty() || p_target_cloud_cr->empty())
        {
            ROS_WARN("Empty input cloud when matching");
            transform_result = init_guess;
            return 1e8;
        }

        pcl::console::TicToc time;
        time.tic();
        // compute normal for each clouds
        getNormal(p_input_cloud_cr, p_input_normal_cr);
        getNormal(p_target_cloud_cr, p_target_normal_cr);
        pcl::concatenateFields(*p_input_cloud_cr, *p_input_normal_cr, *p_input_tn);
        pcl::concatenateFields(*p_target_cloud_cr, *p_target_normal_cr, *p_target_tn);

        // ICP with normal
        pcl::IterativeClosestPointWithNormals<PointTN, PointTN> icp;

        PointCloudTN::Ptr icp_result_cloud(new PointCloudTN);
        PointCloudTN::Ptr icp_result_wrt_base(new PointCloudTN);
        icp.setMaximumIterations(20);
        icp.setMaxCorrespondenceDistance(0.08);
        icp.setTransformationEpsilon(1e-6);
        
        icp.setInputSource(p_input_tn);
        icp.setInputTarget(p_target_tn);

        icp.align(*icp_result_cloud);
        Matrix4d tsfm_icp = icp.getFinalTransformation().cast<double>();

        // info mat computation: firstly transform to base coordinate
        pcl::transformPointCloudWithNormals(*icp_result_cloud, *icp_result_wrt_base, t_cam_wrt_base);
        pcl::transformPointCloudWithNormals(*p_target_tn, *p_target_tn_wrt_base, t_cam_wrt_base);

        transform_info = computeInfomation(icp_result_wrt_base, p_target_tn_wrt_base);
        ROS_INFO("Applied ICP iteration(s) in %lf ms", time.toc());

        if (transform_info.hasNaN())
        {
            ROS_ERROR("INVALID INFO MAT");
            std::cout << "Info matrix 2:\n"
                      << transform_info << std::endl;
        }

        Matrix4d raw_transform_result = init_guess;

        if (icp.hasConverged())
        {
            raw_transform_result = t_cam_wrt_base * tsfm_icp * init_guess_cam * t_base_wrt_cam;
        }
        else
        {
            ROS_WARN("\nICP has not converged.\n");
            raw_transform_result = init_guess;
        }

        InfoMatrix info_ro = 100 * InfoMatrix::Identity();
        transform_result = optimizeF2FMatch(raw_transform_result, init_guess, transform_info, info_ro);

        // check the error of the result and init guess
        // reject results that have super large errors
        auto err = checkError(transform_result, init_guess);
        if (err(0) > 0.05 || err(1) > 0.05)
        {
            ROS_ERROR("Large match error detected: %lf %lf", err(0), err(1));

            if (PRINT_VERBOSE_INFO)
            {
                saveDiagnosticData(raw_transform_result, init_guess,
                                   transform_result, transform_info, info_ro,
                                   p_target_cloud_cr, p_input_cloud_cr);
            }
            // try optimize again towards init guess
            transform_result = optimizeF2FMatch(transform_result, init_guess, transform_info, info_ro);
            err = checkError(transform_result, init_guess);
            ROS_ERROR("Final match error: %lf %lf", err(0), err(1));
            if (err(0) > 0.05 || err(1) > 0.05)
                transform_result = init_guess;
        }

        return icp.getFitnessScore();
    }

    void SubMap::getNormal(
        const PointCloudT::Ptr& input_cloud,
        const PointCloudN::Ptr& normal_cloud
    )
    {
        pcl::search::KdTree<PointT>::Ptr p_tree(new pcl::search::KdTree<PointT>);
        pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
        ne.setInputCloud(input_cloud);
        ne.setSearchMethod(p_tree);
        ne.setKSearch(10);
        ne.compute(*normal_cloud);
    }

    Eigen::Matrix4d SubMap::optimizeF2FMatch(
        Eigen::Matrix4d tf_vo,
        Eigen::Matrix4d tf_ro,
        InfoMatrix info_vo,
        InfoMatrix info_ro)
    {
        using namespace Eigen;

        pg_.reset();

        // add vertices
        Vertex3d v_src(Matrix4d::Identity());
        Vertex3d v_tgt(tf_ro);
        pg_.addVertex(v_src);
        pg_.addVertex(v_tgt);
        // edge of submap-to-submap scan match constraints
        Pose3d t_edge_vo(tf_vo);
        pg_.addEdge(EDGE_TYPE::TRANSFORM, 0, 1, t_edge_vo, info_vo);
        // edge of submap-to-submap legged odom constraints
        Pose3d t_edge_lo(tf_ro);
        pg_.addEdge(EDGE_TYPE::TRANSFORM, 0, 1, t_edge_lo, info_ro);

        // solve using huber loss
        try
        {
            bool ret = pg_.solve(true, 0.08);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            return tf_ro;
        }

        // copy out optimized results
        return pg_.getVertices()->at(1).toMat4d();
    }

    Eigen::Vector2d SubMap::checkError(Eigen::Matrix4d tf_result, Eigen::Matrix4d init_guess)
    {
        Eigen::Affine3d af_res(tf_result);
        Eigen::Affine3d af_ini(init_guess);

        Eigen::Affine3d err = af_res * af_ini.inverse();
        double lin_err = err.translation().norm();
        double ang_err = Eigen::AngleAxisd(err.rotation()).angle();
        ROS_INFO("Match error from guess: %lf %lf", lin_err, ang_err);

        return {lin_err, ang_err};
    }

    InfoMatrix SubMap::computeInfomation(
        const PointCloudTN::Ptr& result_cloud,
        const PointCloudTN::Ptr& target_cloud
    )
    {
        using namespace Eigen;
        size_t pt_counts = target_cloud->width * target_cloud->height;
        InfoMatrix info_mat;
        info_mat.setZero();

        for(auto pt : *target_cloud)
        {
            Vector3d p(pt.x, pt.y, pt.z);
            Vector3d n(pt.normal_x, pt.normal_y, pt.normal_z);
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
        
        return info_mat;
    }

    void SubMap::saveDiagnosticData(
        const Eigen::Matrix4d& tf_vo, 
        const Eigen::Matrix4d& tf_ro, 
        const Eigen::Matrix4d& tf_opt,
        const InfoMatrix& info_vo, 
        const InfoMatrix& info_ro,
        const PointCloudT::Ptr& p_target_cloud, 
        const PointCloudT::Ptr& p_input_cloud )
    {
        static int match_count = 0;

        match_count++;
        char input_cld_name[30];
        char target_cld_name[30];
        char match_info_name[30];

        sprintf(input_cld_name, "Input_%d.ply", match_count);
        sprintf(target_cld_name, "Target_%d.ply", match_count);
        sprintf(match_info_name, "MatchInfo_%d.txt", match_count);

        std::fstream match_info_file;
        match_info_file.open(match_info_name, std::ios::out);

        if (!match_info_file.is_open())
            ROS_ERROR("Open file failed: %s", match_info_name);

        match_info_file << "match_count:\n"
                        << match_count << "\n";
        match_info_file << "Ro result:\n"
                        << tf_ro << "\n";
        match_info_file << "Ro Info:\n"
                        << info_ro << "\n";
        match_info_file << "Vo result:\n"
                        << tf_vo << "\n";
        match_info_file << "Vo Info:\n"
                        << info_vo << "\n";
        match_info_file << "After opt:\n"
                        << tf_opt << "\n";

        pcl::io::savePLYFileASCII(target_cld_name, *p_target_cloud);
        pcl::io::savePLYFileASCII(input_cld_name, *p_input_cloud);
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
        Matrix4d T_o2o = tm_last.inverse().matrix() * current_odom ;
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

    const PointCloudT::Ptr SubMap::getSubmapPoints()
    {
        return this->p_submap_points_;
    }

    const PointCloudT::Ptr SubMap::getCroppedSubmapPoints()
    {
        return this->p_cropped_submap_points_;
    }

    Eigen::Matrix4d SubMap::getSubmapTfCamWrtBase()
    {
        return this->T_cam_wrt_base_.at(0);
    }
}