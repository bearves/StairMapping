#include "SubMap.h"
#include <exception>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp6d.h>
#include <pcl/console/time.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include "PoseGraph.h"

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
            pcl::transformPointCloud(frames_[i], transformed_frame, T_f2sm_[i]);
            p_all_points->operator+=( transformed_frame );
        }
        *p_submap_points_ = *p_all_points;
    }

    double SubMap::match(PointCloudT frame, 
        const Eigen::Matrix4d& init_guess, 
        Eigen::Matrix4d& t_match_result,
        InfoMatrix& info_match_result)
    {
        int point_counts = frame.width * frame.height;
        
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
            // TODO: ICP/NDT match here
            double score = matchIcp(
                frame.makeShared(), 
                p_submap_points_, 
                init_guess, 
                t_match_result,
                info_match_result);

            std::cout << "Score:" << score << std::endl;
            std::cout << "Init guess:\n" << init_guess << std::endl;
            std::cout << "Registration result:\n" << t_match_result << std::endl;
            std::cout << "Info matrix eig:\n" << info_match_result.eigenvalues().real() << std::endl;
            //t_match_result.block<3,3>(0, 0) = Eigen::Matrix3d::Identity();

            //t_match_result = init_guess;
            return score; // best score
        }
    }

    double SubMap::matchIcp(
        const PointCloudT::Ptr& input_cloud, 
        const PointCloudT::Ptr& target_cloud, 
        const Eigen::Matrix4d& init_guess, 
        Eigen::Matrix4d& transform_result,
        InfoMatrix& transform_info)
    {
        PointCloudT::Ptr p_input_cloud_init(new PointCloudT);
        PointCloudT::Ptr p_input_cloud_cr(new PointCloudT);
        PointCloudT::Ptr p_target_cloud_cr(new PointCloudT);
        PointCloudN::Ptr p_input_normal_cr(new PointCloudN);
        PointCloudN::Ptr p_target_normal_cr(new PointCloudN);
        PointCloudTN::Ptr p_input_tn(new PointCloudTN);
        PointCloudTN::Ptr p_target_tn(new PointCloudTN);

        // firstly transform the input cloud with init_guess to 
        // make the two clouds overlap as much as possible
        pcl::transformPointCloud(*input_cloud, *p_input_cloud_init, init_guess);

        // since the stairs are very similar structures
        // crop on Z and X axis to avoid the stair-jumping mismatch
        // aka. match this step to the next
        PointT min_input, max_input, min_target, max_target;
        pcl::getMinMax3D(*p_input_cloud_init, min_input, max_input);
        pcl::getMinMax3D(*target_cloud, min_target, max_target);

        double loose = 0.1;
        double z_crop_max = std::min(max_input.z, max_target.z) + loose;
        double z_crop_min = std::max(min_input.z, min_target.z) - loose;
        double x_crop_max = std::min(max_input.x, max_target.x) + loose;
        double x_crop_min = std::max(min_input.x, min_target.x) - loose;

        PreProcessor::crop(p_input_cloud_init, p_input_cloud_cr, 
            Eigen::Vector3f(x_crop_min, -0.35, z_crop_min), 
            Eigen::Vector3f(x_crop_max, 0.35, z_crop_max));
        PreProcessor::crop(target_cloud, p_target_cloud_cr, 
            Eigen::Vector3f(x_crop_min, -0.35, z_crop_min), 
            Eigen::Vector3f(x_crop_max, 0.35, z_crop_max));

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
        icp.setMaximumIterations(50);
        icp.setMaxCorrespondenceDistance(0.08);
        
        icp.setInputSource(p_input_tn);
        icp.setInputTarget(p_target_tn);

        icp.align(*icp_result_cloud);
        transform_info = computeInfomation(icp_result_cloud, p_target_tn);
        std::cout << "Applied ICP iteration(s) in " << time.toc() << " ms" << std::endl;

        if (icp.hasConverged())
        {
            transform_result = icp.getFinalTransformation().cast<double>() * init_guess;
        }
        else
        {
            std::cout << "\nICP has not converged.\n";
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
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        ne.setSearchMethod(p_tree);
        ne.setInputCloud(input_cloud);
        ne.setKSearch(10);
        ne.compute(*normal_cloud);
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

    const PointCloudT::Ptr SubMap::getSubmapPoints()
    {
        return this->p_submap_points_;
    }
}