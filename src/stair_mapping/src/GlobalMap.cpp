#include "GlobalMap.h"

namespace stair_mapping
{
    GlobalMap::GlobalMap()
        : p_global_map_raw_points_(new PtCld),
          p_global_map_opt_points_(new PtCld),
          p_foothold_ground_patch_(new PtCld),
          last_submap_cnt_(0),
          compensation_coe_(0)
    {
    }

    std::size_t GlobalMap::submapCount()
    {
        return submaps_.size();
    }

    void GlobalMap::addNewSubmap(
        SubMap::Ptr sm,
        Eigen::Matrix4d tf_m2m_laser,
        Eigen::Matrix4d tf_m2m_odom,
        Eigen::Matrix4d tf_m2gm_imu,
        InfoMatrix info_laser)
    {
        // lock since the change of submap number can affect the map building thread
        build_map_mutex_.lock();
        auto submap_cnt = submapCount();
        if (submap_cnt <= 0)
        {
            // first map
            T_m2gm_raw_.push_back(tf_m2m_laser);
            T_m2gm_opt_.push_back(tf_m2m_laser);
            T_m2gm_compensate_.push_back(Eigen::Matrix4d::Identity());
        }
        else
        {
            auto last_tf = T_m2gm_raw_[submap_cnt - 1];
            T_m2gm_raw_.push_back(last_tf * tf_m2m_laser);
            auto last_opt_tf = T_m2gm_opt_[submap_cnt - 1];
            T_m2gm_opt_.push_back(last_opt_tf * tf_m2m_laser);
            T_m2gm_compensate_.push_back(T_m2gm_compensate_[submap_cnt - 1]);
        }
        submaps_.push_back(sm);
        T_m2m_laser_.push_back(tf_m2m_laser);
        info_m2m_laser_.push_back(info_laser);
        T_m2m_odom_.push_back(tf_m2m_odom);
        T_m2gm_imu_.push_back(tf_m2gm_imu);
        build_map_mutex_.unlock();
    }

    SubMap::Ptr GlobalMap::getLastSubMap()
    {
        if (submaps_.size() == 0)
            return nullptr;
        else
            return submaps_[submaps_.size() - 1];
    }

    Eigen::Matrix4d GlobalMap::getLastSubMapRawTf()
    {
        Eigen::Matrix4d last_tf;

        build_map_mutex_.lock();
        if (submaps_.size() == 0)
            last_tf = Eigen::Matrix4d::Identity();
        else
            last_tf = T_m2gm_raw_[T_m2gm_raw_.size() - 1];
        build_map_mutex_.unlock();

        return last_tf;
    }

    Eigen::Matrix4d GlobalMap::getLastSubMapOptTf()
    {
        Eigen::Matrix4d last_opt_tf;

        build_map_mutex_.lock();
        if (submaps_.size() == 0)
            last_opt_tf = Eigen::Matrix4d::Identity();
        else
            last_opt_tf = T_m2gm_compensate_[T_m2gm_compensate_.size() - 1] *
                          T_m2gm_opt_[T_m2gm_opt_.size() - 1];
        build_map_mutex_.unlock();

        return last_opt_tf;
    }

    Eigen::Matrix4d GlobalMap::getCorrectTf()
    {
        Eigen::Matrix4d last_opt_tf;
        Eigen::Matrix4d last_robot_imu_tf;

        build_map_mutex_.lock();
        if (submaps_.size() == 0)
        {
            last_opt_tf = Eigen::Matrix4d::Identity();
            last_robot_imu_tf = last_opt_tf;
        }
        else
        {
            last_opt_tf = T_m2gm_compensate_[T_m2gm_compensate_.size() - 1] *
                          T_m2gm_opt_[T_m2gm_opt_.size() - 1];
            last_robot_imu_tf = T_m2gm_imu_[T_m2gm_imu_.size() - 1];
        }
        build_map_mutex_.unlock();

        Eigen::Affine3d r(last_robot_imu_tf);

        return last_opt_tf * r.inverse().matrix();
    }

    const PtCldPtr GlobalMap::getGlobalMapRawPoints()
    {
        return p_global_map_raw_points_;
    }

    const PtCldPtr GlobalMap::getGlobalMapOptPoints()
    {
        return p_global_map_opt_points_;
    }
    const PtCldPtr GlobalMap::getGroundPatchPoints()
    {
        return p_foothold_ground_patch_;
    }

    std::size_t GlobalMap::updateGlobalMapPoints(bool display_raw_result)
    {
        using namespace Eigen;
        using namespace open3d;

        PtCldPtr p_all_raw_points = std::make_shared<PtCld>();
        PtCldPtr p_all_opt_points = std::make_shared<PtCld>();

        build_map_mutex_.lock();
        int submap_cnt = submapCount();
        build_map_mutex_.unlock();

        // if no submap, no need to generate global map points
        if (submap_cnt <= 0)
            return 0;

        // since only the old data are read and never changed
        // the map building process is thread safe
        int lastn = submap_cnt - 200;

        if (display_raw_result)
        {
            for (int i = 0; i < submap_cnt; i++)
            {
                if (i < lastn)
                    continue;
                auto transformed_raw_pts = *submaps_[i]->getSubmapPoints();
                auto t_cam_wrt_base = submaps_[i]->getSubmapTfCamWrtBase();
                // ^wp = ^wT_{b_i}' * ^bT_c * ^cp
                transformed_raw_pts.Transform(T_m2gm_raw_[i] * t_cam_wrt_base);
                p_all_raw_points->operator+=(transformed_raw_pts);
            }
        }

        open3d::utility::Timer timer;
        timer.Start();

        //for (int i = 0; i < submap_cnt; i++)
        //{
        //    if (i < lastn)
        //        continue;
        //    Matrix4d T_m2gm_refined = T_m2gm_compensate_[i] * T_m2gm_opt_[i];
        //    auto transformed_opt_pts = *submaps_[i]->getCroppedSubmapPoints();
        //    auto t_cam_wrt_base = submaps_[i]->getSubmapTfCamWrtBase();
        //    // ^wp = ^wT_{b_i}' * ^bT_c * ^cp
        //    transformed_opt_pts.Transform(T_m2gm_refined * t_cam_wrt_base);
        //    p_all_opt_points->operator+=(transformed_opt_pts);
        //}

        // use tsdf integration
        double length = 3.0;
        double resolution = 512;
        double sdf_trunc_percentage = 0.01;
        pipelines::integration::ScalableTSDFVolume volume(
            length / (double)resolution, length * sdf_trunc_percentage,
            pipelines::integration::TSDFVolumeColorType::RGB8);

        for (int i = 0; i < submap_cnt; i++)
        {
            if (i < lastn)
                continue;
            Matrix4d T_m2gm_refined = T_m2gm_compensate_[i] * T_m2gm_opt_[i];
            geometry::RGBDImage opt_rgbd_frame;
            auto ret = submaps_[i]->getFirstRGBDFrame(opt_rgbd_frame);
            if (!ret) continue;
            auto intrinsic = submaps_[i]->getIntrinsic();
            auto t_cam_wrt_base = submaps_[i]->getSubmapTfCamWrtBase();
            Matrix4d extrinsic = T_m2gm_refined * t_cam_wrt_base;
            // ^wp = ^wT_{b_i}' * ^bT_c * ^cp
            volume.Integrate(opt_rgbd_frame, intrinsic, extrinsic.inverse());
        }
        p_all_opt_points = volume.ExtractPointCloud();

        // ready for calculating the distance between footholds and generated ground
        Matrix<double, 4, 6> last_tip_points;
        Matrix<double, 1, 6> distance;
        estimateFootGroundDistance(submap_cnt, p_all_opt_points,
                                   last_tip_points, p_foothold_ground_patch_, distance);
        double c = calculateFootGroundCompensation(submap_cnt, last_tip_points, distance);
        compensation_coe_ += c;

        timer.Stop();
        ROS_INFO("Global map generated time: %lf ms", timer.GetDuration());
        ROS_INFO("Overall compensation: %lf", compensation_coe_);

        *p_global_map_opt_points_ = *p_all_opt_points;
        *p_global_map_raw_points_ = *p_all_raw_points;

        return submap_cnt;
    }

    void GlobalMap::estimateFootGroundDistance(
        int submap_count,
        const PtCldPtr &ground,
        Eigen::Matrix<double, 4, 6> &last_tip_points,
        PtCldPtr &ground_patch,
        Eigen::Matrix<double, 1, 6> &distance)
    {
        using namespace Eigen;

        PtCldPtr ground_under_robot = std::make_shared<PtCld>();
        ground_patch->Clear();
        distance.setConstant(1e8); // set to an unreal initial number

        // get the tip points of last submap w.r.t. global cs
        int last_id = submap_count - 1;
        Matrix4d T_m2gm_last = T_m2gm_compensate_[last_id] * T_m2gm_opt_[last_id];
        last_tip_points = submaps_[last_id]->getLastTipPointsWithTransform(T_m2gm_last);

        // crop the X and Y range of the ground since only the part under the robot is used
        double loose = 0.03;
        double y_crop_max = last_tip_points.row(1).maxCoeff() + loose;
        double y_crop_min = last_tip_points.row(1).minCoeff() - loose;
        double x_crop_max = last_tip_points.row(0).maxCoeff() + loose;
        double x_crop_min = last_tip_points.row(0).minCoeff() - loose;

        PreProcessor::crop(ground, ground_under_robot,
                           Vector3d(x_crop_min, y_crop_min, -10),
                           Vector3d(x_crop_max, y_crop_max, 10));

        // check if last tip points are valid
        if (!last_tip_points.isZero() && !ground_under_robot->IsEmpty())
        {
            //std::vector<PointT, Eigen::aligned_allocator<PointT>> center_list[6];
            for (int i = 0; i < 6; i++)
            {
                // skip flying points
                if (last_tip_points.col(i)(3) < 0.95)
                    continue;

                Vector3d foothold = last_tip_points.col(i).topRows(3);
                Vector3d direction = -Vector3d::UnitZ();

                // crop the area under foottip
                PtCldPtr ground_under_foot = std::make_shared<PtCld>();
                loose = 0.015;
                x_crop_max = foothold.x() + loose;
                x_crop_min = foothold.x() - loose;
                y_crop_max = foothold.y() + loose;
                y_crop_min = foothold.y() - loose;

                PreProcessor::crop(ground_under_robot, ground_under_foot,
                                   Vector3d(x_crop_min, y_crop_min, -10),
                                   Vector3d(x_crop_max, y_crop_max, 10));
                auto voxel_ground = ground_under_foot->VoxelDownSample(0.02);

                // find the upmost vexel to the foothold
                double max_distance = 0.1;
                double dz = foothold(2) - voxel_ground->GetMaxBound().z();
                if (fabs(dz) < max_distance) // reject dist too far away
                {
                    distance[i] = dz;
                }
                *ground_patch += *ground_under_foot;
            }
        }
    }

    double GlobalMap::calculateFootGroundCompensation(
        int submap_count,
        const Eigen::Matrix<double, 4, 6> &last_tip_points,
        const Eigen::Matrix<double, 1, 6> &distance)
    {
        using namespace Eigen;

        int last_id = submap_count - 1;
        //Matrix4d T_m2gm_last = T_m2gm_compensate_[last_id] * T_m2gm_opt_[last_id];

        // calculate compensation to reduce the dist
        // firstly we calculate the weighted average distance error as
        //                    sigma_i\in{valid_set} ( (x0-foothold[i].x) * distance[i] )
        //    distance_err = ------------------------------------------------------------------
        //                                     count(valid_set)
        // where x0 = body_center + vision_distance
        int valid_cnt = 0;
        double err_sum = 0;
        //double x0 = Affine3d(T_m2gm_last).translation().x() + 1.5;

        for (int i = 0; i < 6; i++)
        {
            if (fabs(distance[i]) > 0.1) // reject distances too large
                continue;
            Vector3d foothold = last_tip_points.col(i).topRows(3);
            //err_sum += (x0 - foothold.x()) * distance[i];
            err_sum += distance[i];
            valid_cnt++;
        }

        double weighted_err = (valid_cnt == 0) ? 0 : err_sum / valid_cnt;

        // Compensation model:
        // c = k * weighted_dist_err
        return -0.03 * weighted_err;
    }

    bool GlobalMap::runGlobalPoseOptimizer()
    {
        pg_.reset();

        build_map_mutex_.lock();
        auto submap_cnt = submapCount();
        build_map_mutex_.unlock();

        // only optimize every 3 submap have been added
        if (!(submap_cnt > 0 && submap_cnt % 2 == 0))
            return false;

        // if we have run optimizer before at this submap cnt, skip optimizing
        if (submap_cnt == last_submap_cnt_)
            return false;

        // add vertices
        for (int i = 0; i < submap_cnt; i++)
        {
            Vertex3d v(T_m2gm_raw_[i]);
            pg_.addVertex(v);
        }
        // edge of submap-to-submap scan match constraints
        for (int i = 0; i < submap_cnt - 1; i++)
        {
            Pose3d t_edge(T_m2m_laser_[i + 1]);
            pg_.addEdge(EDGE_TYPE::TRANSFORM, i, i + 1, t_edge, info_m2m_laser_[i + 1]);
        }
        // edge of submap-to-submap odom constraints
        for (int i = 0; i < submap_cnt - 1; i++)
        {
            InfoMatrix ifm;
            ifm.setZero();
            // only weight translations
            ifm.diagonal() << 900, 900, 900, 1e-16, 1e-16, 1e-16;
            Pose3d t_edge(T_m2m_odom_[i + 1]);
            pg_.addEdge(EDGE_TYPE::TRANSLATION, i, i + 1, t_edge, ifm);
        }
        // edge of orientation imu constraints
        //if (submap_cnt > 0)
        for (int i = 0; i < submap_cnt - 1; i+=2)
        {
            //Pose3d t_edge(T_m2gm_imu_[submap_cnt - 1]);
            Pose3d t_edge(T_m2gm_imu_[i + 1]);
            InfoMatrix ifm;
            ifm.setZero();
            // only weight orientations
            ifm.diagonal() << 1e-16, 1e-16, 1e-16, 900, 900, 900;
            pg_.addEdge(EDGE_TYPE::ABS_ROTATION, 0, i + 1, t_edge, ifm);
        }

        // solve
        try
        {
            bool ret = pg_.solve();
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            return false;
        }

        // copy out results
        for (int i = 0; i < submap_cnt; i++)
        {
            const Vertex3d v = pg_.getVertices()->at(i);
            T_m2gm_opt_[i] = v.toMat4d();
            double x_frame = T_m2gm_opt_[i](0, 3);
            double y_frame = T_m2gm_opt_[i](1, 3);

            // compensate Z drift using the distance from the origin
            T_m2gm_compensate_[i](2, 3) = compensation_coe_ * sqrt(x_frame * x_frame + y_frame * y_frame);
            // T_m2gm_compensate_[i](2, 3) = 0 * sqrt(x_frame * x_frame + y_frame * y_frame);

            // compensate X drift due to the foot shape by coe * distance_traversed
            T_m2gm_compensate_[i](0, 3) = 0.0 * (x_frame);
            T_m2gm_compensate_[i](1, 3) = 0.0 * (y_frame);
            // T_m2gm_compensate_[i](0, 3) = 0 * (x_frame);
        }

        last_submap_cnt_ = submap_cnt;

        return true;
    }

}