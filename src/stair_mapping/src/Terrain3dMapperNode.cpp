#include "Terrain3dMapperNode.h"
#include <termios.h>
#include "PreProcessor.h"
#include <open3d/Open3D.h>
#include <open3d_conversions/open3d_conversions.h>

namespace stair_mapping
{
    Terrain3dMapperNode::Terrain3dMapperNode(ros::NodeHandle& node) :
        is_imu_transform_ok_(false),
        color_img_sub_(node, "/camera/color/image_raw", 1),
        depth_img_sub_(node, "/camera/aligned_depth_to_color/image_raw", 1),
        cam_info_sub_(node, "/camera/color/camera_info", 1)
    {
        using namespace Eigen;

        node.param("display_process_details", display_process_details_, false);
        node.param("robot_message_version", message_version_, 2);

        SubMap::PRINT_VERBOSE_INFO = display_process_details_;
        PoseGraph::PRINT_VERBOSE_INFO = display_process_details_;

        imu_calibrator_.setParam(node);
        current_tf_of_baselink_wrt_world_ = Matrix4d::Identity();
        current_tf_of_camera_wrt_baselink_ = Matrix4d::Identity();
        current_odom_mat_ = Matrix4d::Identity();

        if (display_process_details_)
        {
            rgbd_converted_pub_ = node.advertise<sensor_msgs::PointCloud2>("rgbd_converted_points", 1);
            preprocess_pub_ = node.advertise<sensor_msgs::PointCloud2>("preprocessed_points", 1);
            submap_pub_ = node.advertise<sensor_msgs::PointCloud2>("submap_points", 1);
            global_map_raw_pub_ = node.advertise<sensor_msgs::PointCloud2>("global_map_raw_points", 1);
            gnd_patch_pub_ = node.advertise<sensor_msgs::PointCloud2>("gnd_patch_points", 1);
        }

        corrected_odom_pub_ = node.advertise<geometry_msgs::PoseStamped>("corrected_robot_pose", 1);
        global_map_opt_pub_ = node.advertise<sensor_msgs::PointCloud2>("global_map_opt_points", 1);
        tip_points_pub_ = node.advertise<sensor_msgs::PointCloud2>("tip_points", 1);

        odom_sub_ = node.subscribe("/qz_state_publisher/robot_odom", 1, &Terrain3dMapperNode::odomMsgCallback, this);
        imu_sub_ = node.subscribe("/qz_state_publisher/robot_imu", 1, &Terrain3dMapperNode::imuMsgCallback, this);
        pcl_sub_ = node.subscribe("/camera/depth/color/points", 1, &Terrain3dMapperNode::pclMsgCallback, this);
        
        p_sync_rgbd_ = std::shared_ptr<RGBDSync>(new RGBDSync(color_img_sub_, depth_img_sub_, cam_info_sub_, 5));
        p_sync_rgbd_->registerCallback(&Terrain3dMapperNode::rgbdImgMsgCallback, this);

        if (message_version_ == 1)
        {
            tip_state_sub_ = node.subscribe("/qz_state_publisher/robot_tip_state", 1, &Terrain3dMapperNode::tipStateCallback, this);
            gait_phase_sub_ = node.subscribe("/qz_state_publisher/robot_gait_phase", 1, &Terrain3dMapperNode::gaitPhaseCallback, this);
        }
        else if (message_version_ == 2)
        {
            tip_state_sub_ = node.subscribe("/qz_state_publisher/robot_tip_state", 1, &Terrain3dMapperNode::tipStateV2Callback, this);
            gait_phase_sub_ = node.subscribe("/qz_state_publisher/robot_gait_phase", 1, &Terrain3dMapperNode::gaitPhaseV2Callback, this);
        }
    }
    void Terrain3dMapperNode::rgbdImgMsgCallback(
        const Img::ConstPtr &color_msg,
        const Img::ConstPtr &depth_msg,
        const CamInfo::ConstPtr &caminfo_msg)
    {
        ROS_INFO("Cam info: fx=%lf fy=%lf cx=%lf cy=%lf", 
            caminfo_msg->P[0], caminfo_msg->P[0], 
            caminfo_msg->P[2], caminfo_msg->P[6]);

        open3d::camera::PinholeCameraIntrinsic intrinsic;
        intrinsic.SetIntrinsics(caminfo_msg->width, caminfo_msg->height, 
                                caminfo_msg->P[0], caminfo_msg->P[5],
                                caminfo_msg->P[2], caminfo_msg->P[6]);
        
        open3d::geometry::Image color,depth;
        color.Prepare(color_msg->width, color_msg->height, 3, 1); // RGB8
        color.data_ = color_msg->data;
        depth.Prepare(depth_msg->width, depth_msg->height, 1, 2); // Mono16
        depth.data_ = depth_msg->data;
        open3d::geometry::RGBDImage rgbd_frame(color, *depth.ConvertDepthToFloatImage());
        
        sensor_msgs::PointCloud2 rgbd_out_cloud2;
        PtCldPtr rgbd_pc = std::make_shared<PtCld>();
        rgbd_pc = open3d::geometry::PointCloud::CreateFromRGBDImage(rgbd_frame, intrinsic);

        if (display_process_details_)
        {
            open3d_conversions::open3dToRos(*rgbd_pc, rgbd_out_cloud2);
            rgbd_out_cloud2.header.frame_id = "camera_depth_optical_frame";
            rgbd_out_cloud2.header.stamp = color_msg->header.stamp;
            rgbd_converted_pub_.publish(rgbd_out_cloud2);
        }

    }

    void Terrain3dMapperNode::pclMsgCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        using namespace Eigen;
        PtCldPtr p_in_cloud = std::make_shared<PtCld>();
        PtCldPtr p_pre_cloud = std::make_shared<PtCld>();

        sensor_msgs::PointCloud2 pre_out_cloud2;
        sensor_msgs::PointCloud2 submap_out_cloud2;

        open3d_conversions::rosToOpen3d(msg, *p_in_cloud);

        // transform using imu
        imu_msg_mtx_.lock();
        Matrix4d T_camera_wrt_base = current_tf_of_camera_wrt_baselink_; // ^bT_c
        Matrix4d T_base_wrt_world = current_tf_of_baselink_wrt_world_; // ^wT_b (rotation part)
        bool is_transform_ok = is_imu_transform_ok_;
        imu_msg_mtx_.unlock();

        // when imu transform is not ready, do nothing to point cloud data
        if (!is_transform_ok) return;

        // get robot tip states
        tip_msg_mtx_.lock();
        // only rotation is used
        auto tip_states = robot_kin_.getTipPosWithTouchState(Matrix4d::Identity());
        tip_msg_mtx_.unlock();

        // preprocess
        terrain_mapper_.preprocess(p_in_cloud, p_pre_cloud);

        if (display_process_details_)
        {
            open3d_conversions::open3dToRos(*p_pre_cloud, pre_out_cloud2);
            pre_out_cloud2.header.frame_id = "camera_depth_optical_frame";
            pre_out_cloud2.header.stamp = msg->header.stamp;
            preprocess_pub_.publish(pre_out_cloud2);
        }

        // match submaps
        PtCldPtr p_submap_cloud = std::make_shared<PtCld>();
        odom_msg_mtx_.lock();
        Matrix4d t_frame_odom = current_odom_mat_;
        odom_msg_mtx_.unlock();
        terrain_mapper_.matchSubmap(p_pre_cloud, p_submap_cloud, t_frame_odom, T_camera_wrt_base, tip_states);

        if (display_process_details_)
        {
            open3d_conversions::open3dToRos(*p_submap_cloud, submap_out_cloud2);
            submap_out_cloud2.header.frame_id = "camera_depth_optical_frame";
            submap_out_cloud2.header.stamp = msg->header.stamp;
            submap_pub_.publish(submap_out_cloud2);
        }
    }

    void Terrain3dMapperNode::imuMsgCallback(const sensor_msgs::ImuConstPtr &msg)
    {
        using namespace Eigen;

        auto imu_tsfm = imu_calibrator_.updateCalibratedImuTf(msg->orientation);
        auto T_base_wrt_world = imu_calibrator_.getTfOfBaselinkWrtWorld();
        auto T_camera_wrt_base = imu_calibrator_.getTfOfCameraWrtBaseLink();

        imu_msg_mtx_.lock();
        current_tf_of_baselink_wrt_world_ = T_base_wrt_world;
        current_tf_of_camera_wrt_baselink_ = T_camera_wrt_base;
        is_imu_transform_ok_ = imu_calibrator_.isImuTransformReady();
        imu_msg_mtx_.unlock();

        imu_tsfm.header.stamp = msg->header.stamp;
        imu_tsfm.header.frame_id = "base_link";
        imu_tsfm.child_frame_id = "base_world"; 
        br_.sendTransform(imu_tsfm);
    }

    void Terrain3dMapperNode::odomMsgCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        Eigen::Matrix4d pose_mat = getPoseMatrix(*msg);
        imu_msg_mtx_.lock();
        auto t_base_wrt_world = current_tf_of_baselink_wrt_world_;
        imu_msg_mtx_.unlock();
        // use imu's rotation with the robot odom's translation
        pose_mat.topLeftCorner(3,3) = t_base_wrt_world.topLeftCorner(3,3);
        publishCorrectedTf(msg->header.stamp, pose_mat);
        odom_msg_mtx_.lock();
        current_odom_mat_ = pose_mat;
        odom_msg_mtx_.unlock();
    }

    Eigen::Matrix4d Terrain3dMapperNode::getPoseMatrix(const nav_msgs::Odometry &odom)
    {
        using namespace Eigen;

        // TODO: this is a bug of the state publisher since
        // the position data is wrongly published in the twist field
        //auto pos = odom.pose.pose.position;
        auto pos = odom.twist.twist.linear;
        auto ori = odom.pose.pose.orientation;
        Translation3d t(pos.x, pos.y, pos.z);
        Quaterniond q(ori.w, ori.x, ori.y, ori.z);
        // since we have corrected rotation using IMU, we ignore the rotation in odom
        Quaterniond q_corrected(1, 0, 0, 0);
        Affine3d pose = t*q_corrected;

        return pose.matrix();
    }

    void Terrain3dMapperNode::gaitPhaseCallback(const mini_bridge::GaitPhaseConstPtr &msg)
    {
        using namespace Eigen;
        tip_msg_mtx_.lock();
        robot_kin_.updateTouchState(msg->header.stamp, msg->touch_possibility);
        tip_msg_mtx_.unlock();
    }

    void Terrain3dMapperNode::tipStateCallback(const mini_bridge::RobotTipStateConstPtr &msg)
    {
        tip_msg_mtx_.lock();
        robot_kin_.updateTipPosition(msg->header.stamp, msg->tip_pos);
        tip_msg_mtx_.unlock();

        sensor_msgs::PointCloud2 pc2;
        open3d_conversions::open3dToRos(*robot_kin_.getTipPoints(), pc2);
        pc2.header.frame_id = "base_link";
        pc2.header.stamp = ros::Time::now();
        tip_points_pub_.publish(pc2);
    }

    void Terrain3dMapperNode::gaitPhaseV2Callback(const mini_bridge::GaitPhaseV2ConstPtr &msg)
    {
        using namespace Eigen;
        tip_msg_mtx_.lock();
        robot_kin_.updateTouchState(msg->header.stamp, msg->touch_possibility);
        tip_msg_mtx_.unlock();
    }

    void Terrain3dMapperNode::tipStateV2Callback(const mini_bridge::RobotTipStateV2ConstPtr &msg)
    {
        tip_msg_mtx_.lock();
        robot_kin_.updateTipPosition(msg->header.stamp, msg->tip_pos);
        tip_msg_mtx_.unlock();

        sensor_msgs::PointCloud2 pc2;
        open3d_conversions::open3dToRos(*robot_kin_.getTipPoints(), pc2);
        pc2.header.frame_id = "base_link";
        pc2.header.stamp = ros::Time::now();
        tip_points_pub_.publish(pc2);
    }

    void Terrain3dMapperNode::startMapServer()
    {
        th_ = std::thread([this](){
            ROS_INFO("Map server started");
            ros::Rate map_publish_rate(5);
            while(ros::ok())
            {
                terrain_mapper_.buildGlobalMap(display_process_details_);
                publishMap();
                map_publish_rate.sleep();
            }
        });
    }

    void Terrain3dMapperNode::publishMap()
    {
        sensor_msgs::PointCloud2 opt_pc2;
        auto p_global_opt_pc = terrain_mapper_.getGlobalMapOptPoints();
        open3d_conversions::open3dToRos(*p_global_opt_pc, opt_pc2);
        opt_pc2.header.frame_id = "map";
        opt_pc2.header.stamp = ros::Time::now();
        global_map_opt_pub_.publish(opt_pc2);

        if (display_process_details_)
        {
            sensor_msgs::PointCloud2 raw_pc2;
            auto p_global_raw_pc = terrain_mapper_.getGlobalMapRawPoints();
            open3d_conversions::open3dToRos(*p_global_raw_pc, raw_pc2);
            raw_pc2.header.frame_id = "map";
            raw_pc2.header.stamp = ros::Time::now();
            global_map_raw_pub_.publish(raw_pc2);

            sensor_msgs::PointCloud2 gnd_patch_pc2;
            auto p_gnd_patch_pc = terrain_mapper_.getGroundPatchPoints();
            open3d_conversions::open3dToRos(*p_gnd_patch_pc, gnd_patch_pc2);
            gnd_patch_pc2.header.frame_id = "map";
            gnd_patch_pc2.header.stamp = ros::Time::now();
            gnd_patch_pub_.publish(gnd_patch_pc2);
        }
    }

    void Terrain3dMapperNode::publishCorrectedTf(const ros::Time &stamp, const Eigen::Matrix4d &original_robot_tf)
    {
        geometry_msgs::TransformStamped corrected_tf;
        geometry_msgs::PoseStamped corrected_pose;

        //transformStamped.header.seq = msg->header.seq;
        auto corrector = terrain_mapper_.getCorrectTf();
        Eigen::Affine3d tf_corrected(corrector * original_robot_tf);
        corrected_tf = tf2::eigenToTransform(tf_corrected);

        corrected_tf.header.stamp = stamp;
        corrected_tf.header.frame_id = "map";
        corrected_tf.child_frame_id = "base_link";

        corrected_pose.header.stamp = stamp;
        corrected_pose.header.frame_id = "map";
        corrected_pose.pose.position.x = corrected_tf.transform.translation.x;
        corrected_pose.pose.position.y = corrected_tf.transform.translation.y;
        corrected_pose.pose.position.z = corrected_tf.transform.translation.z;
        corrected_pose.pose.orientation = corrected_tf.transform.rotation;

        corrected_odom_pub_.publish(corrected_pose);
        br_.sendTransform(corrected_tf);
    }

}