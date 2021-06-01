#include "Terrain3dMapperNode.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <termios.h>
#include "PreProcessor.h"

namespace stair_mapping
{
    Terrain3dMapperNode::Terrain3dMapperNode(ros::NodeHandle& node)
    {
        preprocess_pub_ = node.advertise<sensor_msgs::PointCloud2>("preprocessed_points", 1);
        submap_pub_ = node.advertise<sensor_msgs::PointCloud2>("submap_points", 1);
        global_map_opt_pub_ = node.advertise<sensor_msgs::PointCloud2>("global_map_opt_points", 1);
        global_map_raw_pub_ = node.advertise<sensor_msgs::PointCloud2>("global_map_raw_points", 1);
        corrected_odom_pub_ = node.advertise<geometry_msgs::PoseStamped>("corrected_robot_pose", 1);
        odom_sub_ = node.subscribe("/qz_state_publisher/robot_odom", 1, &Terrain3dMapperNode::odomMsgCallback, this);
        pcl_sub_ = node.subscribe("transformed_points", 1, &Terrain3dMapperNode::pclMsgCallback, this);
    }

    void Terrain3dMapperNode::pclMsgCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        using namespace Eigen;
        PointCloudT::Ptr p_in_cloud(new PointCloudT);
        PointCloudT::Ptr p_pre_cloud(new PointCloudT);

        ROS_INFO("Realsense msg received");

        sensor_msgs::PointCloud2 pre_out_cloud2;
        sensor_msgs::PointCloud2 submap_out_cloud2;

        pcl::fromROSMsg(*msg, *p_in_cloud);

        terrain_mapper_.preprocess(p_in_cloud, p_pre_cloud);

        pcl::toROSMsg(*p_pre_cloud, pre_out_cloud2);
        pre_out_cloud2.header.frame_id = "base_world";
        pre_out_cloud2.header.stamp = msg->header.stamp;
        preprocess_pub_.publish(pre_out_cloud2);

        PointCloudT::Ptr p_submap_cloud(new PointCloudT);

        odom_msg_mtx_.lock();
        Matrix4d t_frame_odom = current_odom_mat_;
        odom_msg_mtx_.unlock();
        terrain_mapper_.matchSubmap(p_pre_cloud, p_submap_cloud, t_frame_odom);

        pcl::toROSMsg(*p_submap_cloud, submap_out_cloud2);
        submap_out_cloud2.header.frame_id = "base_world";
        submap_out_cloud2.header.stamp = msg->header.stamp;
        submap_pub_.publish(submap_out_cloud2);

    }

    void Terrain3dMapperNode::odomMsgCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        Eigen::Matrix4d pose_mat = getPoseMatrix(*msg);
        odom_msg_mtx_.lock();
        current_odom_mat_ = pose_mat;
        odom_msg_mtx_.unlock();

        publishCorrectedTf(msg->header.stamp, pose_mat);
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


    void Terrain3dMapperNode::startMapServer()
    {
        th_ = std::thread([this](){
            ROS_INFO("Map server started");
            ros::Rate map_publish_rate(5);
            while(ros::ok())
            {
                terrain_mapper_.buildGlobalMap();
                publishMap();
                map_publish_rate.sleep();
            }
        });
    }

    void Terrain3dMapperNode::publishMap()
    {
        sensor_msgs::PointCloud2 opt_pc2;
        auto p_global_opt_pc = terrain_mapper_.getGlobalMapOptPoints();
        pcl::toROSMsg(*p_global_opt_pc, opt_pc2);
        opt_pc2.header.frame_id = "map";
        opt_pc2.header.stamp = ros::Time::now();
        global_map_opt_pub_.publish(opt_pc2);

        sensor_msgs::PointCloud2 raw_pc2;
        auto p_global_raw_pc = terrain_mapper_.getGlobalMapRawPoints();
        pcl::toROSMsg(*p_global_raw_pc, raw_pc2);
        raw_pc2.header.frame_id = "map";
        raw_pc2.header.stamp = ros::Time::now();
        global_map_raw_pub_.publish(raw_pc2);
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
        corrected_tf.child_frame_id = "base_world";

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