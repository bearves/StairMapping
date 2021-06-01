#include "HeightMapNode.h"
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace stair_mapping
{
    HeightMapNode::HeightMapNode(ros::NodeHandle& nh)
    : eg_(0.9, 2.4, 0.03, -10),
      global_map_(new PointCloudT)
    {
        height_pcl_pub_ = nh.advertise<sensor_msgs::PointCloud2>("global_height_map", 1);
        global_pcl_sub_ = nh.subscribe("global_map_opt_points", 1, &HeightMapNode::pclDataCallback, this);
        pose_sub_ = nh.subscribe("corrected_robot_pose", 10, &HeightMapNode::poseCallback, this);
    }

    void HeightMapNode::poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        using namespace Eigen;
        // get robot pose
        auto pos = msg->pose.position;
        auto ori = msg->pose.orientation;
        Translation3d t(pos.x, pos.y, pos.z);
        Quaterniond q(ori.w, ori.x, ori.y, ori.z);
        Affine3d pose = t * q;
        // transform global 3d map points to the body cs
        PointCloudT::Ptr local_map(new PointCloudT);
        PointCloudT::Ptr height_map(new PointCloudT);
        mtx_.lock();
        pcl::transformPointCloud(*global_map_, *local_map, pose.inverse().matrix());
        mtx_.unlock();
        // generate height map
        eg_.update(local_map);
        // publish height map
        eg_.getPclFromHeightMap(height_map);

        sensor_msgs::PointCloud2 height_map_2;
        pcl::toROSMsg(*height_map, height_map_2);
        height_map_2.header.frame_id = "base_world";
        height_map_2.header.stamp = msg->header.stamp;

        height_pcl_pub_.publish(height_map_2);
    }

    void HeightMapNode::pclDataCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        // store last 3d map points locally
        mtx_.lock();
        pcl::fromROSMsg(*msg, *global_map_);
        mtx_.unlock();
    }

}