#pragma once

#include <ros/ros.h>
#include <mutex>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include "ElevationGrid.h"

namespace stair_mapping
{
    class HeightMapNode
    {
    public:
        HeightMapNode(ros::NodeHandle &nh);
        void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
        void pclDataCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

    private:
        std::mutex mtx_;

        ros::Subscriber global_pcl_sub_;
        ros::Subscriber pose_sub_;
        ros::Publisher height_pcl_pub_;

        ElevationGrid eg_;
        PointCloudT::Ptr global_map_;
    };

} // namespace stair_mapping
