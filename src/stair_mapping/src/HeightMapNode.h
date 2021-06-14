#pragma once

#include <ros/ros.h>
#include <mutex>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>
#include "ElevationGrid.h"

namespace stair_mapping
{
    class HeightMapNode
    {
    public:
        HeightMapNode(ros::NodeHandle &nh);
        void publishGroundTruth();

    private:
        std::mutex mtx_;

        ros::Subscriber global_pcl_sub_;
        ros::Subscriber pose_sub_;
        ros::Publisher height_pcl_pub_;
        ros::Publisher ground_truth_pcl_pub_;
        bool is_ground_truth_needed_{false};

        ElevationGrid eg_;
        PtCldPtr global_map_;

        std::thread th_;

        // only for test
        PtCldPtr ground_truth_;
        sensor_msgs::PointCloud2 ground_truth_pc2_;

        void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
        void pclDataCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
        void generateGroundTruth();
    };

} // namespace stair_mapping
