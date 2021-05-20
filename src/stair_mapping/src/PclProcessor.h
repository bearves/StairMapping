#pragma once

#include <ros/ros.h>
#include <PointType.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <eigen3/Eigen/Dense>
#include "GlobalMap.h"

namespace stair_mapping 
{
    class PclProcessor
    {
    public:
        PclProcessor(ros::NodeHandle& node);
    private:
        ros::Subscriber pcl_sub_;
        ros::Publisher preprocess_pub_;
        ros::Publisher submap_pub_;
        ros::Publisher height_map_pub_;
        ros::Publisher cost_map_pub_;

        GlobalMap global_map_;

        void pclMsgCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

        void doProcess(const PointCloudT::Ptr &p_in_cloud, PointCloudT::Ptr &p_out_cloud, 
                       Eigen::MatrixXd& height_map, Eigen::MatrixXd& cost_map);
        void generateCostGrid(Eigen::MatrixXd& cost_map, nav_msgs::OccupancyGrid& cost_grid);

        void preProcess(const PointCloudT::Ptr &p_in_cloud, PointCloudT::Ptr &p_out_cloud);
        void submapMatch(const PointCloudT::Ptr &p_in_cloud, PointCloudT::Ptr &p_out_cloud);
    };
}