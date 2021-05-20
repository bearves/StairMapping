#include "PclProcessor.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <termios.h>
#include <PointType.h>
#include <PreProcessor.h>
#include <HeightMap.h>

namespace stair_mapping
{
    PclProcessor::PclProcessor(ros::NodeHandle& node)
    {
        preprocess_pub_ = node.advertise<sensor_msgs::PointCloud2>("preprocessed_points", 1);
        submap_pub_ = node.advertise<sensor_msgs::PointCloud2>("submap_points", 1);
        //height_map_pub_ = node.advertise<sensor_msgs::PointCloud2>("height_map_pcl", 1);
        //cost_map_pub_ = node.advertise<nav_msgs::OccupancyGrid>("terrain_cost_map", 1);
        pcl_sub_ = node.subscribe("transformed_points", 1, &PclProcessor::pclMsgCallback, this);
    }

    void PclProcessor::pclMsgCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        using namespace Eigen;
        PointCloudT::Ptr p_in_cloud(new PointCloudT);
        PointCloudT::Ptr p_pre_cloud(new PointCloudT);

        ROS_INFO("Realsence msg received");

        sensor_msgs::PointCloud2 pre_out_cloud2;
        sensor_msgs::PointCloud2 submap_out_cloud2;

        pcl::fromROSMsg(*msg, *p_in_cloud);

        preProcess(p_in_cloud, p_pre_cloud);

        pcl::toROSMsg(*p_pre_cloud, pre_out_cloud2);
        pre_out_cloud2.header.frame_id = "base_world";
        pre_out_cloud2.header.stamp = ros::Time::now();
        preprocess_pub_.publish(pre_out_cloud2);

        PointCloudT::Ptr p_submap_cloud(new PointCloudT);
        submapMatch(p_pre_cloud, p_submap_cloud);

        pcl::toROSMsg(*p_submap_cloud, submap_out_cloud2);
        submap_out_cloud2.header.frame_id = "base_world";
        submap_out_cloud2.header.stamp = ros::Time::now();
        submap_pub_.publish(submap_out_cloud2);

        ros::Duration d(0.5);
        d.sleep();
    }

    void PclProcessor::preProcess(const PointCloudT::Ptr &p_in_cloud, PointCloudT::Ptr &p_out_cloud)
    {
        // crop
        PointCloudT::Ptr p_cloud_cr(new PointCloudT);
        PreProcessor::crop(p_in_cloud, p_cloud_cr, Eigen::Vector3f(0, -0.5, -2), Eigen::Vector3f(2.3, 0.5, 3));

        // downsampling
        PointCloudT::Ptr p_cloud_ds(new PointCloudT);
        Eigen::Vector2i sizes = PreProcessor::downSample(p_cloud_cr, p_cloud_ds, 0.01);
        ROS_INFO("After downsample size: %d -> %d", sizes[0], sizes[1]);

        p_out_cloud = p_cloud_ds;
    }

    void PclProcessor::submapMatch(const PointCloudT::Ptr &p_in_cloud, PointCloudT::Ptr &p_out_cloud)
    {
        using namespace Eigen;
        // FrontEnd
        // scan-to-submap matcher
        // if no submap exists
        if (global_map_.submapCount() == 0)
        {
            // init new submap with current pcl, set its transform to I
            ROS_INFO("Init new global map");
            SubMap::Ptr p_sm(new SubMap);
            p_sm->init();
            global_map_.addNewSubmap(p_sm, Matrix4d::Identity());
        }

        SubMap::Ptr last_sm = global_map_.getLastMap();
        SubMap::Ptr current_sm;

        if (last_sm == nullptr) return;

        // match current frame to the last submap
        double score = 1e8;
        double SUCCESS_SCORE = 10;
        Matrix4d t_frame_to_last_map = Matrix4d::Identity();

        try
        {
            score = last_sm->match(*p_in_cloud, t_frame_to_last_map);
        }
        catch (std::runtime_error &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }

        // if match succeeds, add current frame to current submap or create new submap, otherwise drop it
        if (score < SUCCESS_SCORE)
        {
            // if a new submap is needed
            if (last_sm->hasEnoughFrame())
            {
                // init new submap with current pcl
                ROS_INFO("Create new submap");

                SubMap::Ptr p_sm(new SubMap);
                p_sm->init();
                p_sm->addFrame(*p_in_cloud, Matrix4d::Identity());
                // match to last submap if exists, record transform Ti
                global_map_.addNewSubmap(p_sm, t_frame_to_last_map);
                current_sm = p_sm;
            }
            else
            {
                // add current pcl to current submap
                last_sm->addFrame(*p_in_cloud, t_frame_to_last_map);
                current_sm = last_sm;
            }
        }

        if (current_sm == nullptr) return;

        p_out_cloud = current_sm->getSubmapPoints();
    }


    void PclProcessor::doProcess(const PointCloudT::Ptr &p_in_cloud, PointCloudT::Ptr &p_out_cloud, 
                       Eigen::MatrixXd& height_map, Eigen::MatrixXd& cost_map)
    {
        // crop
        PointCloudT::Ptr p_cloud_cr(new PointCloudT);
        PreProcessor::crop(p_in_cloud, p_cloud_cr, Eigen::Vector3f(0, -0.5, -0.8), Eigen::Vector3f(2.3, 0.5, 0.2));

        // downsampling
        PointCloudT::Ptr p_cloud_ds(new PointCloudT);
        Eigen::Vector2i sizes = PreProcessor::downSample(p_cloud_cr, p_cloud_ds, 0.01);
        ROS_INFO("After downsample size: %d -> %d", sizes[0], sizes[1]);

        // convert to height map
        PointCloudT::Ptr height_filled_pcl(new PointCloudT);
        PointCloudT::Ptr cost_pcl(new PointCloudT);

        HeightMap::generateHeightMap(p_cloud_ds, height_map);
        HeightMap::fillSmallHoles(height_map);
        HeightMap::generateCostMap(height_map, cost_map);

        HeightMap::getPclFromHeightMap(height_map, height_filled_pcl);
        HeightMap::getPclFromHeightMap(cost_map, cost_pcl);

        p_out_cloud = height_filled_pcl;
    }

    void PclProcessor::generateCostGrid(Eigen::MatrixXd &cost_map, nav_msgs::OccupancyGrid &cost_grid)
    {
        cost_grid.header.frame_id = "base_world";
        cost_grid.header.stamp = ros::Time::now();
        cost_grid.info.height = HeightMap::MAP_ROWS;
        cost_grid.info.width = HeightMap::MAP_COLS;
        cost_grid.info.resolution = HeightMap::GRID_SIZE;
        cost_grid.info.origin.orientation.x = 0;
        cost_grid.info.origin.orientation.y = 0;
        cost_grid.info.origin.orientation.z = 0;
        cost_grid.info.origin.orientation.w = 1;
        cost_grid.info.origin.position.x = HeightMap::MAP_X0;
        cost_grid.info.origin.position.y = -HeightMap::MAP_WIDTH/2;
        cost_grid.info.origin.position.z = -2;
        cost_grid.data.resize(HeightMap::MAP_COLS * HeightMap::MAP_ROWS);

        double cost_threshold = 0.1;

        for (int i = 0; i < HeightMap::MAP_ROWS; i++)
        {
            for (int j = 0; j < HeightMap::MAP_COLS; j++)
            {
                if (cost_map(i, j) < HeightMap::INVALID_VALUE+0.1)
                {
                    cost_grid.data[i * HeightMap::MAP_COLS + j] = -1;
                }
                else if (cost_map(i, j) < cost_threshold)
                {
                    cost_grid.data[i * HeightMap::MAP_COLS + j] = cost_map(i,j)/cost_threshold * 100;
                }
                else
                {
                    cost_grid.data[i * HeightMap::MAP_COLS + j] = 100;
                }
            }
        }
    }
}