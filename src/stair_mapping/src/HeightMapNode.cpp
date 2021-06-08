#include "HeightMapNode.h"
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace stair_mapping
{
    HeightMapNode::HeightMapNode(ros::NodeHandle& nh)
    : eg_(0.9, 2.4, 0.03, -10),
      global_map_(new PointCloudT),
      ground_truth_(new PointCloudT)
    {
        nh.param("need_ground_truth", is_ground_truth_needed_, false);

        height_pcl_pub_ = nh.advertise<sensor_msgs::PointCloud2>("global_height_map", 1);
        if (is_ground_truth_needed_)
        {
            ground_truth_pcl_pub_ = nh.advertise<sensor_msgs::PointCloud2>("ground_truth", 1);
        }
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

    void HeightMapNode::generateGroundTruth()
    {
        double stair_height = 0.131;
        double stair_length = 0.25;
        double stair_width = 1.2;
        double stairx0 = 0.71;
        double offsetz = -0.36;
        int stair_cnt = 9;

        ground_truth_->clear();


        double resolution = 0.01;
        for (int i = -150; i < 300; i++)
        {
            for (int j = -60; j < 61; j++)
            {
                PointT pt;

                pt.x = stairx0 + i * resolution;
                pt.y = j * resolution;

                double d = pt.x - stairx0;
                if (pt.x < stairx0-0.001)
                {
                    // ground
                    pt.z = 0;
                }
                else if (d / stair_length > stair_cnt)
                {
                    // up ground
                    pt.z = stair_cnt * stair_height;
                }
                else
                {
                    // stair steps
                    pt.z = ceil((d+0.005) / stair_length) * stair_height;
                    
                    if (i % (int)round(stair_length/resolution) == 0)
                    {
                        // build vertical plane
                        for (int k = 0; k < stair_height/resolution - 0.0011; k++)
                        {
                            PointT vpt;
                            vpt.x = pt.x; 
                            vpt.y = pt.y;
                            vpt.z = pt.z - (k+1) * resolution;
                            vpt.z += offsetz;
                            ground_truth_->push_back(vpt);
                        }
                    }
                }
                pt.z += offsetz;
                ground_truth_->push_back(pt);
            }
        }
        pcl::toROSMsg(*ground_truth_, ground_truth_pc2_);
    }

    void HeightMapNode::publishGroundTruth()
    {
        if (!is_ground_truth_needed_)
            return;

        generateGroundTruth();

        th_ = std::thread([this](){
            ros::Rate grtruth_publish_rate(1);
            while(ros::ok())
            {
                ground_truth_pc2_.header.frame_id = "map";
                ground_truth_pc2_.header.stamp = ros::Time::now();
                ground_truth_pcl_pub_.publish(ground_truth_pc2_);
                grtruth_publish_rate.sleep();
            }
        });

    }

}