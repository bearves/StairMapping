#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZ PointT;
typedef pcl::Normal PointN;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;
typedef pcl::PointCloud<pcl::Normal> PointCloudN;
typedef pcl::PointXYZINormal PointTN;
typedef pcl::PointCloud<PointTN> PointCloudTN;