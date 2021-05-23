#pragma once

#include "PointType.h"

class HeightMap
{
public:
    static constexpr double MAP_WIDTH = 0.8;
    static constexpr double MAP_LENGTH = 1.6;
    static constexpr double MAP_X0 = 0.7;
    static constexpr double GRID_SIZE = 0.02;
    static constexpr double INVALID_VALUE = -100;
    static constexpr int MAP_ROWS = MAP_WIDTH / GRID_SIZE;
    static constexpr int MAP_COLS = MAP_LENGTH / GRID_SIZE;
    static void generateHeightMap(const PointCloudT::Ptr &p_input_cloud, Eigen::MatrixXd& out_height_map);
    static void generateCostMap(const Eigen::MatrixXd& input_height_map, Eigen::MatrixXd& out_cost_map);
    static void fillSmallHoles(Eigen::MatrixXd& height_map);
    static void getPclFromHeightMap(const Eigen::MatrixXd& input_height_map, PointCloudT::Ptr &p_output_cloud);
    static double countNearbyValidValues(
        const Eigen::MatrixXd& height_map, int row, int col, 
        int neighbur_rows, int neighbur_cols, 
        double& valid_value_mean, double& valid_value_sd);
};