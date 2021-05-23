#include "HeightMap.h"

void HeightMap::generateHeightMap(const PointCloudT::Ptr &p_input_cloud, Eigen::MatrixXd &out_height_map)
{
    out_height_map.resize(MAP_ROWS, MAP_COLS);
    double init_value = HeightMap::INVALID_VALUE;
    out_height_map.setConstant(init_value);
    for(auto& point : p_input_cloud->points)
    {
        int col = (point.x - MAP_X0)/GRID_SIZE;
        int row = point.y/GRID_SIZE + MAP_ROWS / 2; 
        // the point outside height map range
        if (col < 0 || col >= MAP_COLS ||row < 0 || row >= MAP_ROWS)
            continue;

        if (point.z > out_height_map(row, col)) 
            out_height_map(row, col) = point.z;
    }
}

void HeightMap::fillSmallHoles(Eigen::MatrixXd &height_map)
{
    for (int i = 0; i < height_map.rows(); i++)
    {
        for (int j = 0; j < height_map.cols(); j++)
        {
            double z = height_map(i, j);
            if (z < HeightMap::INVALID_VALUE+0.01)
            {
                // stat a neighburhood
                double valid_value_mean = HeightMap::INVALID_VALUE;
                double valid_value_sd = 0;
                double valid_value_rate = countNearbyValidValues(
                    height_map, i, j, 4, 2, 
                    valid_value_mean, valid_value_sd);

                // if there are 50% valid values,
                if (valid_value_rate >= 0.5)
                {
                    // use the mean value of valid values to fill this hole
                    height_map(i, j) = valid_value_mean;
                }
            }
        }
    }
}

void HeightMap::generateCostMap(const Eigen::MatrixXd& input_height_map, Eigen::MatrixXd& out_cost_map)
{
    out_cost_map.resize(MAP_ROWS, MAP_COLS);
    for (int i = 0; i < input_height_map.rows(); i++)
    {
        for (int j = 0; j < input_height_map.cols(); j++)
        {
            double z = input_height_map(i, j);
            if (z < HeightMap::INVALID_VALUE+0.01)
            {
                out_cost_map(i, j) = HeightMap::INVALID_VALUE;
                continue;
            }
            double mean, sd;
            double valid_value_rate = countNearbyValidValues(
                input_height_map, i, j, 2, 2,
                mean, sd);
            out_cost_map(i, j) = sd;
        }
    }
    double max_sd = out_cost_map.maxCoeff();
    std::cout << "MAX SD: " << max_sd << std::endl;
}

double HeightMap::countNearbyValidValues(
    const Eigen::MatrixXd &height_map, int row, int col, 
    int neighbur_rows, int neighbur_cols, 
    double &valid_value_mean, double &valid_value_sd)
{
    int valid_count = 0;
    int total_count = 0;
    double sum = 0;
    std::vector<double> valid_values;
    for(int i = row - neighbur_rows; i <= row + neighbur_rows; i++)
    {
        for (int j = col - neighbur_cols; j <= col + neighbur_cols; j++)
        {
            if (i < 0 || i >= MAP_ROWS || j < 0 || j >= MAP_COLS)
                continue;
            total_count ++;
            double z = height_map(i, j);
            if (z >= HeightMap::INVALID_VALUE+0.01)
            {
                valid_count++;
                sum += z;
                valid_values.push_back(z);
            }
        }
    }
    valid_value_mean = sum / valid_count;
    double sum_sd = 0;
    for (auto v : valid_values)
    {
        sum_sd += sqrt((v - valid_value_mean) * (v - valid_value_mean));
    }
    valid_value_sd = sum_sd / valid_count;
    return valid_count * 1.0 / total_count;
}

void HeightMap::getPclFromHeightMap(const Eigen::MatrixXd &input_height_map, PointCloudT::Ptr &p_output_cloud)
{
    p_output_cloud->clear();
    for (int i = 0; i < input_height_map.rows(); i++)
    {
        for (int j = 0; j < input_height_map.cols(); j++)
        {
            PointT point;
            point.x = j * GRID_SIZE + MAP_X0;
            point.y = (i - MAP_ROWS/2) * GRID_SIZE;
            point.z = input_height_map(i, j);
            if (point.z < -9)
                point.z = -1;
            p_output_cloud->push_back(point);
        }
    }
}