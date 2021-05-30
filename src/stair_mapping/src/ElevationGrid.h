#pragma once

#include "PointType.h"

namespace stair_mapping
{

class ElevationGrid
{
public:
    ElevationGrid(
        double width,
        double length,
        double grid_size,
        double default_height);

    void update(const PointCloudT::Ptr &p_input_cloud);

    void getPclFromHeightMap(PointCloudT::Ptr &p_output_cloud);

private:
    double map_width_;
    double map_length_;
    double grid_size_;
    double default_height_;
    int map_raws_;
    int map_cols_;

    Eigen::MatrixXd elevation_grid_;

    void generateGrid(const PointCloudT::Ptr &p_input_cloud);
    void fillUnknowCell();
    void fillGap(int row, int start_col, int end_col, double height);
};

} // namespace stair_mapping
