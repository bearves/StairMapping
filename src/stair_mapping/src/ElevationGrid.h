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

    void update(const PtCldPtr &p_input_cloud);

    void getPclFromHeightMap(PtCldPtr &p_output_cloud);

private:
    double map_width_;
    double map_length_;
    double grid_size_;
    double default_height_;
    int map_raws_;
    int map_cols_;

    Eigen::MatrixXd elevation_grid_;

    inline bool isUnknown(int row, int col)
    {
        return elevation_grid_(row, col) < (default_height_ + 0.01);
    }

    void generateGrid(const PtCldPtr &p_input_cloud);
    void fillUnknowCell();
    void smoothGrid();
    void fillGap(int row, int start_col, int end_col, double height);
    void applySmoothFilter(int row, int col);
};

} // namespace stair_mapping
