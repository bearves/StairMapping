#include "ElevationGrid.h"

namespace stair_mapping
{
    ElevationGrid::ElevationGrid(
        double width,
        double length,
        double grid_size,
        double default_height)
    {
        map_width_ = width;
        map_length_ = length;
        grid_size_ = grid_size;
        default_height_ = default_height;
        map_raws_ = width / grid_size;
        map_cols_ = length / grid_size;

        elevation_grid_.resize(map_raws_, map_cols_);
    }

    void ElevationGrid::update(const PointCloudT::Ptr &p_input_cloud)
    {
        generateGrid(p_input_cloud);
        fillUnknowCell();
        //smoothGrid();
    }

    void ElevationGrid::fillUnknowCell()
    {
        for (int i = 0; i < elevation_grid_.rows(); i++)
        {
            int last_known_col = -1;
            double last_known_height = default_height_;
            // scan along the x axis
            for (int j = 0; j < elevation_grid_.cols(); j++)
            {
                PointT point;
                point.x = (j - map_cols_/ 2) * grid_size_;
                point.y = (i - map_raws_ / 2) * grid_size_;
                point.z = elevation_grid_(i, j);
                // skip known cell
                if (point.z > default_height_ + 0.01)
                {
                    last_known_col = j;
                    last_known_height = point.z;
                    continue;
                }
                // search next known cell
                int next_known_col = -1;
                double next_known_height = default_height_;

                for(int k = j+1; k < elevation_grid_.cols(); k++)
                {
                    PointT next;
                    next.x = (k - map_cols_ / 2) * grid_size_;
                    next.y = point.y;
                    next.z = elevation_grid_(i, k);
                    if (next.z > default_height_ + 0.01)
                    {
                        next_known_col = k;
                        next_known_height = next.z;
                        break;
                    }
                }
                // cannot find next knowncell along this x axis
                if (next_known_col == -1)
                {
                    // use last known cell's height to fill j to the end this line
                    if (last_known_col != -1)
                    {
                        fillGap(i, j, elevation_grid_.cols(), last_known_height);
                    }
                    else
                    {
                        // the total line is unknown, skip this line
                        break;
                    }
                }
                else
                {
                    // found the next known cell, fill j to next known cell with the smaller height
                    fillGap(i, j, next_known_col, std::fmin(last_known_height,next_known_height));
                }
            }
        }
    }

    void ElevationGrid::fillGap(int row, int start_col, int end_col, double height)
    {
        for(int j = start_col; j < end_col; j++)
        {
            elevation_grid_(row, j) = height;
        }
    }

    void ElevationGrid::generateGrid(const PointCloudT::Ptr &p_input_cloud)
    {
        elevation_grid_.setConstant(default_height_);
        for (auto &point : p_input_cloud->points)
        {
            int col = point.x / grid_size_ + map_cols_ / 2;
            int row = point.y / grid_size_ + map_raws_ / 2;
            // the point outside height map range
            if (col < 0 || col >= map_cols_ || row < 0 || row >= map_raws_)
                continue;

            if (point.z > elevation_grid_(row, col))
                elevation_grid_(row, col) = point.z;
        }

    }

    void ElevationGrid::getPclFromHeightMap(PointCloudT::Ptr &p_output_cloud)
    {
        p_output_cloud->clear();
        for (int i = 0; i < elevation_grid_.rows(); i++)
        {
            for (int j = 0; j < elevation_grid_.cols(); j++)
            {
                PointT point;
                point.x = (j - map_cols_/ 2) * grid_size_;
                point.y = (i - map_raws_ / 2) * grid_size_;
                point.z = elevation_grid_(i, j);
                if (point.z < -9)
                    continue;
                p_output_cloud->push_back(point);
            }
        }
    }
} // namespace stair_mapping
