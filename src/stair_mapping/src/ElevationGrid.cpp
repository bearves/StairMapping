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

    void ElevationGrid::update(const PtCldPtr &p_input_cloud)
    {
        generateGrid(p_input_cloud);
        fillUnknowCell();
        smoothGrid();
    }

    void ElevationGrid::generateGrid(const PtCldPtr &p_input_cloud)
    {
        elevation_grid_.setConstant(default_height_);
        for (auto &point : p_input_cloud->points_)
        {
            int col = point.x() / grid_size_ + map_cols_ / 2;
            int row = point.y() / grid_size_ + map_raws_ / 2;
            // the point outside height map range
            if (col < 0 || col >= map_cols_ || row < 0 || row >= map_raws_)
                continue;

            if (point.z() > elevation_grid_(row, col))
                elevation_grid_(row, col) = point.z();
        }

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
                Eigen::Vector3d point;
                point.x() = (j - map_cols_/ 2) * grid_size_;
                point.y() = (i - map_raws_ / 2) * grid_size_;
                point.z() = elevation_grid_(i, j);
                // skip known cell
                if (!isUnknown(i, j))
                {
                    last_known_col = j;
                    last_known_height = point.z();
                    continue;
                }
                // search next known cell
                int next_known_col = -1;
                double next_known_height = default_height_;

                for(int k = j+1; k < elevation_grid_.cols(); k++)
                {
                    if (!isUnknown(i, k))
                    {
                        next_known_col = k;
                        next_known_height = elevation_grid_(i, k);
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

    void ElevationGrid::smoothGrid()
    {
        for (int i = 0; i < elevation_grid_.rows(); i++)
        {
            for (int j = 0; j < elevation_grid_.cols(); j++)
            {
                applySmoothFilter(i, j);
            }
        }
    }

    void ElevationGrid::applySmoothFilter(int row, int col)
    {
        // skip unknown grid
        if (isUnknown(row, col))
            return;
        
        double center_height = elevation_grid_(row, col);
        double sum = 0;
        int valid_cell_count = 0;
        int radius = 1;
        // scan the cells inside the radius
        // if the neighburhood cell have a close value to 
        // the center cell, take the cell into account
        // and finally normalize all the accounted cells
        for (int u = row - radius; u < row + radius; u++)
        {
            for (int v = col - radius; v < col + radius; v++)
            {
                // skip out-ranged and unknown cells
                if (v < 0 || v >= map_cols_ ||
                    u < 0 || u >= map_raws_ ||
                    isUnknown(u, v))
                {
                    continue;
                }

                double cell_height = elevation_grid_(u, v);
                if (fabs(cell_height - center_height) < 0.04)
                {
                    sum += cell_height;
                    valid_cell_count ++;
                }
            }
        }
        if (valid_cell_count != 0)
            elevation_grid_(row, col) = sum / valid_cell_count;
    }


    void ElevationGrid::getPclFromHeightMap(PtCldPtr &p_output_cloud)
    {
        p_output_cloud->Clear();
        for (int i = 0; i < elevation_grid_.rows(); i++)
        {
            for (int j = 0; j < elevation_grid_.cols(); j++)
            {
                Eigen::Vector3d point;
                point.x() = (j - map_cols_/ 2) * grid_size_;
                point.y() = (i - map_raws_ / 2) * grid_size_;
                point.z() = elevation_grid_(i, j);
                if (isUnknown(i, j))
                    continue;
                p_output_cloud->points_.push_back(point);
            }
        }
    }
} // namespace stair_mapping
