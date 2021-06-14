#pragma once

#include <open3d/Open3D.h>

namespace stair_mapping
{
    typedef open3d::geometry::PointCloud PtCld;
    typedef std::shared_ptr<open3d::geometry::PointCloud> PtCldPtr;
    typedef open3d::t::geometry::PointCloud TsrPtCld;
    typedef std::shared_ptr<open3d::t::geometry::PointCloud> TsrPtCldPtr;
}
