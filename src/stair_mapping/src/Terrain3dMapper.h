#pragma once

#include "PointType.h"
#include "GlobalMap.h"

namespace stair_mapping
{
    class Terrain3dMapper
    {
    public:
        Terrain3dMapper();

        void preprocess(const PtCldPtr &p_in_cloud, PtCldPtr &p_out_cloud);

        void matchSubmap(
            const PtCldPtr &p_in_cloud,
            PtCldPtr &p_out_cloud,
            const Eigen::Matrix4d &t_frame_odom,
            const Eigen::Matrix<double, 4, 6> &tip_states);

        void buildGlobalMap(bool display_raw_result);

        Eigen::Matrix4d getLastSubMapRawTf();
        Eigen::Matrix4d getLastSubMapOptTf();
        Eigen::Matrix4d getCorrectTf();

        const PtCldPtr getGlobalMapRawPoints();
        const PtCldPtr getGlobalMapOptPoints();
        const PtCldPtr getGroundPatchPoints();

    private:
        GlobalMap global_map_;
        PtCldPtr global_raw_points_;
        PtCldPtr global_opt_points_;
        PtCldPtr ground_patch_points_;
        Eigen::Matrix4d correct_tf_;

    };

} // namespace stair_mapping
