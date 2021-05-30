#pragma once

#include <ceres/ceres.h>
#include "PoseErrorTerm.h"

namespace stair_mapping
{

enum EDGE_TYPE
{
    TRANSFORM = 0,
    TRANSLATION = 1,
    ROTATION = 2,
    ABS_ROTATION = 3
};

// The constraint between two vertices in the pose graph. The constraint is the
// transformation from vertex id_begin to vertex id_end.
struct Edge3d
{
    EDGE_TYPE type;
    int id_begin;
    int id_end;

    // The transformation that represents the pose of the end frame E w.r.t. the
    // begin frame B. In other words, it transforms a vector in the E frame to
    // the B frame.
    Pose3d t_be;

    // The inverse of the covariance matrix for the measurement. The order of the
    // entries are x, y, z, delta orientation.
    InfoMatrix information;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


class PoseGraph
{
public:
    PoseGraph();

    virtual ~PoseGraph();

    void reset();

    void addVertex(const Vertex3d init_guess);

    void addEdge(
        EDGE_TYPE type,
        int id_begin, int id_end, 
        const Pose3d t_edge, 
        const InfoMatrix info_mat = InfoMatrix::Identity());

    bool solve();

    const std::vector<Vertex3d>* const getVertices();

private:
    std::vector<Vertex3d> vertex_list_;
    std::vector<Edge3d> edge_list_;
};

} // namespace stair_mapping
