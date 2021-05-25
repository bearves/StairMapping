#include "PoseGraph.h"

namespace stair_mapping
{
    PoseGraph::PoseGraph()
    {
        vertex_list_.clear();
        edge_list_.clear();
    }

    PoseGraph::~PoseGraph()
    {
    }

    void PoseGraph::reset()
    {
        vertex_list_.clear();
        edge_list_.clear();
    }

    void PoseGraph::addVertex(const Vertex3d init_guess)
    {
        vertex_list_.push_back(init_guess);
    }

    void PoseGraph::addEdge(
        int id_begin, int id_end,
        const Pose3d t_edge,
        const InfoMatrix info_mat)
    {
        Edge3d edge;
        edge.id_begin = id_begin;
        edge.id_end = id_end;
        edge.t_be = t_edge;
        edge.information = info_mat;
        edge_list_.push_back(edge);
    }

    bool PoseGraph::solve()
    {
        if (vertex_list_.empty() ||
            edge_list_.empty())
        {
            std::cout << "No constraints, no problem to optimize." << std::endl;
            return false;
        }

        ceres::Problem problem;
        ceres::LossFunction *loss_function = nullptr;
        ceres::LocalParameterization *local_parameterization =
            new ceres::EigenQuaternionParameterization();

        for (const auto &edge : edge_list_)
        {
            if (edge.id_begin >= vertex_list_.size() ||
                edge.id_end >= vertex_list_.size() ||
                edge.id_begin < 0 ||
                edge.id_end < 0)
            {
                std::cout << "Invalid constraints, vtx index out of range." << std::endl;
                continue;
            }

            auto &pose_begin = vertex_list_[edge.id_begin];
            auto &pose_end = vertex_list_[edge.id_end];

            const Eigen::Matrix<double, 6, 6> sqrt_information =
                edge.information.llt().matrixL();

            // Ceres will take ownership of the pointer.
            ceres::CostFunction *cost_function =
                PoseGraph3dErrorTerm::Create(edge.t_be, sqrt_information);

            problem.AddResidualBlock(cost_function,
                                     loss_function,
                                     pose_begin.t_vertex.p.data(),
                                     pose_begin.t_vertex.q.coeffs().data(),
                                     pose_end.t_vertex.p.data(),
                                     pose_end.t_vertex.q.coeffs().data());

            problem.SetParameterization(pose_begin.t_vertex.q.coeffs().data(),
                                        local_parameterization);
            problem.SetParameterization(pose_end.t_vertex.q.coeffs().data(),
                                        local_parameterization);
        }

        // The pose graph optimization problem has six DOFs that are not fully
        // constrained. This is typically referred to as gauge freedom. You can apply
        // a rigid body transformation to all the nodes and the optimization problem
        // will still have the exact same cost. The Levenberg-Marquardt algorithm has
        // internal damping which mitigates this issue, but it is better to properly
        // constrain the gauge freedom. This can be done by setting one of the poses
        // as constant so the optimizer cannot change it.
        auto &pose_start = vertex_list_[0];
        problem.SetParameterBlockConstant(pose_start.t_vertex.p.data());
        problem.SetParameterBlockConstant(pose_start.t_vertex.q.coeffs().data());

        ceres::Solver::Options options;
        ceres::Solver::Summary summary; // summary of the optimiztion result
        options.max_num_iterations = 200;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

        ceres::Solve(options, &problem, &summary);
        std::cout << summary.FullReport() << '\n';
        return summary.IsSolutionUsable();
    }

    const std::vector<Vertex3d> *const PoseGraph::getVertices()
    {
        return &vertex_list_;
    }

} // namespace stair_mapping
