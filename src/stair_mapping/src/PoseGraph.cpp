#include "PoseGraph.h"

namespace stair_mapping
{
    PoseGraph::PoseGraph() : problem_(nullptr),
                             local_parameterization_(new ceres::EigenQuaternionParameterization())
    {
        vertex_list_.clear();
        edge_list_.clear();

        options_.max_num_iterations = 200;
        options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    }

    PoseGraph::~PoseGraph()
    {
        if (problem_ != nullptr)
        {
            delete problem_;
            problem_ = nullptr;
        }
        if (local_parameterization_ != nullptr)
        {
            delete local_parameterization_;
            local_parameterization_ = nullptr;
        }
    }

    void PoseGraph::addVertex()
    {
    }

    void PoseGraph::addEdge()
    {
    }

    void PoseGraph::buildProblem()
    {
        if (problem_ != nullptr)
            delete problem_;
        problem_ = new ceres::Problem();

        if (vertex_list_.empty() ||
            edge_list_.empty())
        {
            std::cout << "No constraints, no problem to optimize." << std::endl;
            return;
        }

        ceres::LossFunction *loss_function = NULL;

        for (const auto& edge : edge_list_)
        {
            if (edge.id_begin >= edge_list_.size() ||
                edge.id_end >= edge_list_.size() ||
                edge.id_begin < 0 ||
                edge.id_end < 0)
            {
                std::cout << "Invalid constraints, vtx index out of range." << std::endl;
                continue;
            }

            auto& pose_begin = vertex_list_[edge.id_begin];
            auto& pose_end = vertex_list_[edge.id_end];

            const Eigen::Matrix<double, 6, 6> sqrt_information =
                edge.information.llt().matrixL();

            // Ceres will take ownership of the pointer.
            ceres::CostFunction *cost_function =
                PoseGraph3dErrorTerm::Create(edge.t_be, sqrt_information);

            problem_->AddResidualBlock(cost_function,
                                      loss_function,
                                      pose_begin.t_vertex.p.data(),
                                      pose_begin.t_vertex.q.coeffs().data(),
                                      pose_end.t_vertex.p.data(),
                                      pose_end.t_vertex.q.coeffs().data());

            problem_->SetParameterization(pose_begin.t_vertex.q.coeffs().data(),
                                         local_parameterization_);
            problem_->SetParameterization(pose_end.t_vertex.q.coeffs().data(),
                                         local_parameterization_);
        }

        // The pose graph optimization problem has six DOFs that are not fully
        // constrained. This is typically referred to as gauge freedom. You can apply
        // a rigid body transformation to all the nodes and the optimization problem
        // will still have the exact same cost. The Levenberg-Marquardt algorithm has
        // internal damping which mitigates this issue, but it is better to properly
        // constrain the gauge freedom. This can be done by setting one of the poses
        // as constant so the optimizer cannot change it.
        auto& pose_start = vertex_list_[0];
        problem_->SetParameterBlockConstant(pose_start.t_vertex.p.data());
        problem_->SetParameterBlockConstant(pose_start.t_vertex.q.coeffs().data());
    }

    bool PoseGraph::solve()
    {
        if (problem_ == nullptr)
        {
            throw std::runtime_error("Problem is null, \
                please build the problem before solving it");
        }
        ceres::Solve(options_, problem_, &summary_);
        std::cout << summary_.FullReport() << '\n';
        return summary_.IsSolutionUsable();
    }

} // namespace stair_mapping
