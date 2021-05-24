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

        CHECK(poses != NULL);
        if (constraints.empty())
        {
            LOG(INFO) << "No constraints, no problem to optimize.";
            return;
        }

        ceres::LossFunction *loss_function = NULL;
        ceres::LocalParameterization *quaternion_local_parameterization =
            new EigenQuaternionParameterization;

        for (VectorOfConstraints::const_iterator constraints_iter =
                 constraints.begin();
             constraints_iter != constraints.end();
             ++constraints_iter)
        {
            const Constraint3d &constraint = *constraints_iter;

            MapOfPoses::iterator pose_begin_iter = poses->find(constraint.id_begin);
            CHECK(pose_begin_iter != poses->end())
                << "Pose with ID: " << constraint.id_begin << " not found.";
            MapOfPoses::iterator pose_end_iter = poses->find(constraint.id_end);
            CHECK(pose_end_iter != poses->end())
                << "Pose with ID: " << constraint.id_end << " not found.";

            const Eigen::Matrix<double, 6, 6> sqrt_information =
                constraint.information.llt().matrixL();
            // Ceres will take ownership of the pointer.
            ceres::CostFunction *cost_function =
                PoseGraph3dErrorTerm::Create(constraint.t_be, sqrt_information);

            problem->AddResidualBlock(cost_function,
                                      loss_function,
                                      pose_begin_iter->second.p.data(),
                                      pose_begin_iter->second.q.coeffs().data(),
                                      pose_end_iter->second.p.data(),
                                      pose_end_iter->second.q.coeffs().data());

            problem->SetParameterization(pose_begin_iter->second.q.coeffs().data(),
                                         quaternion_local_parameterization);
            problem->SetParameterization(pose_end_iter->second.q.coeffs().data(),
                                         quaternion_local_parameterization);
        }

        // The pose graph optimization problem has six DOFs that are not fully
        // constrained. This is typically referred to as gauge freedom. You can apply
        // a rigid body transformation to all the nodes and the optimization problem
        // will still have the exact same cost. The Levenberg-Marquardt algorithm has
        // internal damping which mitigates this issue, but it is better to properly
        // constrain the gauge freedom. This can be done by setting one of the poses
        // as constant so the optimizer cannot change it.
        MapOfPoses::iterator pose_start_iter = poses->begin();
        CHECK(pose_start_iter != poses->end()) << "There are no poses.";
        problem->SetParameterBlockConstant(pose_start_iter->second.p.data());
        problem->SetParameterBlockConstant(pose_start_iter->second.q.coeffs().data());
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
