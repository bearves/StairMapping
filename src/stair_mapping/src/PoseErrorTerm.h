#pragma once

#include <ceres/ceres.h>

namespace stair_mapping
{

typedef Eigen::Matrix<double, 6, 6> InfoMatrix;

// The SE3 transform for reprensenting vertices and constraints
struct Pose3d
{
    Eigen::Vector3d p;
    Eigen::Quaterniond q;

    Pose3d()
    {
        p.setZero();
        q.setIdentity();
    }

    Pose3d(Eigen::Matrix4d t)
    {
        Eigen::Affine3d af(t);
        p = af.translation();
        q = Eigen::Quaterniond(af.rotation());
    }

    const Eigen::Matrix4d toMat4d() const
    {
        Eigen::Matrix4d mat4d = Eigen::Matrix4d::Identity();
        mat4d.block<3,3>(0,0) = q.normalized().toRotationMatrix();
        mat4d.block<3,1>(0,3) = p;
        return mat4d;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// The vertex of the graph
struct Vertex3d
{
    Vertex3d()
        : t_vertex()
    {
    }

    Vertex3d(Eigen::Matrix4d t)
        : t_vertex(t)
    {
    }

    const Eigen::Matrix4d toMat4d() const
    {
        return t_vertex.toMat4d();
    }

    Pose3d t_vertex;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Computes the error term for two poses that have a relative pose measurement
// between them. Let the hat variables be the measurement. We have two poses x_a
// and x_b. Through sensor measurements we can measure the transformation of
// frame B w.r.t frame A denoted as t_ab_hat. We can compute an error metric
// between the current estimate of the poses and the measurement.
//
// In this formulation, we have chosen to represent the rigid transformation as
// a Hamiltonian quaternion, q, and position, p. The quaternion ordering is
// [x, y, z, w].

// The estimated measurement is:
//      t_ab = [ p_ab ]  = [ R(q_a)^T * (p_b - p_a) ]
//             [ q_ab ]    [ q_a^{-1] * q_b         ]
//
// where ^{-1} denotes the inverse and R(q) is the rotation matrix for the
// quaternion. Now we can compute an error metric between the estimated and
// measurement transformation. For the orientation error, we will use the
// standard multiplicative error resulting in:
//
//   error = [ p_ab - \hat{p}_ab                 ]
//           [ 2.0 * Vec(q_ab * \hat{q}_ab^{-1}) ]
//
// where Vec(*) returns the vector (imaginary) part of the quaternion. Since
// the measurement has an uncertainty associated with how accurate it is, we
// will weight the errors by the square root of the measurement information
// matrix:
//
//   residuals = I^{1/2) * error
// where I is the information matrix which is the inverse of the covariance.
class PoseGraph3dErrorTerm
{
public:
    PoseGraph3dErrorTerm(const Pose3d &t_ab_measured,
                            const Eigen::Matrix<double, 6, 6> &sqrt_information)
        : t_ab_measured_(t_ab_measured), sqrt_information_(sqrt_information) {}

    template <typename T>
    bool operator()(const T *const p_a_ptr,
                    const T *const q_a_ptr,
                    const T *const p_b_ptr,
                    const T *const q_b_ptr,
                    T *residuals_ptr) const
    {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

        // Compute the relative transformation between the two frames.
        Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
        Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

        // Represent the displacement between the two frames in the A frame.
        Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

        // Compute the error between the two orientation estimates.
        Eigen::Quaternion<T> delta_q =
            t_ab_measured_.q.template cast<T>() * q_ab_estimated.conjugate();

        // Compute the residuals.
        // [ position         ]   [ delta_p          ]
        // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) =
            p_ab_estimated - t_ab_measured_.p.template cast<T>();
        residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

        // Scale the residuals by the measurement uncertainty.
        residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

        return true;
    }

    static ceres::CostFunction *Create(
        const Pose3d &t_ab_measured,
        const Eigen::Matrix<double, 6, 6> &sqrt_information)
    {
        return new ceres::AutoDiffCostFunction<PoseGraph3dErrorTerm, 6, 3, 4, 3, 4>(
            new PoseGraph3dErrorTerm(t_ab_measured, sqrt_information));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // The measurement for the position of B relative to A in the A frame.
    const Pose3d t_ab_measured_;
    // The square root of the measurement information matrix.
    const Eigen::Matrix<double, 6, 6> sqrt_information_;
};

class PoseGraph3dTranslationErrorTerm
{
public:
    PoseGraph3dTranslationErrorTerm(const Pose3d &t_ab_measured,
                            const Eigen::Matrix<double, 3, 3> &sqrt_information)
        : t_ab_measured_(t_ab_measured), sqrt_information_(sqrt_information) {}

    template <typename T>
    bool operator()(const T *const p_a_ptr,
                    const T *const q_a_ptr,
                    const T *const p_b_ptr,
                    const T *const q_b_ptr,
                    T *residuals_ptr) const
    {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

        // Compute the relative transformation between the two frames.
        Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
        Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

        // Represent the displacement between the two frames in the A frame.
        Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

        // Compute the residuals.
        // [ position         ]   [ delta_p          ]
        Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals(residuals_ptr);
        residuals = p_ab_estimated - t_ab_measured_.p.template cast<T>();

        // Scale the residuals by the measurement uncertainty.
        residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

        return true;
    }

    static ceres::CostFunction *Create(
        const Pose3d &t_ab_measured,
        const Eigen::Matrix<double, 3, 3> &sqrt_information)
    {
        return new ceres::AutoDiffCostFunction<PoseGraph3dTranslationErrorTerm, 3, 3, 4, 3, 4>(
            new PoseGraph3dTranslationErrorTerm(t_ab_measured, sqrt_information));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // The measurement for the position of B relative to A in the A frame.
    const Pose3d t_ab_measured_;
    // The square root of the measurement information matrix.
    const Eigen::Matrix<double, 3, 3> sqrt_information_;
};

class PoseGraph3dRotationErrorTerm
{
public:
    PoseGraph3dRotationErrorTerm(const Pose3d &t_ab_measured,
                            const Eigen::Matrix<double, 3, 3> &sqrt_information)
        : t_ab_measured_(t_ab_measured), sqrt_information_(sqrt_information) {}

    template <typename T>
    bool operator()(const T *const p_a_ptr,
                    const T *const q_a_ptr,
                    const T *const p_b_ptr,
                    const T *const q_b_ptr,
                    T *residuals_ptr) const
    {

        Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

        // Compute the relative transformation between the two frames.
        Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
        Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

        // Compute the error between the two orientation estimates.
        Eigen::Quaternion<T> delta_q =
            t_ab_measured_.q.template cast<T>() * q_ab_estimated.conjugate();

        // Compute the residuals.
        // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
        Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals(residuals_ptr);
        residuals = T(2.0) * delta_q.vec();

        // Scale the residuals by the measurement uncertainty.
        residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

        return true;
    }

    static ceres::CostFunction *Create(
        const Pose3d &t_ab_measured,
        const Eigen::Matrix<double, 3, 3> &sqrt_information)
    {
        return new ceres::AutoDiffCostFunction<PoseGraph3dRotationErrorTerm, 3, 3, 4, 3, 4>(
            new PoseGraph3dRotationErrorTerm(t_ab_measured, sqrt_information));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // The measurement for the position of B relative to A in the A frame.
    const Pose3d t_ab_measured_;
    // The square root of the measurement information matrix.
    const Eigen::Matrix<double, 3, 3> sqrt_information_;
};

class PoseGraph3dAbsoluteRotationErrorTerm
{
public:
    PoseGraph3dAbsoluteRotationErrorTerm(const Pose3d &t_ab_measured,
                            const Eigen::Matrix<double, 3, 3> &sqrt_information)
        : t_ab_measured_(t_ab_measured), sqrt_information_(sqrt_information) {}

    template <typename T>
    bool operator()(const T *const p_a_ptr,
                    const T *const q_a_ptr,
                    const T *const p_b_ptr,
                    const T *const q_b_ptr,
                    T *residuals_ptr) const
    {
        Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

        // Compute the relative transformation between the frame B and the world.
        Eigen::Quaternion<T> q_b_estimated = q_b;

        // Compute the error of the frame B to the world
        Eigen::Quaternion<T> delta_q =
            t_ab_measured_.q.template cast<T>() * q_b_estimated.conjugate();

        // Compute the residuals.
        // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
        Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals(residuals_ptr);
        residuals = T(2.0) * delta_q.vec();
        // residuals[2] = T(0);

        // Scale the residuals by the measurement uncertainty.
        residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

        return true;
    }

    static ceres::CostFunction *Create(
        const Pose3d &t_ab_measured,
        const Eigen::Matrix<double, 3, 3> &sqrt_information)
    {
        return new ceres::AutoDiffCostFunction<PoseGraph3dAbsoluteRotationErrorTerm, 3, 3, 4, 3, 4>(
            new PoseGraph3dAbsoluteRotationErrorTerm(t_ab_measured, sqrt_information));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // The measurement for the position of B relative to A in the A frame.
    const Pose3d t_ab_measured_;
    // The square root of the measurement information matrix.
    const Eigen::Matrix<double, 3, 3> sqrt_information_;
};
    
} // namespace stair_mapping
