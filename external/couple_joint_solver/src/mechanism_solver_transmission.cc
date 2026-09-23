#include <Eigen/LU>
#include <Eigen/SVD>
#include <cmath>
#include <limits>

#include "mechanism_solver.h"
#include "mechanism_solver_internal.h"
#include "mechanism_utils.h"

namespace couple_joint {

MechanismSolver::Residual MechanismSolver::ComputeResidual(double theta1, double theta2, double roll,
                                                           double pitch) const {
    const Eigen::Vector3d p1 = ComputeLeftPlanarEndpoint(theta1);
    const Eigen::Vector3d p2 = ComputeRightPlanarEndpoint(theta2);
    const SpatialPose spatial_pose = ComputeSpatialPose(roll, pitch);
    const Eigen::Vector3d s1 = spatial_pose.left_spatial_endpoint;
    const Eigen::Vector3d s2 = spatial_pose.right_spatial_endpoint;
    return {Squared((p1 - s1).norm()) - Squared(parameters_.left_spatial_length),
            Squared((p2 - s2).norm()) - Squared(parameters_.right_spatial_length)};
}

TransmissionInfo MechanismSolver::ComputeTransmissionInfo(double theta1, double theta2, double roll,
                                                          double pitch) const {
    TransmissionInfo transmission;

    // 重建 F(theta, q) 及其导数中出现的几何量。
    const Eigen::Vector3d p1 = ComputeLeftPlanarEndpoint(theta1);
    const Eigen::Vector3d p2 = ComputeRightPlanarEndpoint(theta2);
    const SpatialPose spatial_pose = ComputeSpatialPose(roll, pitch);
    const Eigen::Vector3d s1 = spatial_pose.left_spatial_endpoint;
    const Eigen::Vector3d s2 = spatial_pose.right_spatial_endpoint;
    const Eigen::Vector3d e1 = p1 - s1;
    const Eigen::Vector3d e2 = p2 - s2;
    const Eigen::Vector3d dp1_dtheta(-parameters_.left_planar_length * std::sin(theta1),
                                     parameters_.left_planar_length * std::cos(theta1), 0.0);
    const Eigen::Vector3d dp2_dtheta(parameters_.right_planar_length * std::sin(theta2),
                                     -parameters_.right_planar_length * std::cos(theta2), 0.0);
    const Eigen::Vector3d ds1_droll = ComputeSpatialRollDerivative(roll, pitch, left_zero_spatial_endpoint_);
    const Eigen::Vector3d ds1_dpitch = ComputeSpatialPitchDerivative(roll, pitch, left_zero_spatial_endpoint_);
    const Eigen::Vector3d ds2_droll = ComputeSpatialRollDerivative(roll, pitch, right_zero_spatial_endpoint_);
    const Eigen::Vector3d ds2_dpitch = ComputeSpatialPitchDerivative(roll, pitch, right_zero_spatial_endpoint_);

    // F_theta 是对角矩阵，因为每个残差 f_i 在输入侧只依赖对应的 theta_i。
    Eigen::Matrix2d fu = Eigen::Matrix2d::Zero();
    fu(0, 0) = 2.0 * e1.dot(dp1_dtheta);
    fu(1, 1) = 2.0 * e2.dot(dp2_dtheta);

    Eigen::Matrix2d fq;
    fq(0, 0) = -2.0 * e1.dot(ds1_droll);
    fq(0, 1) = -2.0 * e1.dot(ds1_dpitch);
    fq(1, 0) = -2.0 * e2.dot(ds2_droll);
    fq(1, 1) = -2.0 * e2.dot(ds2_dpitch);

    // 直接使用 F_theta 和 F_q 的奇异值作为推导对应的奇异性指标。
    const SvdInfo fu_svd = ComputeSvdInfo(fu);
    const SvdInfo fq_svd = ComputeSvdInfo(fq);
    transmission.sigma_min_fu = fu_svd.sigma_min;
    transmission.sigma_min_fq = fq_svd.sigma_min;
    transmission.singular_fu = transmission.sigma_min_fu < internal::kSingularValueTolerance;
    transmission.singular_fq = transmission.sigma_min_fq < internal::kSingularValueTolerance;

    Eigen::Matrix2d velocity_jacobian = Eigen::Matrix2d::Zero();
    if (!transmission.singular_fq) {
        // J_{q theta} = -F_q^{-1} F_theta，通过解线性方程获得，避免显式求逆。
        const Eigen::FullPivLU<Eigen::Matrix2d> lu(fq);
        if (lu.isInvertible()) {
            velocity_jacobian = -lu.solve(fu);
        } else {
            transmission.singular_fq = true;
        }
    }

    const SvdInfo j_svd = ComputeSvdInfo(velocity_jacobian);
    transmission.sigma_min_j = j_svd.sigma_min;
    transmission.condition_j = j_svd.condition;
    transmission.singular_j = transmission.singular_fq || transmission.sigma_min_j < internal::kSingularValueTolerance;

    Eigen::Matrix2d reduction = Eigen::Matrix2d::Zero();
    if (!transmission.singular_j) {
        // 只有当 J_{q theta} 可逆时，反向约化矩阵 J_{theta q} 才有意义。
        const Eigen::FullPivLU<Eigen::Matrix2d> lu(velocity_jacobian);
        if (lu.isInvertible()) {
            reduction = lu.solve(Eigen::Matrix2d::Identity());
        } else {
            transmission.singular_j = true;
        }
    }

    transmission.near_singular = transmission.singular_fu || transmission.singular_fq || transmission.singular_j ||
                                 transmission.sigma_min_j < internal::kNearSingularSigmaMinJLimit ||
                                 transmission.condition_j > internal::kNearSingularConditionJLimit;

    transmission.fu = FromEigen(fu);
    transmission.fq = FromEigen(fq);
    transmission.velocity_jacobian = FromEigen(velocity_jacobian);
    transmission.reduction = FromEigen(reduction);
    return transmission;
}

SolveStatus MechanismSolver::SolveSide(double a, double b, const Eigen::Vector3d& joint_center,
                                       const Eigen::Vector3d& spatial_endpoint, bool right_side, double reference_theta,
                                       double* theta) const {
    // 将 ||d_i + Rz(theta_i) v_i||^2 = b_i^2 整理为 A cos(theta_i) + B sin(theta_i) + C = 0。
    const double joint_to_spatial_x = joint_center.x() - spatial_endpoint.x();
    const double a_term = right_side ? -2.0 * a * joint_to_spatial_x : 2.0 * a * joint_to_spatial_x;
    const double b_term = right_side ? 2.0 * a * spatial_endpoint.y() : -2.0 * a * spatial_endpoint.y();
    const double c_term = Squared(joint_to_spatial_x) + Squared(spatial_endpoint.y()) + Squared(spatial_endpoint.z()) +
                          Squared(a) - Squared(b);
    const double rho = std::sqrt(Squared(a_term) + Squared(b_term));
    if (rho < internal::kTiny) {
        return SolveStatus::DegenerateSideEquation;
    }

    const double cosine_value = -c_term / rho;
    if (cosine_value < -1.0 - 1.0e-6 || cosine_value > 1.0 + 1.0e-6) {
        return SolveStatus::UnreachablePose;
    }

    const double delta = std::atan2(b_term, a_term);
    const double alpha = std::acos(Clamp(cosine_value, -1.0, 1.0));
    const double candidate1 = WrapAngle(delta + alpha);
    const double candidate2 = WrapAngle(delta - alpha);
    // 选取与上一帧 theta_i 最接近的候选解，以保持分支连续性。
    *theta = AngleDistance(candidate1, reference_theta) <= AngleDistance(candidate2, reference_theta) ? candidate1
                                                                                                      : candidate2;
    return SolveStatus::Success;
}

SolveStatus MechanismSolver::StatusForTransmission(const TransmissionInfo& transmission) {
    return transmission.near_singular ? SolveStatus::NearSingular : SolveStatus::Success;
}

MechanismSolver::SvdInfo MechanismSolver::ComputeSvdInfo(const Eigen::Matrix2d& matrix) {
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const double sigma_max = svd.singularValues()(0);
    const double sigma_min = svd.singularValues()(1);
    SvdInfo info;
    info.sigma_max = sigma_max;
    info.sigma_min = sigma_min;
    info.condition =
        sigma_min < internal::kConditionTiny ? std::numeric_limits<double>::infinity() : sigma_max / sigma_min;
    return info;
}

}  // namespace couple_joint
