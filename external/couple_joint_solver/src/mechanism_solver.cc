#include "mechanism_solver.h"

#include <cmath>

#include "mechanism_solver_internal.h"
#include "mechanism_utils.h"

namespace couple_joint {

MechanismSolver::MechanismSolver(const CalibrationInput& input) {
    calibration_status_ = Calibrate(input);
}

CalibrationStatus MechanismSolver::GetCalibrationStatus() const {
    return calibration_status_;
}

MechanismParameters MechanismSolver::GetParameters() const {
    return parameters_;
}

MechanismState MechanismSolver::GetState() const {
    return state_;
}

MechanismGeometry MechanismSolver::GetGeometry() const {
    MechanismGeometry geometry;
    geometry.left_joint_center = FromEigen(Eigen::Vector3d(-parameters_.base_distance / 2.0, 0.0, 0.0));
    geometry.right_joint_center = FromEigen(Eigen::Vector3d(parameters_.base_distance / 2.0, 0.0, 0.0));
    geometry.left_planar_endpoint = FromEigen(ComputeLeftPlanarEndpoint(state_.theta1));
    geometry.right_planar_endpoint = FromEigen(ComputeRightPlanarEndpoint(state_.theta2));
    const SpatialPose spatial_pose = ComputeSpatialPose(state_.roll, state_.pitch);
    geometry.roll_center = FromEigen(spatial_pose.roll_center);
    geometry.pitch_center = FromEigen(spatial_pose.pitch_center);
    geometry.left_spatial_endpoint = FromEigen(spatial_pose.left_spatial_endpoint);
    geometry.right_spatial_endpoint = FromEigen(spatial_pose.right_spatial_endpoint);
    geometry.roll_axis = FromEigen(spatial_pose.roll_axis);
    geometry.pitch_axis = FromEigen(spatial_pose.pitch_axis);
    geometry.planar_direction = {0.0, 0.0, 1.0};
    return geometry;
}

TransmissionInfo MechanismSolver::GetTransmission() const {
    if (calibration_status_ != CalibrationStatus::Success) {
        return {};
    }
    return ComputeTransmissionInfo(state_.theta1, state_.theta2, state_.roll, state_.pitch);
}

const char* MechanismSolver::StatusMessage(CalibrationStatus status) {
    switch (status) {
        case CalibrationStatus::Success:
            return "";
        case CalibrationStatus::CoincidentJointCenters:
            return "Invalid calibration: left and right joint centers are coincident.";
        case CalibrationStatus::DegenerateRotateDirection:
            return "Invalid calibration: rotate direction is degenerate.";
        case CalibrationStatus::BaseAxisNotPerpendicularToRotateDirection:
            return "Invalid calibration: O1_O2 must be perpendicular to rotate direction.";
        case CalibrationStatus::LeftPlanarLinkNotPerpendicularToRotateDirection:
            return "Invalid calibration: O1_P1 must be perpendicular to rotate direction.";
        case CalibrationStatus::RightPlanarLinkNotPerpendicularToRotateDirection:
            return "Invalid calibration: O2_P2 must be perpendicular to rotate direction.";
        case CalibrationStatus::CollinearPlanarLinksWithBaseAxis:
            return "Invalid calibration: initial planar links are collinear with the base axis; change the initial "
                   "pose.";
        case CalibrationStatus::UnstableCanonicalFrame:
            return "Invalid calibration: failed to build a stable frame; change the initial pose.";
        case CalibrationStatus::UnsupportedRotationOrder:
            return "Invalid calibration: rotation order must be set to a supported value.";
        case CalibrationStatus::DegeneratePitchAxis:
            return "Invalid calibration: pitch axis is degenerate; check spatial endpoints.";
        case CalibrationStatus::RollAxisNotPerpendicularToPitchAxis:
            return "Invalid calibration: roll axis must be nearly perpendicular to pitch axis.";
        case CalibrationStatus::ZeroLengthLink:
            return "Invalid calibration: one or more links have zero length.";
        case CalibrationStatus::InitialPoseConstraintViolation:
            return "Invalid calibration: initial pose does not satisfy the closed-chain constraints.";
        case CalibrationStatus::PlanarPointsNotCoplanar:
            return "Invalid calibration: O1, O2, P1 and P2 must be coplanar.";
    }
    return "Unknown calibration status.";
}

const char* MechanismSolver::StatusMessage(SolveStatus status) {
    switch (status) {
        case SolveStatus::Success:
            return "";
        case SolveStatus::NearSingular:
            return "Current pose is near singular; position is updated, but velocity and torque mappings need "
                   "limiting.";
        case SolveStatus::InvalidCalibration:
            return "Solver calibration is invalid.";
        case SolveStatus::DegenerateSideEquation:
            return "Inverse solve failed: one side equation is degenerate.";
        case SolveStatus::UnreachablePose:
            return "Inverse solve failed: target pose is unreachable.";
        case SolveStatus::SingularJacobian:
            return "Solve failed: the local Jacobian is singular.";
        case SolveStatus::ForwardNotConverged:
            return "Forward solve failed: iteration did not converge.";
    }
    return "Unknown solve status.";
}

SolveResult MechanismSolver::SolveInverse(double roll, double pitch) {
    if (calibration_status_ != CalibrationStatus::Success) {
        return {SolveStatus::InvalidCalibration, 0.0, 0, {}};
    }

    // 先构造当前的 S1(q) 和 S2(q)，再分别求解左右两侧的标量方程。
    const SpatialPose spatial_pose = ComputeSpatialPose(roll, pitch);
    const Eigen::Vector3d left_spatial = spatial_pose.left_spatial_endpoint;
    const Eigen::Vector3d right_spatial = spatial_pose.right_spatial_endpoint;

    double theta1 = state_.theta1;
    double theta2 = state_.theta2;
    const Eigen::Vector3d left_joint(-parameters_.base_distance / 2.0, 0.0, 0.0);
    const Eigen::Vector3d right_joint(parameters_.base_distance / 2.0, 0.0, 0.0);
    // 每一侧都化为 A cos(theta_i) + B sin(theta_i) + C = 0，并保留最接近上一帧的分支。
    const SolveStatus left_status = SolveSide(parameters_.left_planar_length, parameters_.left_spatial_length,
                                              left_joint, left_spatial, false, state_.theta1, &theta1);
    if (left_status != SolveStatus::Success) {
        return {left_status, 0.0, 0, GetTransmission()};
    }
    const SolveStatus right_status = SolveSide(parameters_.right_planar_length, parameters_.right_spatial_length,
                                               right_joint, right_spatial, true, state_.theta2, &theta2);
    if (right_status != SolveStatus::Success) {
        return {right_status, 0.0, 0, GetTransmission()};
    }

    const Residual residual = ComputeResidual(theta1, theta2, roll, pitch);
    state_ = {theta1, theta2, roll, pitch};
    const TransmissionInfo transmission = ComputeTransmissionInfo(theta1, theta2, roll, pitch);
    return {StatusForTransmission(transmission), std::sqrt(Squared(residual.f1) + Squared(residual.f2)), 1,
            transmission};
}

SolveResult MechanismSolver::SolveForward(double theta1, double theta2) {
    if (calibration_status_ != CalibrationStatus::Success) {
        return {SolveStatus::InvalidCalibration, 0.0, 0, {}};
    }

    // 按机构尺度放缩残差阈值，使停止条件与几何尺寸匹配。
    const double residual_tolerance = internal::kResidualTolerance * (Squared(parameters_.left_spatial_length) +
                                                                      Squared(parameters_.right_spatial_length) + 1.0);

    // 在 theta 固定时，位置正解等价于在 q = [roll, pitch]^T 上求解 F(theta, q) = 0。
    const auto residual_fn = [this, theta1, theta2](double roll, double pitch) -> Eigen::Vector2d {
        const Residual r = ComputeResidual(theta1, theta2, roll, pitch);
        return {r.f1, r.f2};
    };

    // 牛顿线性系统使用 F_q 作为局部雅可比，并提前做奇异性检查。
    const auto jacobian_fn = [this, theta1, theta2](double roll, double pitch) -> JacobianEval {
        const TransmissionInfo t = ComputeTransmissionInfo(theta1, theta2, roll, pitch);
        return {ToEigen(t.fq), t.singular_fq};
    };

    const ForwardNewtonResult result =
        SolveDampedNewton(residual_fn, jacobian_fn, state_.roll, state_.pitch, residual_tolerance,
                          internal::kStepTolerance, internal::kDamping, internal::kMaxIterations);

    // 在最终迭代点重新评估传动信息，确保状态码与雅可比数据一致。
    const TransmissionInfo transmission = ComputeTransmissionInfo(theta1, theta2, result.roll, result.pitch);

    if (result.singular) {
        return {SolveStatus::SingularJacobian, result.residual_norm, result.iterations, transmission};
    }
    if (!result.converged) {
        return {SolveStatus::ForwardNotConverged, result.residual_norm, result.iterations, transmission};
    }

    state_ = {theta1, theta2, result.roll, result.pitch};
    return {StatusForTransmission(transmission), result.residual_norm, result.iterations, transmission};
}

VelocityTorqueResult MechanismSolver::SolveForwardVelocity(double theta1_dot, double theta2_dot) const {
    if (calibration_status_ != CalibrationStatus::Success) {
        return {SolveStatus::InvalidCalibration, theta1_dot, theta2_dot, 0.0, 0.0, {}};
    }

    const TransmissionInfo transmission = GetTransmission();
    if (transmission.singular_fq) {
        return {SolveStatus::SingularJacobian, theta1_dot, theta2_dot, 0.0, 0.0, transmission};
    }

    const double roll_dot =
        transmission.velocity_jacobian.m00 * theta1_dot + transmission.velocity_jacobian.m01 * theta2_dot;
    const double pitch_dot =
        transmission.velocity_jacobian.m10 * theta1_dot + transmission.velocity_jacobian.m11 * theta2_dot;
    return {StatusForTransmission(transmission), theta1_dot, theta2_dot, roll_dot, pitch_dot, transmission};
}

VelocityTorqueResult MechanismSolver::SolveInverseVelocity(double roll_dot, double pitch_dot) const {
    if (calibration_status_ != CalibrationStatus::Success) {
        return {SolveStatus::InvalidCalibration, 0.0, 0.0, roll_dot, pitch_dot, {}};
    }

    const TransmissionInfo transmission = GetTransmission();
    if (transmission.singular_j) {
        return {SolveStatus::SingularJacobian, 0.0, 0.0, roll_dot, pitch_dot, transmission};
    }

    const double theta1_dot = transmission.reduction.m00 * roll_dot + transmission.reduction.m01 * pitch_dot;
    const double theta2_dot = transmission.reduction.m10 * roll_dot + transmission.reduction.m11 * pitch_dot;
    return {StatusForTransmission(transmission), theta1_dot, theta2_dot, roll_dot, pitch_dot, transmission};
}

VelocityTorqueResult MechanismSolver::SolveForwardTorque(double theta1_torque, double theta2_torque) const {
    if (calibration_status_ != CalibrationStatus::Success) {
        return {SolveStatus::InvalidCalibration, theta1_torque, theta2_torque, 0.0, 0.0, {}};
    }

    const TransmissionInfo transmission = GetTransmission();
    if (transmission.singular_j) {
        return {SolveStatus::SingularJacobian, theta1_torque, theta2_torque, 0.0, 0.0, transmission};
    }

    const double roll_torque = transmission.reduction.m00 * theta1_torque + transmission.reduction.m10 * theta2_torque;
    const double pitch_torque = transmission.reduction.m01 * theta1_torque + transmission.reduction.m11 * theta2_torque;
    return {StatusForTransmission(transmission), theta1_torque, theta2_torque, roll_torque, pitch_torque, transmission};
}

VelocityTorqueResult MechanismSolver::SolveInverseTorque(double roll_torque, double pitch_torque) const {
    if (calibration_status_ != CalibrationStatus::Success) {
        return {SolveStatus::InvalidCalibration, 0.0, 0.0, roll_torque, pitch_torque, {}};
    }

    const TransmissionInfo transmission = GetTransmission();
    if (transmission.singular_fq) {
        return {SolveStatus::SingularJacobian, 0.0, 0.0, roll_torque, pitch_torque, transmission};
    }

    const double theta1_torque =
        transmission.velocity_jacobian.m00 * roll_torque + transmission.velocity_jacobian.m10 * pitch_torque;
    const double theta2_torque =
        transmission.velocity_jacobian.m01 * roll_torque + transmission.velocity_jacobian.m11 * pitch_torque;
    return {StatusForTransmission(transmission), theta1_torque, theta2_torque, roll_torque, pitch_torque, transmission};
}

}  // namespace couple_joint
