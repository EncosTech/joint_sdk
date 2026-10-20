#include "mechanism_solver.h"
#include "mechanism_solver_internal.h"
#include "mechanism_utils.h"

namespace couple_joint {

MechanismSolver::SpatialPose MechanismSolver::ComputeSpatialPose(double roll, double pitch) const {
    SpatialPose pose;
    // 旋转中心和旋转轴的搬运方式取决于先做 pitch 还是先做 roll。
    if (rotation_order_ == RotationOrder::PitchThenRoll) {
        pose.pitch_center = pitch_center_;
        pose.pitch_axis = pitch_axis_;
        pose.roll_center = RotatePoint(pitch_center_, pitch_axis_, pitch, roll_center_);
        pose.roll_axis = RotateVector(pitch_axis_, pitch, roll_axis_);
    } else {
        pose.roll_center = roll_center_;
        pose.roll_axis = roll_axis_;
        pose.pitch_center = RotatePoint(roll_center_, roll_axis_, roll, pitch_center_);
        pose.pitch_axis = RotateVector(roll_axis_, roll, pitch_axis_);
    }
    pose.roll_axis.normalize();
    pose.pitch_axis.normalize();
    pose.left_spatial_endpoint = TransformSpatialPoint(roll, pitch, left_zero_spatial_endpoint_);
    pose.right_spatial_endpoint = TransformSpatialPoint(roll, pitch, right_zero_spatial_endpoint_);
    return pose;
}

Eigen::Vector3d MechanismSolver::TransformSpatialPoint(double roll, double pitch,
                                                       const Eigen::Vector3d& zero_point) const {
    if (rotation_order_ == RotationOrder::PitchThenRoll) {
        // 先在零位坐标下做 pitch，再绕搬运后的 roll 中心 / 轴做 roll。
        const Eigen::Vector3d after_pitch = RotatePoint(pitch_center_, pitch_axis_, pitch, zero_point);
        const Eigen::Vector3d current_roll_center = RotatePoint(pitch_center_, pitch_axis_, pitch, roll_center_);
        Eigen::Vector3d current_roll_axis = RotateVector(pitch_axis_, pitch, roll_axis_);
        current_roll_axis.normalize();
        return RotatePoint(current_roll_center, current_roll_axis, roll, after_pitch);
    }
    // 先做 roll，再绕搬运后的 pitch 中心 / 轴做 pitch。
    const Eigen::Vector3d after_roll = RotatePoint(roll_center_, roll_axis_, roll, zero_point);
    const Eigen::Vector3d current_pitch_center = RotatePoint(roll_center_, roll_axis_, roll, pitch_center_);
    Eigen::Vector3d current_pitch_axis = RotateVector(roll_axis_, roll, pitch_axis_);
    current_pitch_axis.normalize();
    return RotatePoint(current_pitch_center, current_pitch_axis, pitch, after_roll);
}

Eigen::Vector3d MechanismSolver::ComputeSpatialRollDerivative(double roll, double pitch,
                                                              const Eigen::Vector3d& zero_point) const {
    const Eigen::Vector3d plus = TransformSpatialPoint(roll + internal::kDerivativeEps, pitch, zero_point);
    const Eigen::Vector3d minus = TransformSpatialPoint(roll - internal::kDerivativeEps, pitch, zero_point);
    return (plus - minus) / (2.0 * internal::kDerivativeEps);
}

Eigen::Vector3d MechanismSolver::ComputeSpatialPitchDerivative(double roll, double pitch,
                                                               const Eigen::Vector3d& zero_point) const {
    const Eigen::Vector3d plus = TransformSpatialPoint(roll, pitch + internal::kDerivativeEps, zero_point);
    const Eigen::Vector3d minus = TransformSpatialPoint(roll, pitch - internal::kDerivativeEps, zero_point);
    return (plus - minus) / (2.0 * internal::kDerivativeEps);
}

Eigen::Vector3d MechanismSolver::ComputeLeftPlanarEndpoint(double theta1) const {
    return {-parameters_.base_distance / 2.0 + parameters_.left_planar_length * std::cos(theta1),
            parameters_.left_planar_length * std::sin(theta1), 0.0};
}

Eigen::Vector3d MechanismSolver::ComputeRightPlanarEndpoint(double theta2) const {
    return {parameters_.base_distance / 2.0 - parameters_.right_planar_length * std::cos(theta2),
            -parameters_.right_planar_length * std::sin(theta2), 0.0};
}

}  // namespace couple_joint
