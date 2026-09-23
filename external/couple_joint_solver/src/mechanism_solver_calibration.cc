#include <algorithm>
#include <cmath>

#include "mechanism_solver.h"
#include "mechanism_solver_internal.h"
#include "mechanism_utils.h"

namespace couple_joint {

CalibrationStatus MechanismSolver::Calibrate(const CalibrationInput& input) {
    origin_ = Eigen::Vector3d((input.left_joint_center.x + input.right_joint_center.x) / 2.0,
                              (input.left_joint_center.y + input.right_joint_center.y) / 2.0,
                              (input.left_joint_center.z + input.right_joint_center.z) / 2.0);

    // 构造规范坐标系：+X 沿 O1 -> O2，+Z 沿输入平面的法向。
    Eigen::Vector3d axis_x = ToEigen(input.right_joint_center) - ToEigen(input.left_joint_center);
    if (axis_x.norm() < internal::kTiny) {
        return CalibrationStatus::CoincidentJointCenters;
    }
    axis_x.normalize();

    const Eigen::Vector3d left_link = ToEigen(input.left_planar_endpoint) - ToEigen(input.left_joint_center);
    const Eigen::Vector3d right_link = ToEigen(input.right_planar_endpoint) - ToEigen(input.right_joint_center);
    Eigen::Vector3d planar_normal;

    if (input.use_rotate_direction) {
        // 若显式提供了转轴 / 平面法向，则优先使用该方向。
        planar_normal = ToEigen(input.rotate_direction);
        if (planar_normal.norm() < internal::kTiny) {
            return CalibrationStatus::DegenerateRotateDirection;
        }
        planar_normal.normalize();
        if (std::abs(axis_x.dot(planar_normal)) > internal::kAxisPerpendicularTolerance) {
            return CalibrationStatus::BaseAxisNotPerpendicularToRotateDirection;
        }
        const double left_norm = left_link.norm();
        if (left_norm > internal::kTiny &&
            std::abs(left_link.dot(planar_normal)) / left_norm > internal::kAxisPerpendicularTolerance) {
            return CalibrationStatus::LeftPlanarLinkNotPerpendicularToRotateDirection;
        }
        const double right_norm = right_link.norm();
        if (right_norm > internal::kTiny &&
            std::abs(right_link.dot(planar_normal)) / right_norm > internal::kAxisPerpendicularTolerance) {
            return CalibrationStatus::RightPlanarLinkNotPerpendicularToRotateDirection;
        }
    } else {
        // 否则在共面性检查通过后，根据 O1、O2、P1、P2 推断平面法向。
        const CalibrationStatus coplanarity_status = ValidatePlanarJointCoplanarity(input);
        if (coplanarity_status != CalibrationStatus::Success) {
            return coplanarity_status;
        }
        planar_normal = left_link.cross(axis_x);
        if (planar_normal.norm() < internal::kTiny) {
            planar_normal = right_link.cross(axis_x);
        }
        if (planar_normal.norm() < internal::kTiny) {
            return CalibrationStatus::CollinearPlanarLinksWithBaseAxis;
        }
        planar_normal.normalize();
    }

    Eigen::Vector3d axis_z = planar_normal - axis_x * planar_normal.dot(axis_x);
    if (axis_z.norm() < internal::kTiny) {
        return CalibrationStatus::UnstableCanonicalFrame;
    }
    axis_z.normalize();
    Eigen::Vector3d axis_y = axis_z.cross(axis_x);
    axis_y.normalize();

    axis_x_ = axis_x;
    axis_y_ = axis_y;
    axis_z_ = axis_z;
    roll_axis_ = Eigen::Vector3d(0.0, 0.0, 1.0);

    if (input.rotation_order != RotationOrder::PitchThenRoll && input.rotation_order != RotationOrder::RollThenPitch) {
        return CalibrationStatus::UnsupportedRotationOrder;
    }

    const Eigen::Vector3d o1 = ToCanonicalPoint(ToEigen(input.left_joint_center));
    const Eigen::Vector3d o2 = ToCanonicalPoint(ToEigen(input.right_joint_center));
    const Eigen::Vector3d p1 = ToCanonicalPoint(ToEigen(input.left_planar_endpoint));
    const Eigen::Vector3d p2 = ToCanonicalPoint(ToEigen(input.right_planar_endpoint));
    const Eigen::Vector3d roll_center = ToCanonicalPoint(ToEigen(input.roll_center));
    const Eigen::Vector3d pitch_center = ToCanonicalPoint(ToEigen(input.pitch_center));
    const Eigen::Vector3d s1 = ToCanonicalPoint(ToEigen(input.left_spatial_endpoint));
    const Eigen::Vector3d s2 = ToCanonicalPoint(ToEigen(input.right_spatial_endpoint));

    // 在规范模型中，pitch 轴由零位时的 S2 - S1 方向定义。
    Eigen::Vector3d pitch_axis = s2 - s1;
    if (pitch_axis.norm() < internal::kTiny) {
        return CalibrationStatus::DegeneratePitchAxis;
    }
    pitch_axis.normalize();

    const double roll_pitch_dot = roll_axis_.dot(pitch_axis);
    if (std::abs(roll_pitch_dot) > internal::kAxisPerpendicularTolerance) {
        return CalibrationStatus::RollAxisNotPerpendicularToPitchAxis;
    }

    rotation_order_ = input.rotation_order;

    // 缓存后续所有求解都会用到的规范长度与零位点。
    parameters_.base_distance = (o1 - o2).norm();
    parameters_.left_planar_length = (o1 - p1).norm();
    parameters_.right_planar_length = (o2 - p2).norm();
    parameters_.left_spatial_length = (p1 - s1).norm();
    parameters_.right_spatial_length = (p2 - s2).norm();
    parameters_.roll_center = FromEigen(roll_center);
    parameters_.pitch_center = FromEigen(pitch_center);
    parameters_.left_zero_spatial_endpoint = FromEigen(s1);
    parameters_.right_zero_spatial_endpoint = FromEigen(s2);

    if (parameters_.left_planar_length < internal::kTiny || parameters_.right_planar_length < internal::kTiny ||
        parameters_.left_spatial_length < internal::kTiny || parameters_.right_spatial_length < internal::kTiny) {
        return CalibrationStatus::ZeroLengthLink;
    }

    roll_center_ = roll_center;
    pitch_center_ = pitch_center;
    left_zero_spatial_endpoint_ = s1;
    right_zero_spatial_endpoint_ = s2;
    pitch_axis_ = pitch_axis;

    // 验证声明的零位是否已经满足 F(theta, q = 0) = 0。
    parameters_.initial_theta1 = std::atan2(p1.y(), p1.x() + parameters_.base_distance / 2.0);
    parameters_.initial_theta2 = std::atan2(-p2.y(), parameters_.base_distance / 2.0 - p2.x());
    state_ = {parameters_.initial_theta1, parameters_.initial_theta2, 0.0, 0.0};
    const Residual initial_residual = ComputeResidual(state_.theta1, state_.theta2, state_.roll, state_.pitch);
    const double residual_norm = std::sqrt(Squared(initial_residual.f1) + Squared(initial_residual.f2));
    if (residual_norm > 1.0e-8) {
        return CalibrationStatus::InitialPoseConstraintViolation;
    }
    return CalibrationStatus::Success;
}

CalibrationStatus MechanismSolver::ValidatePlanarJointCoplanarity(const CalibrationInput& input) {
    const Eigen::Vector3d base = ToEigen(input.right_joint_center) - ToEigen(input.left_joint_center);
    const Eigen::Vector3d left_link = ToEigen(input.left_planar_endpoint) - ToEigen(input.left_joint_center);
    // 使用任一侧平面连杆构造 O1/O2/P1/P2 所在平面的法向。
    Eigen::Vector3d normal = base.cross(left_link);
    double normal_norm = normal.norm();
    if (normal_norm < internal::kTiny) {
        const Eigen::Vector3d right_link = ToEigen(input.right_planar_endpoint) - ToEigen(input.right_joint_center);
        normal = base.cross(right_link);
        normal_norm = normal.norm();
    }
    if (normal_norm < internal::kTiny) {
        return CalibrationStatus::Success;
    }

    const Eigen::Vector3d right_endpoint_offset =
        ToEigen(input.right_planar_endpoint) - ToEigen(input.left_joint_center);
    const double distance_to_plane = std::abs(right_endpoint_offset.dot(normal)) / normal_norm;
    // 按机构尺寸缩放容差，使大小机构的判据一致。
    const double scale =
        std::max({(ToEigen(input.left_joint_center) - ToEigen(input.right_joint_center)).norm(),
                  (ToEigen(input.left_joint_center) - ToEigen(input.left_planar_endpoint)).norm(),
                  (ToEigen(input.right_joint_center) - ToEigen(input.right_planar_endpoint)).norm(), 1.0});
    if (distance_to_plane > scale * internal::kCoplanarToleranceScale) {
        return CalibrationStatus::PlanarPointsNotCoplanar;
    }
    return CalibrationStatus::Success;
}

Eigen::Vector3d MechanismSolver::ToCanonicalPoint(const Eigen::Vector3d& point) const {
    const Eigen::Vector3d relative = point - origin_;
    return {relative.dot(axis_x_), relative.dot(axis_y_), relative.dot(axis_z_)};
}

}  // namespace couple_joint
