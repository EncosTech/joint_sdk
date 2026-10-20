#include "mechanism_solver.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

constexpr double kTolerance = 1.0e-6;

void ExpectTrue(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "Expected true: " << message << "\n";
        std::exit(1);
    }
}

void ExpectNear(double actual, double expected, double tolerance, const std::string& message) {
    if (std::abs(actual - expected) > tolerance) {
        std::cerr << "Expected near: " << message << ", actual=" << actual << ", expected=" << expected
                  << ", tolerance=" << tolerance << "\n";
        std::exit(1);
    }
}

void ExpectContains(const std::string& text, const std::string& pattern, const std::string& message) {
    if (text.find(pattern) == std::string::npos) {
        std::cerr << "Expected text to contain: " << message << ", text=" << text << ", pattern=" << pattern << "\n";
        std::exit(1);
    }
}

void ExpectStatus(couple_joint::SolveStatus actual, couple_joint::SolveStatus expected, const std::string& message) {
    if (actual != expected) {
        std::cerr << "Expected status: " << message << ", actual=" << static_cast<int>(actual)
                  << ", expected=" << static_cast<int>(expected) << "\n";
        std::exit(1);
    }
}

void ExpectCalibrationStatus(couple_joint::CalibrationStatus actual, couple_joint::CalibrationStatus expected,
                             const std::string& message) {
    if (actual != expected) {
        std::cerr << "Expected calibration status: " << message << ", actual=" << static_cast<int>(actual)
                  << ", expected=" << static_cast<int>(expected) << "\n";
        std::exit(1);
    }
}

void ExpectUsableStatus(couple_joint::SolveStatus actual, const std::string& message) {
    if (actual != couple_joint::SolveStatus::Success && actual != couple_joint::SolveStatus::NearSingular) {
        std::cerr << "Expected usable status: " << message << ", actual=" << static_cast<int>(actual) << "\n";
        std::exit(1);
    }
}

couple_joint::Point3 Subtract(const couple_joint::Point3& lhs, const couple_joint::Point3& rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

couple_joint::Point3 Add(const couple_joint::Point3& lhs, const couple_joint::Point3& rhs) {
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

couple_joint::Point3 Cross(const couple_joint::Point3& lhs, const couple_joint::Point3& rhs) {
    return {lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x};
}

double Dot(const couple_joint::Point3& lhs, const couple_joint::Point3& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

double Norm(const couple_joint::Point3& value) {
    return std::sqrt(Dot(value, value));
}

couple_joint::Point3 Normalize(couple_joint::Point3 value) {
    const double norm = Norm(value);
    return {value.x / norm, value.y / norm, value.z / norm};
}

couple_joint::Point3 RotateAroundAxis(const couple_joint::Point3& center, const couple_joint::Point3& axis,
                                      double angle, const couple_joint::Point3& point) {
    const couple_joint::Point3 normalized_axis = Normalize(axis);
    const couple_joint::Point3 value = Subtract(point, center);
    const couple_joint::Point3 cross = Cross(normalized_axis, value);
    const double dot = Dot(normalized_axis, value);
    const double cos_angle = std::cos(angle);
    const double sin_angle = std::sin(angle);
    return Add(center, {value.x * cos_angle + cross.x * sin_angle + normalized_axis.x * dot * (1.0 - cos_angle),
                        value.y * cos_angle + cross.y * sin_angle + normalized_axis.y * dot * (1.0 - cos_angle),
                        value.z * cos_angle + cross.z * sin_angle + normalized_axis.z * dot * (1.0 - cos_angle)});
}

couple_joint::Point3 RotateVectorAroundAxis(const couple_joint::Point3& axis, double angle,
                                            const couple_joint::Point3& value) {
    const couple_joint::Point3 normalized_axis = Normalize(axis);
    const couple_joint::Point3 cross = Cross(normalized_axis, value);
    const double dot = Dot(normalized_axis, value);
    const double cos_angle = std::cos(angle);
    const double sin_angle = std::sin(angle);
    return {value.x * cos_angle + cross.x * sin_angle + normalized_axis.x * dot * (1.0 - cos_angle),
            value.y * cos_angle + cross.y * sin_angle + normalized_axis.y * dot * (1.0 - cos_angle),
            value.z * cos_angle + cross.z * sin_angle + normalized_axis.z * dot * (1.0 - cos_angle)};
}

couple_joint::Point3 TransformFromCanonical(const couple_joint::Point3& point, const couple_joint::Point3& origin,
                                            const couple_joint::Point3& axis_x, const couple_joint::Point3& axis_y,
                                            const couple_joint::Point3& axis_z) {
    return {
        origin.x + point.x * axis_x.x + point.y * axis_y.x + point.z * axis_z.x,
        origin.y + point.x * axis_x.y + point.y * axis_y.y + point.z * axis_z.y,
        origin.z + point.x * axis_x.z + point.y * axis_y.z + point.z * axis_z.z,
    };
}

void ExpectPointNear(const couple_joint::Point3& actual, const couple_joint::Point3& expected, double tolerance,
                     const std::string& message) {
    ExpectNear(actual.x, expected.x, tolerance, message + " x");
    ExpectNear(actual.y, expected.y, tolerance, message + " y");
    ExpectNear(actual.z, expected.z, tolerance, message + " z");
}

couple_joint::CalibrationInput CreateValidCalibration() {
    couple_joint::CalibrationInput input;
    input.left_joint_center = {-35.462080740, 29.491263548, 58.546474855};
    input.right_joint_center = {-35.462080740, -29.491263548, 58.546474855};
    input.left_planar_endpoint = {-35.5, 11.61629736, 56.42889465};
    input.right_planar_endpoint = {-35.5, -11.61629736, 56.42889465};
    input.roll_center = {0.0, 0.0, 0.0};
    input.pitch_center = {0.0, 0.0, 0.0};
    input.left_spatial_endpoint = {-36.6, 26.0, -5.0};
    input.right_spatial_endpoint = {-36.6, -26.0, -5.0};
    input.rotation_order = couple_joint::RotationOrder::PitchThenRoll;
    return input;
}

couple_joint::CalibrationInput CreateSplitCenterCalibration(couple_joint::RotationOrder rotation_order) {
    couple_joint::CalibrationInput input;
    input.left_joint_center = {-30.0, 0.0, 0.0};
    input.right_joint_center = {30.0, 0.0, 0.0};
    input.left_planar_endpoint = {-20.0, 10.0, 0.0};
    input.right_planar_endpoint = {20.0, 10.0, 0.0};
    input.roll_center = {0.0, 0.0, 0.0};
    input.pitch_center = {0.0, 0.0, 10.0};
    input.left_spatial_endpoint = {-10.0, 5.0, 20.0};
    input.right_spatial_endpoint = {10.0, 5.0, 20.0};
    input.rotation_order = rotation_order;
    return input;
}

couple_joint::CalibrationInput CreateTiltedNormalCalibration() {
    const couple_joint::CalibrationInput canonical =
        CreateSplitCenterCalibration(couple_joint::RotationOrder::PitchThenRoll);
    const couple_joint::Point3 origin = {5.0, -7.0, 11.0};
    const couple_joint::Point3 axis_x = Normalize({1.0, 2.0, 0.0});
    const couple_joint::Point3 axis_z = Normalize({-2.0, 1.0, 3.0});
    const couple_joint::Point3 axis_y = Cross(axis_z, axis_x);

    couple_joint::CalibrationInput input;
    input.left_joint_center = TransformFromCanonical(canonical.left_joint_center, origin, axis_x, axis_y, axis_z);
    input.right_joint_center = TransformFromCanonical(canonical.right_joint_center, origin, axis_x, axis_y, axis_z);
    input.left_planar_endpoint = TransformFromCanonical(canonical.left_planar_endpoint, origin, axis_x, axis_y, axis_z);
    input.right_planar_endpoint =
        TransformFromCanonical(canonical.right_planar_endpoint, origin, axis_x, axis_y, axis_z);
    input.roll_center = TransformFromCanonical(canonical.roll_center, origin, axis_x, axis_y, axis_z);
    input.pitch_center = TransformFromCanonical(canonical.pitch_center, origin, axis_x, axis_y, axis_z);
    input.left_spatial_endpoint =
        TransformFromCanonical(canonical.left_spatial_endpoint, origin, axis_x, axis_y, axis_z);
    input.right_spatial_endpoint =
        TransformFromCanonical(canonical.right_spatial_endpoint, origin, axis_x, axis_y, axis_z);
    input.rotate_direction = axis_z;
    input.use_rotate_direction = true;
    input.rotation_order = couple_joint::RotationOrder::PitchThenRoll;
    return input;
}

couple_joint::Point3 DerivedRotateDirectionFromLeftSide(const couple_joint::CalibrationInput& input) {
    return Normalize(Cross(Subtract(input.left_joint_center, input.left_planar_endpoint),
                           Subtract(input.left_joint_center, input.right_joint_center)));
}

void TestCalibratesInitialPose() {
    couple_joint::MechanismSolver solver(CreateValidCalibration());

    ExpectCalibrationStatus(solver.GetCalibrationStatus(), couple_joint::CalibrationStatus::Success,
                            "valid calibration status");

    const couple_joint::MechanismParameters parameters = solver.GetParameters();
    ExpectNear(parameters.base_distance, 58.982527095, kTolerance, "base distance");
    ExpectNear(parameters.left_planar_length, 18.0, kTolerance, "left planar length");
    ExpectNear(parameters.right_planar_length, 18.0, kTolerance, "right planar length");
    ExpectNear(parameters.left_spatial_length, 63.1, kTolerance, "left spatial length");
    ExpectNear(parameters.right_spatial_length, 63.1, kTolerance, "right spatial length");

    const couple_joint::CalibrationInput input = CreateValidCalibration();
    const couple_joint::Point3 op_normal = Cross(Subtract(input.right_joint_center, input.left_joint_center),
                                                 Subtract(input.left_planar_endpoint, input.left_joint_center));
    const couple_joint::Point3 ps_normal = Cross(Subtract(input.right_planar_endpoint, input.left_planar_endpoint),
                                                 Subtract(input.left_spatial_endpoint, input.left_planar_endpoint));
    const double normal_cosine = std::abs(Dot(op_normal, ps_normal)) / (Norm(op_normal) * Norm(ps_normal));
    ExpectNear(normal_cosine, 1.0, 1.0e-9, "O_P and P_S planes are parallel");

    const couple_joint::MechanismState state = solver.GetState();
    ExpectNear(state.roll, 0.0, kTolerance, "initial roll");
    ExpectNear(state.pitch, 0.0, kTolerance, "initial pitch");
    ExpectNear(state.theta1, -0.117935404, kTolerance, "initial theta1");
    ExpectNear(state.theta2, 0.117935404, kTolerance, "initial theta2");
}

void TestRotateDirectionIsPlanarNormal() {
    couple_joint::MechanismSolver solver(CreateTiltedNormalCalibration());
    ExpectCalibrationStatus(solver.GetCalibrationStatus(), couple_joint::CalibrationStatus::Success,
                            "tilted normal calibration status");

    const couple_joint::MechanismGeometry geometry = solver.GetGeometry();
    ExpectPointNear(geometry.roll_axis, {0.0, 0.0, 1.0}, 1.0e-8, "roll axis is canonical planar normal");
    ExpectPointNear(geometry.planar_direction, {0.0, 0.0, 1.0}, 1.0e-8, "planar direction stores planar normal");
    ExpectNear(geometry.left_planar_endpoint.z, 0.0, 1.0e-8, "left planar endpoint is in canonical plane");
    ExpectNear(geometry.right_planar_endpoint.z, 0.0, 1.0e-8, "right planar endpoint is in canonical plane");
}

void TestDerivedRotateDirectionMatchesExplicitAxis() {
    const couple_joint::CalibrationInput implicit_input = CreateValidCalibration();
    couple_joint::CalibrationInput explicit_input = implicit_input;
    explicit_input.rotate_direction = DerivedRotateDirectionFromLeftSide(implicit_input);
    explicit_input.use_rotate_direction = true;

    couple_joint::MechanismSolver implicit_solver(implicit_input);
    couple_joint::MechanismSolver explicit_solver(explicit_input);
    ExpectCalibrationStatus(implicit_solver.GetCalibrationStatus(), couple_joint::CalibrationStatus::Success,
                            "implicit rotate direction calibration status");
    ExpectCalibrationStatus(explicit_solver.GetCalibrationStatus(), couple_joint::CalibrationStatus::Success,
                            "explicit rotate direction calibration status");

    const couple_joint::MechanismState implicit_state = implicit_solver.GetState();
    const couple_joint::MechanismState explicit_state = explicit_solver.GetState();
    ExpectNear(implicit_state.theta1, explicit_state.theta1, kTolerance, "implicit theta1 matches explicit axis");
    ExpectNear(implicit_state.theta2, explicit_state.theta2, kTolerance, "implicit theta2 matches explicit axis");

    const couple_joint::SolveResult implicit_result = implicit_solver.SolveInverse(0.03, -0.02);
    const couple_joint::SolveResult explicit_result = explicit_solver.SolveInverse(0.03, -0.02);
    ExpectUsableStatus(implicit_result.status, "implicit inverse status");
    ExpectUsableStatus(explicit_result.status, "explicit inverse status");

    const couple_joint::MechanismState implicit_inverse_state = implicit_solver.GetState();
    const couple_joint::MechanismState explicit_inverse_state = explicit_solver.GetState();
    ExpectNear(implicit_inverse_state.theta1, explicit_inverse_state.theta1, 1.0e-5,
               "implicit inverse theta1 matches explicit axis");
    ExpectNear(implicit_inverse_state.theta2, explicit_inverse_state.theta2, 1.0e-5,
               "implicit inverse theta2 matches explicit axis");
}

void TestBothMotorPositiveDirectionsFollowRightHandRule() {
    couple_joint::MechanismSolver solver(CreateValidCalibration());
    ExpectCalibrationStatus(solver.GetCalibrationStatus(), couple_joint::CalibrationStatus::Success,
                            "positive direction calibration status");

    const couple_joint::MechanismState initial_state = solver.GetState();
    const couple_joint::MechanismGeometry initial_geometry = solver.GetGeometry();
    const double delta = 1.0e-4;

    ExpectUsableStatus(solver.SolveForward(initial_state.theta1 + delta, initial_state.theta2 + delta).status,
                       "forward solve with positive motor deltas");
    const couple_joint::MechanismGeometry moved_geometry = solver.GetGeometry();

    const couple_joint::Point3 axis = {0.0, 0.0, 1.0};
    const couple_joint::Point3 left_radius =
        Subtract(initial_geometry.left_planar_endpoint, initial_geometry.left_joint_center);
    const couple_joint::Point3 right_radius =
        Subtract(initial_geometry.right_planar_endpoint, initial_geometry.right_joint_center);
    const couple_joint::Point3 left_expected_tangent = Cross(axis, left_radius);
    const couple_joint::Point3 right_expected_tangent = Cross(axis, right_radius);
    const couple_joint::Point3 left_delta =
        Subtract(moved_geometry.left_planar_endpoint, initial_geometry.left_planar_endpoint);
    const couple_joint::Point3 right_delta =
        Subtract(moved_geometry.right_planar_endpoint, initial_geometry.right_planar_endpoint);

    ExpectTrue(Dot(left_delta, left_expected_tangent) > 0.0, "left motor positive rotation follows right-hand rule");
    ExpectTrue(Dot(right_delta, right_expected_tangent) > 0.0,
               "right motor positive rotation follows right-hand rule");
}

void TestInverseAndForwardRemainConsistent() {
    couple_joint::MechanismSolver solver(CreateValidCalibration());
    ExpectCalibrationStatus(solver.GetCalibrationStatus(), couple_joint::CalibrationStatus::Success,
                            "inverse forward calibration status");

    const couple_joint::SolveResult inverse_result = solver.SolveInverse(0.03, -0.02);
    ExpectStatus(inverse_result.status, couple_joint::SolveStatus::Success, "inverse result");
    const couple_joint::MechanismState inverse_state = solver.GetState();
    ExpectNear(inverse_state.roll, 0.03, kTolerance, "inverse roll");
    ExpectNear(inverse_state.pitch, -0.02, kTolerance, "inverse pitch");

    const couple_joint::SolveResult forward_result = solver.SolveForward(inverse_state.theta1, inverse_state.theta2);
    ExpectStatus(forward_result.status, couple_joint::SolveStatus::Success, "forward result");
    const couple_joint::MechanismState forward_state = solver.GetState();
    ExpectNear(forward_state.roll, 0.03, 1.0e-5, "forward roll");
    ExpectNear(forward_state.pitch, -0.02, 1.0e-5, "forward pitch");
}

void TestTransmissionAndVelocityTorqueMappings() {
    couple_joint::MechanismSolver solver(CreateValidCalibration());
    ExpectCalibrationStatus(solver.GetCalibrationStatus(), couple_joint::CalibrationStatus::Success,
                            "transmission calibration status");

    const couple_joint::SolveResult position_result = solver.SolveInverse(0.03, -0.02);
    ExpectStatus(position_result.status, couple_joint::SolveStatus::Success, "position solve for transmission");
    const couple_joint::TransmissionInfo transmission = position_result.transmission;

    const double identity00 = transmission.velocity_jacobian.m00 * transmission.reduction.m00 +
                              transmission.velocity_jacobian.m01 * transmission.reduction.m10;
    const double identity01 = transmission.velocity_jacobian.m00 * transmission.reduction.m01 +
                              transmission.velocity_jacobian.m01 * transmission.reduction.m11;
    const double identity10 = transmission.velocity_jacobian.m10 * transmission.reduction.m00 +
                              transmission.velocity_jacobian.m11 * transmission.reduction.m10;
    const double identity11 = transmission.velocity_jacobian.m10 * transmission.reduction.m01 +
                              transmission.velocity_jacobian.m11 * transmission.reduction.m11;
    ExpectNear(identity00, 1.0, 1.0e-6, "J * reduction 00");
    ExpectNear(identity01, 0.0, 1.0e-6, "J * reduction 01");
    ExpectNear(identity10, 0.0, 1.0e-6, "J * reduction 10");
    ExpectNear(identity11, 1.0, 1.0e-6, "J * reduction 11");

    const couple_joint::VelocityTorqueResult forward_velocity = solver.SolveForwardVelocity(1.2, -0.4);
    ExpectStatus(forward_velocity.status, couple_joint::SolveStatus::Success, "forward velocity");
    ExpectNear(forward_velocity.roll,
               transmission.velocity_jacobian.m00 * 1.2 + transmission.velocity_jacobian.m01 * -0.4, 1.0e-9,
               "forward velocity roll");
    ExpectNear(forward_velocity.pitch,
               transmission.velocity_jacobian.m10 * 1.2 + transmission.velocity_jacobian.m11 * -0.4, 1.0e-9,
               "forward velocity pitch");

    const couple_joint::VelocityTorqueResult inverse_velocity =
        solver.SolveInverseVelocity(forward_velocity.roll, forward_velocity.pitch);
    ExpectStatus(inverse_velocity.status, couple_joint::SolveStatus::Success, "inverse velocity");
    ExpectNear(inverse_velocity.theta1, 1.2, 1.0e-6, "inverse velocity theta1");
    ExpectNear(inverse_velocity.theta2, -0.4, 1.0e-6, "inverse velocity theta2");

    const couple_joint::VelocityTorqueResult inverse_torque = solver.SolveInverseTorque(2.0, -0.75);
    ExpectStatus(inverse_torque.status, couple_joint::SolveStatus::Success, "inverse torque");
    ExpectNear(inverse_torque.theta1,
               transmission.velocity_jacobian.m00 * 2.0 + transmission.velocity_jacobian.m10 * -0.75, 1.0e-9,
               "inverse torque theta1");
    ExpectNear(inverse_torque.theta2,
               transmission.velocity_jacobian.m01 * 2.0 + transmission.velocity_jacobian.m11 * -0.75, 1.0e-9,
               "inverse torque theta2");

    const couple_joint::VelocityTorqueResult forward_torque =
        solver.SolveForwardTorque(inverse_torque.theta1, inverse_torque.theta2);
    ExpectStatus(forward_torque.status, couple_joint::SolveStatus::Success, "forward torque");
    ExpectNear(forward_torque.roll, 2.0, 1.0e-6, "forward torque roll");
    ExpectNear(forward_torque.pitch, -0.75, 1.0e-6, "forward torque pitch");
}

void ExpectPitchThenRollSplitCenterGeometry() {
    couple_joint::MechanismSolver solver(CreateSplitCenterCalibration(couple_joint::RotationOrder::PitchThenRoll));
    ExpectCalibrationStatus(solver.GetCalibrationStatus(), couple_joint::CalibrationStatus::Success,
                            "pitch then roll split center calibration status");

    const couple_joint::MechanismGeometry initial_geometry = solver.GetGeometry();

    const double roll = 0.08;
    const double pitch = 0.12;
    const couple_joint::SolveResult result = solver.SolveInverse(roll, pitch);
    ExpectUsableStatus(result.status, "pitch then roll split-center inverse");
    const couple_joint::MechanismGeometry geometry = solver.GetGeometry();

    const couple_joint::Point3 roll_axis = initial_geometry.roll_axis;
    const couple_joint::Point3 pitch_axis = initial_geometry.pitch_axis;
    const couple_joint::Point3 roll_center = initial_geometry.roll_center;
    const couple_joint::Point3 pitch_center = initial_geometry.pitch_center;
    const couple_joint::Point3 expected_roll_center = RotateAroundAxis(pitch_center, pitch_axis, pitch, roll_center);
    const couple_joint::Point3 expected_roll_axis = RotateVectorAroundAxis(pitch_axis, pitch, roll_axis);
    const couple_joint::Point3 after_pitch =
        RotateAroundAxis(pitch_center, pitch_axis, pitch, initial_geometry.left_spatial_endpoint);
    const couple_joint::Point3 expected_left =
        RotateAroundAxis(expected_roll_center, expected_roll_axis, roll, after_pitch);

    ExpectPointNear(geometry.pitch_center, pitch_center, 1.0e-8, "pitch center remains fixed");
    ExpectPointNear(geometry.pitch_axis, pitch_axis, 1.0e-8, "pitch axis remains fixed");
    ExpectPointNear(geometry.roll_center, expected_roll_center, 1.0e-8, "roll center follows pitch");
    ExpectPointNear(geometry.roll_axis, expected_roll_axis, 1.0e-8, "roll axis follows pitch");
    ExpectPointNear(geometry.left_spatial_endpoint, expected_left, 1.0e-8, "left point after pitch then roll");
}

void ExpectRollThenPitchSplitCenterGeometry() {
    couple_joint::MechanismSolver solver(CreateSplitCenterCalibration(couple_joint::RotationOrder::RollThenPitch));
    ExpectCalibrationStatus(solver.GetCalibrationStatus(), couple_joint::CalibrationStatus::Success,
                            "roll then pitch split center calibration status");

    const couple_joint::MechanismGeometry initial_geometry = solver.GetGeometry();

    const double roll = 0.08;
    const double pitch = 0.12;
    const couple_joint::SolveResult result = solver.SolveInverse(roll, pitch);
    ExpectUsableStatus(result.status, "roll then pitch split-center inverse");
    const couple_joint::MechanismGeometry geometry = solver.GetGeometry();

    const couple_joint::Point3 roll_axis = initial_geometry.roll_axis;
    const couple_joint::Point3 pitch_axis = initial_geometry.pitch_axis;
    const couple_joint::Point3 roll_center = initial_geometry.roll_center;
    const couple_joint::Point3 pitch_center = initial_geometry.pitch_center;
    const couple_joint::Point3 expected_pitch_center = RotateAroundAxis(roll_center, roll_axis, roll, pitch_center);
    const couple_joint::Point3 expected_pitch_axis = RotateVectorAroundAxis(roll_axis, roll, pitch_axis);
    const couple_joint::Point3 after_roll =
        RotateAroundAxis(roll_center, roll_axis, roll, initial_geometry.left_spatial_endpoint);
    const couple_joint::Point3 expected_left =
        RotateAroundAxis(expected_pitch_center, expected_pitch_axis, pitch, after_roll);

    ExpectPointNear(geometry.roll_center, roll_center, 1.0e-8, "roll center remains fixed");
    ExpectPointNear(geometry.roll_axis, roll_axis, 1.0e-8, "roll axis remains fixed");
    ExpectPointNear(geometry.pitch_center, expected_pitch_center, 1.0e-8, "pitch center follows roll");
    ExpectPointNear(geometry.pitch_axis, expected_pitch_axis, 1.0e-8, "pitch axis follows roll");
    ExpectPointNear(geometry.left_spatial_endpoint, expected_left, 1.0e-8, "left point after roll then pitch");
}

void TestRejectsDegenerateCalibration() {
    couple_joint::CalibrationInput input = CreateValidCalibration();
    input.rotate_direction = Cross(Subtract(input.right_joint_center, input.left_joint_center),
                                   Subtract(input.right_planar_endpoint, input.right_joint_center));
    input.left_planar_endpoint = input.left_joint_center;
    input.use_rotate_direction = true;

    couple_joint::MechanismSolver solver(input);

    ExpectCalibrationStatus(solver.GetCalibrationStatus(), couple_joint::CalibrationStatus::ZeroLengthLink,
                            "degenerate calibration should fail with zero length link");
    ExpectContains(couple_joint::MechanismSolver::StatusMessage(solver.GetCalibrationStatus()), "zero length",
                   "degenerate calibration should provide a message");
}

void TestRejectsNonCoplanarPlanarLinkInputs() {
    couple_joint::CalibrationInput input = CreateValidCalibration();
    input.right_planar_endpoint.z += 1.0;

    couple_joint::MechanismSolver solver(input);

    ExpectCalibrationStatus(solver.GetCalibrationStatus(), couple_joint::CalibrationStatus::PlanarPointsNotCoplanar,
                            "non-coplanar O1/O2/P1/P2 should fail");
    ExpectContains(couple_joint::MechanismSolver::StatusMessage(solver.GetCalibrationStatus()), "coplanar",
                   "non-coplanar calibration should report coplanar error");
}

void TestUnreachableInverseDoesNotOverwriteState() {
    couple_joint::MechanismSolver solver(CreateValidCalibration());
    ExpectCalibrationStatus(solver.GetCalibrationStatus(), couple_joint::CalibrationStatus::Success,
                            "unreachable inverse calibration status");
    const couple_joint::MechanismState before = solver.GetState();

    const couple_joint::SolveResult result = solver.SolveInverse(-3.0, -3.0);

    ExpectStatus(result.status, couple_joint::SolveStatus::UnreachablePose, "unreachable inverse should fail");
    const couple_joint::MechanismState after = solver.GetState();
    ExpectNear(after.theta1, before.theta1, kTolerance, "theta1 remains unchanged");
    ExpectNear(after.theta2, before.theta2, kTolerance, "theta2 remains unchanged");
    ExpectNear(after.roll, before.roll, kTolerance, "roll remains unchanged");
    ExpectNear(after.pitch, before.pitch, kTolerance, "pitch remains unchanged");
}

void TestNearSingularPositionSolveReportsStatusButUpdatesState() {
    couple_joint::MechanismSolver solver(CreateValidCalibration());
    ExpectCalibrationStatus(solver.GetCalibrationStatus(), couple_joint::CalibrationStatus::Success,
                            "near singular calibration status");

    const couple_joint::SolveResult result = solver.SolveInverse(-3.14, -1.15);

    ExpectStatus(result.status, couple_joint::SolveStatus::NearSingular, "near singular inverse status");
    ExpectTrue(result.transmission.near_singular, "near singular transmission flag");
    const couple_joint::MechanismState state = solver.GetState();
    ExpectNear(state.roll, -3.14, kTolerance, "near singular solve updates roll");
    ExpectNear(state.pitch, -1.15, kTolerance, "near singular solve updates pitch");
}

void TestUpdatedNearSingularThresholdsTriggerWarning() {
    couple_joint::MechanismSolver solver(CreateValidCalibration());
    ExpectCalibrationStatus(solver.GetCalibrationStatus(), couple_joint::CalibrationStatus::Success,
                            "updated threshold calibration status");

    const couple_joint::SolveResult result = solver.SolveInverse(-3.14, -1.7);

    ExpectStatus(result.status, couple_joint::SolveStatus::NearSingular, "updated near singular threshold status");
    ExpectTrue(result.transmission.near_singular, "updated near singular threshold flag");
    ExpectTrue(result.transmission.sigma_min_j < 0.2, "updated near singular threshold uses sigma min");
}

}  // namespace

int main() {
    TestCalibratesInitialPose();
    TestRotateDirectionIsPlanarNormal();
    TestDerivedRotateDirectionMatchesExplicitAxis();
    TestBothMotorPositiveDirectionsFollowRightHandRule();
    TestInverseAndForwardRemainConsistent();
    TestTransmissionAndVelocityTorqueMappings();
    ExpectPitchThenRollSplitCenterGeometry();
    ExpectRollThenPitchSplitCenterGeometry();
    TestRejectsDegenerateCalibration();
    TestRejectsNonCoplanarPlanarLinkInputs();
    TestUnreachableInverseDoesNotOverwriteState();
    TestNearSingularPositionSolveReportsStatusButUpdatesState();
    TestUpdatedNearSingularThresholdsTriggerWarning();
    return 0;
}
