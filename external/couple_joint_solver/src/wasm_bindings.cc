#include <emscripten/bind.h>

#include "mechanism_solver.h"

EMSCRIPTEN_BINDINGS(couple_joint_solver) {
    emscripten::value_object<couple_joint::Point3>("Point3")
        .field("x", &couple_joint::Point3::x)
        .field("y", &couple_joint::Point3::y)
        .field("z", &couple_joint::Point3::z);

    emscripten::value_object<couple_joint::CalibrationInput>("CalibrationInput")
        .field("leftJointCenter", &couple_joint::CalibrationInput::left_joint_center)
        .field("rightJointCenter", &couple_joint::CalibrationInput::right_joint_center)
        .field("leftPlanarEndpoint", &couple_joint::CalibrationInput::left_planar_endpoint)
        .field("rightPlanarEndpoint", &couple_joint::CalibrationInput::right_planar_endpoint)
        .field("rollCenter", &couple_joint::CalibrationInput::roll_center)
        .field("pitchCenter", &couple_joint::CalibrationInput::pitch_center)
        .field("leftSpatialEndpoint", &couple_joint::CalibrationInput::left_spatial_endpoint)
        .field("rightSpatialEndpoint", &couple_joint::CalibrationInput::right_spatial_endpoint)
        .field("rotateDirection", &couple_joint::CalibrationInput::rotate_direction)
        .field("useRotateDirection", &couple_joint::CalibrationInput::use_rotate_direction)
        .field("rotationOrder", &couple_joint::CalibrationInput::rotation_order);

    emscripten::value_object<couple_joint::MechanismState>("MechanismState")
        .field("theta1", &couple_joint::MechanismState::theta1)
        .field("theta2", &couple_joint::MechanismState::theta2)
        .field("roll", &couple_joint::MechanismState::roll)
        .field("pitch", &couple_joint::MechanismState::pitch);

    emscripten::value_object<couple_joint::MechanismParameters>("MechanismParameters")
        .field("baseDistance", &couple_joint::MechanismParameters::base_distance)
        .field("leftPlanarLength", &couple_joint::MechanismParameters::left_planar_length)
        .field("rightPlanarLength", &couple_joint::MechanismParameters::right_planar_length)
        .field("leftSpatialLength", &couple_joint::MechanismParameters::left_spatial_length)
        .field("rightSpatialLength", &couple_joint::MechanismParameters::right_spatial_length)
        .field("rollCenter", &couple_joint::MechanismParameters::roll_center)
        .field("pitchCenter", &couple_joint::MechanismParameters::pitch_center)
        .field("leftZeroSpatialEndpoint", &couple_joint::MechanismParameters::left_zero_spatial_endpoint)
        .field("rightZeroSpatialEndpoint", &couple_joint::MechanismParameters::right_zero_spatial_endpoint)
        .field("initialTheta1", &couple_joint::MechanismParameters::initial_theta1)
        .field("initialTheta2", &couple_joint::MechanismParameters::initial_theta2);

    emscripten::value_object<couple_joint::MechanismGeometry>("MechanismGeometry")
        .field("leftJointCenter", &couple_joint::MechanismGeometry::left_joint_center)
        .field("rightJointCenter", &couple_joint::MechanismGeometry::right_joint_center)
        .field("leftPlanarEndpoint", &couple_joint::MechanismGeometry::left_planar_endpoint)
        .field("rightPlanarEndpoint", &couple_joint::MechanismGeometry::right_planar_endpoint)
        .field("rollCenter", &couple_joint::MechanismGeometry::roll_center)
        .field("pitchCenter", &couple_joint::MechanismGeometry::pitch_center)
        .field("leftSpatialEndpoint", &couple_joint::MechanismGeometry::left_spatial_endpoint)
        .field("rightSpatialEndpoint", &couple_joint::MechanismGeometry::right_spatial_endpoint)
        .field("rollAxis", &couple_joint::MechanismGeometry::roll_axis)
        .field("pitchAxis", &couple_joint::MechanismGeometry::pitch_axis)
        .field("planarDirection", &couple_joint::MechanismGeometry::planar_direction);

    emscripten::enum_<couple_joint::CalibrationStatus>("CalibrationStatus")
        .value("Success", couple_joint::CalibrationStatus::Success)
        .value("CoincidentJointCenters", couple_joint::CalibrationStatus::CoincidentJointCenters)
        .value("DegenerateRotateDirection", couple_joint::CalibrationStatus::DegenerateRotateDirection)
        .value("BaseAxisNotPerpendicularToRotateDirection",
               couple_joint::CalibrationStatus::BaseAxisNotPerpendicularToRotateDirection)
        .value("LeftPlanarLinkNotPerpendicularToRotateDirection",
               couple_joint::CalibrationStatus::LeftPlanarLinkNotPerpendicularToRotateDirection)
        .value("RightPlanarLinkNotPerpendicularToRotateDirection",
               couple_joint::CalibrationStatus::RightPlanarLinkNotPerpendicularToRotateDirection)
        .value("CollinearPlanarLinksWithBaseAxis", couple_joint::CalibrationStatus::CollinearPlanarLinksWithBaseAxis)
        .value("UnstableCanonicalFrame", couple_joint::CalibrationStatus::UnstableCanonicalFrame)
        .value("UnsupportedRotationOrder", couple_joint::CalibrationStatus::UnsupportedRotationOrder)
        .value("DegeneratePitchAxis", couple_joint::CalibrationStatus::DegeneratePitchAxis)
        .value("RollAxisNotPerpendicularToPitchAxis",
               couple_joint::CalibrationStatus::RollAxisNotPerpendicularToPitchAxis)
        .value("ZeroLengthLink", couple_joint::CalibrationStatus::ZeroLengthLink)
        .value("InitialPoseConstraintViolation", couple_joint::CalibrationStatus::InitialPoseConstraintViolation)
        .value("PlanarPointsNotCoplanar", couple_joint::CalibrationStatus::PlanarPointsNotCoplanar);

    emscripten::enum_<couple_joint::SolveStatus>("SolveStatus")
        .value("Success", couple_joint::SolveStatus::Success)
        .value("NearSingular", couple_joint::SolveStatus::NearSingular)
        .value("InvalidCalibration", couple_joint::SolveStatus::InvalidCalibration)
        .value("DegenerateSideEquation", couple_joint::SolveStatus::DegenerateSideEquation)
        .value("UnreachablePose", couple_joint::SolveStatus::UnreachablePose)
        .value("SingularJacobian", couple_joint::SolveStatus::SingularJacobian)
        .value("ForwardNotConverged", couple_joint::SolveStatus::ForwardNotConverged);

    emscripten::enum_<couple_joint::RotationOrder>("RotationOrder")
        .value("PitchThenRoll", couple_joint::RotationOrder::PitchThenRoll)
        .value("RollThenPitch", couple_joint::RotationOrder::RollThenPitch);

    emscripten::value_object<couple_joint::Matrix2>("Matrix2")
        .field("m00", &couple_joint::Matrix2::m00)
        .field("m01", &couple_joint::Matrix2::m01)
        .field("m10", &couple_joint::Matrix2::m10)
        .field("m11", &couple_joint::Matrix2::m11);

    emscripten::value_object<couple_joint::TransmissionInfo>("TransmissionInfo")
        .field("fu", &couple_joint::TransmissionInfo::fu)
        .field("fq", &couple_joint::TransmissionInfo::fq)
        .field("velocityJacobian", &couple_joint::TransmissionInfo::velocity_jacobian)
        .field("reduction", &couple_joint::TransmissionInfo::reduction)
        .field("sigmaMinFu", &couple_joint::TransmissionInfo::sigma_min_fu)
        .field("sigmaMinFq", &couple_joint::TransmissionInfo::sigma_min_fq)
        .field("sigmaMinJ", &couple_joint::TransmissionInfo::sigma_min_j)
        .field("conditionJ", &couple_joint::TransmissionInfo::condition_j)
        .field("singularFu", &couple_joint::TransmissionInfo::singular_fu)
        .field("singularFq", &couple_joint::TransmissionInfo::singular_fq)
        .field("singularJ", &couple_joint::TransmissionInfo::singular_j)
        .field("nearSingular", &couple_joint::TransmissionInfo::near_singular);

    emscripten::value_object<couple_joint::SolveResult>("SolveResult")
        .field("status", &couple_joint::SolveResult::status)
        .field("residual", &couple_joint::SolveResult::residual)
        .field("iterations", &couple_joint::SolveResult::iterations)
        .field("transmission", &couple_joint::SolveResult::transmission);

    emscripten::value_object<couple_joint::VelocityTorqueResult>("VelocityTorqueResult")
        .field("status", &couple_joint::VelocityTorqueResult::status)
        .field("theta1", &couple_joint::VelocityTorqueResult::theta1)
        .field("theta2", &couple_joint::VelocityTorqueResult::theta2)
        .field("roll", &couple_joint::VelocityTorqueResult::roll)
        .field("pitch", &couple_joint::VelocityTorqueResult::pitch)
        .field("transmission", &couple_joint::VelocityTorqueResult::transmission);

    emscripten::class_<couple_joint::MechanismSolver>("MechanismSolver")
        .constructor<const couple_joint::CalibrationInput&>()
        .function("getCalibrationStatus", &couple_joint::MechanismSolver::GetCalibrationStatus)
        .function("getParameters", &couple_joint::MechanismSolver::GetParameters)
        .function("getState", &couple_joint::MechanismSolver::GetState)
        .function("getGeometry", &couple_joint::MechanismSolver::GetGeometry)
        .function("getTransmission", &couple_joint::MechanismSolver::GetTransmission)
        .function("solveInverse", &couple_joint::MechanismSolver::SolveInverse)
        .function("solveForward", &couple_joint::MechanismSolver::SolveForward)
        .function("solveForwardVelocity", &couple_joint::MechanismSolver::SolveForwardVelocity)
        .function("solveInverseVelocity", &couple_joint::MechanismSolver::SolveInverseVelocity)
        .function("solveForwardTorque", &couple_joint::MechanismSolver::SolveForwardTorque)
        .function("solveInverseTorque", &couple_joint::MechanismSolver::SolveInverseTorque);
}
