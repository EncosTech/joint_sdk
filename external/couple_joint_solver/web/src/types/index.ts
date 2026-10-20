import * as THREE from 'three';

export type Point3 = {
  x: number;
  y: number;
  z: number;
};

export type CalibrationInput = {
  leftJointCenter: Point3;
  rightJointCenter: Point3;
  leftPlanarEndpoint: Point3;
  rightPlanarEndpoint: Point3;
  rollCenter: Point3;
  pitchCenter: Point3;
  leftSpatialEndpoint: Point3;
  rightSpatialEndpoint: Point3;
  rotateDirection: Point3;
  useRotateDirection: boolean;
  rotationOrder: number;
};

export type AngleLimit = {
  min: number | null;
  max: number | null;
};

export type StoredConfig = {
  calibration: CalibrationInput;
  rollLimit: AngleLimit;
  pitchLimit: AngleLimit;
};

export type LegacyCalibrationInput = Partial<CalibrationInput> & {
  centerJoint?: Point3;
};

export type LegacyStoredConfig = {
  calibration?: LegacyCalibrationInput;
  rollLimit?: Partial<AngleLimit>;
  pitchLimit?: Partial<AngleLimit>;
};

export type MechanismState = {
  theta1: number;
  theta2: number;
  roll: number;
  pitch: number;
};

export type MechanismGeometry = {
  leftJointCenter: Point3;
  rightJointCenter: Point3;
  leftPlanarEndpoint: Point3;
  rightPlanarEndpoint: Point3;
  rollCenter: Point3;
  pitchCenter: Point3;
  leftSpatialEndpoint: Point3;
  rightSpatialEndpoint: Point3;
  rollAxis: Point3;
  pitchAxis: Point3;
  planarDirection: Point3;
};

export type CalibrationStatus = number | { value: number };
export type SolveStatus = number | { value: number };

export type Matrix2 = {
  m00: number;
  m01: number;
  m10: number;
  m11: number;
};

export type TransmissionInfo = {
  fu: Matrix2;
  fq: Matrix2;
  velocityJacobian: Matrix2;
  reduction: Matrix2;
  sigmaMinFu: number;
  sigmaMinFq: number;
  sigmaMinJ: number;
  conditionJ: number;
  singularFu: boolean;
  singularFq: boolean;
  singularJ: boolean;
  nearSingular: boolean;
};

export type SolveResult = {
  status: SolveStatus;
  residual: number;
  iterations: number;
  transmission: TransmissionInfo;
};

export type VelocityTorqueResult = {
  status: SolveStatus;
  theta1: number;
  theta2: number;
  roll: number;
  pitch: number;
  transmission: TransmissionInfo;
};

export type WasmSolver = {
  getCalibrationStatus(): CalibrationStatus;
  getState(): MechanismState;
  getGeometry(): MechanismGeometry;
  getTransmission(): TransmissionInfo;
  solveInverse(roll: number, pitch: number): SolveResult;
  solveForward(theta1: number, theta2: number): SolveResult;
  solveForwardVelocity(theta1Dot: number, theta2Dot: number): VelocityTorqueResult;
  solveInverseVelocity(rollDot: number, pitchDot: number): VelocityTorqueResult;
  solveForwardTorque(theta1Torque: number, theta2Torque: number): VelocityTorqueResult;
  solveInverseTorque(rollTorque: number, pitchTorque: number): VelocityTorqueResult;
};

export type WasmModule = {
  MechanismSolver: new (input: CalibrationInput) => WasmSolver;
};

export type SolverCreationResult = {
  solver: WasmSolver | null;
  error: string;
};

export type CanonicalTransform = {
  origin: THREE.Vector3;
  axisX: THREE.Vector3;
  axisY: THREE.Vector3;
  axisZ: THREE.Vector3;
};
