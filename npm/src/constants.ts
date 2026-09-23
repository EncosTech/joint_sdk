// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

export { MotorModel } from './motor-model.generated.js'

export enum PVTControlMode {
  MotorPVT = 0,
  HostTorque = 1,
  HostKpMotorKd = 2,
}

export enum MotorStopMode {
  FullBrake = 2,
  DynamicBrake = 3,
  RegenerativeBrake = 4,
}

export enum FakeReplyMode {
  Automatic = 0,
  Manual = 1,
}

export enum FakeCommandKind {
  Unknown = 0,
  PVTControl = 1,
  PosControl = 2,
  SpdControl = 3,
  CurControl = 4,
  TorControl = 5,
  Stop = 6,
  Brake = 7,
  SetId = 8,
  SetPos = 9,
  ResetZeroPos = 10,
  GetParameter = 11,
  SetParameter = 12,
}

export enum JointError {
  DoubleCoupleJointNotPaired = 0,
  DoubleCoupleJointDuplicatePair = 1,
  DoubleCoupleJointSameMotor = 2,
  DoubleCoupleJointPending = 3,
  DoubleCoupleJointDuplicateAxis = 4,
  DoubleCoupleJointCalibrationMismatch = 5,
  DoubleCoupleJointCalibrationFailed = 6,
  DoubleCoupleJointSolveFailed = 7,
  DoubleCoupleJointUnsupportedControl = 8,
  DoubleCoupleJointZeroPoseNotSet = 9,
  DoubleCoupleJointMotorLimitMismatch = 10,
  ContinuousJointUnsupportedOperation = 11,
}

export const LogLevel = {
  Trace: 0,
  Debug: 1,
  Info: 2,
  Warn: 3,
  Error: 4,
  Critical: 5,
  Off: 6,
} as const
