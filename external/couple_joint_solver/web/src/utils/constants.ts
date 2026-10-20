export const radiansPerDegree = Math.PI / 180;
export const degreesPerRadian = 180 / Math.PI;
export const defaultAngleRangeDegrees = 30;

export const solveStatus = {
  success: 0,
  nearSingular: 1,
  invalidCalibration: 2,
  degenerateSideEquation: 3,
  unreachablePose: 4,
  singularJacobian: 5,
  forwardNotConverged: 6,
} as const;

export const calibrationStatus = {
  success: 0,
  coincidentJointCenters: 1,
  degenerateRotateDirection: 2,
  baseAxisNotPerpendicularToRotateDirection: 3,
  leftPlanarLinkNotPerpendicularToRotateDirection: 4,
  rightPlanarLinkNotPerpendicularToRotateDirection: 5,
  collinearPlanarLinksWithBaseAxis: 6,
  unstableCanonicalFrame: 7,
  unsupportedRotationOrder: 8,
  degeneratePitchAxis: 9,
  rollAxisNotPerpendicularToPitchAxis: 10,
  zeroLengthLink: 11,
  initialPoseConstraintViolation: 12,
  planarPointsNotCoplanar: 13,
} as const;

export const rotationOrder = {
  pitchThenRoll: 0,
  rollThenPitch: 1,
} as const;
