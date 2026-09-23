import type { CalibrationStatus, SolveStatus } from '../types';
import { calibrationStatus, degreesPerRadian, radiansPerDegree, solveStatus } from './constants';

export function toDegrees(value: number): number {
  return value * degreesPerRadian;
}

export function toRadians(value: number): number {
  return value * radiansPerDegree;
}

export function formatDegrees(value: number): string {
  return toDegrees(value).toFixed(2);
}

export function formatScalar(value: number): string {
  if (Number.isNaN(value)) {
    return 'nan';
  }
  if (!Number.isFinite(value)) {
    return value > 0 ? 'inf' : '-inf';
  }
  const magnitude = Math.abs(value);
  if (magnitude > 0 && (magnitude < 0.001 || magnitude >= 1000)) {
    return value.toExponential(3);
  }
  return value.toFixed(4);
}

export function statusValue(status: SolveStatus | CalibrationStatus): number {
  return typeof status === 'number' ? status : status.value;
}

export function canUseSolveResult(status: SolveStatus): boolean {
  const value = statusValue(status);
  return value === solveStatus.success || value === solveStatus.nearSingular;
}

export function statusMessage(status: SolveStatus | CalibrationStatus, kind: 'solve' | 'calibration'): string {
  const value = statusValue(status);
  if (kind === 'solve') {
    switch (value) {
      case solveStatus.success:
        return '';
      case solveStatus.nearSingular:
        return '当前位姿接近奇异，位置已更新，但速度和力矩映射需要限幅。';
      case solveStatus.invalidCalibration:
        return '解算器标定无效，请返回配置页检查机构参数。';
      case solveStatus.degenerateSideEquation:
        return '逆解失败：单侧方程退化。';
      case solveStatus.unreachablePose:
        return '逆解失败：目标位姿超出可达空间。';
      case solveStatus.singularJacobian:
        return '正解失败：当前雅可比矩阵奇异。';
      case solveStatus.forwardNotConverged:
        return '正解失败：迭代未收敛。';
      default:
        return `未知解算状态：${value}`;
    }
  }

  switch (value) {
    case calibrationStatus.success:
      return '';
    case calibrationStatus.coincidentJointCenters:
      return '标定失败：左右输入转轴中心重合。';
    case calibrationStatus.degenerateRotateDirection:
      return '标定失败：rotate direction 退化。';
    case calibrationStatus.baseAxisNotPerpendicularToRotateDirection:
      return '标定失败：O1_O2 必须与 rotate direction 垂直。';
    case calibrationStatus.leftPlanarLinkNotPerpendicularToRotateDirection:
      return '标定失败：O1_P1 必须与 rotate direction 垂直。';
    case calibrationStatus.rightPlanarLinkNotPerpendicularToRotateDirection:
      return '标定失败：O2_P2 必须与 rotate direction 垂直。';
    case calibrationStatus.collinearPlanarLinksWithBaseAxis:
      return '标定失败：初始平面连杆与基轴共线，无法建立稳定姿态。';
    case calibrationStatus.unstableCanonicalFrame:
      return '标定失败：无法建立稳定的规范坐标系。';
    case calibrationStatus.unsupportedRotationOrder:
      return '标定失败：rotation order 未设置为支持的枚举值。';
    case calibrationStatus.degeneratePitchAxis:
      return '标定失败：pitch 轴退化，请检查空间连接点。';
    case calibrationStatus.rollAxisNotPerpendicularToPitchAxis:
      return '标定失败：roll 轴与 pitch 轴不够垂直。';
    case calibrationStatus.zeroLengthLink:
      return '标定失败：存在零长度连杆。';
    case calibrationStatus.initialPoseConstraintViolation:
      return '标定失败：零位姿态不满足闭链约束。';
    case calibrationStatus.planarPointsNotCoplanar:
      return '标定失败：O1、O2、P1、P2 必须共面。';
    default:
      return `未知标定状态：${value}`;
  }
}
