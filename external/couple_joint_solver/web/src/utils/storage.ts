import type { AngleLimit, CalibrationInput, LegacyCalibrationInput, LegacyStoredConfig, Point3, StoredConfig } from '../types';
import { rotationOrder } from './constants';

export const storageKey = 'coupleJoint:calibration:v5';
export const legacyStorageKeys = ['coupleJoint:calibration:v4', 'coupleJoint:calibration:v3'];

export const defaultConfig: StoredConfig = {
  calibration: {
    leftJointCenter: { x: -35.46208074, y: 29.491263548, z: 58.546474855 },
    rightJointCenter: { x: -35.46208074, y: -29.491263548, z: 58.546474855 },
    leftPlanarEndpoint: { x: -35.5, y: 11.61629736, z: 56.42889465 },
    rightPlanarEndpoint: { x: -35.5, y: -11.61629736, z: 56.42889465 },
    rollCenter: { x: 0, y: 0, z: 0 },
    pitchCenter: { x: 0, y: 0, z: 0 },
    leftSpatialEndpoint: { x: -36.6, y: 26, z: -5 },
    rightSpatialEndpoint: { x: -36.6, y: -26, z: -5 },
    rotateDirection: { x: 0, y: 0, z: 0 },
    useRotateDirection: false,
    rotationOrder: rotationOrder.pitchThenRoll,
  },
  rollLimit: { min: null, max: null },
  pitchLimit: { min: null, max: null },
};

export function isPointLike(value: unknown): value is Point3 {
  const point = value as Point3;
  return (
    typeof point?.x === 'number' &&
    Number.isFinite(point.x) &&
    typeof point.y === 'number' &&
    Number.isFinite(point.y) &&
    typeof point.z === 'number' &&
    Number.isFinite(point.z)
  );
}

export function normalizePoint(value: unknown, fallback: Point3): Point3 {
  return isPointLike(value) ? { x: value.x, y: value.y, z: value.z } : fallback;
}

export function normalizeAngleLimit(value: unknown): AngleLimit {
  const limit = value as Partial<AngleLimit>;
  return {
    min: typeof limit?.min === 'number' && Number.isFinite(limit.min) ? limit.min : null,
    max: typeof limit?.max === 'number' && Number.isFinite(limit.max) ? limit.max : null,
  };
}

export function normalizeCalibrationInput(calibration: LegacyCalibrationInput): CalibrationInput {
  const fallback = defaultConfig.calibration;
  const oldCenter = normalizePoint(calibration.centerJoint, fallback.rollCenter);
  const order = calibration.rotationOrder;
  return {
    leftJointCenter: normalizePoint(calibration.leftJointCenter, fallback.leftJointCenter),
    rightJointCenter: normalizePoint(calibration.rightJointCenter, fallback.rightJointCenter),
    leftPlanarEndpoint: normalizePoint(calibration.leftPlanarEndpoint, fallback.leftPlanarEndpoint),
    rightPlanarEndpoint: normalizePoint(calibration.rightPlanarEndpoint, fallback.rightPlanarEndpoint),
    rollCenter: normalizePoint(calibration.rollCenter, oldCenter),
    pitchCenter: normalizePoint(calibration.pitchCenter, oldCenter),
    leftSpatialEndpoint: normalizePoint(calibration.leftSpatialEndpoint, fallback.leftSpatialEndpoint),
    rightSpatialEndpoint: normalizePoint(calibration.rightSpatialEndpoint, fallback.rightSpatialEndpoint),
    rotateDirection: normalizePoint(calibration.rotateDirection, fallback.rotateDirection),
    useRotateDirection: calibration.useRotateDirection ?? false,
    rotationOrder:
      order === rotationOrder.pitchThenRoll || order === rotationOrder.rollThenPitch
        ? order
        : rotationOrder.pitchThenRoll,
  };
}

export function normalizeStoredConfig(value: unknown): StoredConfig | null {
  const parsed = value as LegacyStoredConfig;
  if (!parsed?.calibration) {
    return null;
  }
  return {
    calibration: normalizeCalibrationInput(parsed.calibration),
    rollLimit: normalizeAngleLimit(parsed.rollLimit),
    pitchLimit: normalizeAngleLimit(parsed.pitchLimit),
  };
}

export function parseAndMigrateConfig(raw: string): StoredConfig | null {
  try {
    return normalizeStoredConfig(JSON.parse(raw));
  } catch {
    return null;
  }
}

export function serializeConfig(config: StoredConfig): string {
  return `${JSON.stringify({ ...config, calibration: normalizeCalibrationInput(config.calibration) }, null, 2)}\n`;
}

export function loadStoredConfig(): StoredConfig | null {
  const raw = window.localStorage.getItem(storageKey);
  if (raw) {
    return parseAndMigrateConfig(raw);
  }
  for (const legacyKey of legacyStorageKeys) {
    const legacyRaw = window.localStorage.getItem(legacyKey);
    const migrated = legacyRaw ? parseAndMigrateConfig(legacyRaw) : null;
    if (migrated) {
      window.localStorage.setItem(storageKey, serializeConfig(migrated));
      return migrated;
    }
  }
  return null;
}
