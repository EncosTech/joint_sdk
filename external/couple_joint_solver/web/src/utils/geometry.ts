import * as THREE from 'three';
import type { CalibrationInput, CanonicalTransform, MechanismGeometry, Point3 } from '../types';

export function pointToVector(point: Point3): THREE.Vector3 {
  return new THREE.Vector3(point.x, point.y, point.z);
}

export function vectorToPoint(vector: THREE.Vector3): Point3 {
  return { x: vector.x, y: vector.y, z: vector.z };
}

export function buildCanonicalTransform(calibration: CalibrationInput): CanonicalTransform {
  const leftJoint = pointToVector(calibration.leftJointCenter);
  const rightJoint = pointToVector(calibration.rightJointCenter);
  const leftEndpoint = pointToVector(calibration.leftPlanarEndpoint);
  const rightEndpoint = pointToVector(calibration.rightPlanarEndpoint);
  const origin = leftJoint.clone().add(rightJoint).multiplyScalar(0.5);
  const axisX = rightJoint.clone().sub(leftJoint).normalize();
  const leftLink = leftEndpoint.clone().sub(leftJoint);
  const rightLink = rightEndpoint.clone().sub(rightJoint);
  let planarNormal = calibration.useRotateDirection
    ? pointToVector(calibration.rotateDirection)
    : new THREE.Vector3().crossVectors(leftLink, axisX);
  if (!calibration.useRotateDirection && planarNormal.lengthSq() < 1.0e-12) {
    planarNormal = new THREE.Vector3().crossVectors(rightLink, axisX);
  }
  const axisZ = planarNormal.addScaledVector(axisX, -planarNormal.dot(axisX)).normalize();
  const axisY = new THREE.Vector3().crossVectors(axisZ, axisX).normalize();
  return { origin, axisX, axisY, axisZ };
}

export function canonicalToImportedPoint(point: Point3, transform: CanonicalTransform): Point3 {
  const imported = transform.origin
    .clone()
    .addScaledVector(transform.axisX, point.x)
    .addScaledVector(transform.axisY, point.y)
    .addScaledVector(transform.axisZ, point.z);
  return vectorToPoint(imported);
}

export function canonicalToImportedVector(vector: Point3, transform: CanonicalTransform): Point3 {
  const imported = new THREE.Vector3()
    .addScaledVector(transform.axisX, vector.x)
    .addScaledVector(transform.axisY, vector.y)
    .addScaledVector(transform.axisZ, vector.z);
  return vectorToPoint(imported);
}

export function importedZUpToThreePoint(point: Point3): Point3 {
  return { x: point.x, y: point.z, z: -point.y };
}

export function importedZUpToThreeVector(vector: Point3): Point3 {
  return { x: vector.x, y: vector.z, z: -vector.y };
}

export function transformGeometryForPreview(geometry: MechanismGeometry, calibration: CalibrationInput): MechanismGeometry {
  const transform = buildCanonicalTransform(calibration);
  const convert = (point: Point3) => importedZUpToThreePoint(canonicalToImportedPoint(point, transform));
  const convertVector = (vector: Point3) => importedZUpToThreeVector(canonicalToImportedVector(vector, transform));
  return {
    leftJointCenter: convert(geometry.leftJointCenter),
    rightJointCenter: convert(geometry.rightJointCenter),
    leftPlanarEndpoint: convert(geometry.leftPlanarEndpoint),
    rightPlanarEndpoint: convert(geometry.rightPlanarEndpoint),
    rollCenter: convert(geometry.rollCenter),
    pitchCenter: convert(geometry.pitchCenter),
    leftSpatialEndpoint: convert(geometry.leftSpatialEndpoint),
    rightSpatialEndpoint: convert(geometry.rightSpatialEndpoint),
    rollAxis: convertVector(geometry.rollAxis),
    pitchAxis: convertVector(geometry.pitchAxis),
    planarDirection: convertVector(geometry.planarDirection),
  };
}

export function setCameraFromOrbit(
  camera: THREE.PerspectiveCamera,
  target: THREE.Vector3,
  azimuth: number,
  elevation: number,
  distance: number,
) {
  const horizontal = Math.cos(elevation) * distance;
  camera.position.set(
    target.x + Math.sin(azimuth) * horizontal,
    target.y + Math.sin(elevation) * distance,
    target.z + Math.cos(azimuth) * horizontal,
  );
  camera.up.set(0, 1, 0);
  camera.lookAt(target);
}

export function geometryPoints(geometry: MechanismGeometry): THREE.Vector3[] {
  return [
    geometry.leftJointCenter,
    geometry.rightJointCenter,
    geometry.leftPlanarEndpoint,
    geometry.rightPlanarEndpoint,
    geometry.rollCenter,
    geometry.pitchCenter,
    geometry.leftSpatialEndpoint,
    geometry.rightSpatialEndpoint,
  ].map(pointToVector);
}
