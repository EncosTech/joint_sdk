import React, { useEffect, useRef } from 'react';
import * as THREE from 'three';
import type { CalibrationInput, MechanismGeometry, Point3 } from '../types';
import { rotationOrder } from '../utils/constants';
import {
  geometryPoints,
  pointToVector,
  setCameraFromOrbit,
  transformGeometryForPreview,
} from '../utils/geometry';

export function useMechanismScene(
  containerRef: React.RefObject<HTMLDivElement | null>,
  geometry: MechanismGeometry | null,
  calibration: CalibrationInput,
) {
  const sceneRef = useRef<{
    renderer: THREE.WebGLRenderer;
    camera: THREE.PerspectiveCamera;
    scene: THREE.Scene;
    objects: THREE.Object3D[];
    orbit: {
      azimuth: number;
      elevation: number;
      distance: number;
      target: THREE.Vector3;
    };
    drag: {
      active: boolean;
      x: number;
      y: number;
    };
    fitted: boolean;
    referencePlaneOffset: number | null;
    referencePlaneKey: string | null;
    lineRadius: number | null;
    sphereRadius: number | null;
  } | null>(null);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) {
      return;
    }

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf5f7fb);
    const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 1000);
    const initialTarget = new THREE.Vector3(0, 0, 0);
    const initialOrbit = {
      azimuth: Math.PI / 5,
      elevation: Math.PI / 6,
      distance: 160,
      target: initialTarget,
    };
    setCameraFromOrbit(camera, initialOrbit.target, initialOrbit.azimuth, initialOrbit.elevation, initialOrbit.distance);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(renderer.domElement);

    const ambient = new THREE.AmbientLight(0xffffff, 0.75);
    const directional = new THREE.DirectionalLight(0xffffff, 0.9);
    directional.position.set(2, -4, 5);
    scene.add(ambient, directional);

    const resize = () => {
      const width = container.clientWidth;
      const height = container.clientHeight;
      renderer.setSize(width, height);
      camera.aspect = width / Math.max(height, 1);
      camera.updateProjectionMatrix();
    };
    resize();
    window.addEventListener('resize', resize);

    const applyOrbit = () => {
      const current = sceneRef.current;
      if (!current) {
        return;
      }
      setCameraFromOrbit(current.camera, current.orbit.target, current.orbit.azimuth, current.orbit.elevation, current.orbit.distance);
    };

    const handlePointerDown = (event: PointerEvent) => {
      const current = sceneRef.current;
      if (!current) {
        return;
      }
      current.drag = { active: true, x: event.clientX, y: event.clientY };
      renderer.domElement.setPointerCapture(event.pointerId);
    };

    const handlePointerMove = (event: PointerEvent) => {
      const current = sceneRef.current;
      if (!current || !current.drag.active) {
        return;
      }
      const delta_x = event.clientX - current.drag.x;
      const delta_y = event.clientY - current.drag.y;
      current.drag = { active: true, x: event.clientX, y: event.clientY };
      current.orbit.azimuth -= delta_x * 0.008;
      current.orbit.elevation = Math.max(-1.35, Math.min(1.35, current.orbit.elevation + delta_y * 0.008));
      applyOrbit();
    };

    const handlePointerUp = (event: PointerEvent) => {
      const current = sceneRef.current;
      if (!current) {
        return;
      }
      current.drag.active = false;
      if (renderer.domElement.hasPointerCapture(event.pointerId)) {
        renderer.domElement.releasePointerCapture(event.pointerId);
      }
    };

    const handleWheel = (event: WheelEvent) => {
      event.preventDefault();
      const current = sceneRef.current;
      if (!current) {
        return;
      }
      current.orbit.distance = Math.max(20, Math.min(600, current.orbit.distance * (1 + event.deltaY * 0.001)));
      applyOrbit();
    };

    renderer.domElement.addEventListener('pointerdown', handlePointerDown);
    renderer.domElement.addEventListener('pointermove', handlePointerMove);
    renderer.domElement.addEventListener('pointerup', handlePointerUp);
    renderer.domElement.addEventListener('pointercancel', handlePointerUp);
    renderer.domElement.addEventListener('wheel', handleWheel, { passive: false });

    let animationId = 0;
    const render = () => {
      animationId = window.requestAnimationFrame(render);
      renderer.render(scene, camera);
    };
    render();

    sceneRef.current = {
      renderer,
      camera,
      scene,
      objects: [],
      orbit: initialOrbit,
      drag: { active: false, x: 0, y: 0 },
      fitted: false,
      referencePlaneOffset: null,
      referencePlaneKey: null,
      lineRadius: null,
      sphereRadius: null,
    };
    return () => {
      window.cancelAnimationFrame(animationId);
      window.removeEventListener('resize', resize);
      renderer.domElement.removeEventListener('pointerdown', handlePointerDown);
      renderer.domElement.removeEventListener('pointermove', handlePointerMove);
      renderer.domElement.removeEventListener('pointerup', handlePointerUp);
      renderer.domElement.removeEventListener('pointercancel', handlePointerUp);
      renderer.domElement.removeEventListener('wheel', handleWheel);
      renderer.dispose();
      container.removeChild(renderer.domElement);
      sceneRef.current = null;
    };
  }, [containerRef]);

  useEffect(() => {
    const state = sceneRef.current;
    if (!state || !geometry) {
      return;
    }
    const previewGeometry = transformGeometryForPreview(geometry, calibration);
    const referencePlaneKey = JSON.stringify(calibration);
    if (state.referencePlaneKey !== referencePlaneKey) {
      state.referencePlaneKey = referencePlaneKey;
      state.referencePlaneOffset = null;
    }
    state.objects.forEach((object) => state.scene.remove(object));
    state.objects = [];

    const box = new THREE.Box3().setFromPoints(geometryPoints(previewGeometry));
    const size = new THREE.Vector3();
    const center = new THREE.Vector3();
    box.getSize(size);
    box.getCenter(center);
    if (state.lineRadius === null || state.sphereRadius === null) {
      const maxEdge = Math.max(size.x, size.y, size.z);
      state.lineRadius = (maxEdge / 50) * 0.1;
      state.sphereRadius = (maxEdge / 50) * 0.3;
    }
    const lineRadius = state.lineRadius;
    const radius = state.sphereRadius;

    if (!state.fitted) {
      state.orbit.target = center;
      state.orbit.distance = Math.max(size.length() * 1.8, 80);
      setCameraFromOrbit(state.camera, state.orbit.target, state.orbit.azimuth, state.orbit.elevation, state.orbit.distance);
      state.fitted = true;
    }

    const addLine = (start: Point3, end: Point3, color: number, radius: number, name = '') => {
      const startVector = pointToVector(start);
      const endVector = pointToVector(end);
      const direction = endVector.clone().sub(startVector);
      const length = direction.length();
      const geometryCylinder = new THREE.CylinderGeometry(radius, radius, length, 18);
      const material = new THREE.MeshStandardMaterial({ color, roughness: 0.55, metalness: 0.08 });
      const mesh = new THREE.Mesh(geometryCylinder, material);
      mesh.name = name;
      mesh.position.copy(startVector.clone().add(endVector).multiplyScalar(0.5));
      mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction.normalize());
      state.scene.add(mesh);
      state.objects.push(mesh);
    };

    const addSphere = (point: Point3, color: number) => {
      const sphere = new THREE.Mesh(
        new THREE.SphereGeometry(radius, 24, 16),
        new THREE.MeshStandardMaterial({ color, roughness: 0.4 }),
      );
      sphere.position.copy(pointToVector(point));
      state.scene.add(sphere);
      state.objects.push(sphere);
    };

    const addAxisMarker = (center: Point3, axis: Point3, color: number, name: string) => {
      const centerVector = pointToVector(center);
      const axisVector = pointToVector(axis).normalize();
      const halfLength = 12;
      const start = centerVector.clone().addScaledVector(axisVector, -halfLength);
      const end = centerVector.clone().addScaledVector(axisVector, halfLength);
      addLine({ x: start.x, y: start.y, z: start.z }, { x: end.x, y: end.y, z: end.z }, color, lineRadius, name);
    };

    const addReferencePlane = () => {
      const left = pointToVector(previewGeometry.leftSpatialEndpoint);
      const right = pointToVector(previewGeometry.rightSpatialEndpoint);
      const center = pointToVector(
        calibration.rotationOrder === rotationOrder.pitchThenRoll
          ? previewGeometry.rollCenter
          : previewGeometry.pitchCenter,
      );
      const midpoint = left.clone().add(right).multiplyScalar(0.5);
      const x_axis = right.clone().sub(left).normalize();
      const planar_normal = pointToVector(previewGeometry.planarDirection).normalize();
      let y_axis = midpoint.clone().sub(center);
      if (y_axis.lengthSq() < 1.0e-12) {
        y_axis = new THREE.Vector3().crossVectors(planar_normal, x_axis);
      }
      y_axis.normalize();
      if (state.referencePlaneOffset === null) {
        const y_projected = y_axis.clone().addScaledVector(x_axis, -y_axis.dot(x_axis));
        const planar_projected = new THREE.Vector3().crossVectors(planar_normal, x_axis);
        const y_length = y_projected.length();
        const planar_length = planar_projected.length();
        if (y_length > 1.0e-6 && planar_length > 1.0e-6) {
          y_projected.divideScalar(y_length);
          planar_projected.divideScalar(planar_length);
          const cross = new THREE.Vector3().crossVectors(y_projected, planar_projected);
          state.referencePlaneOffset = Math.atan2(cross.dot(x_axis), y_projected.dot(planar_projected)) + Math.PI / 2;
        } else {
          state.referencePlaneOffset = 0;
        }
      }
      if (state.referencePlaneOffset) {
        y_axis.applyAxisAngle(x_axis, state.referencePlaneOffset);
      }
      const z_axis = new THREE.Vector3().crossVectors(x_axis, y_axis).normalize();
      y_axis.copy(new THREE.Vector3().crossVectors(z_axis, x_axis).normalize());
      const plane_center = center.clone();
      const link_length = left.distanceTo(right);
      const plane_width = Math.max(link_length, 1);
      const plane_height = Math.max(center.distanceTo(midpoint) * 2.2, 20);
      const plane = new THREE.Mesh(
        new THREE.PlaneGeometry(plane_width, plane_height),
        new THREE.MeshStandardMaterial({
          color: 0x0f766e,
          opacity: 0.24,
          transparent: true,
          roughness: 0.6,
          side: THREE.DoubleSide,
        }),
      );
      plane.name = 'attitude-reference-plane';
      plane.position.copy(plane_center);
      plane.setRotationFromMatrix(new THREE.Matrix4().makeBasis(x_axis, y_axis, z_axis));
      const edges = new THREE.LineSegments(
        new THREE.EdgesGeometry(plane.geometry),
        new THREE.LineBasicMaterial({ color: 0x0f766e, transparent: true, opacity: 0.75 }),
      );
      edges.position.copy(plane.position);
      edges.quaternion.copy(plane.quaternion);
      state.scene.add(plane, edges);
      state.objects.push(plane, edges);

      const half_width = plane_width / 2;
      const half_height = plane_height / 2;
      const cornersLeft = [
        plane_center.clone().addScaledVector(x_axis, -half_width).addScaledVector(y_axis, -half_height),
        plane_center.clone().addScaledVector(x_axis, -half_width).addScaledVector(y_axis, half_height),
      ];
      const cornersRight = [
        plane_center.clone().addScaledVector(x_axis, half_width).addScaledVector(y_axis, half_height),
        plane_center.clone().addScaledVector(x_axis, half_width).addScaledVector(y_axis, -half_height),
      ];
      cornersLeft.forEach((corner) => {
        addLine(left, { x: corner.x, y: corner.y, z: corner.z }, 0x0f766e, lineRadius, 'reference-plane-corner-link');
      });
      cornersRight.forEach((corner) => {
        addLine(right, { x: corner.x, y: corner.y, z: corner.z }, 0x0f766e, lineRadius, 'reference-plane-corner-link');
      });
    };

    addLine(previewGeometry.leftJointCenter, previewGeometry.rightJointCenter, 0x475569, lineRadius);
    addLine(previewGeometry.leftJointCenter, previewGeometry.leftPlanarEndpoint, 0x2563eb, lineRadius);
    addLine(previewGeometry.rightJointCenter, previewGeometry.rightPlanarEndpoint, 0x2563eb, lineRadius);
    addLine(previewGeometry.leftPlanarEndpoint, previewGeometry.leftSpatialEndpoint, 0x16a34a, lineRadius);
    addLine(previewGeometry.rightPlanarEndpoint, previewGeometry.rightSpatialEndpoint, 0x16a34a, lineRadius);
    addLine(previewGeometry.leftSpatialEndpoint, previewGeometry.rightSpatialEndpoint, 0xdc2626, lineRadius);
    addAxisMarker(previewGeometry.rollCenter, previewGeometry.rollAxis, 0xf59e0b, 'roll-axis');
    addAxisMarker(previewGeometry.pitchCenter, previewGeometry.pitchAxis, 0x7c3aed, 'pitch-axis');
    addReferencePlane();

    [
      previewGeometry.leftJointCenter,
      previewGeometry.rightJointCenter,
      previewGeometry.leftPlanarEndpoint,
      previewGeometry.rightPlanarEndpoint,
      previewGeometry.rollCenter,
      previewGeometry.pitchCenter,
      previewGeometry.leftSpatialEndpoint,
      previewGeometry.rightSpatialEndpoint,
    ].forEach((point, index) => addSphere(point, index === 4 ? 0xf59e0b : index === 5 ? 0x7c3aed : 0x111827));
  }, [geometry, calibration]);
}
