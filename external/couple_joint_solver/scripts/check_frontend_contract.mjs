import { readFileSync, readdirSync, statSync } from 'node:fs';
import { join, extname } from 'node:path';

function collectFiles(dir, extensions) {
  const files = [];
  for (const entry of readdirSync(dir)) {
    const fullPath = join(dir, entry);
    const st = statSync(fullPath);
    if (st.isDirectory()) {
      files.push(...collectFiles(fullPath, extensions));
    } else if (st.isFile() && extensions.includes(extname(fullPath))) {
      files.push(fullPath);
    }
  }
  return files;
}

const srcDir = 'web/src';
const sourceFiles = collectFiles(srcDir, ['.ts', '.tsx']);
const source = sourceFiles.map((f) => readFileSync(f, 'utf8')).join('\n');

const checks = [
  ['default angle range is 30 degrees', 'const defaultAngleRangeDegrees = 30;'],
  ['scene drag handler is present', 'handlePointerMove'],
  ['scene wheel zoom handler is present', 'handleWheel'],
  ['attitude reference plane is present', 'attitude-reference-plane'],
  ['reference plane is centered on the active rotation center', 'const plane_center = center.clone();'],
  ['reference plane uses the later rotation center for pitch-then-roll', '? previewGeometry.rollCenter'],
  ['reference plane uses the later rotation center for roll-then-pitch', ': previewGeometry.pitchCenter'],
  ['reference plane follows middle link attitude with fallback', 'let y_axis = midpoint.clone().sub(center);'],
  ['reference plane fallback derives from planar normal', 'crossVectors(planar_normal, x_axis)'],
  ['reference plane corner links are present', 'reference-plane-corner-link'],
  ['canonical geometry is restored to imported coordinates', 'canonicalToImportedPoint'],
  ['imported Z axis is mapped to the Three.js vertical axis', 'importedZUpToThreePoint'],
  ['rotate direction is treated as planar normal', 'let planarNormal = calibration.useRotateDirection'],
  ['canonical Z axis is built from planar normal', 'const axisZ = planarNormal.addScaledVector(axisX, -planarNormal.dot(axisX)).normalize();'],
  ['canonical Y axis is derived from planar normal and base axis', 'const axisY = new THREE.Vector3().crossVectors(axisZ, axisX).normalize();'],
  ['roll center is part of frontend calibration', 'rollCenter: Point3;'],
  ['pitch center is part of frontend calibration', 'pitchCenter: Point3;'],
  ['legacy center joint import is migrated', 'centerJoint?: Point3;'],
  ['configuration migration helper is present', 'function parseAndMigrateConfig'],
  ['configuration export is present', 'const exportConfigFile ='],
  ['configuration file import is present', 'const importConfigFile ='],
  ['configuration clipboard import is present', 'const importClipboard ='],
  ['rotation order selector has styled control', 'rotation-order-control'],
  ['rotation order options expose selected state', 'rotation-order-option selected'],
  ['roll axis marker is drawn', "'roll-axis'"],
  ['pitch axis marker is drawn', "'pitch-axis'"],
  ['solver constructor errors are routed to the UI', 'function createSolver'],
  ['solve status enum drives result handling', 'function canUseSolveResult'],
  ['transmission ratio is displayed', 'function TransmissionPanel'],
  ['human readable reduction title is displayed', '姿态到电机的映射矩阵'],
  ['human readable jacobian title is displayed', '电机到姿态的映射矩阵'],
  ['matrix rows are labeled by output variables', "rowLabels={['Theta 1', 'Theta 2']}"],
  ['matrix columns are labeled by input variables', "columnLabels={['Roll', 'Pitch']}"],
  ['condition number label is human readable', '雅可比条件数'],
  ['minimum singular value label is human readable', '最小奇异值'],
  ['near singular solve status is handled', 'nearSingular'],
];

const missing = checks.filter(([, pattern]) => !source.includes(pattern));

if (missing.length > 0) {
  for (const [description] of missing) {
    console.error(`Missing frontend contract: ${description}`);
  }
  process.exit(1);
}

console.log('frontend contract checks passed');
