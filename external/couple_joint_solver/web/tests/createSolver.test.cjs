const test = require('node:test');
const assert = require('node:assert/strict');

const compiledRoot = process.env.CREATE_SOLVER_TEST_ROOT;
if (!compiledRoot) {
  throw new Error('CREATE_SOLVER_TEST_ROOT is required');
}

const { defaultConfig } = require(`${compiledRoot}/utils/storage.js`);
const { calibrationStatus } = require(`${compiledRoot}/utils/constants.js`);
const { createSolver } = require(`${compiledRoot}/utils/wasm.js`);

function createModuleWithStatus(status) {
  class FakeSolver {
    getCalibrationStatus() {
      return status;
    }
  }

  return {
    MechanismSolver: FakeSolver,
  };
}

test('createSolver accepts numeric success calibration status', () => {
  const result = createSolver(createModuleWithStatus(calibrationStatus.success), defaultConfig.calibration);

  assert.ok(result.solver);
  assert.equal(result.error, '');
});

test('createSolver accepts embind enum-object success calibration status', () => {
  const result = createSolver(createModuleWithStatus({ value: calibrationStatus.success }), defaultConfig.calibration);

  assert.ok(result.solver);
  assert.equal(result.error, '');
});
