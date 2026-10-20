import type { CalibrationInput, SolverCreationResult, WasmModule } from '../types';
import { calibrationStatus } from './constants';
import { statusMessage, statusValue } from './math';
import { normalizeCalibrationInput } from './storage';

export async function loadWasm(): Promise<WasmModule> {
  const wasm_url = '/wasm/couple_joint_solver.js';
  const native_import = new Function('url', 'return import(url)') as (url: string) => Promise<{ default: () => Promise<WasmModule> }>;
  const factory = (await native_import(wasm_url)).default;
  return factory();
}

export function toErrorMessage(error: unknown): string {
  return error instanceof Error ? error.message : String(error);
}

export function createSolver(module: WasmModule, calibration: CalibrationInput): SolverCreationResult {
  try {
    const solver = new module.MechanismSolver(normalizeCalibrationInput(calibration));
    if (typeof solver.getCalibrationStatus === 'function') {
      const calibrationState = solver.getCalibrationStatus();
      if (statusValue(calibrationState) !== calibrationStatus.success) {
        return { solver: null, error: statusMessage(calibrationState, 'calibration') };
      }
    }
    return { solver, error: '' };
  } catch (error: unknown) {
    return { solver: null, error: toErrorMessage(error) };
  }
}
