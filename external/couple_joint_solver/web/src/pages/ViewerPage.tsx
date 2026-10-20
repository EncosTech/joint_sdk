import { useMemo, useRef, useState } from 'react';
import { RotateCcw, Trash2 } from 'lucide-react';
import type { MechanismGeometry, MechanismState, SolveResult, StoredConfig, TransmissionInfo, WasmSolver } from '../types';
import { defaultAngleRangeDegrees } from '../utils/constants';
import { toDegrees, toRadians, canUseSolveResult, statusMessage } from '../utils/math';
import { useMechanismScene } from '../hooks/useMechanismScene';
import { Slider } from '../components/Slider';
import { TransmissionPanel } from '../components/TransmissionPanel';

export function ViewerPage({
  solver,
  config,
  onReset,
}: {
  solver: WasmSolver;
  config: StoredConfig;
  onReset: () => void;
}) {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const [state, setState] = useState<MechanismState>(solver.getState());
  const [geometry, setGeometry] = useState<MechanismGeometry>(solver.getGeometry());
  const [transmission, setTransmission] = useState<TransmissionInfo>(solver.getTransmission());
  const [error, setError] = useState('');
  useMechanismScene(containerRef, geometry, config.calibration);

  const ranges = useMemo(() => {
    const initial = solver.getState();
    const theta1 = toDegrees(initial.theta1);
    const theta2 = toDegrees(initial.theta2);
    const roll = toDegrees(initial.roll);
    const pitch = toDegrees(initial.pitch);
    return {
      theta1: { min: theta1 - defaultAngleRangeDegrees, max: theta1 + defaultAngleRangeDegrees },
      theta2: { min: theta2 - defaultAngleRangeDegrees, max: theta2 + defaultAngleRangeDegrees },
      roll: {
        min: config.rollLimit.min ?? roll - defaultAngleRangeDegrees,
        max: config.rollLimit.max ?? roll + defaultAngleRangeDegrees,
      },
      pitch: {
        min: config.pitchLimit.min ?? pitch - defaultAngleRangeDegrees,
        max: config.pitchLimit.max ?? pitch + defaultAngleRangeDegrees,
      },
    };
  }, [config, solver]);

  const sync = (result: SolveResult) => {
    if (!canUseSolveResult(result.status)) {
      setError(statusMessage(result.status, 'solve'));
      return;
    }
    setError(statusMessage(result.status, 'solve'));
    setState(solver.getState());
    setGeometry(solver.getGeometry());
    setTransmission(result.transmission);
  };

  return (
    <main className="viewer-page">
      <aside className="control-panel">
        <div className="panel-heading compact">
          <h1>机构验证</h1>
          <button type="button" className="icon-button" onClick={onReset} title="重新配置参数">
            <Trash2 size={18} />
          </button>
        </div>
        {error ? <div className="error-banner">{error}</div> : null}
        <Slider label="Theta 1" value={state.theta1} min={ranges.theta1.min} max={ranges.theta1.max} onChange={(value) => sync(solver.solveForward(value, state.theta2))} />
        <Slider label="Theta 2" value={state.theta2} min={ranges.theta2.min} max={ranges.theta2.max} onChange={(value) => sync(solver.solveForward(state.theta1, value))} />
        <Slider label="Roll" value={state.roll} min={ranges.roll.min} max={ranges.roll.max} onChange={(value) => sync(solver.solveInverse(value, state.pitch))} />
        <Slider label="Pitch" value={state.pitch} min={ranges.pitch.min} max={ranges.pitch.max} onChange={(value) => sync(solver.solveInverse(state.roll, value))} />
        <TransmissionPanel transmission={transmission} />
        <button type="button" className="secondary-button" onClick={() => sync(solver.solveInverse(0, 0))}>
          <RotateCcw size={18} />
          回到初始位姿
        </button>
      </aside>
      <section className="scene-panel" ref={containerRef} />
    </main>
  );
}
