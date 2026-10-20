import { toDegrees, toRadians, formatDegrees } from '../utils/math';

export function Slider({
  label,
  value,
  min,
  max,
  onChange,
}: {
  label: string;
  value: number;
  min: number;
  max: number;
  onChange: (value: number) => void;
}) {
  return (
    <label className="slider-row">
      <span>{label}</span>
      <input
        type="range"
        min={min}
        max={max}
        step="0.1"
        value={toDegrees(value)}
        onChange={(event) => onChange(toRadians(Number(event.target.value)))}
      />
      <output>{formatDegrees(value)} deg</output>
    </label>
  );
}
