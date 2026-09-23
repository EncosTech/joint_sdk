import type { Point3 } from '../types';

export function PointFields({
  label,
  value,
  onChange,
}: {
  label: string;
  value: Point3;
  onChange: (point: Point3) => void;
}) {
  const update = (axis: keyof Point3, nextValue: string) => {
    onChange({ ...value, [axis]: Number(nextValue) });
  };

  return (
    <div className="point-row">
      <div className="point-label">{label}</div>
      {(['x', 'y', 'z'] as const).map((axis) => (
        <label key={axis}>
          <span>{axis.toUpperCase()}</span>
          <input type="number" step="0.01" value={value[axis]} onChange={(event) => update(axis, event.target.value)} />
        </label>
      ))}
    </div>
  );
}
