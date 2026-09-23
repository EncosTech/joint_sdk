import type { Matrix2 } from '../types';
import { formatScalar } from '../utils/math';

export function MatrixReadout({
  title,
  rowLabels,
  columnLabels,
  matrix,
}: {
  title: string;
  rowLabels: [string, string];
  columnLabels: [string, string];
  matrix: Matrix2;
}) {
  return (
    <div className="matrix-readout">
      <span>{title}</span>
      <div className="matrix-grid">
        <span className="matrix-corner" />
        <span className="matrix-label">{columnLabels[0]}</span>
        <span className="matrix-label">{columnLabels[1]}</span>
        <span className="matrix-label">{rowLabels[0]}</span>
        <output>{formatScalar(matrix.m00)}</output>
        <output>{formatScalar(matrix.m01)}</output>
        <span className="matrix-label">{rowLabels[1]}</span>
        <output>{formatScalar(matrix.m10)}</output>
        <output>{formatScalar(matrix.m11)}</output>
      </div>
    </div>
  );
}
