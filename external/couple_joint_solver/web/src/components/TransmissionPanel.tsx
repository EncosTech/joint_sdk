import type { TransmissionInfo } from '../types';
import { formatScalar } from '../utils/math';
import { MatrixReadout } from './MatrixReadout';

export function TransmissionPanel({ transmission }: { transmission: TransmissionInfo }) {
  return (
    <section className="transmission-panel">
      <h2>减速比</h2>
      <MatrixReadout
        title="姿态到电机的映射矩阵"
        rowLabels={['Theta 1', 'Theta 2']}
        columnLabels={['Roll', 'Pitch']}
        matrix={transmission.reduction}
      />
      <MatrixReadout
        title="电机到姿态的映射矩阵"
        rowLabels={['Roll', 'Pitch']}
        columnLabels={['Theta 1', 'Theta 2']}
        matrix={transmission.velocityJacobian}
      />
      <div className="singularity-readout">
        <span>雅可比条件数</span>
        <output>{formatScalar(transmission.conditionJ)}</output>
        <span>最小奇异值</span>
        <output>{formatScalar(transmission.sigmaMinJ)}</output>
      </div>
      <div className={transmission.nearSingular ? 'status-pill warning' : 'status-pill'}>
        {transmission.nearSingular ? '接近奇异' : '正常'}
      </div>
    </section>
  );
}
