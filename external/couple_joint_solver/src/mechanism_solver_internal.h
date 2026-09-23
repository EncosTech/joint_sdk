#pragma once

namespace couple_joint::internal {

constexpr double kTiny = 1.0e-10;                   ///< 通用零值判定阈值，用于避免除零和退化比较。
constexpr double kResidualTolerance = 1.0e-6;       ///< 闭链残差或牛顿收敛的目标精度阈值。
constexpr double kStepTolerance = 1.0e-10;          ///< 迭代步长足够小时的停止阈值。
constexpr double kDamping = 1.0e-5;                 ///< 阻尼牛顿法中加入到法方程的阻尼项系数。
constexpr double kDerivativeEps = 1.0e-6;           ///< 数值差分近似导数时使用的扰动步长。
constexpr double kCoplanarToleranceScale = 1.0e-6;  ///< 判断 O1/O2/P1/P2 共面时按几何尺度放大的容差系数。
constexpr double kSingularValueTolerance = 1.0e-6;  ///< 判断 F_theta、F_q 或 J 是否数值奇异的奇异值阈值。
constexpr double kNearSingularSigmaMinJLimit = 0.2;  ///< 依据 J 的最小奇异值判定近奇异的经验下限。
constexpr double kConditionTiny = 1.0e-15;  ///< 计算条件数时防止除以极小奇异值的保护阈值。
constexpr double kNearSingularConditionJLimit = 8.0;  ///< 依据 J 的条件数判定近奇异的经验上限。
constexpr double kAxisPerpendicularTolerance = 0.05;  ///< 检查旋转轴近似正交时允许的点积容差。
constexpr int kMaxIterations = 64;                    ///< 阻尼牛顿正解允许的最大迭代次数。

}  // namespace couple_joint::internal
