#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <functional>

namespace couple_joint {

struct Point3;
struct Matrix2;
/** @brief q = [roll, pitch]^T 的阻尼牛顿求解结果。 */
struct ForwardNewtonResult {
    double roll{0.0};           ///< 最终 roll 估计值。
    double pitch{0.0};          ///< 最终 pitch 估计值。
    double residual_norm{0.0};  ///< 最终残差向量的 2 范数。
    int iterations{0};          ///< 已执行的迭代次数。
    bool converged{false};      ///< 残差达到阈值时为 true。
    bool singular{false};       ///< 局部雅可比奇异时为 true。
};

/** @brief 阻尼牛顿迭代中使用的 Jacobian 评估结果 F_q(q_k)。 */
struct JacobianEval {
    Eigen::Matrix2d matrix;  ///< 当前评估得到的雅可比矩阵。
    bool singular{false};    ///< 针对该矩阵预先判定的奇异标志。
};

/** @brief 将对外的 Point3 转换为 Eigen::Vector3d。 */
Eigen::Vector3d ToEigen(const Point3& p);
/** @brief 将 Eigen::Vector3d 转换为对外的 Point3。 */
Point3 FromEigen(const Eigen::Vector3d& v);
/** @brief 将对外的 Matrix2 转换为 Eigen::Matrix2d。 */
Eigen::Matrix2d ToEigen(const Matrix2& m);
/** @brief 将 Eigen::Matrix2d 转换为对外的 Matrix2。 */
Matrix2 FromEigen(const Eigen::Matrix2d& m);

/** @brief 将向量绕任意轴旋转 `angle`。 */
Eigen::Vector3d RotateVector(const Eigen::Vector3d& axis, double angle, const Eigen::Vector3d& value);
/** @brief 将点绕任意中心 / 轴组合旋转 `angle`。 */
Eigen::Vector3d RotatePoint(const Eigen::Vector3d& center, const Eigen::Vector3d& axis, double angle,
                            const Eigen::Vector3d& point);

/** @brief 将 `value` 限制到闭区间 [`minimum`, `maximum`]。 */
double Clamp(double value, double minimum, double maximum);
/** @brief 返回 `value * value`。 */
double Squared(double value);
/** @brief 将角度包装到区间 (-pi, pi]。 */
double WrapAngle(double angle);
/** @brief 返回两个角度之间的包装角距离。 */
double AngleDistance(double lhs, double rhs);
/**
 * @brief 使用阻尼最小二乘 / LM 风格步长求解 F_q(q_k) Delta q = -F(q_k)。
 * @param residual_fn 在固定驱动输入下计算 F(theta, q)。
 * @param jacobian_fn 在当前 q 上计算 F_q。
 * @param initial_roll roll 初始猜测。
 * @param initial_pitch pitch 初始猜测。
 * @param residual_tolerance 残差范数的收敛阈值。
 * @param step_tolerance 步长范数的停滞阈值。
 * @param damping 施加在 J^T J + lambda^2 I 上的阻尼 lambda。
 * @param max_iterations 最大迭代次数。
 * @return 最终迭代点及其收敛诊断信息。
 */
ForwardNewtonResult SolveDampedNewton(const std::function<Eigen::Vector2d(double, double)>& residual_fn,
                                      const std::function<JacobianEval(double, double)>& jacobian_fn,
                                      double initial_roll, double initial_pitch, double residual_tolerance,
                                      double step_tolerance, double damping, int max_iterations);

}  // namespace couple_joint
