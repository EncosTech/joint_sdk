#include "mechanism_utils.h"

#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <limits>

#include "mechanism_solver.h"

namespace couple_joint {

Eigen::Vector3d ToEigen(const Point3& p) {
    return Eigen::Vector3d(p.x, p.y, p.z);
}

Point3 FromEigen(const Eigen::Vector3d& v) {
    return {v.x(), v.y(), v.z()};
}

Eigen::Matrix2d ToEigen(const Matrix2& m) {
    Eigen::Matrix2d result;
    result << m.m00, m.m01, m.m10, m.m11;
    return result;
}

Matrix2 FromEigen(const Eigen::Matrix2d& m) {
    return {m(0, 0), m(0, 1), m(1, 0), m(1, 1)};
}

Eigen::Vector3d RotateVector(const Eigen::Vector3d& axis, double angle, const Eigen::Vector3d& value) {
    return Eigen::AngleAxisd(angle, axis.normalized()) * value;
}

Eigen::Vector3d RotatePoint(const Eigen::Vector3d& center, const Eigen::Vector3d& axis, double angle,
                            const Eigen::Vector3d& point) {
    return center + Eigen::AngleAxisd(angle, axis.normalized()) * (point - center);
}

double Clamp(double value, double minimum, double maximum) {
    return std::max(minimum, std::min(maximum, value));
}

double Squared(double value) {
    return value * value;
}

double WrapAngle(double angle) {
    constexpr double kPi = 3.141592653589793238462643383279502884;
    while (angle <= -kPi) {
        angle += 2.0 * kPi;
    }
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    return angle;
}

double AngleDistance(double lhs, double rhs) {
    return std::abs(WrapAngle(lhs - rhs));
}

ForwardNewtonResult SolveDampedNewton(const std::function<Eigen::Vector2d(double, double)>& residual_fn,
                                      const std::function<JacobianEval(double, double)>& jacobian_fn,
                                      double initial_roll, double initial_pitch, double residual_tolerance,
                                      double step_tolerance, double damping, int max_iterations) {
    double roll = initial_roll;
    double pitch = initial_pitch;
    double residual_norm = std::numeric_limits<double>::infinity();

    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        // 在线性化之前先检查当前闭链残差 F(theta, q_k)。
        const Eigen::Vector2d residual = residual_fn(roll, pitch);
        residual_norm = residual.norm();
        if (residual_norm < residual_tolerance) {
            return {roll, pitch, residual_norm, iteration, true, false};
        }

        const JacobianEval jacobian = jacobian_fn(roll, pitch);
        if (jacobian.singular) {
            return {roll, pitch, residual_norm, iteration, false, true};
        }

        // 求解阻尼法方程 (J^T J + lambda^2 I) Delta q = -J^T F。
        const Eigen::Matrix2d& j = jacobian.matrix;
        Eigen::Matrix2d a = j.transpose() * j;
        a(0, 0) += damping * damping;
        a(1, 1) += damping * damping;
        const Eigen::Vector2d b = -j.transpose() * residual;
        const Eigen::FullPivLU<Eigen::Matrix2d> lu(a);
        if (!lu.isInvertible()) {
            return {roll, pitch, residual_norm, iteration, false, true};
        }

        const Eigen::Vector2d delta = lu.solve(b);
        // 对每个坐标增量限幅，避免牛顿步跳到错误分支。
        roll = WrapAngle(roll + Clamp(delta(0), -0.25, 0.25));
        pitch = WrapAngle(pitch + Clamp(delta(1), -0.25, 0.25));
        if (delta.norm() < step_tolerance) {
            break;
        }
    }

    const Eigen::Vector2d final_residual = residual_fn(roll, pitch);
    residual_norm = final_residual.norm();
    const bool converged = residual_norm < residual_tolerance;
    return {roll, pitch, residual_norm, max_iterations, converged, false};
}

}  // namespace couple_joint
