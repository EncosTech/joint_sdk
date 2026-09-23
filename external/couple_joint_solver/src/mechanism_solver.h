#pragma once

#include <Eigen/Core>
namespace couple_joint {

/** @brief 广义坐标 q = [roll, pitch]^T 的旋转复合顺序。 */
enum class RotationOrder {
    /** @brief 先绕 O_pitch 做 pitch，再绕随之搬运后的 roll 轴做 roll。 */
    PitchThenRoll = 0,
    /** @brief 先绕 O_roll 做 roll，再绕随之搬运后的 pitch 轴做 pitch。 */
    RollThenPitch = 1,
};

/** @brief 在求解器规范坐标系中表达的笛卡尔点。 */
struct Point3 {
    double x{0.0};  ///< X 坐标分量。
    double y{0.0};  ///< Y 坐标分量。
    double z{0.0};  ///< Z 坐标分量。
};

/**
 * @brief 用于构建规范机构模型的标定数据。
 *
 * 符号映射遵循 `docs/mechanism.md`：
 * - O1/O2：左右输入转轴中心；
 * - P1/P2：零位平面连杆末端；
 * - S1/S2：零位空间连杆连接点；
 * - O_roll/O_pitch：中间连杆的 roll / pitch 旋转中心。
 */
struct CalibrationInput {
    Point3 left_joint_center;          ///< O1
    Point3 right_joint_center;         ///< O2
    Point3 left_planar_endpoint;       ///< q = 0 时的 P1
    Point3 right_planar_endpoint;      ///< q = 0 时的 P2
    Point3 roll_center;                ///< O_roll
    Point3 pitch_center;               ///< O_pitch
    Point3 left_spatial_endpoint;      ///< q = 0 时的 S1
    Point3 right_spatial_endpoint;     ///< q = 0 时的 S2
    Point3 rotate_direction;           ///< 可选的平面法向 / 转轴方向。
    bool use_rotate_direction{false};  ///< 为 true 时优先使用 `rotate_direction`，不再从点集推断法向。
    RotationOrder rotation_order{static_cast<RotationOrder>(-1)};  ///< q 对应的旋转顺序。
};

/** @brief 机构广义状态，其中 theta = [theta1, theta2]^T，q = [roll, pitch]^T。 */
struct MechanismState {
    double theta1{0.0};  ///< 左侧驱动角 theta1。
    double theta2{0.0};  ///< 右侧驱动角 theta2。
    double roll{0.0};    ///< roll = phi。
    double pitch{0.0};   ///< pitch = psi。
};

/**
 * @brief 由标定数据推导出的规范几何参数。
 *
 * 各长度与机构文档中的记号对应：
 * - `left/right_planar_length` 对应 |P_i - O_i|；
 * - `left/right_spatial_length` 对应 b_i = |P_i - S_i|。
 */
struct MechanismParameters {
    double base_distance{0.0};           ///< 规范坐标系中的 |O2 - O1|。
    double left_planar_length{0.0};      ///< |P1 - O1|。
    double right_planar_length{0.0};     ///< |P2 - O2|。
    double left_spatial_length{0.0};     ///< b1 = |P1 - S1|。
    double right_spatial_length{0.0};    ///< b2 = |P2 - S2|。
    Point3 roll_center;                  ///< 规范坐标系中的 O_roll。
    Point3 pitch_center;                 ///< 规范坐标系中的 O_pitch。
    Point3 left_zero_spatial_endpoint;   ///< q = 0 时的 S1。
    Point3 right_zero_spatial_endpoint;  ///< q = 0 时的 S2。
    double initial_theta1{0.0};          ///< 满足零位闭链约束的 theta1。
    double initial_theta2{0.0};          ///< 满足零位闭链约束的 theta2。
};

/** @brief 根据当前状态重建得到的瞬时机构几何。 */
struct MechanismGeometry {
    Point3 left_joint_center;       ///< O1
    Point3 right_joint_center;      ///< O2
    Point3 left_planar_endpoint;    ///< P1(theta1)
    Point3 right_planar_endpoint;   ///< P2(theta2)
    Point3 roll_center;             ///< 当前 O_roll
    Point3 pitch_center;            ///< 当前 O_pitch
    Point3 left_spatial_endpoint;   ///< 当前 S1(q)
    Point3 right_spatial_endpoint;  ///< 当前 S2(q)
    Point3 roll_axis;               ///< 当前 roll 轴方向。
    Point3 pitch_axis;              ///< 当前 pitch 轴方向。
    Point3 planar_direction;        ///< 规范坐标系的 +Z 平面法向。
};

/** @brief 标定阶段的状态分类。 */
enum class CalibrationStatus {
    /** @brief 标定成功。 */
    Success = 0,
    /** @brief 左右输入转轴中心重合，无法建立基轴。 */
    CoincidentJointCenters = 1,
    /** @brief 提供的 `rotate_direction` 退化为零向量。 */
    DegenerateRotateDirection = 2,
    /** @brief 基轴 O1O2 与转轴方向不垂直。 */
    BaseAxisNotPerpendicularToRotateDirection = 3,
    /** @brief 左侧平面连杆 O1P1 与转轴方向不垂直。 */
    LeftPlanarLinkNotPerpendicularToRotateDirection = 4,
    /** @brief 右侧平面连杆 O2P2 与转轴方向不垂直。 */
    RightPlanarLinkNotPerpendicularToRotateDirection = 5,
    /** @brief 平面连杆方向与基轴共线，无法稳定建立规范坐标系。 */
    CollinearPlanarLinksWithBaseAxis = 6,
    /** @brief 构造出的规范坐标系数值不稳定。 */
    UnstableCanonicalFrame = 7,
    /** @brief 输入的旋转顺序不在支持范围内。 */
    UnsupportedRotationOrder = 8,
    /** @brief pitch 轴退化为零向量。 */
    DegeneratePitchAxis = 9,
    /** @brief roll 轴与 pitch 轴不满足正交要求。 */
    RollAxisNotPerpendicularToPitchAxis = 10,
    /** @brief 某条机构连杆长度为零。 */
    ZeroLengthLink = 11,
    /** @brief 标定零位不满足闭链约束。 */
    InitialPoseConstraintViolation = 12,
    /** @brief O1、O2、P1、P2 未落在同一平面内。 */
    PlanarPointsNotCoplanar = 13,
};

/** @brief 标定、位姿、速度和力矩映射共用的求解状态分类。 */
enum class SolveStatus {
    /** @brief 求解成功。 */
    Success = 0,
    /** @brief 求解成功，但当前状态接近奇异位形。 */
    NearSingular = 1,
    /** @brief 在标定失败的求解器实例上调用了求解接口。 */
    InvalidCalibration = 2,
    /** @brief 单侧逆解标量方程退化，无法稳定选枝。 */
    DegenerateSideEquation = 3,
    /** @brief 目标位姿或目标状态超出机构可达范围。 */
    UnreachablePose = 4,
    /** @brief 局部雅可比奇异，无法继续映射。 */
    SingularJacobian = 5,
    /** @brief 正解的阻尼牛顿迭代未在限制步数内收敛。 */
    ForwardNotConverged = 6,
};

/** @brief 对外 API 使用的简单 2x2 行主序矩阵容器。 */
struct Matrix2 {
    double m00{0.0};  ///< 第 0 行第 0 列元素。
    double m01{0.0};  ///< 第 0 行第 1 列元素。
    double m10{0.0};  ///< 第 1 行第 0 列元素。
    double m11{0.0};  ///< 第 1 行第 1 列元素。
};

/**
 * @brief 由闭链约束推导得到的微分传动信息。
 *
 * 各符号与文档保持一致：
 * - `fu` 存储 F_theta = dF / dtheta；
 * - `fq` 存储 F_q = dF / dq；
 * - `velocity_jacobian` 存储 J_{q theta} = -F_q^{-1} F_theta；
 * - `reduction` 在可逆时存储 J_{theta q} = J_{q theta}^{-1}。
 */
struct TransmissionInfo {
    Matrix2 fu;                 ///< F_theta
    Matrix2 fq;                 ///< F_q
    Matrix2 velocity_jacobian;  ///< J_{q theta}
    Matrix2 reduction;          ///< J_{theta q}
    double sigma_min_fu{0.0};   ///< F_theta 的最小奇异值。
    double sigma_min_fq{0.0};   ///< F_q 的最小奇异值。
    double sigma_min_j{0.0};    ///< J_{q theta} 的最小奇异值。
    double condition_j{0.0};    ///< J_{q theta} 的条件数。
    bool singular_fu{false};    ///< F_theta 数值上奇异时为 true。
    bool singular_fq{false};    ///< F_q 数值上奇异时为 true。
    bool singular_j{false};     ///< J_{q theta} 数值上奇异时为 true。
    bool near_singular{false};  ///< 工程上用于近奇异预警的标志。
};

/** @brief 正逆位置求解共用的结果结构。 */
struct SolveResult {
    SolveStatus status{SolveStatus::Success};  ///< 最终求解状态。
    double residual{0.0};                      ///< ||F(theta, q)||_2 或等价标量残差。
    int iterations{0};                         ///< 迭代类求解的迭代次数。
    TransmissionInfo transmission;             ///< 返回状态处的微分传动信息。
};

/** @brief theta 空间与 q 空间之间速度或力矩映射的结果。 */
struct VelocityTorqueResult {
    SolveStatus status{SolveStatus::Success};  ///< 映射状态。
    double theta1{0.0};                        ///< theta 空间分量 1。
    double theta2{0.0};                        ///< theta 空间分量 2。
    double roll{0.0};                          ///< q 空间的 roll 分量。
    double pitch{0.0};                         ///< q 空间的 pitch 分量。
    TransmissionInfo transmission;             ///< 本次映射使用的微分传动信息。
};

/**
 * @brief 在规范 Z-up 坐标系中工作的闭链机构求解器。
 *
 * 求解器内部维护当前状态 `(theta, q)`，并提供：
 * - 位置逆解：q -> theta；
 * - 位置正解：theta -> q；
 * - 基于 J_{q theta} 的速度映射；
 * - 基于雅可比转置关系的力矩映射。
 */
class MechanismSolver {
public:
    /**
     * @brief 根据标定输入构造求解器，并验证零位是否满足闭链约束。
     * @param input 标定输入，包含 O1/O2/P1/P2、S1/S2、O_roll/O_pitch 和旋转顺序。
     */
    explicit MechanismSolver(const CalibrationInput& input);

    /**
     * @brief 返回当前求解器的标定状态。
     * @return 当前实例保存的标定状态码。
     */
    CalibrationStatus GetCalibrationStatus() const;
    /**
     * @brief 返回由标定得到的规范几何参数。
     * @return 标定阶段推导出的规范机构参数。
     */
    MechanismParameters GetParameters() const;
    /**
     * @brief 返回当前状态 `(theta1, theta2, roll, pitch)`。
     * @return 求解器内部维护的当前广义状态。
     */
    MechanismState GetState() const;
    /**
     * @brief 返回重建得到的 O_i、P_i(theta)、S_i(q) 以及当前旋转轴。
     * @return 当前状态对应的瞬时机构几何。
     */
    MechanismGeometry GetGeometry() const;
    /**
     * @brief 返回当前状态处的 F_theta、F_q、J_{q theta} 及奇异性指标。
     * @return 当前内部状态对应的微分传动信息。
     */
    TransmissionInfo GetTransmission() const;
    /**
     * @brief 将标定状态转换为可读消息。
     * @param status 待解释的标定状态码。
     * @return 对应的人类可读状态消息。
     */
    static const char* StatusMessage(CalibrationStatus status);
    /**
     * @brief 将求解状态转换为可读消息。
     * @param status 待解释的求解状态码。
     * @return 对应的人类可读状态消息。
     */
    static const char* StatusMessage(SolveStatus status);

    /**
     * @brief 由 q = [roll, pitch]^T 求解 theta = [theta1, theta2]^T 的位置逆解。
     * @param roll Roll 坐标 phi。
     * @param pitch Pitch 坐标 psi。
     * @return 选定分支的驱动角结果及对应传动信息。
     */
    SolveResult SolveInverse(double roll, double pitch);
    /**
     * @brief 使用阻尼牛顿迭代由 theta 求解 q 的位置正解。
     * @param theta1 左侧驱动角。
     * @param theta2 右侧驱动角。
     * @return 收敛后的 q 与传动信息，或对应的错误 / 奇异状态。
     */
    SolveResult SolveForward(double theta1, double theta2);
    /**
     * @brief 将驱动速度 `dot(theta)` 映射为输出速度 `dot(q)`。
     * @param theta1_dot 左侧驱动角速度 `dot(theta1)`。
     * @param theta2_dot 右侧驱动角速度 `dot(theta2)`。
     * @return 输出速度、状态码以及使用的传动信息。
     */
    VelocityTorqueResult SolveForwardVelocity(double theta1_dot, double theta2_dot) const;
    /**
     * @brief 将输出速度 `dot(q)` 映射为驱动速度 `dot(theta)`。
     * @param roll_dot 输出 roll 角速度 `dot(roll)`。
     * @param pitch_dot 输出 pitch 角速度 `dot(pitch)`。
     * @return 驱动速度、状态码以及使用的传动信息。
     */
    VelocityTorqueResult SolveInverseVelocity(double roll_dot, double pitch_dot) const;
    /**
     * @brief 使用 J_{q theta}^{-T} 将驱动力矩映射为输出广义力矩。
     * @param theta1_torque 左侧驱动力矩。
     * @param theta2_torque 右侧驱动力矩。
     * @return 输出广义力矩、状态码以及使用的传动信息。
     */
    VelocityTorqueResult SolveForwardTorque(double theta1_torque, double theta2_torque) const;
    /**
     * @brief 使用 J_{q theta}^T 将输出广义力矩映射为驱动力矩。
     * @param roll_torque 输出 roll 广义力矩。
     * @param pitch_torque 输出 pitch 广义力矩。
     * @return 驱动力矩、状态码以及使用的传动信息。
     */
    VelocityTorqueResult SolveInverseTorque(double roll_torque, double pitch_torque) const;

private:
    /** @brief 闭链残差 F(theta, q) = [f1, f2]^T。 */
    struct Residual {
        double f1{0.0};  ///< 左侧闭链约束残差。
        double f2{0.0};  ///< 右侧闭链约束残差。
    };

    /** @brief 2x2 矩阵的最小/最大奇异值及条件数。 */
    struct SvdInfo {
        double sigma_min{0.0};  ///< 最小奇异值。
        double sigma_max{0.0};  ///< 最大奇异值。
        double condition{0.0};  ///< 条件数，退化时可视为无穷大。
    };

    /** @brief 中间连杆当前位姿数据，包含搬运后的旋转中心和旋转轴。 */
    struct SpatialPose {
        Eigen::Vector3d left_spatial_endpoint;   ///< 当前姿态下的 S1(q)。
        Eigen::Vector3d right_spatial_endpoint;  ///< 当前姿态下的 S2(q)。
        Eigen::Vector3d roll_center;             ///< 当前姿态下的 O_roll。
        Eigen::Vector3d pitch_center;            ///< 当前姿态下的 O_pitch。
        Eigen::Vector3d roll_axis;               ///< 当前姿态下的 roll 轴方向。
        Eigen::Vector3d pitch_axis;              ///< 当前姿态下的 pitch 轴方向。
    };

    /**
     * @brief 将标定数据转换为规范参数，并验证零位闭链是否成立。
     * @param input 原始标定输入。
     * @return 标定成功或失败的原因码。
     */
    CalibrationStatus Calibrate(const CalibrationInput& input);
    /**
     * @brief 在构建规范坐标系之前检查 O1、O2、P1、P2 是否共面。
     * @param input 原始标定输入。
     * @return 共面检查结果对应的标定状态码。
     */
    CalibrationStatus ValidatePlanarJointCoplanarity(const CalibrationInput& input);
    /**
     * @brief 将世界坐标中的点投影到以平面法向为 +Z 的规范坐标系。
     * @param point 世界坐标系中的输入点。
     * @return 对应的规范坐标系点坐标。
     */
    Eigen::Vector3d ToCanonicalPoint(const Eigen::Vector3d& point) const;
    /**
     * @brief 计算当前的 S1(q)、S2(q)、O_roll(q)、O_pitch(q) 以及搬运后的旋转轴。
     * @param roll 当前 roll 坐标。
     * @param pitch 当前 pitch 坐标。
     * @return 当前中间连杆位姿及其派生几何。
     */
    SpatialPose ComputeSpatialPose(double roll, double pitch) const;
    /**
     * @brief 按配置的旋转链将零位空间点 S_i 变换到当前姿态。
     * @param roll 当前 roll 坐标。
     * @param pitch 当前 pitch 坐标。
     * @param zero_point 规范模型零位下的空间点。
     * @return 经过当前姿态变换后的空间点坐标。
     */
    Eigen::Vector3d TransformSpatialPoint(double roll, double pitch, const Eigen::Vector3d& zero_point) const;
    /**
     * @brief 数值近似 dS_i / droll。
     * @param roll 当前 roll 坐标。
     * @param pitch 当前 pitch 坐标。
     * @param zero_point 规范模型零位下的空间点。
     * @return 当前姿态下空间点对 roll 的偏导数。
     */
    Eigen::Vector3d ComputeSpatialRollDerivative(double roll, double pitch, const Eigen::Vector3d& zero_point) const;
    /**
     * @brief 数值近似 dS_i / dpitch。
     * @param roll 当前 roll 坐标。
     * @param pitch 当前 pitch 坐标。
     * @param zero_point 规范模型零位下的空间点。
     * @return 当前姿态下空间点对 pitch 的偏导数。
     */
    Eigen::Vector3d ComputeSpatialPitchDerivative(double roll, double pitch, const Eigen::Vector3d& zero_point) const;
    /**
     * @brief 计算 P1(theta1) = O1 + Rz(theta1) v1。
     * @param theta1 左侧驱动角。
     * @return 当前 `theta1` 对应的左侧平面端点位置。
     */
    Eigen::Vector3d ComputeLeftPlanarEndpoint(double theta1) const;
    /**
     * @brief 计算 P2(theta2) = O2 + Rz(theta2) v2。
     * @param theta2 右侧驱动角。
     * @return 当前 `theta2` 对应的右侧平面端点位置。
     */
    Eigen::Vector3d ComputeRightPlanarEndpoint(double theta2) const;
    /**
     * @brief 计算闭链残差 F(theta, q)。
     * @param theta1 左侧驱动角。
     * @param theta2 右侧驱动角。
     * @param roll 当前 roll 坐标。
     * @param pitch 当前 pitch 坐标。
     * @return 两条闭链约束组成的残差向量。
     */
    Residual ComputeResidual(double theta1, double theta2, double roll, double pitch) const;
    /**
     * @brief 在候选状态处构造 F_theta、F_q、J_{q theta} 及奇异性指标。
     * @param theta1 左侧驱动角。
     * @param theta2 右侧驱动角。
     * @param roll 当前 roll 坐标。
     * @param pitch 当前 pitch 坐标。
     * @return 候选状态处的微分传动信息。
     */
    TransmissionInfo ComputeTransmissionInfo(double theta1, double theta2, double roll, double pitch) const;
    /**
     * @brief 求解单侧标量分支方程 A cos(theta_i) + B sin(theta_i) + C = 0。
     * @param a 方程中余弦项系数 A。
     * @param b 方程中正弦项系数 B。
     * @param joint_center 当前侧输入转轴中心 O_i。
     * @param spatial_endpoint 当前姿态下的空间端点 S_i(q)。
     * @param right_side 为 true 表示求右侧分支，否则求左侧分支。
     * @param reference_theta 用于选取连续分支的参考角度。
     * @param theta 输出参数，写入选定分支的驱动角。
     * @return 单侧方程的求解状态。
     */
    SolveStatus SolveSide(double a, double b, const Eigen::Vector3d& joint_center,
                          const Eigen::Vector3d& spatial_endpoint, bool right_side, double reference_theta,
                          double* theta) const;
    /**
     * @brief 将传动指标转换为对外求解状态。
     * @param transmission 当前状态对应的微分传动信息。
     * @return 面向调用者的求解状态码。
     */
    static SolveStatus StatusForTransmission(const TransmissionInfo& transmission);
    /**
     * @brief 计算 2x2 矩阵的奇异值摘要信息。
     * @param matrix 待分析的 2x2 矩阵。
     * @return 最小奇异值、最大奇异值和条件数。
     */
    static SvdInfo ComputeSvdInfo(const Eigen::Matrix2d& matrix);

    CalibrationStatus calibration_status_{CalibrationStatus::Success};  ///< 当前实例的标定状态缓存。
    Eigen::Vector3d origin_;                                            ///< 规范坐标系原点。
    Eigen::Vector3d axis_x_;                                            ///< 规范坐标系 X 轴方向。
    Eigen::Vector3d axis_y_;                                            ///< 规范坐标系 Y 轴方向。
    Eigen::Vector3d axis_z_;                       ///< 规范坐标系 Z 轴方向，即输入平面的法向。
    Eigen::Vector3d roll_axis_;                    ///< 规范模型零位下的 roll 轴方向。
    Eigen::Vector3d pitch_axis_;                   ///< 规范模型零位下的 pitch 轴方向。
    Eigen::Vector3d roll_center_;                  ///< 规范模型零位下的 O_roll。
    Eigen::Vector3d pitch_center_;                 ///< 规范模型零位下的 O_pitch。
    Eigen::Vector3d left_zero_spatial_endpoint_;   ///< 规范模型零位下的 S1。
    Eigen::Vector3d right_zero_spatial_endpoint_;  ///< 规范模型零位下的 S2。
    RotationOrder rotation_order_{RotationOrder::PitchThenRoll};  ///< 当前采用的 q 旋转复合顺序。
    MechanismParameters parameters_;                              ///< 标定后得到的规范几何参数。
    MechanismState state_;  ///< 当前内部状态 `(theta1, theta2, roll, pitch)`。
};

}  // namespace couple_joint
