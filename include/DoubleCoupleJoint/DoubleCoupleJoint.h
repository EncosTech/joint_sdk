// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

#pragma once

#ifndef ENCOS_DOUBLE_COUPLE_JOINT_H_
#define ENCOS_DOUBLE_COUPLE_JOINT_H_

#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "BaseJoint.h"
#include "JsonSchemaGenerator.h"
#include "external/json.hpp"
#include "mechanism_solver.h"

namespace couple_joint {

/**
 * @brief 从 JSON 对象解析三维点。
 * @param j 包含 x、y、z 字段的 JSON 对象。
 * @param point 输出的三维点。
 */
inline void from_json(const nlohmann::json& j, Point3& point) {
    j.at("x").get_to(point.x);
    j.at("y").get_to(point.y);
    j.at("z").get_to(point.z);
}

/**
 * @brief 从 JSON 字符串或整数解析双关节旋转顺序。
 * @param j 字符串或整数 JSON 值；支持 PitchThenRoll/RollThenPitch 及 0/1。
 * @param rotation_order 输出的旋转顺序。
 * @throw std::invalid_argument 当值不是已知旋转顺序时抛出。
 */
inline void from_json(const nlohmann::json& j, RotationOrder& rotation_order) {
    if (j.is_number_integer()) {
        const int value = j.get<int>();
        if (value == 0) {
            rotation_order = RotationOrder::PitchThenRoll;
            return;
        }
        if (value == 1) {
            rotation_order = RotationOrder::RollThenPitch;
            return;
        }
        throw std::invalid_argument("Unknown DoubleCoupleJoint rotationOrder: " +
                                    std::to_string(value));
    }

    const auto value = j.get<std::string>();
    if (value == "PitchThenRoll" || value == "pitch_then_roll") {
        rotation_order = RotationOrder::PitchThenRoll;
        return;
    }
    if (value == "RollThenPitch" || value == "roll_then_pitch") {
        rotation_order = RotationOrder::RollThenPitch;
        return;
    }
    throw std::invalid_argument("Unknown DoubleCoupleJoint rotationOrder: " + value);
}

/**
 * @brief 从 JSON 对象解析双关节闭链标定参数。
 * @param j 包含关节中心、端点和旋转顺序字段的 JSON 对象。
 * @param calibration 输出的标定参数。
 */
inline void from_json(const nlohmann::json& j, CalibrationInput& calibration) {
    j.at("leftJointCenter").get_to(calibration.left_joint_center);
    j.at("rightJointCenter").get_to(calibration.right_joint_center);
    j.at("leftPlanarEndpoint").get_to(calibration.left_planar_endpoint);
    j.at("rightPlanarEndpoint").get_to(calibration.right_planar_endpoint);
    j.at("rollCenter").get_to(calibration.roll_center);
    j.at("pitchCenter").get_to(calibration.pitch_center);
    j.at("leftSpatialEndpoint").get_to(calibration.left_spatial_endpoint);
    j.at("rightSpatialEndpoint").get_to(calibration.right_spatial_endpoint);
    j.at("rotationOrder").get_to(calibration.rotation_order);

    const auto rotate_direction_it = j.find("rotateDirection");
    calibration.use_rotate_direction = rotate_direction_it != j.end();
    if (calibration.use_rotate_direction) {
        rotate_direction_it->get_to(calibration.rotate_direction);
    } else {
        calibration.rotate_direction = {};
    }
}

}  // namespace couple_joint

namespace encos {

// 前向声明
class MasterJoint;
class SlaveJoint;

template <typename... JointTypes>
class JointManager;

/**
 * @brief DoubleCoupleJoint 对应的输出轴。
 */
enum class DoubleCoupleJointAxis {
    Roll,
    Pitch,
};

/**
 * @brief 从 JSON 字符串解析双电机关节输出轴。
 * @param j 字符串 JSON 值；支持 roll/Roll/pitch/Pitch。
 * @param axis 输出的轴枚举。
 * @throw std::invalid_argument 当字符串不是已知轴名时抛出。
 */
inline void from_json(const nlohmann::json& j, DoubleCoupleJointAxis& axis) {
    const auto value = j.get<std::string>();
    if (value == "roll" || value == "Roll") {
        axis = DoubleCoupleJointAxis::Roll;
        return;
    }
    if (value == "pitch" || value == "Pitch") {
        axis = DoubleCoupleJointAxis::Pitch;
        return;
    }
    throw std::invalid_argument("Unknown DoubleCoupleJoint axis: " + value);
}

/**
 * @brief 双电机关节配置。
 *
 * 每个 DoubleCoupleJoint 实例只代表闭链输出的一个自由度，axis 指定该实例对应 roll 或
 * pitch。同一对电机必须创建两个实例并分别声明 roll/pitch。
 */
class DoubleCoupleJoint {
public:
    struct Config {
        // clang-format off
        GENERATE_CONFIG(DoubleCoupleJoint,
                        ((motorParams1, MotorParams, Complex, true, "Motor 1 initialization parameters"))
                        ((motorParams2, MotorParams, Complex, true, "Motor 2 initialization parameters"))
                        ((axis, DoubleCoupleJointAxis, Axis, true, "Output axis: roll or pitch"))
                        ((calibration, couple_joint::CalibrationInput, Calibration, true, "Closed-chain calibration parameters"))
                        ((motorLimit1, std::optional<Range<float>>, Range, false, "Motor 1 relative position limit"))
                        ((motorLimit2, std::optional<Range<float>>, Range, false, "Motor 2 relative position limit"))
                        ((limit, Range<float>, Range, true, "Position limit range"))
                        ((maxSpd, float, Float, true, "Maximum speed"))
                        ((maxTor, float, Float, true, "Maximum torque"))
                        ((reverse, bool, Bool, true, "Reverse direction")))
        // clang-format on
    };

    /**
     * @brief 生成电机对配对键，用于 DoubleCoupleJoint 的 Master/Slave 配对。
     *
     * 根据两个 MotorParams 生成格式为 "adapterKey/bus/motorId|adapterKey/bus/motorId"
     * 的配对键，adapterKey 保留引用类型及名称长度。若两个 MotorParams 指向同一物理电机
     * （adapterId 与合成总线索引
     * slaveId << 16 | busId、motorId 均相同），则返回空字符串以拒绝自身配对。
     *
     * @note adapterId 按取值比较：索引引用与名称引用即使解析到同一个适配器也判为
     * 不同，同一对电机的两份配置必须使用同一种引用形式。
     *
     * @note motorParams1 和 motorParams2 在两次 AddJoint 调用中顺序必须一致。
     * @param a 第一个电机参数。
     * @param b 第二个电机参数。
     * @return 配对键；两个参数指向同一物理电机时返回空字符串。
     */
    static std::string MakePairKey(const MotorParams& a, const MotorParams& b) {
        if (a.adapterId == b.adapterId && a.BusIndex() == b.BusIndex() && a.motorId == b.motorId) {
            return "";
        }
        auto id_a = a.AdapterIdentityKey() + "/" + std::to_string(a.BusIndex()) + "/" +
                    std::to_string(a.motorId);
        auto id_b = b.AdapterIdentityKey() + "/" + std::to_string(b.BusIndex()) + "/" +
                    std::to_string(b.motorId);
        return id_a + "|" + id_b;
    }

    /**
     * @brief 工厂方法，创建 SlaveJoint 或 MasterJoint。
     *
     * - 当 motor1/motor2 均为 nullptr 时创建 SlaveJoint（暂存配置，等待配对）；
     * - 当 motor1/motor2 均非空时创建 MasterJoint，创建前校验配对约束：
     *   slave 参数不能为空、master 与 slave 的 axis 必须不同、calibration 必须一致。
     *   配对成功后将 master 绑定到 slave。
     *
     * @param config  双电机关节配置
     * @param motor1  预创建的电机 1 实例（slave 路径传 nullptr）
     * @param motor2  预创建的电机 2 实例（slave 路径传 nullptr）
     * @param slave   master 创建时传入的待配对 slave 实例（创建 slave 时为 nullptr）
     * @return 新创建的 BaseJoint 派生类实例
     * @throws JointException 配对约束不满足时抛出相应错误
     */
    static BaseJointPtr CreateJoint(const Config& config, Motor* motor1 = nullptr,
                                    Motor* motor2 = nullptr,
                                    std::shared_ptr<SlaveJoint> slave = nullptr);

    /**
     * @brief 比较两个 CalibrationInput 是否在容差范围内相等。
     *
     * 逐个比较 Point3 类型的几何字段（left_joint_center 等），
     * 以及 use_rotate_direction、rotation_order 等标量字段。
     * rotate_direction 仅在 lhs.use_rotate_direction 为 true 时才参与比较。
     *
     * @param lhs 标定输入 A
     * @param rhs 标定输入 B
     * @return true 表示两个标定输入在容差内相等
     */
    static bool CalibrationEqual(const couple_joint::CalibrationInput& lhs,
                                 const couple_joint::CalibrationInput& rhs) {
        const bool same_calibration =
            PointEqual(lhs.left_joint_center, rhs.left_joint_center) &&
            PointEqual(lhs.right_joint_center, rhs.right_joint_center) &&
            PointEqual(lhs.left_planar_endpoint, rhs.left_planar_endpoint) &&
            PointEqual(lhs.right_planar_endpoint, rhs.right_planar_endpoint) &&
            PointEqual(lhs.roll_center, rhs.roll_center) &&
            PointEqual(lhs.pitch_center, rhs.pitch_center) &&
            PointEqual(lhs.left_spatial_endpoint, rhs.left_spatial_endpoint) &&
            PointEqual(lhs.right_spatial_endpoint, rhs.right_spatial_endpoint) &&
            lhs.use_rotate_direction == rhs.use_rotate_direction &&
            lhs.rotation_order == rhs.rotation_order;
        if (!same_calibration) {
            return false;
        }
        return !lhs.use_rotate_direction || PointEqual(lhs.rotate_direction, rhs.rotate_direction);
    }

    /**
     * @brief 校验 master 配置能否与待配对 SlaveJoint 成为一对。
     * @param config master 侧双电机关节配置。
     * @param slave 待配对的 slave 关节实例。
     * @throw JointException 当 slave 缺失、轴重复或标定不一致时抛出。
     */
    static void ValidatePair(const Config& config, const std::shared_ptr<SlaveJoint>& slave);

private:
    static constexpr double kCalibrationTolerance = 1.0e-6;

    /**
     * @brief 按标定容差比较两个三维点。
     * @param lhs 左侧三维点。
     * @param rhs 右侧三维点。
     * @return true 表示三个坐标分量都在容差范围内相等。
     */
    static bool PointEqual(const couple_joint::Point3& lhs, const couple_joint::Point3& rhs) {
        return std::abs(lhs.x - rhs.x) <= kCalibrationTolerance &&
               std::abs(lhs.y - rhs.y) <= kCalibrationTolerance &&
               std::abs(lhs.z - rhs.z) <= kCalibrationTolerance;
    }

    /**
     * @brief 校验双关节电机相对位置限位。
     * @param limit 待校验的电机相对位置限位。
     * @throw std::invalid_argument 当限位包含非有限值或 min > max 时抛出。
     */
    static void ValidateMotorLimit(Range<float> limit);

    /**
     * @brief 按标定容差比较两个范围。
     * @param lhs 左侧范围。
     * @param rhs 右侧范围。
     * @return true 表示 min 和 max 都在容差范围内相等。
     */
    static bool RangeEqual(const Range<float>& lhs, const Range<float>& rhs);

    /**
     * @brief 合并 master/slave 两侧可选电机限位。
     * @param master_limit master 侧配置的电机限位，可为空。
     * @param slave_limit slave 侧配置的电机限位，可为空。
     * @return 合并后的电机限位；两侧都未配置时返回 std::nullopt。
     * @throw JointException 当两侧均配置但范围不一致时抛出。
     * @throw std::invalid_argument 当任一已配置限位无效时抛出。
     */
    static std::optional<Range<float>> MergeMotorLimit(
        const std::optional<Range<float>>& master_limit,
        const std::optional<Range<float>>& slave_limit);
};

}  // namespace encos

// MasterJoint depends on the common DoubleCoupleJoint definitions above.
#include "DoubleCoupleJoint/MasterJoint.h"

#endif  // ENCOS_DOUBLE_COUPLE_JOINT_H_
