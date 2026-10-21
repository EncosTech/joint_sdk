// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

#pragma once

#ifndef ENCOS_MASTER_JOINT_H_
#define ENCOS_MASTER_JOINT_H_

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "DoubleCoupleJoint/DoubleCoupleJoint.h"
#include "JointSdkLog.h"

namespace encos {

template <typename... JointTypes>
class JointManager;

class SlaveJoint;
void RecordSlaveStatusForLog(const std::shared_ptr<SlaveJoint>& slave,
                             const JointStatus& status) noexcept;

/**
 * @brief 双电机关节主关节。
 */
class MasterJoint : public BaseJoint, public std::enable_shared_from_this<MasterJoint> {
    friend class JointManager<>;

public:
    /**
     * @brief 析构主关节并注销底层电机状态回调。
     */
    ~MasterJoint() override {
        if (motor1_) {
            motor1_->SetOnStatus(nullptr);
        }
        if (motor2_) {
            motor2_->SetOnStatus(nullptr);
        }
    }

    // Master 关节对外控制接口。

    /** @brief 两台电机均带机械抱闸时才支持整关节抱闸。 */
    bool HasBrake() const override {
        return motor1_->HasBrake() && motor2_->HasBrake();
    }

    /** @brief 两次调用合帧：首条返回 true 表示缓存，第二条按最后值下发并返回双电机结果。 */
    bool Brake(bool enabled) override {
        RecordBrakeCommandForLog(enabled);
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (!HasBrake()) {
            WarnProtection("Brake", "The current joint does not use brake-equipped motors");
            return false;
        }
        if (!brake_batch_pending_) {
            brake_batch_motion_calls_ = 0;
            brake_batch_pending_ = true;
            return true;
        }
        brake_batch_pending_ = false;
        // 即使第一台返回失败，也必须向第二台下发同一关节操作。
        const bool motor1_ok = motor1_->Brake(enabled);
        const bool motor2_ok = motor2_->Brake(enabled);
        return motor1_ok && motor2_ok;
    }

    /** @brief 任一电机抱紧即返回 true，并向另一台未抱紧的电机补发抱闸命令。 */
    bool GetBrakeStatus() override {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (!HasBrake()) {
            WarnProtection("GetBrakeStatus",
                           "The current joint does not use brake-equipped motors");
            return false;
        }
        const bool motor1_engaged = motor1_->GetParameter<MotorParameter::BrakeStatus>() == 0;
        const bool motor2_engaged = motor2_->GetParameter<MotorParameter::BrakeStatus>() == 0;
        if (motor1_engaged != motor2_engaged) {
            auto* released_motor = motor1_engaged ? motor2_ : motor1_;
            if (!released_motor->Brake(true)) {
                WarnProtection("GetBrakeStatus",
                               motor1_engaged ? "电机2补抱闸失败" : "电机1补抱闸失败");
            }
        }
        return motor1_engaged || motor2_engaged;
    }

    /**
     * @brief 向两个电机发送停止指令。
     * @param mode 停止模式。
     * @param current 停止电流；非有限值会被归零。
     * @param feedback 底层电机反馈模式。
     */
    void Stop(MotorStopMode mode, float current = 0, int feedback = 1) override {
        RecordStopCommandForLog(mode, current, feedback);
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        ResetControlBatchLocked();
        ClearHoldProtectionLocked();
        SendMotorStopLocked(mode, current, feedback);
    }

    /**
     * @brief 设置关节限位（关节坐标系）。
     * @param limit 限位范围，若 reverse 为 true 则内部翻转存储。
     */
    void SetLimit(Range<float> limit) override {
        SetValidatedLimit(limit);
    }

    /**
     * @brief 在主轴上执行 PVT 位置-速度-扭矩控制。
     * @param kp 位置比例增益。
     * @param kd 速度微分增益。
     * @param pos 目标位置，单位 rad，主轴关节坐标系。
     * @param spd 目标速度，单位 rad/s，主轴关节坐标系。
     * @param torque 目标扭矩，单位 Nm，主轴关节坐标系。
     */
    void PVTControl(float kp, float kd, float pos, float spd, float torque) override {
        PVTControl(kp, kd, pos, spd, torque, PVTControlMode::MotorPVT);
    }

    /**
     * @brief 在主轴上执行带上位机力矩模式选择的 PVT 控制。
     * @param torque_mode 同一合帧批次两次调用必须选择相同的上位机模式才生效；
     * 模式不一致或所需反馈不可用时退回 MotorPVT。
     */
    void PVTControl(float kp, float kd, float pos, float spd, float torque,
                    PVTControlMode torque_mode) override {
        RecordPVTCommandForLog(kp, kd, pos, spd, torque, torque_mode);
        ControlPVT(axis_, kp, kd, pos, spd, torque, torque_mode);
    }

    void SetJointLogContext(std::shared_ptr<JointLogContext> context, std::string name) override {
        BaseJoint::SetJointLogContext(std::move(context), std::move(name));
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        UpdateMotorStatusCallbacksLocked();
    }

    /**
     * @brief 在主轴上执行位置控制。
     * @param position 目标位置，单位 rad，主轴关节坐标系。
     * @param speed 速度上限，单位 rad/s；0 时使用主轴最大速度，负值拒绝。
     * @param current 电流上限，单位 A；0 保留底层默认含义，越界拒绝。
     * @param feedback 底层电机反馈模式。
     */
    void PosControl(float position, float speed = 0, float current = 0, int feedback = 1) override {
        RecordPosCommandForLog(position, speed, current, feedback);
        ControlPosition(axis_, position, speed, current, feedback);
    }

    /**
     * @brief 在主轴上执行速度控制。
     * @param speed 目标速度，单位 rad/s，主轴关节坐标系。
     * @param current 电流上限，单位 A；0 保留底层默认含义，越界拒绝。
     * @param feedback 底层电机反馈模式。
     */
    void SpdControl(float speed, float current = 0, int feedback = 1) override {
        RecordSpdCommandForLog(speed, current, feedback);
        ControlDynamic(DynamicControlMode::Speed, axis_, speed, current, feedback);
    }

    /**
     * @brief 在主轴上执行电流控制。
     * @param current 目标电流，单位 A，主轴关节坐标系方向。
     * @param feedback 底层电机反馈模式。
     */
    void CurControl(float current = 0, int feedback = 1) override {
        RecordCurCommandForLog(current, feedback);
        ControlDynamic(DynamicControlMode::Current, axis_, current, 0.0f, feedback);
    }

    /**
     * @brief 在主轴上执行扭矩控制。
     * @param torque 目标扭矩，单位 Nm，主轴关节坐标系。
     * @param feedback 底层电机反馈模式。
     */
    void TorControl(float torque = 0, int feedback = 1) override {
        RecordTorCommandForLog(torque, feedback);
        ControlDynamic(DynamicControlMode::Torque, axis_, torque, 0.0f, feedback);
    }

    /**
     * @brief 未实现：闭链关节不支持位置偏移获取。
     * @param dir 限位方向，当前未使用。
     * @return 始终返回 NaN。
     */
    float PosAsOffset([[maybe_unused]] int dir) override {
        return std::nanf("");
    }

    /**
     * @brief 获取主轴的状态信息。
     * @param autoSendCommand 为 true 且缓存状态不可用时发送一次反馈命令。
     * @param lifeCycleDeduction 读取状态时扣减的生命周期值。
     * @return 主轴关节坐标系下的状态；状态不可用时返回 std::nullopt。
     * @warning 关节运动过程中不得将 autoSendCommand 设为 true；内部零目标 PVT 反馈命令
     *          可能改变双电机关节运动状态。
     */
    std::optional<JointStatus> GetStatus(bool autoSendCommand = false,
                                         int lifeCycleDeduction = 1) override {
        return GetStatusForAxis(axis_, autoSendCommand, lifeCycleDeduction);
    }

    /**
     * @brief 注册主轴状态异步回调。
     * @param onStatus 状态回调；传入 nullptr 时取消主轴回调。
     */
    void SetOnStatus(std::function<void(const JointStatus&)> onStatus) override {
        SetOnStatusForAxis(axis_, std::move(onStatus));
    }

    // 双电机诊断接口。

    /**
     * @brief 读取电机 1 的位置值（原始电机坐标系，用于调试诊断）。
     * @param read_cache true 时优先读取状态缓存；false 时直接读取电机参数。
     * @param life_cycle_deduction 读取状态缓存时扣减的生命周期值。
     * @return 电机 1 坐标系下的位置，单位 rad。
     */
    float GetMotor1Position(bool read_cache = true, int life_cycle_deduction = 0) {
        return ReadMotorPosition(motor1_, read_cache, life_cycle_deduction);
    }

    /**
     * @brief 读取电机 2 的位置值（原始电机坐标系，用于调试诊断）。
     * @param read_cache true 时优先读取状态缓存；false 时直接读取电机参数。
     * @param life_cycle_deduction 读取状态缓存时扣减的生命周期值。
     * @return 电机 2 坐标系下的位置，单位 rad。
     */
    float GetMotor2Position(bool read_cache = true, int life_cycle_deduction = 0) {
        return ReadMotorPosition(motor2_, read_cache, life_cycle_deduction);
    }

private:
    friend struct DoubleCoupleJoint;
    friend class SlaveJoint;

    // Internal protection keeps its batch latch; explicit Stop cancels the batch above.
    void SendMotorStopLocked(MotorStopMode mode, float current, int feedback) {
        const bool finite_current = std::isfinite(current);
        LogProtection("Stop", "non-finite current; using zero", !finite_current);
        if (!finite_current) {
            current = 0.0f;
        }
        motor1_->Stop<0>(mode, current, feedback);
        motor2_->Stop<0>(mode, current, feedback);
    }

    /**
     * @brief 保护日志点作用域名。
     *
     * 双轴关节的两条轴、两台电机在同一帧里会各自调用同一个辅助函数，同名保护点若共用
     * 一个标志位，一条轴的“未触发”会把另一条轴刚置位的标志位清掉，导致逐帧抖动。
     */
    static const char* AxisScope(DoubleCoupleJointAxis axis) {
        return axis == DoubleCoupleJointAxis::Roll ? "roll" : "pitch";
    }

    static constexpr double kRangeTolerance = 1.0e-6;
    static constexpr double kLimitProtectionTolerance = 1.0e-3;
    static constexpr double kSingularMetricTolerance = 1.0e-9;
    static constexpr double kSingularDirectionMetricThreshold = 0.2;
    static constexpr double kSingularDirectionProbeStep = 0.01;
    static constexpr double kSingularConditionTolerance = 1.0e-3;
    static constexpr double kPi = 3.14159265358979323846;
    static constexpr float kMinimumMotorKd = 0.1f;
    const char* active_control_mode_{"control"};  ///< 由 mutex_ 保护的当前保护日志上下文

    /**
     * @brief 归一化相对零位的电机角度，避免逆解主值减初始角后跳圈。
     * @param position 相对零位的电机角度。
     * @param scope 保护日志点作用域名（motor1/motor2）。
     */
    float NormalizeMotorPosition(double position, const char* scope) const {
        const double normalized = std::remainder(position, 2.0 * kPi);
        LogProtection(active_control_mode_, "motor angle normalized to [-pi, pi]",
                      std::abs(normalized - position) > kRangeTolerance, {}, scope);
        return static_cast<float>(normalized);
    }

    struct AxisRuntimeParams {
        Range<float> limit_{0.0f, 0.0f};
        float maxSpd_{0.0f};
        float maxTor_{0.0f};
        bool reverse_{false};
    };

    struct JointPose {
        double roll{0.0};
        double pitch{0.0};
    };

    struct MotorPositions {
        float motor1{0.0f};
        float motor2{0.0f};
    };

    struct MotorCurrents {
        float motor1{0.0f};
        float motor2{0.0f};
    };

    struct AxisPVTCommand {
        float kp{0.0f};
        float kd{0.0f};
        double speed{0.0};
        double torque{0.0};
    };

    struct AxisPositionCommand {
        float speed{0.0f};
        float current{0.0f};
    };

    /// 动态类控制的三种模式，决定合帧时使用哪一组轴槽位与哪个联合映射。
    enum class DynamicControlMode {
        Speed,
        Torque,
        Current,
    };

    /// 动态类控制单个轴的目标与电流上限缓存。
    struct AxisDynamicCommand {
        double value{0.0};  ///< 求解器坐标系下的轴目标（速度 rad/s / 扭矩 Nm / 电流 A）
        float current{0.0f};  ///< 两台电机共用的电流上限，入口验证两台电机均允许
    };

    struct ActualPoseState {
        JointPose pose{};
        MotorPositions positions{};
        couple_joint::MechanismSolver solver;
    };

    struct JointFeedbackState {
        JointPose pose{};
        JointStatus roll_status{};
        JointStatus pitch_status{};
        bool position_solved{false};
    };

    /**
     * @brief 构造 MasterJoint：初始化求解器、创建双电机、绑定从关节。
     * @param config master 侧双电机关节配置。
     * @param motor1 已创建的第 1 个底层电机。
     * @param motor2 已创建的第 2 个底层电机。
     * @param slave 待绑定的从关节。
     * @param motor_limit1 合并后的电机 1 相对位置限位。
     * @param motor_limit2 合并后的电机 2 相对位置限位。
     * @throw std::invalid_argument 当电机为空、力矩常数无效或限位无效时抛出。
     * @throw JointException 当闭链求解器标定失败时抛出。
     */
    explicit MasterJoint(const DoubleCoupleJoint::Config& config, Motor* motor1, Motor* motor2,
                         std::shared_ptr<SlaveJoint> slave,
                         std::optional<Range<float>> motor_limit1,
                         std::optional<Range<float>> motor_limit2)
        : BaseJoint{config.limit, config.maxSpd, config.maxTor, config.reverse},
          axis_(config.axis),
          calibration_(config.calibration) {
        if (!motor1 || !motor2) {
            throw std::invalid_argument("MasterJoint motors must not be null");
        }
        solver_ = std::make_unique<couple_joint::MechanismSolver>(config.calibration);
        const auto calibration_status = solver_->GetCalibrationStatus();
        if (calibration_status != couple_joint::CalibrationStatus::Success) {
            throw JointException(
                JointError::kDoubleCoupleJointCalibrationFailed,
                std::string("DoubleCoupleJoint solver calibration failed: ") +
                    couple_joint::MechanismSolver::StatusMessage(calibration_status));
        }
        solver_parameters_ = solver_->GetParameters();
        motor1_ = motor1;
        motor2_ = motor2;
        motor1_ranges_ = motor1_->GetProperties();
        motor1_kt_ = motor1_ranges_.kt;
        motor2_ranges_ = motor2_->GetProperties();
        motor2_kt_ = motor2_ranges_.kt;
        motor1_position_limit_ = motor_limit1.value_or(motor1_ranges_.position);
        motor2_position_limit_ = motor_limit2.value_or(motor2_ranges_.position);
        ValidateOrderedRange(motor1_position_limit_);
        ValidateOrderedRange(motor2_position_limit_);
        const auto motor1_command_position_range =
            CoverRange(motor1_ranges_.position, motor1_position_limit_);
        if (motor_limit1 && (motor1_command_position_range.min < motor1_ranges_.position.min ||
                             motor1_command_position_range.max > motor1_ranges_.position.max)) {
            if (motor1_->SetPVTPosRange(motor1_command_position_range, false)) {
                motor1_ranges_.position = motor1_command_position_range;
            }
        }
        const auto motor2_command_position_range =
            CoverRange(motor2_ranges_.position, motor2_position_limit_);
        if (motor_limit2 && (motor2_command_position_range.min < motor2_ranges_.position.min ||
                             motor2_command_position_range.max > motor2_ranges_.position.max)) {
            if (motor2_->SetPVTPosRange(motor2_command_position_range, false)) {
                motor2_ranges_.position = motor2_command_position_range;
            }
        }
        if (!IsValidTorqueConstant(motor1_kt_) || !IsValidTorqueConstant(motor2_kt_)) {
            throw std::invalid_argument("MasterJoint motor kt must be finite and positive");
        }
        slave_ = slave;
        InitializePoseFromCurrentMotorPositions();
    }

    /**
     * @brief 绑定从关节轴标识。
     * @param axis 从关节对应的输出轴。
     */
    void BindSlaveAxis(DoubleCoupleJointAxis axis) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        slave_axis_ = axis;
    }

    /**
     * @brief 判断运动学求解状态是否为可用状态（Success 或 NearSingular）。
     * @param status 求解器返回的状态。
     * @return true 表示该状态可继续进行软件限位或控制映射判断。
     */
    static bool IsUsable(couple_joint::SolveStatus status) {
        return status == couple_joint::SolveStatus::Success ||
               status == couple_joint::SolveStatus::NearSingular;
    }

    /**
     * @brief 设置位姿结构中指定轴的值。
     * @param axis 需要写入的输出轴。
     * @param value 写入值，单位 rad，求解器坐标系。
     * @param pose 被修改的位姿对象，不能为空。
     */
    static void SetAxisValue(DoubleCoupleJointAxis axis, double value, JointPose* pose) {
        if (axis == DoubleCoupleJointAxis::Roll) {
            pose->roll = value;
        } else {
            pose->pitch = value;
        }
    }

    /**
     * @brief 读取位姿结构中指定轴的值。
     * @param axis 需要读取的输出轴。
     * @param pose 输入位姿对象。
     * @return 指定轴的值，单位 rad，求解器坐标系。
     */
    static double AxisValue(DoubleCoupleJointAxis axis, const JointPose& pose) {
        return axis == DoubleCoupleJointAxis::Roll ? pose.roll : pose.pitch;
    }

    /**
     * @brief 计算覆盖两个范围的最小范围。
     * @param lhs 第一个范围。
     * @param rhs 第二个范围。
     * @return 同时覆盖 lhs 和 rhs 的范围。
     */
    static Range<float> CoverRange(Range<float> lhs, Range<float> rhs) {
        return Range<float>{std::min(lhs.min, rhs.min), std::max(lhs.max, rhs.max)};
    }

    /**
     * @brief 判断传动信息是否包含硬奇异（不可通过的控制奇异）。
     * @param transmission 求解器返回的微分传动信息。
     * @return true 表示 F_theta、F_q 或 J 已数值奇异。
     */
    static bool HasHardSingularity(const couple_joint::TransmissionInfo& transmission) {
        return transmission.singular_fu || transmission.singular_fq || transmission.singular_j;
    }

    /**
     * @brief 判断浮点值是否在指定范围内（含容差）。
     * @param value 待检查值。
     * @param range 允许范围。
     * @return true 表示 value 为有限值且在 range 容差范围内。
     */
    static bool IsInRange(float value, const Range<float>& range) {
        return std::isfinite(value) && value >= range.min - kRangeTolerance &&
               value <= range.max + kRangeTolerance;
    }

    /**
     * @brief 判断浮点值是否在带安全缓冲的范围内。
     * @param value 待检查值。
     * @param range 原始允许范围。
     * @return true 表示 value 为有限值且位于 range 去除 kLimitSlack 后的范围内。
     */
    static bool IsInRangeWithSlack(float value, const Range<float>& range) {
        return std::isfinite(value) && value >= range.min + kLimitSlack - kRangeTolerance &&
               value <= range.max - kLimitSlack + kRangeTolerance;
    }

    /**
     * @brief 判断命令是否继续朝软件限位外运动。
     * @param value 当前值。
     * @param range 软件限位范围。
     * @param command 控制命令方向；正数表示朝 max，负数表示朝 min。
     * @return true 表示当前位置已在缓冲区且命令仍朝硬限位方向。
     */
    static bool IsCommandLeavingLimit(float value, const Range<float>& range, double command) {
        if (!std::isfinite(value) || !std::isfinite(command) || command == 0.0) {
            return false;
        }
        if (value <= range.min + kLimitSlack + kLimitProtectionTolerance && command < 0.0) {
            return true;
        }
        return value >= range.max - kLimitSlack - kLimitProtectionTolerance && command > 0.0;
    }

    /**
     * @brief 判断指定关节轴命令是否朝自身软件限位外运动。
     * @param axis 被控制的输出轴。
     * @param pose 当前求解器坐标系位姿。
     * @param solver_command 求解器坐标系下的速度、力矩或电流方向命令。
     * @return true 表示命令需要被限位保护拦截。
     */
    bool AxisCommandLeavingLimit(DoubleCoupleJointAxis axis, const JointPose& pose,
                                 double solver_command) const {
        const bool blocked = IsCommandLeavingLimit(static_cast<float>(AxisValue(axis, pose)),
                                                   AxisParams(axis).limit_, solver_command);
        LogProtection(active_control_mode_, "joint outward limit", blocked, {}, AxisScope(axis));
        return blocked;
    }

    /**
     * @brief 判断任一电机命令是否朝自身软件限位外运动。
     * @param positions 当前双电机相对零位位置。
     * @param motor1_command 电机 1 控制命令方向。
     * @param motor2_command 电机 2 控制命令方向。
     * @return true 表示至少一个电机命令需要被限位保护拦截。
     */
    bool MotorCommandsLeavingLimit(const MotorPositions& positions, double motor1_command,
                                   double motor2_command) const {
        const bool blocked =
            IsCommandLeavingLimit(positions.motor1, motor1_position_limit_, motor1_command) ||
            IsCommandLeavingLimit(positions.motor2, motor2_position_limit_, motor2_command);
        LogProtection(active_control_mode_, "motor outward limit", blocked);
        return blocked;
    }

    static bool IsPVTCommandLeavingLimit(float actual, const Range<float>& range, double target,
                                         double speed, double effort) {
        const double lower = static_cast<double>(range.min) + kLimitSlack;
        const double upper = static_cast<double>(range.max) - kLimitSlack;
        bool leaving_limit = false;
        if (actual <= lower + kLimitProtectionTolerance) {
            leaving_limit = target < actual || speed < 0.0 || effort < 0.0;
        }
        if (actual >= upper - kLimitProtectionTolerance) {
            leaving_limit = leaving_limit || target > actual || speed > 0.0 || effort > 0.0;
        }
        return leaving_limit;
    }

    bool JointPVTCommandsLeavingLimit(const JointPose& actual_pose, const JointPose& target_pose,
                                      const JointPose& speed, double roll_effort,
                                      double pitch_effort) const {
        const auto leaves = [&](DoubleCoupleJointAxis axis, double actual, double target,
                                double speed, double effort) {
            if (axis != axis_ && slave_.expired()) {
                return false;
            }
            return IsPVTCommandLeavingLimit(static_cast<float>(actual), AxisParams(axis).limit_,
                                            target, speed, effort);
        };
        const bool blocked = leaves(DoubleCoupleJointAxis::Roll, actual_pose.roll, target_pose.roll,
                                    speed.roll, roll_effort) ||
                             leaves(DoubleCoupleJointAxis::Pitch, actual_pose.pitch,
                                    target_pose.pitch, speed.pitch, pitch_effort);
        LogProtection(active_control_mode_, "PVT joint outward limit", blocked);
        return blocked;
    }

    bool MotorPVTCommandsLeavingLimit(const MotorPositions& actual_positions,
                                      const MotorPositions& target_positions, double motor1_speed,
                                      double motor2_speed, double motor1_effort,
                                      double motor2_effort) const {
        const bool blocked =
            IsPVTCommandLeavingLimit(actual_positions.motor1, motor1_position_limit_,
                                     target_positions.motor1, motor1_speed, motor1_effort) ||
            IsPVTCommandLeavingLimit(actual_positions.motor2, motor2_position_limit_,
                                     target_positions.motor2, motor2_speed, motor2_effort);
        LogProtection(active_control_mode_, "PVT motor outward limit", blocked);
        return blocked;
    }

    /**
     * @brief 判断电机力矩常数是否可用于电流/力矩换算。
     * @param kt 待检查的力矩常数。
     * @return true 表示 kt 为有限正数。
     */
    static bool IsValidTorqueConstant(float kt) {
        return std::isfinite(kt) && kt > 0.0f;
    }

    /**
     * @brief 获取指定轴的运行时参数（限速、限扭、反向标志）。
     * @param axis 需要查询的输出轴。
     * @return 该轴对应的限位、最大速度、最大扭矩和 reverse 标志。
     * @throw JointException 当查询从轴但从关节尚未绑定时抛出。
     */
    AxisRuntimeParams AxisParams(DoubleCoupleJointAxis axis) const;

    /**
     * @brief 电机 1 位置转换到求解器 theta1 角度。
     * @param motor_position 电机 1 相对零位的位置。
     * @return 求解器 theta1 绝对角度。
     */
    double Motor1ToSolverTheta(float motor_position) const {
        return solver_parameters_.initial_theta1 + motor_position;
    }

    /**
     * @brief 电机 2 位置转换到求解器 theta2 角度。
     * @param motor_position 电机 2 相对零位的位置。
     * @return 求解器 theta2 绝对角度。
     */
    double Motor2ToSolverTheta(float motor_position) const {
        return solver_parameters_.initial_theta2 + motor_position;
    }

    /**
     * @brief 取两个浮点状态值中的较大值（处理非有限值）。
     * @param lhs 左侧状态值。
     * @param rhs 右侧状态值。
     * @return 两个有限值中的较大值；若一侧非有限则返回另一侧。
     */
    static float MaxStatusValue(float lhs, float rhs) {
        if (!std::isfinite(lhs)) {
            return rhs;
        }
        if (!std::isfinite(rhs)) {
            return lhs;
        }
        return std::max(lhs, rhs);
    }

    /**
     * @brief 取两个电机错误状态中更严重的一个。
     * @param lhs 左侧电机错误状态。
     * @param rhs 右侧电机错误状态。
     * @return NoError 以外枚举值更大的错误状态；一侧 NoError 时返回另一侧。
     */
    static MotorError MaxStatusError(MotorError lhs, MotorError rhs) {
        if (lhs == MotorError::NoError) {
            return rhs;
        }
        if (rhs == MotorError::NoError) {
            return lhs;
        }
        return static_cast<int>(lhs) >= static_cast<int>(rhs) ? lhs : rhs;
    }

    /**
     * @brief 判断求解结果是否可执行（状态可用且无硬奇异）。
     * @param status 求解器状态。
     * @param transmission 求解器传动信息。
     * @return true 表示状态可用且不存在硬奇异。
     */
    static bool IsExecutableSolveResult(couple_joint::SolveStatus status,
                                        const couple_joint::TransmissionInfo& transmission) {
        return IsUsable(status) && !HasHardSingularity(transmission);
    }

    /**
     * @brief 判断双电机位置是否均在允许范围内。
     * @param positions 待检查的双电机相对零位位置。
     * @param require_slack true 时要求位置位于带 kLimitSlack 的安全范围内。
     * @return true 表示两个电机位置都满足对应范围约束。
     */
    bool MotorPositionsInRange(const MotorPositions& positions, bool require_slack = false) const {
        if (require_slack) {
            return IsInRangeWithSlack(positions.motor1, motor1_position_limit_) &&
                   IsInRangeWithSlack(positions.motor2, motor2_position_limit_);
        }
        return IsInRange(positions.motor1, motor1_position_limit_) &&
               IsInRange(positions.motor2, motor2_position_limit_);
    }

    bool MotorPositionsInPVTRange(const MotorPositions& positions) const {
        return IsInRange(positions.motor1, motor1_ranges_.position) &&
               IsInRange(positions.motor2, motor2_ranges_.position);
    }

    /**
     * @brief 计算指定输出轴的奇异方向指标。
     * @param transmission 当前位姿处的传动信息。
     * @param axis 需要评估的输出轴。
     * @return reduction 对应列范数的倒数；值越小表示该轴越接近奇异弱方向。
     */
    static double AxisDirectionMetric(const couple_joint::TransmissionInfo& transmission,
                                      DoubleCoupleJointAxis axis) {
        const auto& R = transmission.reduction;
        const double col0 = axis == DoubleCoupleJointAxis::Roll ? R.m00 : R.m01;
        const double col1 = axis == DoubleCoupleJointAxis::Roll ? R.m10 : R.m11;
        const double norm = std::hypot(col0, col1);
        if (!std::isfinite(norm)) {
            return 0.0;
        }
        if (norm <= kSingularMetricTolerance) {
            return std::numeric_limits<double>::infinity();
        }
        return 1.0 / norm;
    }

    /**
     * @brief 判断指定输出轴当前是否应被奇异限位逻辑关注。
     * @param transmission 当前位姿处的传动信息。
     * @param axis 需要判断的输出轴。
     * @return true 表示该轴在 near_singular 区域内属于受限轴。
     */
    static bool IsAxisConstrainedBySingularity(const couple_joint::TransmissionInfo& transmission,
                                               DoubleCoupleJointAxis axis) {
        if (!transmission.near_singular) {
            return false;
        }

        const double roll_metric = AxisDirectionMetric(transmission, DoubleCoupleJointAxis::Roll);
        const double pitch_metric = AxisDirectionMetric(transmission, DoubleCoupleJointAxis::Pitch);
        const bool roll_limited = roll_metric <= kSingularDirectionMetricThreshold;
        const bool pitch_limited = pitch_metric <= kSingularDirectionMetricThreshold;
        if (roll_limited || pitch_limited) {
            return axis == DoubleCoupleJointAxis::Roll ? roll_limited : pitch_limited;
        }

        return axis == DoubleCoupleJointAxis::Roll ? roll_metric <= pitch_metric
                                                   : pitch_metric <= roll_metric;
    }

    /**
     * @brief 比较试探点是否比当前点更接近奇异。
     * @param current 当前点的传动信息。
     * @param probe 试探点的传动信息。
     * @return true 表示试探点进入硬奇异、sigma_min_j 变小或 condition_j 变大。
     */
    static bool TransmissionWorseTowardSingularity(const couple_joint::TransmissionInfo& current,
                                                   const couple_joint::TransmissionInfo& probe) {
        if (HasHardSingularity(probe)) {
            return true;
        }
        if (current.near_singular && !probe.near_singular) {
            return false;
        }
        if (std::isfinite(current.sigma_min_j) && std::isfinite(probe.sigma_min_j) &&
            probe.sigma_min_j + kSingularMetricTolerance < current.sigma_min_j) {
            return true;
        }
        if (std::isfinite(current.condition_j) && std::isfinite(probe.condition_j) &&
            probe.condition_j > current.condition_j * (1.0 + kSingularConditionTolerance)) {
            return true;
        }
        return false;
    }

    /**
     * @brief 判断指定轴命令是否朝奇异区更深处运动。
     * @param current_pose 当前求解器坐标系位姿。
     * @param transmission 当前位姿处的传动信息。
     * @param axis 被控制的输出轴。
     * @param command 求解器坐标系下的命令方向；正负号用于判断运动方向。
     * @return true 表示命令需要被位置保持保护拦截。
     */
    bool IsCommandEnteringSingularity(const JointPose& current_pose,
                                      const couple_joint::TransmissionInfo& transmission,
                                      DoubleCoupleJointAxis axis, double command) const {
        if (std::abs(command) <= kRangeTolerance || !transmission.near_singular ||
            !IsAxisConstrainedBySingularity(transmission, axis)) {
            // 命令为零、未进入近奇异区或该轴不受奇异约束：本轴不在奇异保护中。
            // near_singular 门控收在函数内部，保证调用方每周期都能观测到条件转假。
            const char* scope = AxisScope(axis);
            LogProtection(active_control_mode_, "singularity probe unreachable/hard singular",
                          false, {}, scope);
            LogProtection(active_control_mode_, "command enters dead point", false, {}, scope);
            return false;
        }

        JointPose probe_pose = current_pose;
        const double direction = command > 0.0 ? 1.0 : -1.0;
        SetAxisValue(axis, AxisValue(axis, probe_pose) + direction * kSingularDirectionProbeStep,
                     &probe_pose);

        auto probe_solver = *solver_;
        const auto probe = probe_solver.SolveInverse(probe_pose.roll, probe_pose.pitch);
        if (!IsUsable(probe.status) || HasHardSingularity(probe.transmission)) {
            const char* scope = AxisScope(axis);
            LogProtection(active_control_mode_, "singularity probe unreachable/hard singular", true,
                          {}, scope);
            LogProtection(active_control_mode_, "command enters dead point", false, {}, scope);
            return true;
        }
        const bool blocked = TransmissionWorseTowardSingularity(transmission, probe.transmission);
        const char* scope = AxisScope(axis);
        LogProtection(active_control_mode_, "command enters dead point", blocked, {}, scope);
        LogProtection(active_control_mode_, "singularity probe unreachable/hard singular", false,
                      {}, scope);
        return blocked;
    }

    bool PVTCommandsEnteringSingularity(const JointPose& actual_pose, const JointPose& target_pose,
                                        const couple_joint::TransmissionInfo& transmission,
                                        const JointPose& speed, double roll_effort,
                                        double pitch_effort) const {
        const auto enters = [&](DoubleCoupleJointAxis axis, double target, double speed,
                                double effort) {
            const double position_delta = target - AxisValue(axis, actual_pose);
            return IsCommandEnteringSingularity(actual_pose, transmission, axis, position_delta) ||
                   IsCommandEnteringSingularity(actual_pose, transmission, axis, speed) ||
                   IsCommandEnteringSingularity(actual_pose, transmission, axis, effort);
        };
        return enters(DoubleCoupleJointAxis::Roll, target_pose.roll, speed.roll, roll_effort) ||
               enters(DoubleCoupleJointAxis::Pitch, target_pose.pitch, speed.pitch, pitch_effort);
    }

    bool MapJointSpeed(const couple_joint::MechanismSolver& pose_solver, double solver_roll_speed,
                       double solver_pitch_speed, float* motor1_speed, float* motor2_speed) const {
        const auto velocity =
            pose_solver.SolveInverseVelocity(solver_roll_speed, solver_pitch_speed);
        if (!IsUsable(velocity.status) || HasHardSingularity(velocity.transmission)) {
            return false;
        }
        *motor1_speed = static_cast<float>(velocity.theta1);
        *motor2_speed = static_cast<float>(velocity.theta2);
        return std::isfinite(*motor1_speed) && std::isfinite(*motor2_speed);
    }

    bool MapJointTorque(const couple_joint::MechanismSolver& pose_solver, double solver_roll_torque,
                        double solver_pitch_torque, float* motor1_torque,
                        float* motor2_torque) const {
        const auto torque_result =
            pose_solver.SolveInverseTorque(solver_roll_torque, solver_pitch_torque);
        if (!IsUsable(torque_result.status) || HasHardSingularity(torque_result.transmission)) {
            return false;
        }
        *motor1_torque = static_cast<float>(torque_result.theta1);
        *motor2_torque = static_cast<float>(torque_result.theta2);
        return std::isfinite(*motor1_torque) && std::isfinite(*motor2_torque);
    }

    /**
     * @brief 将两轴关节增益按 diag(J^T K J) 转换为两台电机的独立增益。
     */
    bool MapPVTGains(const couple_joint::MechanismSolver& pose_solver, const AxisPVTCommand& roll,
                     const AxisPVTCommand& pitch, float* motor1_kp, float* motor2_kp,
                     float* motor1_kd, float* motor2_kd) const {
        const auto transmission = pose_solver.GetTransmission();
        if (HasHardSingularity(transmission)) {
            return false;
        }
        const auto& J = transmission.velocity_jacobian;
        *motor1_kp = static_cast<float>(J.m00 * J.m00 * roll.kp + J.m10 * J.m10 * pitch.kp);
        *motor2_kp = static_cast<float>(J.m01 * J.m01 * roll.kp + J.m11 * J.m11 * pitch.kp);
        *motor1_kd = static_cast<float>(J.m00 * J.m00 * roll.kd + J.m10 * J.m10 * pitch.kd);
        *motor2_kd = static_cast<float>(J.m01 * J.m01 * roll.kd + J.m11 * J.m11 * pitch.kd);
        if (*motor1_kd > 0.0f) {
            LogProtection(active_control_mode_, "motor1 minimum kd", *motor1_kd < kMinimumMotorKd);
            *motor1_kd = std::max(*motor1_kd, kMinimumMotorKd);
        } else {
            // kd 为 0 时不参与最小阻尼保护，显式清除标志位
            LogProtection(active_control_mode_, "motor1 minimum kd", false);
        }
        if (*motor2_kd > 0.0f) {
            LogProtection(active_control_mode_, "motor2 minimum kd", *motor2_kd < kMinimumMotorKd);
            *motor2_kd = std::max(*motor2_kd, kMinimumMotorKd);
        } else {
            LogProtection(active_control_mode_, "motor2 minimum kd", false);
        }
        return std::isfinite(*motor1_kp) && std::isfinite(*motor2_kp) &&
               std::isfinite(*motor1_kd) && std::isfinite(*motor2_kd);
    }

    static bool MatchMotorTerm(double torque, float gain, float actual, float* target) {
        if (!std::isfinite(torque) || !std::isfinite(gain) || !std::isfinite(actual) ||
            gain < 0.0f) {
            return false;
        }
        if (gain <= std::numeric_limits<float>::epsilon()) {
            *target = actual;
            return std::abs(torque) <= kRangeTolerance;
        }
        const double value = static_cast<double>(actual) + torque / static_cast<double>(gain);
        if (!std::isfinite(value)) {
            return false;
        }
        *target = static_cast<float>(value);
        return std::isfinite(*target);
    }

    /** @brief 按完整关节 Kd 力矩反算电机目标速度，补偿交叉耦合项。 */
    bool MatchCoupledDampingTargets(const couple_joint::MechanismSolver& actual_solver,
                                    const AxisPVTCommand& roll, const AxisPVTCommand& pitch,
                                    float motor1_kd, float motor2_kd, float actual_motor1_speed,
                                    float actual_motor2_speed, float* target_speed1,
                                    float* target_speed2) const {
        const auto velocity =
            actual_solver.SolveForwardVelocity(actual_motor1_speed, actual_motor2_speed);
        if (!IsExecutableSolveResult(velocity.status, velocity.transmission)) {
            return false;
        }
        const JointPose kd_joint{roll.kd * (roll.speed - velocity.roll),
                                 pitch.kd * (pitch.speed - velocity.pitch)};
        float kd_torque1 = 0.0f;
        float kd_torque2 = 0.0f;
        if (!MapJointTorque(actual_solver, kd_joint.roll, kd_joint.pitch, &kd_torque1,
                            &kd_torque2) ||
            !MatchMotorTerm(kd_torque1, motor1_kd, actual_motor1_speed, target_speed1) ||
            !MatchMotorTerm(kd_torque2, motor2_kd, actual_motor2_speed, target_speed2)) {
            return false;
        }
        return true;
    }

    /**
     * @brief 基于力矩比的双轴电流分配：两条虚拟轴电流到双电机电流。
     *
     * 按该轴 1 Nm 对应的双电机电流权重分别分配，再叠加两轴输出；不做电机限幅。
     * @param pose_solver 已位于目标位姿的求解器副本。
     * @param roll_current 求解器坐标系下的 roll 轴电流命令。
     * @param pitch_current 求解器坐标系下的 pitch 轴电流命令。
     * @param motor_currents 输出双电机电流。
     * @return true 表示分配成功且电流有限。
     */
    bool MapJointCurrent(const couple_joint::MechanismSolver& pose_solver, double roll_current,
                         double pitch_current, MotorCurrents* motor_currents) const {
        if (!std::isfinite(roll_current) || !std::isfinite(pitch_current) || motor1_kt_ <= 0.0f ||
            motor2_kt_ <= 0.0f) {
            return false;
        }
        if (roll_current == 0.0 && pitch_current == 0.0) {
            *motor_currents = MotorCurrents{};
            return true;
        }

        double motor1_current = 0.0;
        double motor2_current = 0.0;
        for (const auto axis : {DoubleCoupleJointAxis::Roll, DoubleCoupleJointAxis::Pitch}) {
            const double commanded =
                axis == DoubleCoupleJointAxis::Roll ? roll_current : pitch_current;
            if (commanded == 0.0) {
                continue;
            }
            const double unit_axis_torque = std::copysign(1.0, commanded);
            const auto torque_result = axis == DoubleCoupleJointAxis::Roll
                                           ? pose_solver.SolveInverseTorque(unit_axis_torque, 0.0)
                                           : pose_solver.SolveInverseTorque(0.0, unit_axis_torque);
            if (!IsUsable(torque_result.status) || HasHardSingularity(torque_result.transmission)) {
                return false;
            }

            const double weight1 = torque_result.theta1 / motor1_kt_;
            const double weight2 = torque_result.theta2 / motor2_kt_;
            const double weight_sum = std::abs(weight1) + std::abs(weight2);
            if (!std::isfinite(weight_sum) ||
                weight_sum <= std::numeric_limits<double>::epsilon()) {
                return false;
            }

            // weight_sum A produces one Nm on this virtual axis.
            if (std::abs(commanded) > AxisParams(axis).maxTor_ * weight_sum) {
                return false;
            }
            const double current_magnitude = std::abs(commanded);
            motor1_current += (weight1 * current_magnitude) / weight_sum;
            motor2_current += (weight2 * current_magnitude) / weight_sum;
        }

        motor_currents->motor1 = static_cast<float>(motor1_current);
        motor_currents->motor2 = static_cast<float>(motor2_current);
        return std::isfinite(motor_currents->motor1) && std::isfinite(motor_currents->motor2);
    }

    void StoreActualPoseLocked(const couple_joint::MechanismSolver& feedback_solver,
                               const MotorPositions& positions) {
        const auto state = feedback_solver.GetState();
        actual_pose_state_.emplace(
            ActualPoseState{JointPose{state.roll, state.pitch}, positions, feedback_solver});
    }

    std::optional<ActualPoseState> ResolveActualPoseLocked(
        const std::optional<MotorStatus>& motor1_status,
        const std::optional<MotorStatus>& motor2_status, bool& feedback_solved) {
        const bool feedback_available = motor1_status && motor2_status &&
                                        std::isfinite(motor1_status->position) &&
                                        std::isfinite(motor2_status->position);
        LogProtection(active_control_mode_, "feedback unavailable; cached actual pose",
                      !feedback_available);
        if (feedback_available) {
            const MotorPositions status_positions{motor1_status->position, motor2_status->position};
            auto feedback_solver = actual_pose_state_ ? actual_pose_state_->solver : *solver_;
            const auto forward =
                feedback_solver.SolveForward(Motor1ToSolverTheta(motor1_status->position),
                                             Motor2ToSolverTheta(motor2_status->position));
            const bool forward_executable =
                IsExecutableSolveResult(forward.status, forward.transmission);
            LogProtection(active_control_mode_, "forward solve failed; cached actual pose",
                          !forward_executable);
            if (forward_executable) {
                StoreActualPoseLocked(feedback_solver, status_positions);
                feedback_solved = true;
            }
        }
        if (!actual_pose_state_) {
            return std::nullopt;
        }
        return actual_pose_state_;
    }

    /**
     * @brief 从当前双电机位置初始化闭链目标位姿缓存。
     * @throw JointException 当电机位置不可用或当前闭链姿态不可执行时抛出。
     */
    void InitializePoseFromCurrentMotorPositions() {
        const float motor1_position = ReadMotorPosition(motor1_, false, 0);
        const float motor2_position = ReadMotorPosition(motor2_, false, 0);
        if (!std::isfinite(motor1_position) || !std::isfinite(motor2_position)) {
            throw JointException(JointError::kDoubleCoupleJointSolveFailed,
                                 "DoubleCoupleJoint initial motor position is not finite");
        }
        const auto forward = solver_->SolveForward(Motor1ToSolverTheta(motor1_position),
                                                   Motor2ToSolverTheta(motor2_position));
        if (!IsExecutableSolveResult(forward.status, forward.transmission)) {
            throw JointException(
                JointError::kDoubleCoupleJointSolveFailed,
                std::string("DoubleCoupleJoint initial motor pose is not executable: ") +
                    couple_joint::MechanismSolver::StatusMessage(forward.status));
        }
        const auto state = solver_->GetState();
        target_roll_ = state.roll;
        target_pitch_ = state.pitch;
        StoreActualPoseLocked(*solver_, MotorPositions{motor1_position, motor2_position});
    }

    /**
     * @brief 提交并缓存目标位姿及求解器状态。
     * @param pose 已执行成功的目标位姿。
     * @param pose_solver 已求解到目标位姿的求解器副本。
     */
    void CommitTargetPose(const JointPose& pose, const couple_joint::MechanismSolver& pose_solver) {
        *solver_ = pose_solver;
        target_roll_ = pose.roll;
        target_pitch_ = pose.pitch;
    }

    // 半帧只保存候选命令；已提交目标与实际反馈永远不需要回滚。
    struct PositionBatch {
        JointPose target;
        AxisPVTCommand roll_pvt;
        AxisPVTCommand pitch_pvt;
        std::optional<AxisPositionCommand> roll_position;
        std::optional<AxisPositionCommand> pitch_position;
        std::optional<PVTControlMode> pvt_mode;
    };

    struct DynamicBatch {
        DynamicControlMode mode;
        std::array<std::optional<AxisDynamicCommand>, 2> commands;
    };

    // Each motion frame takes two calls; expire on the fifth call after a brake half-frame.
    void AgeBrakeBatchLocked() {
        if (brake_batch_pending_ && ++brake_batch_motion_calls_ > 4) {
            brake_batch_pending_ = false;
        }
    }

    bool CanFeedbackRefreshTargetLocked() const {
        return !position_batch_ && !dynamic_batch_ && !position_target_feedback_lock_axis_;
    }

    /** @brief 取消候选命令；显式 Stop 同时清除已记住的轴指令。 */
    void ResetControlBatchLocked() {
        brake_batch_pending_ = false;
        position_batch_.reset();
        dynamic_batch_.reset();
        dynamic_commands_ = {};
        roll_pvt_command_ = {};
        pitch_pvt_command_ = {};
        position_target_feedback_lock_axis_.reset();
    }

    /** @brief 丢弃半帧，按机械约束保持位置；无法保持时对双电机执行 FullBrake。 */
    void ProtectLocked(int feedback, std::optional<MotorPositions> positions = std::nullopt) {
        position_batch_.reset();
        dynamic_batch_.reset();
        dynamic_commands_ = {};
        LogProtection(active_control_mode_, "position hold", true);
        try {
            if (!positions) {
                const auto first = motor1_->GetStatus(0);
                const auto second = motor2_->GetStatus(0);
                if (first && second)
                    positions = MotorPositions{first->position, second->position};
            }
            do {
                if (!positions || !std::isfinite(positions->motor1) ||
                    !std::isfinite(positions->motor2))
                    break;
                const float max_torque =
                    std::max(AxisParams(axis_).maxTor_, AxisParams(slave_axis_).maxTor_);
                const float speed1 = motor1_ranges_.speed.max;
                const float speed2 = motor2_ranges_.speed.max;
                const float current1 =
                    std::min(max_torque / motor1_kt_, motor1_ranges_.current.max);
                const float current2 =
                    std::min(max_torque / motor2_kt_, motor2_ranges_.current.max);
                if (!std::isfinite(speed1) || speed1 <= 0.0f || !std::isfinite(speed2) ||
                    speed2 <= 0.0f || !std::isfinite(current1) || current1 <= 0.0f ||
                    !std::isfinite(current2) || current2 <= 0.0f)
                    break;

                const float lower1 =
                    std::max(motor1_position_limit_.min + kLimitSlack, -static_cast<float>(kPi));
                const float upper1 =
                    std::min(motor1_position_limit_.max - kLimitSlack, static_cast<float>(kPi));
                const float lower2 =
                    std::max(motor2_position_limit_.min + kLimitSlack, -static_cast<float>(kPi));
                const float upper2 =
                    std::min(motor2_position_limit_.max - kLimitSlack, static_cast<float>(kPi));
                const bool slack_available = lower1 <= upper1 && lower2 <= upper2;
                LogProtection(active_control_mode_, "motor limit cannot contain two-degree slack",
                              !slack_available);
                if (!slack_available)
                    break;

                // 仅故障保持需要把反馈收回机械边界；原始反馈不能取模跳圈。
                const float position1 = ClampControl(positions->motor1, lower1, upper1,
                                                     active_control_mode_, "motor1 position");
                const float position2 = ClampControl(positions->motor2, lower2, upper2,
                                                     active_control_mode_, "motor2 position");
                auto solver = actual_pose_state_ ? actual_pose_state_->solver : *solver_;
                const auto forward = solver.SolveForward(Motor1ToSolverTheta(position1),
                                                         Motor2ToSolverTheta(position2));
                const auto pose = solver.GetState();
                const bool valid =
                    IsExecutableSolveResult(forward.status, forward.transmission) &&
                    IsInRange(pose.roll, AxisParams(DoubleCoupleJointAxis::Roll).limit_) &&
                    IsInRange(pose.pitch, AxisParams(DoubleCoupleJointAxis::Pitch).limit_);
                LogProtection(active_control_mode_,
                              "clipped position violates closed-chain/joint limits", !valid);
                if (!valid)
                    break;
                motor1_->PosControl<0>(position1, speed1, current1, feedback);
                motor2_->PosControl<0>(position2, speed2, current2, feedback);
                return;
            } while (false);
        } catch (const std::exception& ex) {
            JointSdkLogWarn("DoubleCoupleJoint hold failed: {}", ex.what());
        }
        LogProtection(active_control_mode_, "hold unavailable; FullBrake", true);
        SendMotorStopLocked(MotorStopMode::FullBrake, 0.0f, feedback);
    }

    /**
     * @brief 清除「位置保持」类保护日志点。
     *
     * 这几个错误点只在保护真分支里被置位，自身观测不到条件转假：一次成功下发跟踪命令、
     * 或 Stop 显式作废批次之后在此统一清除。调用方必须已持有 mutex_。
     */
    void ClearHoldProtectionLocked() {
        LogProtection(active_control_mode_, "position hold", false);
        LogProtection(active_control_mode_, "hold unavailable; FullBrake", false);
        LogProtection(active_control_mode_, "feedback/hold unavailable; FullBrake", false);
        LogProtection(active_control_mode_, "hold-position failed", false);
    }

    /**
     * @brief 从双电机位置反馈正解刷新目标位姿缓存；调用方必须已持有 mutex_。
     * @param life_cycle_deduction 读取电机状态时扣减的生命周期值。
     * @param respect_batch_gate 为 true 时遵守合帧门控（半帧挂起期间不覆盖位置目标）；
     *        动态类控制不缓存位置目标，需要传 false 以免可恢复的位姿被判为不可执行。
     * @return true 表示反馈位置有效、正向运动学成功且缓存已刷新。
     */
    bool RefreshTargetPoseFromFeedbackLocked(int life_cycle_deduction,
                                             bool respect_batch_gate = true) {
        auto motor1_status = motor1_->GetStatus(life_cycle_deduction);
        auto motor2_status = motor2_->GetStatus(life_cycle_deduction);
        if (!motor1_status || !motor2_status || !std::isfinite(motor1_status->position) ||
            !std::isfinite(motor2_status->position)) {
            return false;
        }
        const MotorPositions feedback_positions{motor1_status->position, motor2_status->position};
        auto feedback_solver = actual_pose_state_ ? actual_pose_state_->solver : *solver_;
        const auto forward =
            feedback_solver.SolveForward(Motor1ToSolverTheta(motor1_status->position),
                                         Motor2ToSolverTheta(motor2_status->position));
        const bool forward_executable =
            IsExecutableSolveResult(forward.status, forward.transmission);
        LogProtectionMessage(active_control_mode_, "target feedback refresh failed",
                             !forward_executable,
                             "Joint protection: joint={}, mode={}, reason=target feedback refresh "
                             "failed, status={}, singular_j={}, state=enter",
                             protection_name_, active_control_mode_,
                             couple_joint::MechanismSolver::StatusMessage(forward.status),
                             forward.transmission.singular_j);
        if (!forward_executable) {
            return false;
        }
        StoreActualPoseLocked(feedback_solver, feedback_positions);
        if (respect_batch_gate && !CanFeedbackRefreshTargetLocked()) {
            return false;
        }
        const auto state = feedback_solver.GetState();
        *solver_ = feedback_solver;
        target_roll_ = state.roll;
        target_pitch_ = state.pitch;
        return true;
    }

    /**
     * @brief PVT 控制模式：目标角度不按软件限位裁剪，实际电机或虚拟关节位置进入保护区后
     * 拦截继续向外或深入奇异区的命令。
     * @param axis 被控制的输出轴。
     * @param kp 位置比例增益。
     * @param kd 速度微分增益。
     * @param pos 目标位置，单位 rad，关节坐标系。
     * @param spd 目标速度，单位 rad/s，关节坐标系。
     * @param torque 目标扭矩，单位 Nm，关节坐标系。
     * @param torque_mode PVT 解算模式。
     */
    void ControlPVT(DoubleCoupleJointAxis axis, float kp, float kd, float pos, float spd,
                    float torque, PVTControlMode torque_mode) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        AgeBrakeBatchLocked();
        active_control_mode_ = "PVT";
        const auto params = AxisParams(axis);
        if (!IsValidPVTControlMode(torque_mode) || !std::isfinite(kp) || kp < 0.0f ||
            !std::isfinite(kd) || kd < 0.0f || !std::isfinite(pos) || !std::isfinite(spd) ||
            !std::isfinite(torque) || std::abs(spd) > params.maxSpd_ ||
            std::abs(torque) > params.maxTor_) {
            WarnProtection(active_control_mode_, "invalid PVT input");
            ProtectLocked(1);
            throw std::invalid_argument("DoubleCoupleJoint invalid PVT input");
        }

        // 首条命令只写候选，两次调用仍可来自同一个轴。
        if (dynamic_batch_) {
            dynamic_batch_.reset();
            dynamic_commands_ = {};
        }
        const bool first = !position_batch_;
        if (first) {
            position_batch_ = PositionBatch{{target_roll_, target_pitch_},
                                            roll_pvt_command_,
                                            pitch_pvt_command_,
                                            {},
                                            {},
                                            torque_mode};
        }
        auto& pending = *position_batch_;
        SetAxisValue(axis, params.reverse_ ? -pos : pos, &pending.target);
        auto& command = axis == DoubleCoupleJointAxis::Roll ? pending.roll_pvt : pending.pitch_pvt;
        command = AxisPVTCommand{kp, kd, params.reverse_ ? -spd : spd,
                                 params.reverse_ ? -torque : torque};
        if (first) {
            return;
        }
        const auto batch = pending;
        position_batch_.reset();

        // 实际位置和传动信息只解析一次，三种 PVT 模式共用同一反馈快照。
        const auto motor1_status = motor1_->GetStatus(0);
        const auto motor2_status = motor2_->GetStatus(0);
        bool feedback_solved = false;
        const auto actual = ResolveActualPoseLocked(motor1_status, motor2_status, feedback_solved);
        if (!actual) {
            ProtectLocked(1);
            throw JointException(JointError::kDoubleCoupleJointSolveFailed,
                                 "DoubleCoupleJoint actual motor pose is unavailable");
        }
        auto target_solver = *solver_;
        const auto inverse = target_solver.SolveInverse(batch.target.roll, batch.target.pitch);
        const auto target_state = target_solver.GetState();
        MotorPositions positions{
            NormalizeMotorPosition(target_state.theta1 - solver_parameters_.initial_theta1,
                                   "motor1"),
            NormalizeMotorPosition(target_state.theta2 - solver_parameters_.initial_theta2,
                                   "motor2")};
        const bool target_valid = IsExecutableSolveResult(inverse.status, inverse.transmission) &&
                                  MotorPositionsInPVTRange(positions);
        LogProtection(active_control_mode_, "target inverse solve/protocol range rejected",
                      !target_valid, {}, AxisScope(axis));
        if (!target_valid) {
            position_target_feedback_lock_axis_ = axis;
            ProtectLocked(1, actual->positions);
            return;
        }
        position_target_feedback_lock_axis_.reset();

        PVTControlMode mode =
            batch.pvt_mode == torque_mode ? torque_mode : PVTControlMode::MotorPVT;
        JointPose joint_torque{batch.roll_pvt.torque, batch.pitch_pvt.torque};
        couple_joint::VelocityTorqueResult velocity{};
        bool velocity_available = false;
        if (feedback_solved && std::isfinite(motor1_status->speed) &&
            std::isfinite(motor2_status->speed)) {
            velocity =
                actual->solver.SolveForwardVelocity(motor1_status->speed, motor2_status->speed);
            velocity_available = IsExecutableSolveResult(velocity.status, velocity.transmission) &&
                                 std::isfinite(velocity.roll) && std::isfinite(velocity.pitch);
        }
        if (mode != PVTControlMode::MotorPVT) {
            const bool host_available = feedback_solved && velocity_available;
            LogProtection(active_control_mode_, "host feedback unavailable; motor PVT fallback",
                          !host_available);
            if (!host_available) {
                mode = PVTControlMode::MotorPVT;
            } else {
                joint_torque.roll += batch.roll_pvt.kp * (batch.target.roll - actual->pose.roll);
                joint_torque.pitch +=
                    batch.pitch_pvt.kp * (batch.target.pitch - actual->pose.pitch);
                if (mode == PVTControlMode::HostTorque) {
                    joint_torque.roll += batch.roll_pvt.kd * (batch.roll_pvt.speed - velocity.roll);
                    joint_torque.pitch +=
                        batch.pitch_pvt.kd * (batch.pitch_pvt.speed - velocity.pitch);
                }
                if (!std::isfinite(joint_torque.roll) || !std::isfinite(joint_torque.pitch) ||
                    std::abs(joint_torque.roll) > AxisParams(DoubleCoupleJointAxis::Roll).maxTor_ ||
                    std::abs(joint_torque.pitch) >
                        AxisParams(DoubleCoupleJointAxis::Pitch).maxTor_) {
                    ProtectLocked(1, actual->positions);
                    throw JointException(JointError::kDoubleCoupleJointSolveFailed,
                                         "DoubleCoupleJoint host torque exceeds joint limits");
                }
            }
        }

        // 数学映射不再改变幅度，PVT 的协议饱和由底层电机 SDK 完成。
        float speed1, speed2, torque1, torque2, kp1, kp2, kd1, kd2;
        if (!MapJointSpeed(actual->solver, batch.roll_pvt.speed, batch.pitch_pvt.speed, &speed1,
                           &speed2) ||
            !MapJointTorque(actual->solver, joint_torque.roll, joint_torque.pitch, &torque1,
                            &torque2) ||
            !MapPVTGains(actual->solver, batch.roll_pvt, batch.pitch_pvt, &kp1, &kp2, &kd1, &kd2)) {
            ProtectLocked(1, actual->positions);
            throw JointException(JointError::kDoubleCoupleJointSolveFailed,
                                 "DoubleCoupleJoint PVT mapping failed");
        }
        if (mode == PVTControlMode::HostKpMotorKd) {
            if (!MatchCoupledDampingTargets(actual->solver, batch.roll_pvt, batch.pitch_pvt, kd1,
                                            kd2, motor1_status->speed, motor2_status->speed,
                                            &speed1, &speed2)) {
                ProtectLocked(1, actual->positions);
                throw JointException(JointError::kDoubleCoupleJointSolveFailed,
                                     "DoubleCoupleJoint coupled PVT target mapping failed");
            }
            if (!MotorPositionsInPVTRange(positions) || !IsInRange(speed1, motor1_ranges_.speed) ||
                !IsInRange(speed2, motor2_ranges_.speed)) {
                ProtectLocked(1, actual->positions);
                throw JointException(JointError::kDoubleCoupleJointSolveFailed,
                                     "DoubleCoupleJoint coupled PVT target outside output range");
            }
        }
        const JointPose joint_speed{batch.roll_pvt.speed, batch.pitch_pvt.speed};
        if (JointPVTCommandsLeavingLimit(actual->pose, batch.target, joint_speed, joint_torque.roll,
                                         joint_torque.pitch) ||
            MotorPVTCommandsLeavingLimit(actual->positions, positions, speed1, speed2, torque1,
                                         torque2) ||
            PVTCommandsEnteringSingularity(actual->pose, batch.target,
                                           actual->solver.GetTransmission(), joint_speed,
                                           joint_torque.roll, joint_torque.pitch)) {
            ProtectLocked(1, actual->positions);
            return;
        }

        if (mode == PVTControlMode::HostTorque) {
            // TorControl 没有 SDK 饱和，必须在发送任一台电机前验证整帧。
            if (!IsInRange(torque1, motor1_ranges_.torque) ||
                !IsInRange(torque2, motor2_ranges_.torque) || torque1 < -327.68f ||
                torque1 > 327.67f || torque2 < -327.68f || torque2 > 327.67f) {
                ProtectLocked(1, actual->positions);
                throw JointException(JointError::kDoubleCoupleJointSolveFailed,
                                     "DoubleCoupleJoint motor torque outside output range");
            }
            motor1_->TorControl<0>(torque1, 1);
            motor2_->TorControl<0>(torque2, 1);
        } else {
            const bool host_kp = mode == PVTControlMode::HostKpMotorKd;
            motor1_->PVTControl<0>(host_kp ? 0.0f : kp1, kd1,
                                   host_kp ? actual->positions.motor1 : positions.motor1, speed1,
                                   torque1);
            motor2_->PVTControl<0>(host_kp ? 0.0f : kp2, kd2,
                                   host_kp ? actual->positions.motor2 : positions.motor2, speed2,
                                   torque2);
        }
        CommitTargetPose(batch.target, target_solver);
        roll_pvt_command_ = batch.roll_pvt;
        pitch_pvt_command_ = batch.pitch_pvt;
        ClearHoldProtectionLocked();
    }

    static float MergeCurrentLimit(float a, float b) {
        // 零表示底层默认值；仅在两侧均有显式限制时取较小值。
        if (a == 0.0f)
            return b;
        if (b == 0.0f)
            return a;
        return std::min(a, b);
    }

    /**
     * @brief 位置控制模式：当前位姿与目标位姿均可执行时直接发送，否则 Stop。
     * @param axis 被控制的输出轴。
     * @param position 目标位置，单位 rad，关节坐标系。
     * @param speed 速度上限，单位 rad/s。
     * @param current 电流上限，单位 A。
     * @param feedback 底层电机反馈模式。
     */
    void ControlPosition(DoubleCoupleJointAxis axis, float position, float speed, float current,
                         int feedback) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        AgeBrakeBatchLocked();
        active_control_mode_ = "PosControl";
        const auto params = AxisParams(axis);
        const double solver_position = params.reverse_ ? -position : position;
        if (!std::isfinite(position) || !std::isfinite(speed) || !std::isfinite(current) ||
            speed < 0.0f || speed > params.maxSpd_ || current < 0.0f || current > 409.5f ||
            current > params.maxTor_ / motor1_kt_ || current > params.maxTor_ / motor2_kt_ ||
            feedback < 0 || feedback > 3 ||
            !IsInRangeWithSlack(static_cast<float>(solver_position), params.limit_)) {
            WarnProtection(active_control_mode_, "invalid position input");
            ProtectLocked(feedback >= 0 && feedback <= 3 ? feedback : 1);
            throw std::invalid_argument("DoubleCoupleJoint invalid position input");
        }
        if (dynamic_batch_) {
            dynamic_batch_.reset();
            dynamic_commands_ = {};
        }
        const bool first = !position_batch_;
        if (first) {
            position_batch_ = PositionBatch{
                {target_roll_, target_pitch_}, roll_pvt_command_, pitch_pvt_command_, {}, {}, {}};
        }
        auto& pending = *position_batch_;
        SetAxisValue(axis, solver_position, &pending.target);
        auto& command =
            axis == DoubleCoupleJointAxis::Roll ? pending.roll_position : pending.pitch_position;
        AxisPositionCommand next{speed > 0.0f ? speed : params.maxSpd_, current};
        if (command) {
            next.speed = std::min(next.speed, command->speed);
            next.current = MergeCurrentLimit(next.current, command->current);
        }
        command = next;
        if (first) {
            return;
        }
        const auto batch = pending;
        position_batch_.reset();

        // 完整候选只求解一次；失败不会污染已提交目标。
        auto solver = *solver_;
        const auto inverse = solver.SolveInverse(batch.target.roll, batch.target.pitch);
        const auto& transmission = inverse.transmission;
        const auto state = solver.GetState();
        const MotorPositions positions{
            NormalizeMotorPosition(state.theta1 - solver_parameters_.initial_theta1, "motor1"),
            NormalizeMotorPosition(state.theta2 - solver_parameters_.initial_theta2, "motor2")};
        const bool reachable = IsExecutableSolveResult(inverse.status, transmission) &&
                               !transmission.near_singular &&
                               MotorPositionsInRange(positions, true) &&
                               IsInRange(static_cast<float>(batch.target.roll),
                                         AxisParams(DoubleCoupleJointAxis::Roll).limit_) &&
                               IsInRange(static_cast<float>(batch.target.pitch),
                                         AxisParams(DoubleCoupleJointAxis::Pitch).limit_);
        LogProtection(active_control_mode_, "target unreachable/near singular/outside motor slack",
                      !reachable, {}, AxisScope(axis));
        if (!reachable) {
            position_target_feedback_lock_axis_ = axis;
            const auto state = solver_->GetState();
            ProtectLocked(
                feedback,
                MotorPositions{NormalizeMotorPosition(
                                   state.theta1 - solver_parameters_.initial_theta1, "motor1"),
                               NormalizeMotorPosition(
                                   state.theta2 - solver_parameters_.initial_theta2, "motor2")});
            return;
        }
        position_target_feedback_lock_axis_.reset();
        const double roll_delta = batch.target.roll - target_roll_;
        const double pitch_delta = batch.target.pitch - target_pitch_;
        const float roll_speed = batch.roll_position
                                     ? batch.roll_position->speed
                                     : AxisParams(DoubleCoupleJointAxis::Roll).maxSpd_;
        const float pitch_speed = batch.pitch_position
                                      ? batch.pitch_position->speed
                                      : AxisParams(DoubleCoupleJointAxis::Pitch).maxSpd_;
        const double duration =
            std::max(std::abs(roll_delta) / roll_speed, std::abs(pitch_delta) / pitch_speed);
        const double roll_velocity = duration > 0.0                        ? roll_delta / duration
                                     : axis == DoubleCoupleJointAxis::Roll ? next.speed
                                                                           : 0.0;
        const double pitch_velocity = duration > 0.0 ? pitch_delta / duration
                                      : axis == DoubleCoupleJointAxis::Pitch ? next.speed
                                                                             : 0.0;
        float speed1, speed2;
        if (!MapJointSpeed(solver, roll_velocity, pitch_velocity, &speed1, &speed2)) {
            ProtectLocked(feedback);
            throw JointException(JointError::kDoubleCoupleJointSolveFailed,
                                 "DoubleCoupleJoint position velocity mapping failed");
        }
        // 位置模式的两台电机速度是独立上限，按 Jacobian 行和约束关节速度。
        const auto& J = transmission.velocity_jacobian;
        const double roll_bound = std::abs(J.m00 * speed1) + std::abs(J.m01 * speed2);
        const double pitch_bound = std::abs(J.m10 * speed1) + std::abs(J.m11 * speed2);
        double scale = 1.0;
        if (roll_bound > roll_speed)
            scale = std::min(scale, roll_speed / roll_bound);
        if (pitch_bound > pitch_speed)
            scale = std::min(scale, pitch_speed / pitch_bound);
        speed1 = static_cast<float>(std::abs(speed1) * scale);
        speed2 = static_cast<float>(std::abs(speed2) * scale);
        float current_limit = next.current;
        for (const auto& item : {batch.roll_position, batch.pitch_position}) {
            if (item) {
                current_limit = MergeCurrentLimit(current_limit, item->current);
            }
        }
        if (speed1 > 3276.7 * kPi / 30.0 || speed2 > 3276.7 * kPi / 30.0 ||
            !IsInRange(speed1, motor1_ranges_.speed) || !IsInRange(speed2, motor2_ranges_.speed) ||
            (current_limit > 0.0f && (!IsInRange(current_limit, motor1_ranges_.current) ||
                                      !IsInRange(current_limit, motor2_ranges_.current)))) {
            ProtectLocked(feedback);
            throw JointException(JointError::kDoubleCoupleJointSolveFailed,
                                 "DoubleCoupleJoint position speed outside protocol range");
        }
        motor1_->PosControl<0>(positions.motor1, speed1, current_limit, feedback);
        motor2_->PosControl<0>(positions.motor2, speed2, current_limit, feedback);
        CommitTargetPose(batch.target, solver);
        roll_pvt_command_ = batch.roll_pvt;
        pitch_pvt_command_ = batch.pitch_pvt;
        ClearHoldProtectionLocked();
    }

    /**
     * @brief 动态类控制（Spd/Tor/Cur）的统一实现
     *
     * 与位置类控制一致地参与合帧：本批次第一条轴命令只缓存该轴目标与电流上限，第二条
     * 轴命令把两轴目标一起做联合映射后一次下发两台电机；未在本批次命令过的轴沿用该轴
     * 上次命令值（从未命令过则为 0）。严重奇异方向与越限时按本模式语义把命令置零或保持。
     * @param mode 动态控制模式。
     * @param axis 被控制的输出轴。
     * @param command 关节坐标系目标（速度 rad/s / 扭矩 Nm / 电流 A）。
     * @param current 电流上限，单位 A；仅速度模式使用。
     * @param feedback 底层电机反馈模式。
     */
    void ControlDynamic(DynamicControlMode mode, DoubleCoupleJointAxis axis, float command,
                        float current, int feedback) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        AgeBrakeBatchLocked();
        active_control_mode_ = mode == DynamicControlMode::Speed    ? "SpdControl"
                               : mode == DynamicControlMode::Torque ? "TorControl"
                                                                    : "CurControl";
        const auto params = AxisParams(axis);
        if (!std::isfinite(command) || !std::isfinite(current) || current < 0.0f ||
            current > 6553.5f || feedback < 0 || feedback > 3 ||
            current > params.maxTor_ / motor1_kt_ || current > params.maxTor_ / motor2_kt_ ||
            (current > 0.0f && (!IsInRange(current, motor1_ranges_.current) ||
                                !IsInRange(current, motor2_ranges_.current))) ||
            (mode == DynamicControlMode::Speed && std::abs(command) > params.maxSpd_) ||
            (mode == DynamicControlMode::Torque && std::abs(command) > params.maxTor_)) {
            WarnProtection(active_control_mode_, "invalid dynamic input");
            ProtectLocked(feedback >= 0 && feedback <= 3 ? feedback : 1);
            throw std::invalid_argument("DoubleCoupleJoint invalid dynamic input");
        }
        position_target_feedback_lock_axis_.reset();
        if (position_batch_ || (dynamic_batch_ && dynamic_batch_->mode != mode)) {
            position_batch_.reset();
            dynamic_batch_.reset();
            dynamic_commands_ = {};
        }
        const bool first = !dynamic_batch_;
        const auto mode_index = static_cast<std::size_t>(mode);
        if (first) {
            dynamic_batch_ = DynamicBatch{mode, dynamic_commands_[mode_index]};
        }
        const std::size_t axis_index = axis == DoubleCoupleJointAxis::Roll ? 0U : 1U;
        dynamic_batch_->commands[axis_index] =
            AxisDynamicCommand{params.reverse_ ? -command : command, current};
        RefreshTargetPoseFromFeedbackLocked(0, false);
        if (first) {
            return;
        }
        const auto batch = *dynamic_batch_;
        dynamic_batch_.reset();
        const JointPose pose{target_roll_, target_pitch_};
        auto solver = *solver_;
        const auto inverse = solver.SolveInverse(pose.roll, pose.pitch);
        const auto& transmission = inverse.transmission;
        const auto state = solver.GetState();
        const MotorPositions positions{
            NormalizeMotorPosition(state.theta1 - solver_parameters_.initial_theta1, "motor1"),
            NormalizeMotorPosition(state.theta2 - solver_parameters_.initial_theta2, "motor2")};
        if (!IsExecutableSolveResult(inverse.status, transmission) ||
            !MotorPositionsInRange(positions)) {
            ProtectLocked(feedback);
            throw JointException(JointError::kDoubleCoupleJointSolveFailed,
                                 "DoubleCoupleJoint current pose is not executable");
        }

        // 每轴只检查一次运动方向，再进行双轴联合映射。
        double values[2] = {batch.commands[0].value_or(AxisDynamicCommand{}).value,
                            batch.commands[1].value_or(AxisDynamicCommand{}).value};
        for (std::size_t i = 0; i < 2; ++i) {
            const auto output_axis =
                i == 0 ? DoubleCoupleJointAxis::Roll : DoubleCoupleJointAxis::Pitch;
            if (IsCommandEnteringSingularity(pose, transmission, output_axis, values[i])) {
                ProtectLocked(feedback, positions);
                return;
            }
            if (AxisCommandLeavingLimit(output_axis, pose, values[i])) {
                values[i] = 0.0;
            }
        }
        float value1 = 0.0f, value2 = 0.0f;
        bool mapped = false;
        switch (mode) {
            case DynamicControlMode::Speed:
                mapped = MapJointSpeed(solver, values[0], values[1], &value1, &value2);
                break;
            case DynamicControlMode::Torque:
                mapped = MapJointTorque(solver, values[0], values[1], &value1, &value2);
                break;
            case DynamicControlMode::Current: {
                MotorCurrents currents;
                mapped = MapJointCurrent(solver, values[0], values[1], &currents);
                value1 = currents.motor1;
                value2 = currents.motor2;
                break;
            }
        }
        const auto range1 = mode == DynamicControlMode::Speed    ? motor1_ranges_.speed
                            : mode == DynamicControlMode::Torque ? motor1_ranges_.torque
                                                                 : motor1_ranges_.current;
        const auto range2 = mode == DynamicControlMode::Speed    ? motor2_ranges_.speed
                            : mode == DynamicControlMode::Torque ? motor2_ranges_.torque
                                                                 : motor2_ranges_.current;
        const bool encodable = mode == DynamicControlMode::Speed
                                   ? std::isfinite(value1 * static_cast<float>(30.0 / kPi)) &&
                                         std::isfinite(value2 * static_cast<float>(30.0 / kPi))
                                   : value1 >= -327.68f && value1 <= 327.67f &&
                                         value2 >= -327.68f && value2 <= 327.67f;
        if (!mapped || !encodable || !IsInRange(value1, range1) || !IsInRange(value2, range2)) {
            ProtectLocked(feedback, positions);
            throw JointException(JointError::kDoubleCoupleJointSolveFailed,
                                 "DoubleCoupleJoint dynamic output outside valid range");
        }
        if (MotorCommandsLeavingLimit(positions, value1, value2)) {
            value1 = value2 = 0.0f;
        }
        float current_limit = current;
        const auto& other = batch.commands[1U - axis_index];
        if (other)
            current_limit = MergeCurrentLimit(current_limit, other->current);
        switch (mode) {
            case DynamicControlMode::Speed:
                motor1_->SpdControl<0>(value1, current_limit, feedback);
                motor2_->SpdControl<0>(value2, current_limit, feedback);
                break;
            case DynamicControlMode::Torque:
                motor1_->TorControl<0>(value1, feedback);
                motor2_->TorControl<0>(value2, feedback);
                break;
            case DynamicControlMode::Current:
                motor1_->CurControl<0>(value1, feedback);
                motor2_->CurControl<0>(value2, feedback);
                break;
        }
        dynamic_commands_[mode_index] = batch.commands;
        ClearHoldProtectionLocked();
    }

    /**
     * @brief 拉取指定轴的状态：读取双电机状态并执行正向运动学。
     * @param axis 需要返回状态的输出轴。
     * @param autoSendCommand 为 true 且缓存状态不可用时发送一次反馈命令。
     * @param lifeCycleDeduction 读取电机状态时扣减的生命周期值。
     * @return 指定轴关节坐标系下的状态；状态不可用时返回 std::nullopt。
     * @warning 关节运动过程中不得将 autoSendCommand 设为 true；内部零目标 PVT 反馈命令
     *          可能改变双电机关节运动状态。
     */
    std::optional<JointStatus> GetStatusForAxis(DoubleCoupleJointAxis axis, bool autoSendCommand,
                                                int lifeCycleDeduction) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        auto motor1_status = motor1_->GetStatus(lifeCycleDeduction);
        auto motor2_status = motor2_->GetStatus(lifeCycleDeduction);
        if (!motor1_status || !motor2_status) {
            if (autoSendCommand) {
                motor1_->PVTControl<0>(0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
                motor2_->PVTControl<0>(0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
            }
            return std::nullopt;
        }
        try {
            const auto feedback_state =
                BuildJointFeedbackStateLocked(*motor1_status, *motor2_status);
            if (feedback_state.position_solved && CanFeedbackRefreshTargetLocked()) {
                target_roll_ = feedback_state.pose.roll;
                target_pitch_ = feedback_state.pose.pitch;
            }
            return axis == DoubleCoupleJointAxis::Roll ? feedback_state.roll_status
                                                       : feedback_state.pitch_status;
        } catch (const JointException& ex) {
            JointSdkLogWarn("DoubleCoupleJoint status feedback failed: error={}, message={}",
                            static_cast<int>(ex.Error()), ex.what());
            return std::nullopt;
        }
    }

    /**
     * @brief 正向运动学：从双电机状态构建关节反馈状态。
     *
     * 电机反馈报文可能只携带部分字段（位置/速度/电流可能为 NaN），
     * 本方法对每个运动学通道独立判断数据有效性：
     * - 双电机位置均有效时执行正向位置运动学，否则使用缓存的 target_roll_/target_pitch_
     * - 双电机速度均有效时执行正向速度运动学，否则速度字段为 NaN
     * - 双电机电流均有效时执行正向力矩运动学，否则力矩字段为 NaN
     * - 温度和错误码始终可用
     *
     * @param motor1_status 电机 1 最新状态。
     * @param motor2_status 电机 2 最新状态。
     * @return 同时包含 roll/pitch 合成状态、求解器位姿和位置是否成功正解的结构。
     * @note 无位置数据时，速度/力矩基于 *solver_ 的缓存位姿计算（近似，非精确）。
     * @note 调用方必须已持有 mutex_。
     */
    JointFeedbackState BuildJointFeedbackStateLocked(const MotorStatus& motor1_status,
                                                     const MotorStatus& motor2_status) {
        const auto roll_params = AxisParams(DoubleCoupleJointAxis::Roll);
        const auto pitch_params = AxisParams(DoubleCoupleJointAxis::Pitch);
        const auto solver_to_joint = [](double value, const AxisRuntimeParams& params) {
            return static_cast<float>(params.reverse_ ? -value : value);
        };

        // 位置：双电机均有效时才执行正向运动学，否则使用缓存位姿
        double solver_roll = target_roll_;
        double solver_pitch = target_pitch_;
        bool position_attempted = false;
        bool position_solved = false;
        auto feedback_solver = actual_pose_state_ ? actual_pose_state_->solver : *solver_;
        const MotorPositions feedback_positions{motor1_status.position, motor2_status.position};
        if (std::isfinite(motor1_status.position) && std::isfinite(motor2_status.position)) {
            position_attempted = true;
            const auto forward =
                feedback_solver.SolveForward(Motor1ToSolverTheta(motor1_status.position),
                                             Motor2ToSolverTheta(motor2_status.position));
            if (IsExecutableSolveResult(forward.status, forward.transmission)) {
                const auto state = feedback_solver.GetState();
                solver_roll = state.roll;
                solver_pitch = state.pitch;
                position_solved = true;
            } else {
                JointSdkLogWarn(
                    "DoubleCoupleJoint forward kinematics failed: status={}, singular_j={}",
                    couple_joint::MechanismSolver::StatusMessage(forward.status),
                    forward.transmission.singular_j);
            }
        }

        double solver_roll_spd = std::numeric_limits<double>::quiet_NaN();
        double solver_pitch_spd = std::numeric_limits<double>::quiet_NaN();
        double solver_roll_tor = std::numeric_limits<double>::quiet_NaN();
        double solver_pitch_tor = std::numeric_limits<double>::quiet_NaN();

        // SolveForward 失败时 state_ 保持旧缓存值，与当前电机位置不匹配；
        // 此时 velocity/torque 映射将基于错误位姿，必须跳过。
        if (!position_attempted || position_solved) {
            const auto& kinematics_solver = position_solved ? feedback_solver : *solver_;
            if (std::isfinite(motor1_status.speed) && std::isfinite(motor2_status.speed)) {
                const auto velocity = kinematics_solver.SolveForwardVelocity(motor1_status.speed,
                                                                             motor2_status.speed);
                if (IsExecutableSolveResult(velocity.status, velocity.transmission)) {
                    solver_roll_spd = velocity.roll;
                    solver_pitch_spd = velocity.pitch;
                } else {
                    JointSdkLogWarn(
                        "DoubleCoupleJoint forward velocity failed: status={}, singular_j={}",
                        couple_joint::MechanismSolver::StatusMessage(velocity.status),
                        velocity.transmission.singular_j);
                }
            }

            if (std::isfinite(motor1_status.current) && std::isfinite(motor2_status.current)) {
                const auto torque = kinematics_solver.SolveForwardTorque(
                    motor1_status.current * motor1_kt_, motor2_status.current * motor2_kt_);
                if (IsExecutableSolveResult(torque.status, torque.transmission)) {
                    solver_roll_tor = torque.roll;
                    solver_pitch_tor = torque.pitch;
                } else {
                    JointSdkLogWarn(
                        "DoubleCoupleJoint forward torque failed: status={}, singular_j={}",
                        couple_joint::MechanismSolver::StatusMessage(torque.status),
                        torque.transmission.singular_j);
                }
            }
        }
        if (position_solved) {
            StoreActualPoseLocked(feedback_solver, feedback_positions);
            if (CanFeedbackRefreshTargetLocked()) {
                *solver_ = feedback_solver;
            }
        }

        // 温度和错误码始终可用
        const float mos_temperature =
            MaxStatusValue(motor1_status.mos_temperature, motor2_status.mos_temperature);
        const float motor_temperature =
            MaxStatusValue(motor1_status.motor_temperature, motor2_status.motor_temperature);
        const MotorError error = MaxStatusError(motor1_status.error, motor2_status.error);

        JointStatus roll_status{};
        roll_status.pos = solver_to_joint(solver_roll, roll_params);
        roll_status.spd = solver_to_joint(solver_roll_spd, roll_params);
        roll_status.tor = solver_to_joint(solver_roll_tor, roll_params);
        roll_status.mosTemperature = mos_temperature;
        roll_status.motorTemperature = motor_temperature;
        roll_status.error = error;

        JointStatus pitch_status{};
        pitch_status.pos = solver_to_joint(solver_pitch, pitch_params);
        pitch_status.spd = solver_to_joint(solver_pitch_spd, pitch_params);
        pitch_status.tor = solver_to_joint(solver_pitch_tor, pitch_params);
        pitch_status.mosTemperature = mos_temperature;
        pitch_status.motorTemperature = motor_temperature;
        pitch_status.error = error;

        return JointFeedbackState{JointPose{solver_roll, solver_pitch}, roll_status, pitch_status,
                                  position_solved};
    }

    /**
     * @brief 按用户回调需求安装底层电机状态回调。
     * @note 调用方必须已持有 mutex_。
     */
    void UpdateMotorStatusCallbacksLocked() {
        const bool callbacks_required =
            IsJointLogEnabled() || roll_status_callback_ || pitch_status_callback_;
        if (callbacks_required && !status_callbacks_installed_) {
            const std::weak_ptr<MasterJoint> self = weak_from_this();
            motor1_->SetOnStatus([self](const MotorStatus& status) {
                if (auto joint = self.lock()) {
                    joint->OnMotorStatus(1, status);
                }
            });
            motor2_->SetOnStatus([self](const MotorStatus& status) {
                if (auto joint = self.lock()) {
                    joint->OnMotorStatus(2, status);
                }
            });
            status_callbacks_installed_ = true;
        } else if (!callbacks_required && status_callbacks_installed_) {
            motor1_->SetOnStatus(nullptr);
            motor2_->SetOnStatus(nullptr);
            status_callbacks_installed_ = false;
        }
    }

    /**
     * @brief 注册指定轴的异步状态回调。
     * @param axis 需要注册回调的输出轴。
     * @param onStatus 状态回调；传入 nullptr 时取消该轴回调。
     */
    void SetOnStatusForAxis(DoubleCoupleJointAxis axis,
                            std::function<void(const JointStatus&)> onStatus) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (axis == DoubleCoupleJointAxis::Roll) {
            roll_status_callback_ = std::move(onStatus);
        } else {
            pitch_status_callback_ = std::move(onStatus);
        }
        UpdateMotorStatusCallbacksLocked();
    }

    /**
     * @brief 电机状态异步回调：缓冲状态、执行正向运动学、分发回调。
     * @param motor_index 触发回调的电机序号，1 表示 motor1，其它值按 motor2 处理。
     * @param status 底层电机上报的状态。
     */
    void OnMotorStatus(int motor_index, const MotorStatus& status) {
        std::function<void(const JointStatus&)> roll_callback;
        std::function<void(const JointStatus&)> pitch_callback;
        JointStatus roll_status{};
        JointStatus pitch_status{};
        bool has_roll_status = false;
        bool has_pitch_status = false;

        try {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            if (motor_index == 1) {
                last_motor1_status_ = status;
                has_last_motor1_status_ = true;
            } else {
                last_motor2_status_ = status;
                has_last_motor2_status_ = true;
            }
            if (!has_last_motor1_status_ || !has_last_motor2_status_) {
                return;
            }
            const auto feedback_state =
                BuildJointFeedbackStateLocked(last_motor1_status_, last_motor2_status_);
            if (feedback_state.position_solved && CanFeedbackRefreshTargetLocked()) {
                target_roll_ = feedback_state.pose.roll;
                target_pitch_ = feedback_state.pose.pitch;
            }
            if (axis_ == DoubleCoupleJointAxis::Roll) {
                RecordStatusForLog(feedback_state.roll_status);
                if (auto slave = slave_.lock()) {
                    RecordSlaveStatusForLog(slave, feedback_state.pitch_status);
                }
            } else {
                RecordStatusForLog(feedback_state.pitch_status);
                if (auto slave = slave_.lock()) {
                    RecordSlaveStatusForLog(slave, feedback_state.roll_status);
                }
            }
            if (roll_status_callback_) {
                roll_status = feedback_state.roll_status;
                roll_callback = roll_status_callback_;
                has_roll_status = true;
            }
            if (pitch_status_callback_) {
                pitch_status = feedback_state.pitch_status;
                pitch_callback = pitch_status_callback_;
                has_pitch_status = true;
            }
        } catch (const JointException& ex) {
            JointSdkLogWarn(
                "DoubleCoupleJoint status feedback ignored for motor {}: error={}, message={}",
                motor_index, static_cast<int>(ex.Error()), ex.what());
            return;
        }

        if (has_roll_status) {
            roll_callback(roll_status);
        }
        if (has_pitch_status) {
            pitch_callback(pitch_status);
        }
    }

    /**
     * @brief 读取电机位置值（优先使用缓存状态）。
     * @param motor 需要读取的电机。
     * @param read_cache true 时优先读取状态缓存；false 时直接读取电机参数。
     * @param life_cycle_deduction 读取状态缓存时扣减的生命周期值。
     * @return 电机坐标系下的位置，单位 rad。
     */
    static float ReadMotorPosition(Motor* motor, bool read_cache, int life_cycle_deduction) {
        if (!read_cache) {
            return motor->GetParameter<MotorParameter::Position>();
        }
        auto status = motor->GetStatus(life_cycle_deduction);
        if (!status || std::isnan(status->position)) {
            return motor->GetParameter<MotorParameter::Position>();
        }
        return status->position;
    }

    Motor* motor1_{nullptr};  ///< Bus-owned, non-owning motor instance
    Motor* motor2_{nullptr};  ///< Bus-owned, non-owning motor instance
    std::weak_ptr<SlaveJoint> slave_;
    DoubleCoupleJointAxis axis_{DoubleCoupleJointAxis::Roll};
    DoubleCoupleJointAxis slave_axis_{DoubleCoupleJointAxis::Pitch};
    couple_joint::CalibrationInput calibration_;
    std::unique_ptr<couple_joint::MechanismSolver> solver_;
    std::optional<ActualPoseState> actual_pose_state_;
    couple_joint::MechanismParameters solver_parameters_;
    MotorProperties motor1_ranges_{};
    MotorProperties motor2_ranges_{};
    Range<float> motor1_position_limit_{0.0f, 0.0f};
    Range<float> motor2_position_limit_{0.0f, 0.0f};
    float motor1_kt_{1.0f};
    float motor2_kt_{1.0f};
    AxisPVTCommand roll_pvt_command_{};
    AxisPVTCommand pitch_pvt_command_{};
    bool brake_batch_pending_{false};
    unsigned brake_batch_motion_calls_{0};
    std::optional<PositionBatch> position_batch_;
    std::optional<DynamicBatch> dynamic_batch_;
    std::array<std::array<std::optional<AxisDynamicCommand>, 2>, 3> dynamic_commands_{};
    double target_roll_{0.0};
    double target_pitch_{0.0};
    std::optional<DoubleCoupleJointAxis> position_target_feedback_lock_axis_;

    std::recursive_mutex mutex_;
    std::function<void(const JointStatus&)> roll_status_callback_;
    std::function<void(const JointStatus&)> pitch_status_callback_;
    bool status_callbacks_installed_{false};
    MotorStatus last_motor1_status_{};
    MotorStatus last_motor2_status_{};
    bool has_last_motor1_status_{false};
    bool has_last_motor2_status_{false};
};

}  // namespace encos

#include "DoubleCoupleJoint/SlaveJoint.h"

#endif  // ENCOS_MASTER_JOINT_H_
