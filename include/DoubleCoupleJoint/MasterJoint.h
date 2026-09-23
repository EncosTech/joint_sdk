// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

#pragma once

#ifndef ENCOS_MASTER_JOINT_H_
#define ENCOS_MASTER_JOINT_H_

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
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

    /**
     * @brief 重置双电机零位并验证原点位姿的可执行性。
     * @return true 表示两台电机零位重置成功且求解器原点位姿可执行。
     */
    bool SetZero() override {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        track_actual_feedback_ = true;
        waiting_for_post_zero_feedback_ = true;
        motor1_feedback_after_zero_ = false;
        motor2_feedback_after_zero_ = false;
        zero_reset_in_progress_ = true;
        zero_reset_faulted_ = false;
        UpdateMotorStatusCallbacksLocked();
        feedback_epoch_.fetch_add(1, std::memory_order_acq_rel);
        const bool motor1_ok = motor1_->ResetZeroPos();
        const bool motor2_ok = motor2_->ResetZeroPos();
        zero_reset_in_progress_ = false;
        feedback_epoch_.fetch_add(1, std::memory_order_acq_rel);
        if (!motor1_ok || !motor2_ok) {
            zero_reset_faulted_ = true;
            actual_pose_state_.reset();
            return false;
        }
        const auto result = solver_->SolveInverse(0.0, 0.0);
        if (!IsExecutableSolveResult(result.status, result.transmission)) {
            zero_reset_faulted_ = true;
            actual_pose_state_.reset();
            return false;
        }
        target_roll_ = 0.0;
        target_pitch_ = 0.0;
        StoreActualPoseLocked(*solver_, MotorPositions{});
        waiting_for_post_zero_feedback_ =
            !motor1_feedback_after_zero_ || !motor2_feedback_after_zero_;
        ResetPositionControlBatchLocked();
        ClearHoldProtectionLocked();
        return true;
    }

    /**
     * @brief 向两个电机发送停止指令。
     * @param mode 停止模式。
     * @param current 停止电流；非有限值会被归零。
     * @param feedback 底层电机反馈模式。
     */
    void Stop(MotorStopMode mode, float current = 0, int feedback = 1) override {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        RestorePositionControlBatchStartLocked(MotorPositions{});
        ResetPositionControlBatchLocked();
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
        ControlPVT(axis_, kp, kd, pos, spd, torque, torque_mode);
    }

    /**
     * @brief 在主轴上执行位置控制。
     * @param position 目标位置，单位 rad，主轴关节坐标系。
     * @param speed 速度上限，单位 rad/s；<=0 时使用主轴最大速度。
     * @param current 电流上限，单位 A；<=0 时按最大扭矩和 kt 计算。
     * @param feedback 底层电机反馈模式。
     */
    void PosControl(float position, float speed = 0, float current = 0, int feedback = 1) override {
        ControlPosition(axis_, position, speed, current, feedback);
    }

    /**
     * @brief 在主轴上执行速度控制。
     * @param speed 目标速度，单位 rad/s，主轴关节坐标系。
     * @param current 电流上限，单位 A；<=0 时按最大扭矩和 kt 计算。
     * @param feedback 底层电机反馈模式。
     */
    void SpdControl(float speed, float current = 0, int feedback = 1) override {
        ControlSpeed(axis_, speed, current, feedback);
    }

    /**
     * @brief 在主轴上执行电流控制。
     * @param current 目标电流，单位 A，主轴关节坐标系方向。
     * @param feedback 底层电机反馈模式。
     */
    void CurControl(float current = 0, int feedback = 1) override {
        ControlCurrent(axis_, current, feedback);
    }

    /**
     * @brief 在主轴上执行扭矩控制。
     * @param torque 目标扭矩，单位 Nm，主轴关节坐标系。
     * @param feedback 底层电机反馈模式。
     */
    void TorControl(float torque = 0, int feedback = 1) override {
        ControlTorque(axis_, torque, feedback);
    }

    /**
     * @brief 未实现：闭链关节不支持硬件限位搜索。
     * @param dir 搜索方向，当前未使用。
     * @param spd 搜索速度，当前未使用。
     * @param cur 搜索电流，当前未使用。
     * @param timeout 搜索超时时间，当前未使用。
     * @return 始终返回 NaN。
     */
    float GotoLimit([[maybe_unused]] int dir, [[maybe_unused]] float spd = 2.f,
                    [[maybe_unused]] float cur = 2.f,
                    [[maybe_unused]] std::chrono::milliseconds timeout = std::chrono::milliseconds{
                        5000}) override {
        return std::nanf("");
    }

    /**
     * @brief 未实现：闭链关节不支持归零运动。
     * @param offset 零点偏移，当前未使用。
     * @param spd 回零速度，当前未使用。
     * @param cur 回零电流，当前未使用。
     * @param timeout 回零超时时间，当前未使用。
     * @return 始终返回 false。
     */
    bool GotoZero([[maybe_unused]] float offset, [[maybe_unused]] float spd = 2.f,
                  [[maybe_unused]] float cur = 2.f,
                  [[maybe_unused]] std::chrono::milliseconds timeout = std::chrono::milliseconds{
                      5000}) override {
        return false;
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
     * @brief 未实现：闭链关节校准在构造时通过求解器完成。
     * @param dir 校准方向，当前未使用。
     * @param spd 校准速度，当前未使用。
     * @param cur 校准电流，当前未使用。
     * @param timeout 校准超时时间，当前未使用。
     * @return 始终返回 false。
     */
    bool Calibrate([[maybe_unused]] int dir, [[maybe_unused]] float spd = 2.f,
                   [[maybe_unused]] float cur = 2.f,
                   [[maybe_unused]] std::chrono::milliseconds timeout = std::chrono::milliseconds{
                       5000}) override {
        return false;
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

    // Slave 关节通过 SlaveJoint 转发到 MasterJoint 的控制接口。

    /**
     * @brief 在从轴上执行 PVT 控制（由 SlaveJoint 转发调用）。
     * @param kp 位置比例增益。
     * @param kd 速度微分增益。
     * @param pos 目标位置，单位 rad，从轴关节坐标系。
     * @param spd 目标速度，单位 rad/s，从轴关节坐标系。
     * @param torque 目标扭矩，单位 Nm，从轴关节坐标系。
     */
    void PVTControlForSlave(float kp, float kd, float pos, float spd, float torque) {
        PVTControlForSlave(kp, kd, pos, spd, torque, PVTControlMode::MotorPVT);
    }

    /**
     * @brief 在从轴上执行带上位机力矩模式选择的 PVT 控制。
     * @param torque_mode PVT 解算模式；同一批次两轴模式一致时启用上位机解算。
     */
    void PVTControlForSlave(float kp, float kd, float pos, float spd, float torque,
                            PVTControlMode torque_mode) {
        ControlPVT(slave_axis_, kp, kd, pos, spd, torque, torque_mode);
    }

    /**
     * @brief 在从轴上执行位置控制（由 SlaveJoint 转发调用）。
     * @param position 目标位置，单位 rad，从轴关节坐标系。
     * @param speed 速度上限，单位 rad/s。
     * @param current 电流上限，单位 A。
     * @param feedback 底层电机反馈模式。
     */
    void PosControlForSlave(float position, float speed, float current, int feedback) {
        ControlPosition(slave_axis_, position, speed, current, feedback);
    }

    /**
     * @brief 在从轴上执行速度控制（由 SlaveJoint 转发调用）。
     * @param speed 目标速度，单位 rad/s，从轴关节坐标系。
     * @param current 电流上限，单位 A。
     * @param feedback 底层电机反馈模式。
     */
    void SpdControlForSlave(float speed, float current, int feedback) {
        ControlSpeed(slave_axis_, speed, current, feedback);
    }

    /**
     * @brief 在从轴上执行电流控制（由 SlaveJoint 转发调用）。
     * @param current 目标电流，单位 A，从轴关节坐标系方向。
     * @param feedback 底层电机反馈模式。
     */
    void CurControlForSlave(float current, int feedback) {
        ControlCurrent(slave_axis_, current, feedback);
    }

    /**
     * @brief 在从轴上执行扭矩控制（由 SlaveJoint 转发调用）。
     * @param torque 目标扭矩，单位 Nm，从轴关节坐标系。
     * @param feedback 底层电机反馈模式。
     */
    void TorControlForSlave(float torque, int feedback) {
        ControlTorque(slave_axis_, torque, feedback);
    }

    /**
     * @brief 获取从轴的状态信息（由 SlaveJoint 转发调用）。
     * @param autoSendCommand 为 true 且缓存状态不可用时发送一次反馈命令。
     * @param lifeCycleDeduction 读取状态时扣减的生命周期值。
     * @return 从轴关节坐标系下的状态；状态不可用时返回 std::nullopt。
     * @warning 关节运动过程中不得将 autoSendCommand 设为 true；内部零目标 PVT 反馈命令
     *          可能改变双电机关节运动状态。
     */
    std::optional<JointStatus> GetStatusForSlave(bool autoSendCommand, int lifeCycleDeduction) {
        return GetStatusForAxis(slave_axis_, autoSendCommand, lifeCycleDeduction);
    }

    /**
     * @brief 注册从轴状态异步回调（由 SlaveJoint 转发调用）。
     * @param onStatus 状态回调；传入 nullptr 时取消从轴回调。
     */
    void SetOnStatusForSlave(std::function<void(const JointStatus&)> onStatus) {
        SetOnStatusForAxis(slave_axis_, std::move(onStatus));
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
        double position{0.0};
        float speed{0.0f};
        float current1{0.0f};
        float current2{0.0f};
    };

    /// 合帧批次所属的控制类别；PVT/Pos 属位置类，Spd/Tor/Cur 属动态类。
    enum class CoupledBatchClass {
        None,
        Position,
        Dynamic,
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
        float current1{0.0f};  ///< 电机 1 电流上限
        float current2{0.0f};  ///< 电机 2 电流上限
    };

    enum class HostTorquePVTResult {
        Unavailable,
        Sent,
        Protected,
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
        motor1_ranges_ = motor1_->GetPVTRanges();
        motor1_kt_ = motor1_ranges_.kt;
        motor2_ranges_ = motor2_->GetPVTRanges();
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
     * @brief 将目标值夹紧到带限位缓冲的范围内。
     * @param value 待夹紧值。
     * @param range 原始允许范围。
     * @return 夹紧后的值，保证不越过 kLimitSlack 缓冲边界。
     */
    static double ClampToLimitSlack(double value, const Range<float>& range) {
        if (value < range.min + kLimitSlack) {
            value = range.min + kLimitSlack;
        }
        if (value > range.max - kLimitSlack) {
            value = range.max - kLimitSlack;
        }
        return value;
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
                                      double roll_effort, double pitch_effort) const {
        const auto leaves = [&](DoubleCoupleJointAxis axis, double actual, double target,
                                double speed, double effort) {
            if (axis != axis_ && slave_.expired()) {
                return false;
            }
            return IsPVTCommandLeavingLimit(static_cast<float>(actual), AxisParams(axis).limit_,
                                            target, speed, effort);
        };
        const bool blocked = leaves(DoubleCoupleJointAxis::Roll, actual_pose.roll, target_pose.roll,
                                    roll_pvt_command_.speed, roll_effort) ||
                             leaves(DoubleCoupleJointAxis::Pitch, actual_pose.pitch,
                                    target_pose.pitch, pitch_pvt_command_.speed, pitch_effort);
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
     * @brief 验证控制输入均为有限值，失败时保持当前位置或停止。
     * @tparam Values 控制输入参数类型。
     * @param feedback 保护命令使用的底层反馈模式。
     * @param values 需要检查的控制输入。
     * @throw std::invalid_argument 当任一输入不是有限值时抛出。
     */
    template <typename... Values>
    void RejectNonFiniteControlInput(int feedback, Values... values) {
        if ((std::isfinite(values) && ...)) {
            return;
        }
        WarnProtection(active_control_mode_, "non-finite input");
        HoldCurrentMotorPositionsOrStopLocked(feedback);
        throw std::invalid_argument("DoubleCoupleJoint control input must be finite");
    }

    /**
     * @brief 计算将值缩放到范围内的比例因子。
     * @param value 待缩放值。
     * @param range 允许范围。
     * @return 可乘到 value 上的比例；无法通过正比例缩放进入范围时返回 std::nullopt。
     */
    static std::optional<double> ScaleForRange(double value, const Range<float>& range) {
        if (!std::isfinite(value)) {
            return std::nullopt;
        }
        if (value >= range.min - kRangeTolerance && value <= range.max + kRangeTolerance) {
            return 1.0;
        }
        if (value > range.max) {
            if (value <= 0.0 || range.max <= 0.0f) {
                return std::nullopt;
            }
            return std::clamp(static_cast<double>(range.max) / value, 0.0, 1.0);
        }
        if (value < range.min) {
            if (value >= 0.0 || range.min >= 0.0f) {
                return std::nullopt;
            }
            return std::clamp(static_cast<double>(range.min) / value, 0.0, 1.0);
        }
        return std::nullopt;
    }

    /**
     * @brief 获取指定轴的运行时参数（限速、限扭、反向标志）。
     * @param axis 需要查询的输出轴。
     * @return 该轴对应的限位、最大速度、最大扭矩和 reverse 标志。
     * @throw JointException 当查询从轴但从关节尚未绑定时抛出。
     */
    AxisRuntimeParams AxisParams(DoubleCoupleJointAxis axis) const;

    /**
     * @brief 关节坐标系值转换到求解器坐标系值（处理 reverse 标志）。
     * @param axis 值所属的输出轴。
     * @param value 关节坐标系下的值。
     * @return 求解器坐标系下的值。
     */
    double JointToSolver(DoubleCoupleJointAxis axis, double value) const {
        return AxisParams(axis).reverse_ ? -value : value;
    }

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
     * @brief 根据最大扭矩和力矩常数约束电流值。
     * @param current 调用方请求的电流上限。
     * @param max_tor 对应关节轴最大扭矩。
     * @param kt 电机力矩常数。
     * @param scope 保护日志点作用域名（motor1/motor2）。
     * @return 允许下发的电流上限；current <= 0 时返回 0 表示使用底层默认策略。
     * @throw std::invalid_argument 当 kt 无效且 current > 0 时抛出。
     */
    float ResolveMotorCurrent(float current, float max_tor, float kt, const char* scope) const {
        if (current <= 0.0f) {
            return 0.0f;
        }
        if (!IsValidTorqueConstant(kt)) {
            throw std::invalid_argument("MasterJoint motor kt must be finite and positive");
        }
        const float max_current = max_tor / kt;
        return ClampControl(current, 0.0f, max_current, active_control_mode_, "current", scope);
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
     * @brief 评估位姿是否可执行：求解逆运动学、检查状态、验证电机位置范围。
     * @param pose 待评估的求解器坐标系位姿。
     * @param solver 用于求解的求解器实例，成功时其内部状态更新到 pose。
     * @param out_status 输出求解状态，不能为空。
     * @param out_transmission 输出传动信息，不能为空。
     * @param out_positions 若非空，输出相对零位电机位置。
     * @param require_position_slack true 时要求目标电机位置带限位缓冲。
     * @param allow_near_singular false 时将 near_singular 也视为不可执行。
     * @return true 表示位姿可执行且电机位置满足范围约束。
     */
    bool EvaluatePose(const JointPose& pose, couple_joint::MechanismSolver* solver,
                      couple_joint::SolveStatus* out_status,
                      couple_joint::TransmissionInfo* out_transmission,
                      MotorPositions* out_positions, bool require_position_slack = false,
                      bool allow_near_singular = true) const {
        const auto result = solver->SolveInverse(pose.roll, pose.pitch);
        *out_status = result.status;
        *out_transmission = result.transmission;
        if (!IsExecutableSolveResult(result.status, result.transmission) ||
            (!allow_near_singular && result.transmission.near_singular)) {
            if (out_positions)
                *out_positions = MotorPositions{};
            return false;
        }
        const auto state = solver->GetState();
        const MotorPositions positions{
            NormalizeMotorPosition(state.theta1 - solver_parameters_.initial_theta1, "motor1"),
            NormalizeMotorPosition(state.theta2 - solver_parameters_.initial_theta2, "motor2"),
        };
        if (out_positions)
            *out_positions = positions;
        return MotorPositionsInRange(positions, require_position_slack);
    }

    /**
     * @brief 评估 PVT 目标是否可编码，不应用关节或电机软件位置限位。
     */
    bool EvaluatePVTPose(const JointPose& pose, couple_joint::MechanismSolver* solver,
                         couple_joint::SolveStatus* out_status,
                         couple_joint::TransmissionInfo* out_transmission,
                         MotorPositions* out_positions) const {
        const auto result = solver->SolveInverse(pose.roll, pose.pitch);
        *out_status = result.status;
        *out_transmission = result.transmission;
        if (!IsExecutableSolveResult(result.status, result.transmission)) {
            if (out_positions) {
                *out_positions = MotorPositions{};
            }
            return false;
        }
        const auto state = solver->GetState();
        const MotorPositions positions{
            NormalizeMotorPosition(state.theta1 - solver_parameters_.initial_theta1, "motor1"),
            NormalizeMotorPosition(state.theta2 - solver_parameters_.initial_theta2, "motor2"),
        };
        if (out_positions) {
            *out_positions = positions;
        }
        return MotorPositionsInPVTRange(positions);
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
                                        double roll_effort, double pitch_effort) const {
        const auto enters = [&](DoubleCoupleJointAxis axis, double target, double speed,
                                double effort) {
            const double position_delta = target - AxisValue(axis, actual_pose);
            return IsCommandEnteringSingularity(actual_pose, transmission, axis, position_delta) ||
                   IsCommandEnteringSingularity(actual_pose, transmission, axis, speed) ||
                   IsCommandEnteringSingularity(actual_pose, transmission, axis, effort);
        };
        return enters(DoubleCoupleJointAxis::Roll, target_pose.roll, roll_pvt_command_.speed,
                      roll_effort) ||
               enters(DoubleCoupleJointAxis::Pitch, target_pose.pitch, pitch_pvt_command_.speed,
                      pitch_effort);
    }

    /**
     * @brief 对两个值进行一致比例缩放，确保均在各自范围内。
     * @param value1 第一个待缩放值，输入输出参数。
     * @param range1 第一个值的允许范围。
     * @param value2 第二个待缩放值，输入输出参数。
     * @param range2 第二个值的允许范围。
     * @return true 表示缩放后两个值都在对应范围内。
     */
    bool ApplyUniformScale(float* value1, const Range<float>& range1, float* value2,
                           const Range<float>& range2) const {
        const auto scale1 = ScaleForRange(*value1, range1);
        const auto scale2 = ScaleForRange(*value2, range2);
        if (!scale1 || !scale2) {
            return false;
        }
        const double scale = std::min(*scale1, *scale2);
        LogProtection(active_control_mode_, "motor command range scaling", scale < 1.0);
        *value1 = static_cast<float>(*value1 * scale);
        *value2 = static_cast<float>(*value2 * scale);
        return IsInRange(*value1, range1) && IsInRange(*value2, range2);
    }

    /**
     * @brief 逆速度运动学映射：关节速度到双电机速度。
     * @param axis 被控制的输出轴。
     * @param pose_solver 已位于当前位姿的求解器副本。
     * @param solver_speed 求解器坐标系下的关节速度。
     * @param motor1_speed 输出电机 1 速度。
     * @param motor2_speed 输出电机 2 速度。
     * @return true 表示映射成功且双电机速度已按范围缩放。
     */
    bool MapAxisSpeed(DoubleCoupleJointAxis axis, const couple_joint::MechanismSolver& pose_solver,
                      double solver_speed, float* motor1_speed, float* motor2_speed) const {
        return MapJointSpeed(pose_solver, axis == DoubleCoupleJointAxis::Roll ? solver_speed : 0.0,
                             axis == DoubleCoupleJointAxis::Pitch ? solver_speed : 0.0,
                             motor1_speed, motor2_speed);
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
        return ApplyUniformScale(motor1_speed, motor1_ranges_.speed, motor2_speed,
                                 motor2_ranges_.speed);
    }

    /**
     * @brief 逆力矩运动学映射：关节力矩到双电机力矩。
     * @param axis 被控制的输出轴。
     * @param pose_solver 已位于当前位姿的求解器副本。
     * @param solver_torque 求解器坐标系下的关节扭矩。
     * @param motor1_torque 输出电机 1 扭矩。
     * @param motor2_torque 输出电机 2 扭矩。
     * @return true 表示映射成功且双电机扭矩已按范围缩放。
     */
    bool MapAxisTorque(DoubleCoupleJointAxis axis, const couple_joint::MechanismSolver& pose_solver,
                       double solver_torque, float* motor1_torque, float* motor2_torque) const {
        return MapJointTorque(pose_solver,
                              axis == DoubleCoupleJointAxis::Roll ? solver_torque : 0.0,
                              axis == DoubleCoupleJointAxis::Pitch ? solver_torque : 0.0,
                              motor1_torque, motor2_torque);
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
        return ApplyUniformScale(motor1_torque, motor1_ranges_.torque, motor2_torque,
                                 motor2_ranges_.torque);
    }

    AxisPVTCommand& PVTCommandForAxis(DoubleCoupleJointAxis axis) {
        return axis == DoubleCoupleJointAxis::Roll ? roll_pvt_command_ : pitch_pvt_command_;
    }

    /**
     * @brief 在 SDK 层记录协议限幅，再交由电机执行 PVT 编码。
     * @param motor 目标电机。
     * @param ranges 该电机的 PVT 协议范围。
     * @param kp 位置增益。
     * @param kd 微分增益。
     * @param position 目标位置。
     * @param speed 目标速度。
     * @param torque 目标扭矩。
     * @param scope 保护日志点作用域名（motor1/motor2）。
     */
    void SendMotorPVTLocked(Motor* motor, const MotorPVTRanges& ranges, float kp, float kd,
                            float position, float speed, float torque, const char* scope) const {
        position = ClampControl(position, -static_cast<float>(kPi), static_cast<float>(kPi),
                                active_control_mode_, "motor principal position", scope);
        motor->PVTControl<0>(
            ClampControl(kp, ranges.kp.min, ranges.kp.max, active_control_mode_, "motor kp", scope),
            ClampControl(kd, ranges.kd.min, ranges.kd.max, active_control_mode_, "motor kd", scope),
            ClampControl(position, ranges.position.min, ranges.position.max, active_control_mode_,
                         "motor position", scope),
            ClampControl(speed, ranges.speed.min, ranges.speed.max, active_control_mode_,
                         "motor speed", scope),
            ClampControl(torque, ranges.torque.min, ranges.torque.max, active_control_mode_,
                         "motor torque", scope));
    }

    /**
     * @brief 将两轴关节增益按 diag(J^T K J) 转换为两台电机的独立增益。
     */
    bool MapPVTGains(const couple_joint::MechanismSolver& pose_solver, float* motor1_kp,
                     float* motor2_kp, float* motor1_kd, float* motor2_kd) const {
        const auto transmission = pose_solver.GetTransmission();
        if (HasHardSingularity(transmission)) {
            return false;
        }
        const auto& J = transmission.velocity_jacobian;
        *motor1_kp = static_cast<float>(J.m00 * J.m00 * roll_pvt_command_.kp +
                                        J.m10 * J.m10 * pitch_pvt_command_.kp);
        *motor2_kp = static_cast<float>(J.m01 * J.m01 * roll_pvt_command_.kp +
                                        J.m11 * J.m11 * pitch_pvt_command_.kp);
        *motor1_kd = static_cast<float>(J.m00 * J.m00 * roll_pvt_command_.kd +
                                        J.m10 * J.m10 * pitch_pvt_command_.kd);
        *motor2_kd = static_cast<float>(J.m01 * J.m01 * roll_pvt_command_.kd +
                                        J.m11 * J.m11 * pitch_pvt_command_.kd);
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

    /**
     * @brief SDK 解算力矩；HostTorque 包含速度反馈，HostKpMotorKd 保留电机本地阻尼。
     * @return 力矩命令发送结果；Unavailable 表示应退回普通 PVT。
     */
    HostTorquePVTResult TryHostTorquePVTLocked(const JointPose& target_pose,
                                               const MotorPositions& target_positions,
                                               PVTControlMode mode) {
        if (waiting_for_post_zero_feedback_ || zero_reset_faulted_) {
            return HostTorquePVTResult::Unavailable;
        }
        const auto motor1_status = motor1_->GetStatus(0);
        const auto motor2_status = motor2_->GetStatus(0);
        if (!motor1_status || !motor2_status || !std::isfinite(motor1_status->position) ||
            !std::isfinite(motor2_status->position)) {
            return HostTorquePVTResult::Unavailable;
        }
        const MotorPositions feedback_positions{motor1_status->position, motor2_status->position};

        auto feedback_solver = actual_pose_state_ ? actual_pose_state_->solver : *solver_;
        const auto forward =
            feedback_solver.SolveForward(Motor1ToSolverTheta(motor1_status->position),
                                         Motor2ToSolverTheta(motor2_status->position));
        if (!IsExecutableSolveResult(forward.status, forward.transmission)) {
            return HostTorquePVTResult::Unavailable;
        }
        StoreActualPoseLocked(feedback_solver, feedback_positions);
        const auto feedback_state = feedback_solver.GetState();
        const JointPose feedback_pose{feedback_state.roll, feedback_state.pitch};
        const auto roll_params = AxisParams(DoubleCoupleJointAxis::Roll);
        const auto pitch_params = AxisParams(DoubleCoupleJointAxis::Pitch);
        double roll_torque = roll_pvt_command_.kp * (target_pose.roll - feedback_pose.roll) +
                             roll_pvt_command_.torque;
        double pitch_torque = pitch_pvt_command_.kp * (target_pose.pitch - feedback_pose.pitch) +
                              pitch_pvt_command_.torque;
        if (mode == PVTControlMode::HostTorque) {
            if (!std::isfinite(motor1_status->speed) || !std::isfinite(motor2_status->speed)) {
                return HostTorquePVTResult::Unavailable;
            }
            const auto velocity =
                feedback_solver.SolveForwardVelocity(motor1_status->speed, motor2_status->speed);
            if (!IsExecutableSolveResult(velocity.status, velocity.transmission) ||
                !std::isfinite(velocity.roll) || !std::isfinite(velocity.pitch)) {
                return HostTorquePVTResult::Unavailable;
            }
            roll_torque += roll_pvt_command_.kd * (roll_pvt_command_.speed - velocity.roll);
            pitch_torque += pitch_pvt_command_.kd * (pitch_pvt_command_.speed - velocity.pitch);
        }
        roll_torque = ClampControl(roll_torque, -roll_params.maxTor_, roll_params.maxTor_,
                                   active_control_mode_, "roll torque");
        pitch_torque = ClampControl(pitch_torque, -pitch_params.maxTor_, pitch_params.maxTor_,
                                    active_control_mode_, "pitch torque");

        const auto torque_result = feedback_solver.SolveInverseTorque(roll_torque, pitch_torque);
        if (!IsExecutableSolveResult(torque_result.status, torque_result.transmission)) {
            return HostTorquePVTResult::Unavailable;
        }
        float motor1_torque = static_cast<float>(torque_result.theta1);
        float motor2_torque = static_cast<float>(torque_result.theta2);
        if (!ApplyUniformScale(&motor1_torque, motor1_ranges_.torque, &motor2_torque,
                               motor2_ranges_.torque)) {
            return HostTorquePVTResult::Unavailable;
        }

        float motor1_speed = 0.0f;
        float motor2_speed = 0.0f;
        if (!MapJointSpeed(feedback_solver, roll_pvt_command_.speed, pitch_pvt_command_.speed,
                           &motor1_speed, &motor2_speed)) {
            return HostTorquePVTResult::Unavailable;
        }

        if (JointPVTCommandsLeavingLimit(feedback_pose, target_pose, roll_torque, pitch_torque) ||
            MotorPVTCommandsLeavingLimit(feedback_positions, target_positions, motor1_speed,
                                         motor2_speed, motor1_torque, motor2_torque) ||
            PVTCommandsEnteringSingularity(feedback_pose, target_pose, forward.transmission,
                                           roll_torque, pitch_torque)) {
            HoldMotorPositionsOrStopLocked(feedback_positions, 1);
            return HostTorquePVTResult::Protected;
        }

        if (mode == PVTControlMode::HostTorque) {
            motor1_->TorControl<0>(motor1_torque, 1);
            motor2_->TorControl<0>(motor2_torque, 1);
            return HostTorquePVTResult::Sent;
        }

        float kp1, kp2, kd1, kd2;
        if (!MapPVTGains(feedback_solver, &kp1, &kp2, &kd1, &kd2)) {
            return HostTorquePVTResult::Unavailable;
        }
        SendMotorPVTLocked(motor1_, motor1_ranges_, 0.0f, kd1, feedback_positions.motor1,
                           motor1_speed, motor1_torque, "motor1");
        SendMotorPVTLocked(motor2_, motor2_ranges_, 0.0f, kd2, feedback_positions.motor2,
                           motor2_speed, motor2_torque, "motor2");
        return HostTorquePVTResult::Sent;
    }

    /**
     * @brief 基于力矩比的电流分配：关节电流到双电机电流。
     * @param axis 被控制的输出轴。
     * @param pose_solver 已位于当前位姿的求解器副本。
     * @param solver_current 求解器坐标系下的关节电流命令。
     * @param motor_currents 输出双电机电流。
     * @return true 表示分配成功且电流已按电机范围缩放。
     */
    bool MapAxisCurrent(DoubleCoupleJointAxis axis,
                        const couple_joint::MechanismSolver& pose_solver, double solver_current,
                        MotorCurrents* motor_currents) const {
        if (!std::isfinite(solver_current) || motor1_kt_ <= 0.0f || motor2_kt_ <= 0.0f) {
            return false;
        }
        if (solver_current == 0.0) {
            *motor_currents = MotorCurrents{};
            return true;
        }

        const double unit_axis_torque = std::copysign(1.0, solver_current);
        const auto torque_result = axis == DoubleCoupleJointAxis::Roll
                                       ? pose_solver.SolveInverseTorque(unit_axis_torque, 0.0)
                                       : pose_solver.SolveInverseTorque(0.0, unit_axis_torque);
        if (!IsUsable(torque_result.status) || HasHardSingularity(torque_result.transmission)) {
            return false;
        }

        const double weight1 = torque_result.theta1 / motor1_kt_;
        const double weight2 = torque_result.theta2 / motor2_kt_;
        const double weight_sum = std::abs(weight1) + std::abs(weight2);
        if (!std::isfinite(weight_sum) || weight_sum <= std::numeric_limits<double>::epsilon()) {
            return false;
        }

        // weight_sum A produces one Nm on the requested virtual axis.
        // Scale the pair together to preserve zero torque on the other axis.
        const double current_magnitude =
            std::min(std::abs(solver_current), AxisParams(axis).maxTor_ * weight_sum);
        LogProtection(active_control_mode_, "joint torque limit in current mapping",
                      current_magnitude < std::abs(solver_current));
        motor_currents->motor1 = static_cast<float>(weight1 * current_magnitude / weight_sum);
        motor_currents->motor2 = static_cast<float>(weight2 * current_magnitude / weight_sum);
        return ApplyUniformScale(&motor_currents->motor1, motor1_ranges_.current,
                                 &motor_currents->motor2, motor2_ranges_.current);
    }

    /**
     * @brief 基于力矩比的双轴电流分配：两条虚拟轴电流到双电机电流。
     *
     * 对每个被命令的轴按 MapAxisCurrent 的权重公式求出"该轴 1 Nm 对应的双电机电流
     * 权重"，再按各轴自身幅度线性叠加（J^T 两列独立缩放）。只有单轴非零时与
     * MapAxisCurrent 逐位等价。
     * @param pose_solver 已位于目标位姿的求解器副本。
     * @param roll_current 求解器坐标系下的 roll 轴电流命令。
     * @param pitch_current 求解器坐标系下的 pitch 轴电流命令。
     * @param motor_currents 输出双电机电流。
     * @return true 表示分配成功且电流已按电机范围缩放。
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
            const double current_magnitude =
                std::min(std::abs(commanded), AxisParams(axis).maxTor_ * weight_sum);
            LogProtection(active_control_mode_, "joint torque limit in current mapping",
                          current_magnitude < std::abs(commanded));
            motor1_current += (weight1 * current_magnitude) / weight_sum;
            motor2_current += (weight2 * current_magnitude) / weight_sum;
        }

        motor_currents->motor1 = static_cast<float>(motor1_current);
        motor_currents->motor2 = static_cast<float>(motor2_current);
        return ApplyUniformScale(&motor_currents->motor1, motor1_ranges_.current,
                                 &motor_currents->motor2, motor2_ranges_.current);
    }

    void StoreActualPoseLocked(const couple_joint::MechanismSolver& feedback_solver,
                               const MotorPositions& positions) {
        const auto state = feedback_solver.GetState();
        actual_pose_state_.emplace(
            ActualPoseState{JointPose{state.roll, state.pitch}, positions, feedback_solver});
    }

    std::optional<ActualPoseState> ResolveActualPoseLocked() {
        if (waiting_for_post_zero_feedback_ || zero_reset_faulted_) {
            return actual_pose_state_;
        }
        const auto motor1_status = motor1_->GetStatus(0);
        const auto motor2_status = motor2_->GetStatus(0);
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

    /**
     * @brief 判断下一条位置类控制是否为两轴合帧批次的第一条。
     * @return true 表示当前没有等待第二轴完成的半帧目标。
     */
    bool IsFirstPositionControlBatchCommandLocked() const {
        return (position_control_count_ % 2U) == 0U;
    }

    bool CanFeedbackRefreshTargetLocked() const {
        return IsFirstPositionControlBatchCommandLocked() && !PositionTargetFeedbackLocked();
    }

    /**
     * @brief 记录位置类控制批次开始前的稳定目标和电机保持位置。
     * @param pose 当前稳定目标位姿。
     * @param pose_solver 已求解到当前稳定目标的求解器副本。
     * @param positions 当前稳定目标对应的双电机位置。
     */
    void CapturePositionControlBatchStartLocked(const JointPose& pose,
                                                const couple_joint::MechanismSolver& pose_solver,
                                                const MotorPositions& positions) {
        if (!IsFirstPositionControlBatchCommandLocked()) {
            return;
        }
        position_batch_start_pose_ = pose;
        position_batch_start_solver_.emplace(pose_solver);
        position_batch_start_positions_ = positions;
        position_batch_hold_axis_.reset();
    }

    /**
     * @brief 清除位置类控制批次状态，不改变软限位保护锁。
     */
    void ClearPositionControlBatchProgressLocked() {
        position_control_count_ = 0;
        position_batch_start_pose_.reset();
        position_batch_start_solver_.reset();
        position_batch_start_positions_.reset();
        position_batch_hold_axis_.reset();
        pvt_batch_torque_mode_.reset();
        roll_position_command_.reset();
        pitch_position_command_.reset();
        // 动态类共用同一个奇偶闩锁，丢弃位置类半帧时动态半帧同样失效。
        ClearDynamicControlBatchLocked();
    }

    /**
     * @brief 清除动态类控制的轴目标槽位与批次标签。
     *
     * 槽位跨批次保留（未在本批次命令过的轴沿用上次值），只有 Stop、SetZero 这类显式
     * 作废命令以及丢弃半帧时才清空，避免被丢弃的命令在后续合帧里复活。
     */
    void ClearDynamicControlBatchLocked() {
        batch_class_ = CoupledBatchClass::None;
        dynamic_batch_mode_.reset();
        roll_speed_command_.reset();
        pitch_speed_command_.reset();
        roll_torque_command_.reset();
        pitch_torque_command_.reset();
        roll_current_command_.reset();
        pitch_current_command_.reset();
    }

    /**
     * @brief 按模式与轴取动态控制槽位。
     * @param mode 动态控制模式。
     * @param axis 被控制的输出轴。
     * @return 对应槽位的引用。
     */
    std::optional<AxisDynamicCommand>& DynamicCommandSlotLocked(DynamicControlMode mode,
                                                                DoubleCoupleJointAxis axis) {
        const bool is_roll = axis == DoubleCoupleJointAxis::Roll;
        switch (mode) {
            case DynamicControlMode::Speed:
                return is_roll ? roll_speed_command_ : pitch_speed_command_;
            case DynamicControlMode::Torque:
                return is_roll ? roll_torque_command_ : pitch_torque_command_;
            case DynamicControlMode::Current:
            default:
                return is_roll ? roll_current_command_ : pitch_current_command_;
        }
    }

    /**
     * @brief 进入动态类控制前对齐合帧闩锁。
     *
     * 挂起的是位置类半帧、或另一种动态模式的半帧时丢弃该半帧，本次调用作为新批次第
     * 一条处理；同类半帧保留，由本次调用完成合帧。
     * @param mode 本次调用的动态控制模式。
     */
    void AlignDynamicControlBatchLocked(DynamicControlMode mode) {
        ClearPositionTargetFeedbackLockLocked();
        const bool half_frame_pending = !IsFirstPositionControlBatchCommandLocked();
        if (half_frame_pending && (batch_class_ != CoupledBatchClass::Dynamic ||
                                   !dynamic_batch_mode_ || *dynamic_batch_mode_ != mode)) {
            ClearPositionControlBatchProgressLocked();
        }
        dynamic_batch_mode_ = mode;
    }

    /**
     * @brief 丢弃挂起的动态类半帧，供位置类控制入口调用。
     */
    void DiscardDynamicControlBatchLocked() {
        if (batch_class_ == CoupledBatchClass::Dynamic) {
            ClearPositionControlBatchProgressLocked();
        }
    }

    /**
     * @brief 动态类共用合帧门控：第一条只缓存轴目标，第二条继续下发。
     * @param mode 本次调用的动态控制模式。
     * @return true 表示本次应仅缓存目标并跳过电机命令。
     */
    bool ShouldDeferDynamicControlLocked(DynamicControlMode mode) {
        const bool first = IncrementAndTestFirstBatchCommandLocked();
        batch_class_ = CoupledBatchClass::Dynamic;
        dynamic_batch_mode_ = mode;
        return first;
    }

    /**
     * @brief 成功下发完整两轴目标后，丢弃批次起点缓存但保留奇偶计数。
     */
    void FinishPositionControlBatchLocked() {
        position_batch_start_pose_.reset();
        position_batch_start_solver_.reset();
        position_batch_start_positions_.reset();
        position_batch_hold_axis_.reset();
        pvt_batch_torque_mode_.reset();
        roll_position_command_.reset();
        pitch_position_command_.reset();
        batch_class_ = CoupledBatchClass::None;
        dynamic_batch_mode_.reset();
    }

    /**
     * @brief 目标被保护拒绝时，恢复到本批次开始前的稳定目标。
     * @param fallback_positions 没有批次起点缓存时使用的保持位置。
     * @return 应下发的位置保持电机位置。
     */
    MotorPositions RestorePositionControlBatchStartLocked(
        const MotorPositions& fallback_positions) {
        if (position_batch_start_pose_ && position_batch_start_solver_) {
            CommitTargetPose(*position_batch_start_pose_, *position_batch_start_solver_);
        }
        return position_batch_start_positions_.value_or(fallback_positions);
    }

    /**
     * @brief 位置目标保护下发保持命令；同一两轴批次内只下发一次。
     * @param axis 当前控制轴。
     * @param positions 需要保持的双电机位置。
     * @param feedback 底层电机反馈模式。
     */
    void HoldPositionControlBatchLocked(DoubleCoupleJointAxis axis, const MotorPositions& positions,
                                        int feedback) {
        if (!IsFirstPositionControlBatchCommandLocked() && position_batch_hold_axis_) {
            if (*position_batch_hold_axis_ != axis) {
                ClearPositionControlBatchProgressLocked();
                return;
            }
            ClearPositionControlBatchProgressLocked();
        }

        const bool first_command = IsFirstPositionControlBatchCommandLocked();
        HoldMotorPositionsOrStopLocked(positions, feedback);
        if (first_command) {
            position_control_count_ = 1;
            position_batch_hold_axis_ = axis;
        } else {
            ClearPositionControlBatchProgressLocked();
        }
    }

    /**
     * @brief 推进合帧奇偶闩锁并判断本次是否为批次的第一条命令。
     * @return true 表示本次是批次第一条，应仅缓存目标。
     */
    bool IncrementAndTestFirstBatchCommandLocked() {
        ++position_control_count_;
        return (position_control_count_ % 2U) != 0U;
    }

    /**
     * @brief PVT/Pos 共用的位置类控制合帧：奇数次只提交目标缓存，偶数次继续下发。
     * @param pose 已验证可执行的目标位姿。
     * @param pose_solver 已求解到目标位姿的求解器副本。
     * @return true 表示本次应仅缓存目标并跳过电机命令。
     */
    bool ShouldDeferPositionControlLocked(const JointPose& pose,
                                          const couple_joint::MechanismSolver& pose_solver) {
        if (!IncrementAndTestFirstBatchCommandLocked()) {
            return false;
        }
        batch_class_ = CoupledBatchClass::Position;
        CommitTargetPose(pose, pose_solver);
        return true;
    }

    /**
     * @brief 清除位置类控制合帧状态；调用方必须已持有 mutex_。
     */
    void ResetPositionControlBatchLocked() {
        ClearPositionControlBatchProgressLocked();
        ClearPositionTargetFeedbackLockLocked();
    }

    /**
     * @brief 判断位置目标保护是否正在阻止反馈覆盖目标缓存。
     * @return true 表示某个输出轴的位置目标已被软限位保护拦截。
     */
    bool PositionTargetFeedbackLocked() const {
        return position_target_feedback_lock_axis_.has_value();
    }

    /**
     * @brief 判断另一个输出轴是否已经触发位置目标保护。
     * @param axis 当前控制轴。
     * @return true 表示当前轴不能覆盖已锁住的位置保持目标。
     */
    bool OtherAxisPositionTargetFeedbackLocked(DoubleCoupleJointAxis axis) const {
        return position_target_feedback_lock_axis_ && *position_target_feedback_lock_axis_ != axis;
    }

    /**
     * @brief 进入位置目标保护后，禁止反馈把目标缓存拖向扰动位置。
     * @param axis 触发保护的输出轴。
     */
    void LockPositionTargetFeedbackLocked(DoubleCoupleJointAxis axis) {
        position_target_feedback_lock_axis_ = axis;
    }

    /**
     * @brief 受限轴收到可执行目标后恢复反馈目标刷新。
     * @param axis 当前可执行控制的输出轴。
     */
    void ClearPositionTargetFeedbackLockForAxisLocked(DoubleCoupleJointAxis axis) {
        if (position_target_feedback_lock_axis_ && *position_target_feedback_lock_axis_ == axis) {
            position_target_feedback_lock_axis_.reset();
        }
    }

    /**
     * @brief 无条件恢复反馈目标刷新。
     */
    void ClearPositionTargetFeedbackLockLocked() {
        position_target_feedback_lock_axis_.reset();
    }

    /**
     * @brief 计算位置保持命令使用的速度和电流上限。
     * @param motor1_speed 输出电机 1 位置控制速度上限。
     * @param motor2_speed 输出电机 2 位置控制速度上限。
     * @param motor1_current 输出电机 1 位置控制电流上限。
     * @param motor2_current 输出电机 2 位置控制电流上限。
     * @return true 表示所有输出值有效且可用于位置保持。
     */
    bool PositionHoldCommandLimits(float* motor1_speed, float* motor2_speed, float* motor1_current,
                                   float* motor2_current) const {
        const auto master_params = AxisParams(axis_);
        const auto slave_params = AxisParams(slave_axis_);
        const float hold_max_tor = std::max(master_params.maxTor_, slave_params.maxTor_);

        if (!std::isfinite(motor1_ranges_.speed.max) || motor1_ranges_.speed.max <= 0.0f ||
            !std::isfinite(motor2_ranges_.speed.max) || motor2_ranges_.speed.max <= 0.0f ||
            !std::isfinite(motor1_ranges_.current.max) || motor1_ranges_.current.max <= 0.0f ||
            !std::isfinite(motor2_ranges_.current.max) || motor2_ranges_.current.max <= 0.0f ||
            !IsValidTorqueConstant(motor1_kt_) || !IsValidTorqueConstant(motor2_kt_) ||
            !std::isfinite(hold_max_tor) || hold_max_tor <= 0.0f) {
            return false;
        }

        const float current1 = hold_max_tor / motor1_kt_;
        const float current2 = hold_max_tor / motor2_kt_;
        if (!std::isfinite(current1) || current1 <= 0.0f || !std::isfinite(current2) ||
            current2 <= 0.0f) {
            return false;
        }

        *motor1_speed = motor1_ranges_.speed.max;
        *motor2_speed = motor2_ranges_.speed.max;
        *motor1_current = std::min(current1, motor1_ranges_.current.max);
        *motor2_current = std::min(current2, motor2_ranges_.current.max);
        return true;
    }

    /**
     * @brief 按指定电机位置发送双电机位置保持命令。
     * @param positions 需要保持的双电机相对零位位置。
     * @param feedback 底层电机反馈模式。
     * @return true 表示两台电机位置保持命令都已成功下发。
     * @note 调用方必须已持有 mutex_。
     */
    bool SendMotorPositionControlLocked(const MotorPositions& positions, float speed1, float speed2,
                                        float current1, float current2, int feedback) {
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
        if (!slack_available) {
            // 限位区间不可用：本次不判断限幅后的闭链可执行性，按未触发处理
            LogProtection(active_control_mode_,
                          "clipped position violates closed-chain/joint limits", false);
            return false;
        }
        // 原始反馈只限幅，不能取模后跳到另一圈；逆解结果已在 EvaluatePose/PVTPose 归一化。
        const float position1 =
            ClampControl(positions.motor1, lower1, upper1, active_control_mode_, "motor1 position");
        const float position2 =
            ClampControl(positions.motor2, lower2, upper2, active_control_mode_, "motor2 position");
        // Independent motor clipping can move either virtual axis outward.
        auto hold_solver = actual_pose_state_ ? actual_pose_state_->solver : *solver_;
        const auto forward = hold_solver.SolveForward(Motor1ToSolverTheta(position1),
                                                      Motor2ToSolverTheta(position2));
        const auto hold_pose = hold_solver.GetState();
        const bool clipped_pose_in_range =
            IsExecutableSolveResult(forward.status, forward.transmission) &&
            IsInRange(hold_pose.roll, AxisParams(DoubleCoupleJointAxis::Roll).limit_) &&
            IsInRange(hold_pose.pitch, AxisParams(DoubleCoupleJointAxis::Pitch).limit_);
        LogProtection(active_control_mode_, "clipped position violates closed-chain/joint limits",
                      !clipped_pose_in_range);
        if (!clipped_pose_in_range) {
            return false;
        }
        motor1_->PosControl<0>(position1, speed1, current1, feedback);
        motor2_->PosControl<0>(position2, speed2, current2, feedback);
        return true;
    }

    bool HoldMotorPositionsLocked(const MotorPositions& positions, int feedback) {
        // 只在真分支里被调用，标志位由一次成功下发跟踪命令或 Stop/SetZero 清除
        LogProtection(active_control_mode_, "position hold", true);
        if (!std::isfinite(positions.motor1) || !std::isfinite(positions.motor2)) {
            return false;
        }
        try {
            float motor1_speed = 0.0f;
            float motor2_speed = 0.0f;
            float motor1_current = 0.0f;
            float motor2_current = 0.0f;
            if (!PositionHoldCommandLimits(&motor1_speed, &motor2_speed, &motor1_current,
                                           &motor2_current)) {
                return false;
            }
            return SendMotorPositionControlLocked(positions, motor1_speed, motor2_speed,
                                                  motor1_current, motor2_current, feedback);
        } catch (const JointException& ex) {
            LogProtectionMessage(active_control_mode_, "hold-position failed", true,
                                 "Joint protection: joint={}, mode={}, reason=hold-position "
                                 "failed, error={}, message={}, state=enter",
                                 protection_name_, active_control_mode_,
                                 static_cast<int>(ex.Error()), ex.what());
            return false;
        } catch (const std::exception& ex) {
            LogProtectionMessage(active_control_mode_, "hold-position failed", true,
                                 "Joint protection: joint={}, mode={}, reason=hold-position "
                                 "failed, message={}, state=enter",
                                 protection_name_, active_control_mode_, ex.what());
            return false;
        }
    }

    /**
     * @brief 优先按指定位置保持，失败时退回 FullBrake。
     * @param positions 需要保持的双电机相对零位位置。
     * @param feedback 底层电机反馈模式。
     * @note 调用方必须已持有 mutex_。
     */
    void HoldMotorPositionsOrStopLocked(const MotorPositions& positions, int feedback) {
        if (HoldMotorPositionsLocked(positions, feedback)) {
            return;
        }
        LogProtection(active_control_mode_, "hold unavailable; FullBrake", true);
        SendMotorStopLocked(MotorStopMode::FullBrake, 0.0f, feedback);
    }

    /**
     * @brief 读取缓存反馈并下发当前位置保持命令；调用方必须已持有 mutex_。
     * @param feedback 底层电机反馈模式。
     * @return true 表示读取到有效缓存位置并成功下发位置保持命令。
     */
    bool HoldCurrentMotorPositionsLocked(int feedback) {
        auto motor1_status = motor1_->GetStatus(0);
        auto motor2_status = motor2_->GetStatus(0);
        if (!motor1_status || !motor2_status || !std::isfinite(motor1_status->position) ||
            !std::isfinite(motor2_status->position)) {
            return false;
        }

        return HoldMotorPositionsLocked(
            MotorPositions{motor1_status->position, motor2_status->position}, feedback);
    }

    /**
     * @brief 报错保护：优先位置保持，缺少有效反馈时退回 FullBrake。
     * @param feedback 底层电机反馈模式。
     * @note 调用方必须已持有 mutex_。
     */
    void HoldCurrentMotorPositionsOrStopLocked(int feedback) {
        if (HoldCurrentMotorPositionsLocked(feedback)) {
            return;
        }
        LogProtection(active_control_mode_, "feedback/hold unavailable; FullBrake", true);
        SendMotorStopLocked(MotorStopMode::FullBrake, 0.0f, feedback);
    }

    /**
     * @brief 清除「位置保持」类保护日志点。
     *
     * 这几个错误点只在保护真分支里被置位，自身观测不到条件转假：一次成功下发跟踪命令、
     * 或 Stop/SetZero 显式作废批次之后在此统一清除。调用方必须已持有 mutex_。
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
        if (waiting_for_post_zero_feedback_ || zero_reset_faulted_) {
            return false;
        }
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
        active_control_mode_ = torque_mode == PVTControlMode::MotorPVT     ? "PVT"
                               : torque_mode == PVTControlMode::HostTorque ? "PVT HostTorque"
                                                                           : "PVT HostKpMotorKd";
        if (!IsValidPVTControlMode(torque_mode)) {
            RestorePositionControlBatchStartLocked(MotorPositions{});
            ResetPositionControlBatchLocked();
            HoldCurrentMotorPositionsOrStopLocked(1);
            throw std::invalid_argument("Invalid PVT control mode");
        }
        DiscardDynamicControlBatchLocked();
        RejectNonFiniteControlInput(1, kp, kd, pos, spd, torque);
        const auto params = AxisParams(axis);
        const double solver_pos = JointToSolver(axis, pos);
        const JointPose current_pose{target_roll_, target_pitch_};

        // Layer 1: 当前位姿必须可执行
        auto solver = *solver_;
        couple_joint::SolveStatus status;
        couple_joint::TransmissionInfo transmission;
        MotorPositions current_positions;
        if (!EvaluatePVTPose(current_pose, &solver, &status, &transmission, &current_positions)) {
            WarnProtection(active_control_mode_, "current pose not executable");
            HoldCurrentMotorPositionsOrStopLocked(1);
            throw JointException(JointError::kDoubleCoupleJointSolveFailed,
                                 "DoubleCoupleJoint current pose is not executable");
        }
        CapturePositionControlBatchStartLocked(current_pose, solver, current_positions);
        const bool paired_axis_latched = OtherAxisPositionTargetFeedbackLocked(axis);
        LogProtection(active_control_mode_, "paired axis target protection latched",
                      paired_axis_latched, {}, AxisScope(axis));
        if (paired_axis_latched) {
            HoldPositionControlBatchLocked(axis, current_positions, 1);
            return;
        }

        JointPose target_pose = current_pose;
        SetAxisValue(axis, solver_pos, &target_pose);

        // Layer 2: 目标位姿必须可执行
        MotorPositions target_positions;
        const bool target_evaluable =
            EvaluatePVTPose(target_pose, &solver, &status, &transmission, &target_positions);
        LogProtection(active_control_mode_, "target inverse solve/protocol range rejected",
                      !target_evaluable, {}, AxisScope(axis));
        if (!target_evaluable) {
            LockPositionTargetFeedbackLocked(axis);
            const auto hold_positions = RestorePositionControlBatchStartLocked(current_positions);
            HoldPositionControlBatchLocked(axis, hold_positions, 1);
            return;
        }
        ClearPositionTargetFeedbackLockForAxisLocked(axis);
        const double solver_speed =
            JointToSolver(axis, ClampControl(spd, -params.maxSpd_, params.maxSpd_,
                                             active_control_mode_, "speed", AxisScope(axis)));
        const double solver_torque =
            JointToSolver(axis, ClampControl(torque, -params.maxTor_, params.maxTor_,
                                             active_control_mode_, "torque", AxisScope(axis)));
        PVTCommandForAxis(axis) = AxisPVTCommand{kp, kd, solver_speed, solver_torque};
        const bool first_batch_command = IsFirstPositionControlBatchCommandLocked();
        if (first_batch_command) {
            pvt_batch_torque_mode_ = torque_mode;
        }
        const bool use_host_torque = !first_batch_command &&
                                     torque_mode != PVTControlMode::MotorPVT &&
                                     pvt_batch_torque_mode_ == torque_mode;
        if (ShouldDeferPositionControlLocked(target_pose, solver)) {
            return;
        }
        if (use_host_torque) {
            const auto host_result =
                TryHostTorquePVTLocked(target_pose, target_positions, torque_mode);
            const bool host_unavailable = host_result == HostTorquePVTResult::Unavailable;
            LogProtection(active_control_mode_, "host feedback unavailable; motor PVT fallback",
                          host_unavailable, {}, AxisScope(axis));
            if (!host_unavailable) {
                if (host_result == HostTorquePVTResult::Sent) {
                    CommitTargetPose(target_pose, solver);
                    ClearHoldProtectionLocked();
                } else {
                    RestorePositionControlBatchStartLocked(current_positions);
                }
                FinishPositionControlBatchLocked();
                return;
            }
        }

        const auto actual_pose = ResolveActualPoseLocked();
        if (!actual_pose) {
            if (zero_reset_faulted_) {
                WarnProtection(active_control_mode_, "zero reset fault; FullBrake");
                Stop(MotorStopMode::FullBrake, 0.0f, 1);
            } else {
                HoldCurrentMotorPositionsOrStopLocked(1);
            }
            throw JointException(JointError::kDoubleCoupleJointSolveFailed,
                                 "DoubleCoupleJoint actual motor pose is unavailable");
        }

        float motor1_speed = 0.0f;
        float motor2_speed = 0.0f;
        float motor1_torque = 0.0f;
        float motor2_torque = 0.0f;
        float motor1_kp = 0.0f;
        float motor2_kp = 0.0f;
        float motor1_kd = 0.0f;
        float motor2_kd = 0.0f;
        if (!MapJointSpeed(actual_pose->solver, roll_pvt_command_.speed, pitch_pvt_command_.speed,
                           &motor1_speed, &motor2_speed) ||
            !MapJointTorque(actual_pose->solver, roll_pvt_command_.torque,
                            pitch_pvt_command_.torque, &motor1_torque, &motor2_torque) ||
            !MapPVTGains(actual_pose->solver, &motor1_kp, &motor2_kp, &motor1_kd, &motor2_kd)) {
            WarnProtection(active_control_mode_, "PVT gain/velocity/torque mapping failed");
            HoldMotorPositionsOrStopLocked(actual_pose->positions, 1);
            throw JointException(JointError::kDoubleCoupleJointSolveFailed,
                                 "DoubleCoupleJoint PVT gain, velocity, or torque mapping failed");
        }
        const auto actual_transmission = actual_pose->solver.GetTransmission();
        if (JointPVTCommandsLeavingLimit(actual_pose->pose, target_pose, roll_pvt_command_.torque,
                                         pitch_pvt_command_.torque) ||
            MotorPVTCommandsLeavingLimit(actual_pose->positions, target_positions, motor1_speed,
                                         motor2_speed, motor1_torque, motor2_torque) ||
            PVTCommandsEnteringSingularity(actual_pose->pose, target_pose, actual_transmission,
                                           roll_pvt_command_.torque, pitch_pvt_command_.torque)) {
            RestorePositionControlBatchStartLocked(actual_pose->positions);
            HoldMotorPositionsOrStopLocked(actual_pose->positions, 1);
            FinishPositionControlBatchLocked();
            return;
        }
        SendMotorPVTLocked(motor1_, motor1_ranges_, motor1_kp, motor1_kd, target_positions.motor1,
                           motor1_speed, motor1_torque, "motor1");
        SendMotorPVTLocked(motor2_, motor2_ranges_, motor2_kp, motor2_kd, target_positions.motor2,
                           motor2_speed, motor2_torque, "motor2");
        CommitTargetPose(target_pose, solver);
        FinishPositionControlBatchLocked();
        ClearHoldProtectionLocked();
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
        active_control_mode_ = "PosControl";
        DiscardDynamicControlBatchLocked();
        RejectNonFiniteControlInput(feedback, position, speed, current);
        const auto params = AxisParams(axis);
        double solver_position = ClampToLimitSlack(JointToSolver(axis, position), params.limit_);
        LogProtection(active_control_mode_, "joint position limit",
                      solver_position != JointToSolver(axis, position), {}, AxisScope(axis));
        JointPose current_pose{target_roll_, target_pitch_};

        // Layer 1: 当前位姿必须可执行
        auto solver = *solver_;
        couple_joint::SolveStatus status;
        couple_joint::TransmissionInfo transmission;
        MotorPositions current_positions;
        bool current_pose_executable =
            EvaluatePose(current_pose, &solver, &status, &transmission, &current_positions);
        if (!current_pose_executable && IsFirstPositionControlBatchCommandLocked() &&
            RefreshTargetPoseFromFeedbackLocked(0)) {
            current_pose = JointPose{target_roll_, target_pitch_};
            solver = *solver_;
            current_pose_executable =
                EvaluatePose(current_pose, &solver, &status, &transmission, &current_positions);
        }
        if (!current_pose_executable) {
            HoldCurrentMotorPositionsOrStopLocked(feedback);
            throw JointException(JointError::kDoubleCoupleJointSolveFailed,
                                 "DoubleCoupleJoint current pose is not executable");
        }
        CapturePositionControlBatchStartLocked(current_pose, solver, current_positions);
        if (OtherAxisPositionTargetFeedbackLocked(axis)) {
            HoldPositionControlBatchLocked(axis, current_positions, feedback);
            return;
        }

        JointPose target_pose = current_pose;
        SetAxisValue(axis, solver_position, &target_pose);

        // Layer 2: 目标位姿必须可执行
        MotorPositions target_positions;
        const bool target_reachable = EvaluatePose(target_pose, &solver, &status, &transmission,
                                                   &target_positions, true, false);
        LogProtection(active_control_mode_, "target unreachable/near singular/outside motor slack",
                      !target_reachable, {}, AxisScope(axis));
        if (!target_reachable) {
            LockPositionTargetFeedbackLocked(axis);
            const auto hold_positions = RestorePositionControlBatchStartLocked(current_positions);
            HoldPositionControlBatchLocked(axis, hold_positions, feedback);
            return;
        }
        ClearPositionTargetFeedbackLockForAxisLocked(axis);
        const float axis_speed = speed > 0.0f ? std::min(speed, params.maxSpd_) : params.maxSpd_;
        LogProtection(active_control_mode_, "speed limit", speed > params.maxSpd_, {},
                      AxisScope(axis));
        auto& command =
            axis == DoubleCoupleJointAxis::Roll ? roll_position_command_ : pitch_position_command_;
        AxisPositionCommand next{
            solver_position, axis_speed,
            ResolveMotorCurrent(current, params.maxTor_, motor1_kt_, "motor1"),
            ResolveMotorCurrent(current, params.maxTor_, motor2_kt_, "motor2")};
        if (command) {
            next.speed = std::min(next.speed, command->speed);
            next.current1 = std::min(next.current1, command->current1);
            next.current2 = std::min(next.current2, command->current2);
        }
        command = next;
        if (ShouldDeferPositionControlLocked(target_pose, solver)) {
            return;
        }

        const auto start = position_batch_start_pose_.value_or(current_pose);
        const double roll_delta = target_pose.roll - start.roll;
        const double pitch_delta = target_pose.pitch - start.pitch;
        const float roll_speed = roll_position_command_
                                     ? roll_position_command_->speed
                                     : AxisParams(DoubleCoupleJointAxis::Roll).maxSpd_;
        const float pitch_speed = pitch_position_command_
                                      ? pitch_position_command_->speed
                                      : AxisParams(DoubleCoupleJointAxis::Pitch).maxSpd_;
        const double duration =
            std::max(std::abs(roll_delta) / roll_speed, std::abs(pitch_delta) / pitch_speed);
        float motor1_speed = 0.0f;
        float motor2_speed = 0.0f;
        // Both axes share a movement duration so neither requested speed is discarded.
        const bool mapped =
            duration > 0.0 ? MapJointSpeed(solver, roll_delta / duration, pitch_delta / duration,
                                           &motor1_speed, &motor2_speed)
                           : MapAxisSpeed(axis, solver, next.speed, &motor1_speed, &motor2_speed);
        if (!mapped) {
            WarnProtection(active_control_mode_, "position velocity mapping failed");
            HoldMotorPositionsOrStopLocked(current_positions, feedback);
            throw JointException(JointError::kDoubleCoupleJointSolveFailed,
                                 "DoubleCoupleJoint position velocity mapping failed");
        }
        // PosControl sends independent motor speed caps, not signed synchronized velocities.
        // Bound each Jacobian row even if only one motor reaches its cap.
        const auto& J = solver.GetTransmission().velocity_jacobian;
        const double roll_bound = std::abs(J.m00 * motor1_speed) + std::abs(J.m01 * motor2_speed);
        const double pitch_bound = std::abs(J.m10 * motor1_speed) + std::abs(J.m11 * motor2_speed);
        double speed_scale = 1.0;
        if (roll_bound > roll_speed)
            speed_scale = std::min(speed_scale, roll_speed / roll_bound);
        if (pitch_bound > pitch_speed)
            speed_scale = std::min(speed_scale, pitch_speed / pitch_bound);
        motor1_speed *= speed_scale;
        motor2_speed *= speed_scale;
        float current1 = next.current1;
        float current2 = next.current2;
        for (const auto& axis_command : {roll_position_command_, pitch_position_command_}) {
            if (axis_command) {
                current1 = std::min(current1, axis_command->current1);
                current2 = std::min(current2, axis_command->current2);
            }
        }
        const bool sent =
            SendMotorPositionControlLocked(target_positions, std::abs(motor1_speed),
                                           std::abs(motor2_speed), current1, current2, feedback);
        LogProtection(active_control_mode_, "position output unavailable; FullBrake", !sent, {},
                      AxisScope(axis));
        if (!sent) {
            Stop(MotorStopMode::FullBrake, 0.0f, feedback);
            return;
        }
        CommitTargetPose(target_pose, solver);
        FinishPositionControlBatchLocked();
        ClearHoldProtectionLocked();
    }

    /**
     * @brief 速度控制模式：两轴合帧后联合逆速度映射下发。
     * @param axis 被控制的输出轴。
     * @param speed 目标速度，单位 rad/s，关节坐标系。
     * @param current 电流上限，单位 A。
     * @param feedback 底层电机反馈模式。
     */
    void ControlSpeed(DoubleCoupleJointAxis axis, float speed, float current, int feedback) {
        ControlDynamic(DynamicControlMode::Speed, axis, speed, current, feedback);
    }

    /**
     * @brief 扭矩控制模式：两轴合帧后联合逆力矩映射下发。
     * @param axis 被控制的输出轴。
     * @param torque 目标扭矩，单位 Nm，关节坐标系。
     * @param feedback 底层电机反馈模式。
     */
    void ControlTorque(DoubleCoupleJointAxis axis, float torque, int feedback) {
        ControlDynamic(DynamicControlMode::Torque, axis, torque, 0.0f, feedback);
    }

    /**
     * @brief 电流控制模式：两轴合帧后联合电流分配下发。
     * @param axis 被控制的输出轴。
     * @param current 目标电流，单位 A，关节坐标系方向。
     * @param feedback 底层电机反馈模式。
     */
    void ControlCurrent(DoubleCoupleJointAxis axis, float current, int feedback) {
        ControlDynamic(DynamicControlMode::Current, axis, current, 0.0f, feedback);
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
        active_control_mode_ = DynamicControlModeName(mode);
        RejectNonFiniteControlInput(feedback, command);
        if (mode == DynamicControlMode::Speed) {
            RejectNonFiniteControlInput(feedback, current);
        }
        AlignDynamicControlBatchLocked(mode);
        JointPose current_pose{target_roll_, target_pitch_};

        // Layer 1: 当前位姿必须可执行（1 次 SolveInverse）
        auto solver = *solver_;
        couple_joint::SolveStatus status;
        couple_joint::TransmissionInfo transmission;
        MotorPositions current_positions;
        bool current_pose_executable =
            EvaluatePose(current_pose, &solver, &status, &transmission, &current_positions);
        if (!current_pose_executable &&
            RefreshTargetPoseFromFeedbackLocked(0, /*respect_batch_gate=*/false)) {
            current_pose = JointPose{target_roll_, target_pitch_};
            solver = *solver_;
            current_pose_executable =
                EvaluatePose(current_pose, &solver, &status, &transmission, &current_positions);
        }
        if (!current_pose_executable) {
            ClearPositionControlBatchProgressLocked();
            HoldCurrentMotorPositionsOrStopLocked(feedback);
            throw JointException(JointError::kDoubleCoupleJointSolveFailed,
                                 "DoubleCoupleJoint current pose is not executable");
        }

        // 本轴命令：关节侧限幅后换算到求解器坐标系
        const auto params = AxisParams(axis);
        const double solver_command = DynamicAxisCommandInSolver(axis, mode, params, command);

        // 方向安全：近奇异时检查是否朝奇异区运动（near_singular 门控在函数内部，0 次 SolveInverse）
        if (IsCommandEnteringSingularity(current_pose, transmission, axis, solver_command)) {
            ClearPositionControlBatchProgressLocked();
            HoldMotorPositionsOrStopLocked(current_positions, feedback);
            return;
        }

        DynamicCommandSlotLocked(mode, axis) = AxisDynamicCommand{
            solver_command, ResolveMotorCurrent(current, params.maxTor_, motor1_kt_, "motor1"),
            ResolveMotorCurrent(current, params.maxTor_, motor2_kt_, "motor2")};
        if (ShouldDeferDynamicControlLocked(mode)) {
            RefreshTargetPoseFromFeedbackLocked(0, /*respect_batch_gate=*/false);
            return;
        }

        FlushDynamicControlLocked(mode, axis, current_pose, transmission, solver, current_positions,
                                  feedback);
    }

    /**
     * @brief 动态控制模式名称，用于保护日志。
     * @param mode 动态控制模式。
     * @return 模式名称字符串。
     */
    static const char* DynamicControlModeName(DynamicControlMode mode) {
        switch (mode) {
            case DynamicControlMode::Speed:
                return "SpdControl";
            case DynamicControlMode::Torque:
                return "TorControl";
            case DynamicControlMode::Current:
            default:
                return "CurControl";
        }
    }

    /**
     * @brief 动态控制越限置零时使用的保护原因。
     * @param mode 动态控制模式。
     * @return 保护原因字符串。
     */
    static const char* DynamicZeroReason(DynamicControlMode mode) {
        switch (mode) {
            case DynamicControlMode::Speed:
                return "joint/motor outward limit; zero speed";
            case DynamicControlMode::Torque:
                return "joint/motor outward limit; zero torque";
            case DynamicControlMode::Current:
            default:
                return "joint/motor outward limit; zero current";
        }
    }

    /**
     * @brief 把关节坐标系动态命令限幅并换算到求解器坐标系。
     *
     * 电流模式的关节扭矩限幅在 MapJointCurrent 内部处理，这里不做轴侧限幅。
     * @param axis 被控制的输出轴。
     * @param mode 动态控制模式。
     * @param params 该轴的运行参数。
     * @param command 关节坐标系目标。
     * @return 求解器坐标系下的轴命令。
     */
    double DynamicAxisCommandInSolver(DoubleCoupleJointAxis axis, DynamicControlMode mode,
                                      const AxisRuntimeParams& params, float command) {
        switch (mode) {
            case DynamicControlMode::Speed:
                return JointToSolver(axis,
                                     ClampControl(command, -params.maxSpd_, params.maxSpd_,
                                                  active_control_mode_, "speed", AxisScope(axis)));
            case DynamicControlMode::Torque:
                return JointToSolver(axis,
                                     ClampControl(command, -params.maxTor_, params.maxTor_,
                                                  active_control_mode_, "torque", AxisScope(axis)));
            case DynamicControlMode::Current:
            default:
                return JointToSolver(axis, command);
        }
    }

    /**
     * @brief 下发前按当前位姿复核单轴动态命令，越限或朝奇异区运动时把该轴命令置零。
     * @param mode 动态控制模式。
     * @param pose 下发时刻的目标位姿。
     * @param transmission 下发时刻的传动信息。
     * @param axis 被复核的输出轴。
     * @param value 求解器坐标系下的轴命令。
     * @return 复核通过时返回原值，被拦截时返回 0。
     */
    double SanitizeDynamicAxisCommandLocked(DynamicControlMode mode, const JointPose& pose,
                                            const couple_joint::TransmissionInfo& transmission,
                                            DoubleCoupleJointAxis axis, double value) {
        const char* scope = AxisScope(axis);
        if (value == 0.0) {
            // 命令为零：该轴既未被越限拦截，也不在奇异方向保护中
            LogProtection(active_control_mode_, DynamicZeroReason(mode), false, {}, scope);
            LogProtection(active_control_mode_, "singular direction; zero command", false, {},
                          scope);
            return 0.0;
        }
        const bool leaving_limit = AxisCommandLeavingLimit(axis, pose, value);
        LogProtection(active_control_mode_, DynamicZeroReason(mode), leaving_limit, {}, scope);
        if (leaving_limit) {
            // 越限拦截优先于奇异方向判断，本次不判奇异方向，按未触发处理
            LogProtection(active_control_mode_, "singular direction; zero command", false, {},
                          scope);
            return 0.0;
        }
        const bool entering_singularity =
            IsCommandEnteringSingularity(pose, transmission, axis, value);
        LogProtection(active_control_mode_, "singular direction; zero command",
                      entering_singularity, {}, scope);
        if (entering_singularity) {
            return 0.0;
        }
        return value;
    }

    /**
     * @brief 按模式对两轴轴命令做联合映射。
     * @param mode 动态控制模式。
     * @param solver 已位于目标位姿的求解器副本。
     * @param roll_command 求解器坐标系下的 roll 轴命令。
     * @param pitch_command 求解器坐标系下的 pitch 轴命令。
     * @param motor1_value 输出电机 1 命令。
     * @param motor2_value 输出电机 2 命令。
     * @return true 表示映射成功。
     */
    bool MapJointDynamicCommandLocked(DynamicControlMode mode,
                                      const couple_joint::MechanismSolver& solver,
                                      double roll_command, double pitch_command,
                                      float* motor1_value, float* motor2_value) {
        switch (mode) {
            case DynamicControlMode::Speed:
                return MapJointSpeed(solver, roll_command, pitch_command, motor1_value,
                                     motor2_value);
            case DynamicControlMode::Torque:
                return MapJointTorque(solver, roll_command, pitch_command, motor1_value,
                                      motor2_value);
            case DynamicControlMode::Current:
            default: {
                MotorCurrents currents{};
                if (!MapJointCurrent(solver, roll_command, pitch_command, &currents)) {
                    return false;
                }
                *motor1_value = currents.motor1;
                *motor2_value = currents.motor2;
                return true;
            }
        }
    }

    /**
     * @brief 按模式把联合映射结果下发给两台电机。
     * @param mode 动态控制模式。
     * @param motor1_value 电机 1 命令。
     * @param motor2_value 电机 2 命令。
     * @param motor1_current 电机 1 电流上限，仅速度模式使用。
     * @param motor2_current 电机 2 电流上限，仅速度模式使用。
     * @param feedback 底层电机反馈模式。
     */
    void SendMotorDynamicLocked(DynamicControlMode mode, float motor1_value, float motor2_value,
                                float motor1_current, float motor2_current, int feedback) {
        switch (mode) {
            case DynamicControlMode::Speed:
                motor1_->SpdControl<0>(motor1_value, motor1_current, feedback);
                motor2_->SpdControl<0>(motor2_value, motor2_current, feedback);
                break;
            case DynamicControlMode::Torque:
                motor1_->TorControl<0>(motor1_value, feedback);
                motor2_->TorControl<0>(motor2_value, feedback);
                break;
            case DynamicControlMode::Current:
            default:
                motor1_->CurControl<0>(motor1_value, feedback);
                motor2_->CurControl<0>(motor2_value, feedback);
                break;
        }
    }

    /**
     * @brief 动态类合帧下发：两条轴命令都到齐后一次下发两台电机。
     * @param mode 动态控制模式。
     * @param axis 本次调用的控制轴，即合帧的第二条命令。
     * @param current_pose 下发时刻的目标位姿。
     * @param transmission 下发时刻的传动信息，用于近奇异方向复核。
     * @param solver 已位于 current_pose 的求解器副本。
     * @param current_positions 下发时刻的双电机位置。
     * @param feedback 底层电机反馈模式，取完成调用的取值。
     * @note 电流上限以本轴命令（本次调用刚写入的槽位，必然有值）为基准，另一轴只有确实
     *       存在槽位时才参与收紧。从未被命令过的轴沿用默认 AxisDynamicCommand{}，其
     *       current 为 0，若一并取 min 会把整帧限流到 0 A、使关节静默失去力矩输出。
     */
    void FlushDynamicControlLocked(DynamicControlMode mode, DoubleCoupleJointAxis axis,
                                   const JointPose& current_pose,
                                   const couple_joint::TransmissionInfo& transmission,
                                   const couple_joint::MechanismSolver& solver,
                                   const MotorPositions& current_positions, int feedback) {
        const auto& roll_slot = DynamicCommandSlotLocked(mode, DoubleCoupleJointAxis::Roll);
        const auto& pitch_slot = DynamicCommandSlotLocked(mode, DoubleCoupleJointAxis::Pitch);
        const auto& self_slot = axis == DoubleCoupleJointAxis::Roll ? roll_slot : pitch_slot;
        const auto& other_slot = axis == DoubleCoupleJointAxis::Roll ? pitch_slot : roll_slot;

        // 本轴槽位由本次 ControlDynamic 调用写入；另一轴的本批次值与电流上限一并沿用上次命令
        float motor1_current = self_slot->current1;
        float motor2_current = self_slot->current2;
        if (other_slot) {
            // 两台电机各取两条调用中较小的电流上限
            motor1_current = std::min(motor1_current, other_slot->current1);
            motor2_current = std::min(motor2_current, other_slot->current2);
        }

        const double roll_command = SanitizeDynamicAxisCommandLocked(
            mode, current_pose, transmission, DoubleCoupleJointAxis::Roll,
            roll_slot.value_or(AxisDynamicCommand{}).value);
        const double pitch_command = SanitizeDynamicAxisCommandLocked(
            mode, current_pose, transmission, DoubleCoupleJointAxis::Pitch,
            pitch_slot.value_or(AxisDynamicCommand{}).value);

        float motor1_value = 0.0f;
        float motor2_value = 0.0f;
        if (!MapJointDynamicCommandLocked(mode, solver, roll_command, pitch_command, &motor1_value,
                                          &motor2_value)) {
            WarnProtection(active_control_mode_, "inverse dynamic mapping failed");
            ClearPositionControlBatchProgressLocked();
            HoldMotorPositionsOrStopLocked(current_positions, feedback);
            throw JointException(JointError::kDoubleCoupleJointSolveFailed,
                                 "DoubleCoupleJoint inverse dynamic mapping failed");
        }
        const bool motor_leaving_limit =
            MotorCommandsLeavingLimit(current_positions, motor1_value, motor2_value);
        LogProtection(active_control_mode_, DynamicZeroReason(mode), motor_leaving_limit);
        if (motor_leaving_limit) {
            motor1_value = 0.0f;
            motor2_value = 0.0f;
        }
        SendMotorDynamicLocked(mode, motor1_value, motor2_value, motor1_current, motor2_current,
                               feedback);
        FinishPositionControlBatchLocked();
        ClearHoldProtectionLocked();
        RefreshTargetPoseFromFeedbackLocked(0, /*respect_batch_gate=*/false);
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
        if (!waiting_for_post_zero_feedback_ && !zero_reset_faulted_ &&
            std::isfinite(motor1_status.position) && std::isfinite(motor2_status.position)) {
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
     * @brief 按用户回调或内部实际位姿跟踪需求安装底层电机状态回调。
     * @note 调用方必须已持有 mutex_。
     */
    void UpdateMotorStatusCallbacksLocked() {
        const bool callbacks_required =
            roll_status_callback_ || pitch_status_callback_ || track_actual_feedback_;
        if (callbacks_required && !status_callbacks_installed_) {
            const std::weak_ptr<MasterJoint> self = weak_from_this();
            motor1_->SetOnStatus([self](const MotorStatus& status) {
                if (auto joint = self.lock()) {
                    const auto feedback_epoch =
                        joint->feedback_epoch_.load(std::memory_order_acquire);
                    joint->OnMotorStatus(1, status, feedback_epoch);
                }
            });
            motor2_->SetOnStatus([self](const MotorStatus& status) {
                if (auto joint = self.lock()) {
                    const auto feedback_epoch =
                        joint->feedback_epoch_.load(std::memory_order_acquire);
                    joint->OnMotorStatus(2, status, feedback_epoch);
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
     * @param feedback_epoch 回调进入时采样的零位坐标系代次。
     */
    void OnMotorStatus(int motor_index, const MotorStatus& status, std::uint64_t feedback_epoch) {
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
            if (waiting_for_post_zero_feedback_) {
                const auto current_epoch = feedback_epoch_.load(std::memory_order_acquire);
                if (zero_reset_in_progress_ || zero_reset_faulted_ ||
                    feedback_epoch != current_epoch || (feedback_epoch % 2U) != 0U) {
                    return;
                }
                if (motor_index == 1) {
                    motor1_feedback_after_zero_ = true;
                } else {
                    motor2_feedback_after_zero_ = true;
                }
                if (!motor1_feedback_after_zero_ || !motor2_feedback_after_zero_) {
                    return;
                }
                waiting_for_post_zero_feedback_ = false;
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
    bool track_actual_feedback_{false};
    bool waiting_for_post_zero_feedback_{false};
    bool motor1_feedback_after_zero_{false};
    bool motor2_feedback_after_zero_{false};
    bool zero_reset_in_progress_{false};
    bool zero_reset_faulted_{false};
    std::atomic<std::uint64_t> feedback_epoch_{0};
    couple_joint::MechanismParameters solver_parameters_;
    MotorPVTRanges motor1_ranges_{};
    MotorPVTRanges motor2_ranges_{};
    Range<float> motor1_position_limit_{0.0f, 0.0f};
    Range<float> motor2_position_limit_{0.0f, 0.0f};
    float motor1_kt_{1.0f};
    float motor2_kt_{1.0f};
    AxisPVTCommand roll_pvt_command_{};
    AxisPVTCommand pitch_pvt_command_{};
    std::optional<AxisPositionCommand> roll_position_command_;
    std::optional<AxisPositionCommand> pitch_position_command_;
    CoupledBatchClass batch_class_{CoupledBatchClass::None};
    std::optional<DynamicControlMode> dynamic_batch_mode_;
    std::optional<AxisDynamicCommand> roll_speed_command_;
    std::optional<AxisDynamicCommand> pitch_speed_command_;
    std::optional<AxisDynamicCommand> roll_torque_command_;
    std::optional<AxisDynamicCommand> pitch_torque_command_;
    std::optional<AxisDynamicCommand> roll_current_command_;
    std::optional<AxisDynamicCommand> pitch_current_command_;
    double target_roll_{0.0};
    double target_pitch_{0.0};
    std::size_t position_control_count_{0};
    std::optional<JointPose> position_batch_start_pose_;
    std::optional<couple_joint::MechanismSolver> position_batch_start_solver_;
    std::optional<MotorPositions> position_batch_start_positions_;
    std::optional<DoubleCoupleJointAxis> position_batch_hold_axis_;
    std::optional<PVTControlMode> pvt_batch_torque_mode_;
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
