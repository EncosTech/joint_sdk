// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>

#include "BaseJoint.h"
#include "JsonSchemaGenerator.h"

namespace encos {

template <typename... JointTypes>
class JointManager;

/**
 * @brief 旋转关节实现类
 *
 * 基于Motor的旋转关节控制类，支持位置、速度、电流、扭矩控制，
 * 并提供限位保护和方向反转功能
 */
class RJoint : public BaseJoint {
    friend class JointManager<>;

public:
    /**
     * @brief RJoint配置结构体
     *
     * 使用GENERATE_CONFIG宏生成字段声明、JSON Schema和反序列化函数
     */
    struct Config {
        // clang-format off
        GENERATE_CONFIG(RJoint,
                        ((motorParams, MotorParams, Complex, true, "Motor initialization parameters"))
                        ((limit, Range<float>, Range, true, "Position limit range"))
                        ((maxSpd, float, Float, true, "Maximum speed"))
                        ((maxTor, float, Float, true, "Maximum torque"))
                        ((reverse, bool, Bool, true, "Reverse direction")))
        // clang-format on
    };

    /**
     * @brief 由 Config 和预创建 Motor 构造
     *
     * @param config 关节配置
     * @param motor 底层电机实例
     */
    explicit RJoint(const Config& config, Motor* motor)
        : BaseJoint{config.limit, config.maxSpd, config.maxTor, config.reverse} {
        if (!motor) {
            throw std::invalid_argument("RJoint motor must not be null");
        }
        motor_ = motor;
        const auto ranges = motor_->GetPVTRanges();
        if (!std::isfinite(ranges.kt) || ranges.kt <= 0.0f) {
            throw std::invalid_argument("RJoint motor kt must be positive");
        }
        maxCur_ = config.maxTor / ranges.kt;
    }

    /**
     * @brief 设置旋转关节的软件位置限位。
     * @param limit 关节坐标系下的新限位范围。
     */
    void SetLimit(Range<float> limit) override {
        SetValidatedLimit(limit);
    }

    /**
     * @brief 执行旋转关节 PVT 控制；目标位置不限幅，实际位置进入保护区后拦截向外命令。
     * @param kp 位置比例增益。
     * @param kd 速度微分增益。
     * @param pos 目标位置，单位 rad，关节坐标系。
     * @param spd 目标速度，单位 rad/s，关节坐标系。
     * @param torque 目标扭矩，单位 Nm，关节坐标系。
     * @throw std::invalid_argument 当任一控制输入不是有限值时抛出。
     */
    void PVTControl(float kp, float kd, float pos, float spd, float torque) override {
        RejectNonFiniteControlInput("PVT", kp, kd, pos, spd, torque);
        spd = ClampControl(spd, -maxSpd_, maxSpd_, "PVT", "speed");
        torque = ClampControl(torque, -maxTor_, maxTor_, "PVT", "torque");
        ApplyReverse(pos, spd, torque);

        const float motor_pos = ReadControlMotorPosition(1);
        const bool feedback_available = std::isfinite(motor_pos);
        LogProtection("PVT", "position feedback unavailable; FullBrake", !feedback_available);
        const bool hold_at_limit =
            feedback_available && ShouldHoldPVTAtLimit(motor_pos, pos, spd, torque);
        LogProtection("PVT", "outward position limit; position hold", hold_at_limit);
        if (!feedback_available) {
            Stop(MotorStopMode::FullBrake, 0.0f, 1);
            return;
        }
        if (hold_at_limit) {
            SendLimitHold(motor_pos, 1);
            return;
        }
        const auto ranges = motor_->GetPVTRanges();
        motor_->PVTControl<0>(
            ClampControl(kp, ranges.kp.min, ranges.kp.max, "PVT", "motor kp"),
            ClampControl(kd, ranges.kd.min, ranges.kd.max, "PVT", "motor kd"),
            ClampControl(pos, ranges.position.min, ranges.position.max, "PVT", "motor position"),
            ClampControl(spd, ranges.speed.min, ranges.speed.max, "PVT", "motor speed"),
            ClampControl(torque, ranges.torque.min, ranges.torque.max, "PVT", "motor torque"));
    }

    /**
     * @brief 执行旋转关节位置控制。
     * @param pos 目标位置，单位 rad，关节坐标系。
     * @param spd 速度上限，单位 rad/s；<=0 时使用关节最大速度。
     * @param current 电流上限，单位 A；<=0 时按最大扭矩和 kt 计算。
     * @param feedback 底层电机反馈模式。
     * @throw std::invalid_argument 当任一控制输入不是有限值时抛出。
     */
    void PosControl(float pos, float spd = 0, float current = 0, int feedback = 1) override {
        RejectNonFiniteControlInput("PosControl", pos, spd, current);
        if (reverse_)
            pos = -pos;
        LogProtection("PosControl", "speed limit", spd > maxSpd_);
        LogProtection("PosControl", "current limit", current > maxCur_);
        if (spd <= 0 || spd > maxSpd_)
            spd = maxSpd_;
        if (current <= 0 || current > maxCur_)
            current = maxCur_;

        ClampPosition(pos);
        motor_->PosControl<0>(pos, spd, current, feedback);
    }

    /**
     * @brief 执行旋转关节速度控制。
     * @param spd 目标速度，单位 rad/s，关节坐标系。
     * @param current 电流上限，单位 A；<=0 时按最大扭矩和 kt 计算。
     * @param feedback 底层电机反馈模式。
     * @throw std::invalid_argument 当任一控制输入不是有限值时抛出。
     */
    void SpdControl(float spd, float current = 0, int feedback = 1) override {
        RejectNonFiniteControlInput("SpdControl", spd, current);
        if (reverse_)
            spd = -spd;
        LogProtection("SpdControl", "current limit", current > maxCur_);
        if (current <= 0 || current > maxCur_)
            current = maxCur_;
        spd = ClampControl(spd, -maxSpd_, maxSpd_, "SpdControl", "speed");

        const float motor_pos = ReadControlMotorPosition(feedback);
        const bool hold_at_limit =
            !std::isnan(motor_pos) && ShouldHoldDynamicAtLimit(motor_pos, spd);
        LogProtection("SpdControl", "outward position limit; position hold", hold_at_limit);
        if (hold_at_limit) {
            SendLimitHold(motor_pos, feedback);
            return;
        }
        motor_->SpdControl<0>(spd, current, feedback);
    }

    /**
     * @brief 执行旋转关节电流控制。
     * @param current 目标电流，单位 A，正负方向按关节坐标系解释。
     * @param feedback 底层电机反馈模式。
     * @throw std::invalid_argument 当 current 不是有限值时抛出。
     */
    void CurControl(float current = 0, int feedback = 1) override {
        RejectNonFiniteControlInput("CurControl", current);
        if (reverse_)
            current = -current;
        current = ClampControl(current, -maxCur_, maxCur_, "CurControl", "current");

        const float motor_pos = ReadControlMotorPosition(feedback);
        const bool hold_at_limit =
            !std::isnan(motor_pos) && ShouldHoldDynamicAtLimit(motor_pos, current);
        LogProtection("CurControl", "outward position limit; position hold", hold_at_limit);
        if (hold_at_limit) {
            SendLimitHold(motor_pos, feedback);
            return;
        }
        motor_->CurControl<0>(current, feedback);
    }

    /**
     * @brief 执行旋转关节扭矩控制。
     * @param torque 目标扭矩，单位 Nm，关节坐标系。
     * @param feedback 底层电机反馈模式。
     * @throw std::invalid_argument 当 torque 不是有限值时抛出。
     */
    void TorControl(float torque = 0, int feedback = 1) override {
        RejectNonFiniteControlInput("TorControl", torque);
        if (reverse_)
            torque = -torque;
        torque = ClampControl(torque, -maxTor_, maxTor_, "TorControl", "torque");

        const float motor_pos = ReadControlMotorPosition(feedback);
        const bool hold_at_limit =
            !std::isnan(motor_pos) && ShouldHoldDynamicAtLimit(motor_pos, torque);
        LogProtection("TorControl", "outward position limit; position hold", hold_at_limit);
        if (hold_at_limit) {
            SendLimitHold(motor_pos, feedback);
            return;
        }
        motor_->TorControl<0>(torque, feedback);
    }

    /**
     * @brief 向底层电机发送停止命令。
     * @param mode 停止模式。
     * @param current 停止电流，非有限值会被归零。
     * @param feedback 底层电机反馈模式。
     */
    void Stop(MotorStopMode mode, float current = 0, int feedback = 1) override {
        const bool finite_current = std::isfinite(current);
        LogProtection("Stop", "non-finite current; using zero", !finite_current);
        if (!finite_current) {
            current = 0.0f;
        }
        motor_->Stop<0>(mode, current, feedback);
    }

    /**
     * @brief 将当前电机位置重置为零点。
     * @return 底层电机 ResetZeroPos() 的执行结果。
     */
    bool SetZero() override {
        return motor_->ResetZeroPos();
    }

    /**
     * @brief 驱动旋转关节搜索指定方向的限位。
     * @param dir 搜索方向，关节坐标系；正数表示正向，负数表示负向。
     * @param spd 搜索速度，单位 rad/s。
     * @param cur 搜索电流上限，单位 A。
     * @param timeout 搜索超时时间。
     * @return 零点偏移量，单位 rad，关节坐标系；失败时返回 NaN。
     */
    float GotoLimit(int dir, float spd = 2.f, float cur = 2.f,
                    std::chrono::milliseconds timeout = std::chrono::milliseconds{5000}) override {
        if (reverse_)
            dir = -dir;
        float offset = motor_->GotoLimit(limit_, dir, spd, cur, timeout);
        if (reverse_)
            offset = -offset;
        return offset;
    }

    /**
     * @brief 按偏移量驱动关节回到零点。
     * @param offset 零点偏移量，单位 rad，关节坐标系。
     * @param spd 回零速度，单位 rad/s。
     * @param cur 回零电流上限，单位 A。
     * @param timeout 回零超时时间。
     * @return true 表示底层电机回零成功。
     */
    bool GotoZero(float offset, float spd = 2.f, float cur = 2.f,
                  std::chrono::milliseconds timeout = std::chrono::milliseconds{5000}) override {
        if (reverse_)
            offset = -offset;
        return motor_->GotoZero(offset, spd, cur, timeout);
    }

    /**
     * @brief 将当前位置换算成指定限位方向的零点偏移量。
     * @param dir 限位方向，关节坐标系；正数使用 max，负数使用 min。
     * @return 当前位置相对对应限位的偏移量，单位 rad；位置不可用时返回 NaN。
     */
    float PosAsOffset(int dir) override {
        if (reverse_)
            dir = -dir;
        float motorPos = ReadMotorPosition(false);
        if (std::isnan(motorPos))
            return std::nanf("");
        float limitPos = (dir > 0 ? limit_.max : limit_.min);
        float offset = motorPos - limitPos;
        if (reverse_)
            offset = -offset;
        return offset;
    }

    /**
     * @brief 执行旋转关节限位校准流程。
     * @param dir 校准方向，关节坐标系。
     * @param spd 校准速度，单位 rad/s。
     * @param cur 校准电流上限，单位 A。
     * @param timeout 限位搜索和回零各自使用的超时时间。
     * @return true 表示限位搜索和回零均成功。
     */
    bool Calibrate(int dir, float spd = 2.f, float cur = 2.f,
                   std::chrono::milliseconds timeout = std::chrono::milliseconds{5000}) override {
        float offset = GotoLimit(dir, spd, cur, timeout);
        if (std::isnan(offset))
            return false;
        return GotoZero(offset, spd, cur, timeout);
    }

    /**
     * @brief 读取旋转关节状态。
     * @param autoSendCommand 为 true 且缓存状态不可用时，发送一次 PVT 查询命令。
     * @param lifeCycleDeduction 读取状态时扣减的生命周期值。
     * @return 关节坐标系下的状态；没有可用电机状态时返回 std::nullopt。
     * @warning 关节运动过程中不得将 autoSendCommand 设为 true；内部零目标 PVT 反馈命令
     *          可能改变关节运动状态。
     */
    std::optional<JointStatus> GetStatus(bool autoSendCommand = false,
                                         int lifeCycleDeduction = 1) override {
        auto status = motor_->GetStatus(lifeCycleDeduction);
        if (!status) {
            if (autoSendCommand) {
                motor_->PVTControl<0>(0, 1, 0, 0, 0);
            }
            return std::nullopt;
        }
        const float kt = motor_->GetPVTRanges().kt;
        float dir = reverse_ ? -1.f : 1.f;
        JointStatus joint_status{};
        joint_status.pos = dir * status->position;
        joint_status.spd = dir * status->speed;
        joint_status.tor = dir * status->current * kt;
        joint_status.mosTemperature = status->mos_temperature;
        joint_status.motorTemperature = status->motor_temperature;
        joint_status.error = status->error;
        return joint_status;
    }

    /**
     * @brief 注册或取消旋转关节状态回调。
     * @param onStatus 状态回调；传入 nullptr 时取消底层电机回调。
     */
    void SetOnStatus(std::function<void(const JointStatus&)> onStatus) override {
        if (!onStatus) {
            motor_->SetOnStatus(nullptr);
            return;
        }
        const float kt = motor_->GetPVTRanges().kt;
        const bool rev = reverse_;
        motor_->SetOnStatus([onStatus = std::move(onStatus), kt, rev](const MotorStatus& s) {
            JointStatus joint_status{};
            joint_status.pos = rev ? -s.position : s.position;
            joint_status.spd = rev ? -s.speed : s.speed;
            joint_status.tor = rev ? -s.current * kt : s.current * kt;
            joint_status.mosTemperature = s.mos_temperature;
            joint_status.motorTemperature = s.motor_temperature;
            joint_status.error = s.error;
            onStatus(joint_status);
        });
    }

    /**
     * @brief 读取底层电机原始位置。
     * @param read_cache true 时优先读取缓存状态；false 时直接读取电机参数。
     * @param life_cycle_deduction 读取缓存状态时扣减的生命周期值。
     * @return 电机坐标系下的位置，单位 rad。
     */
    float GetMotorPosition(bool read_cache = true, int life_cycle_deduction = 0) {
        return ReadMotorPosition(read_cache, life_cycle_deduction);
    }

    /**
     * @brief 获取反转后的当前位置（关节坐标系）
     * @return 关节坐标系的位置
     */
    float GetReversedPosition() {
        float pos = ReadMotorPosition();
        return reverse_ ? -pos : pos;
    }

    /**
     * @brief 获取电机扭矩常数。
     * @return 扭矩常数，单位 Nm/A。
     */
    float GetTorqueConstant() const {
        return motor_->GetPVTRanges().kt;
    }

    /**
     * @brief 获取关节允许的最大电流。
     * @return 最大电流绝对值，单位 A。
     */
    float GetMaxCurrent() const {
        return maxCur_;
    }

    /**
     * @brief 获取关节允许的最大扭矩。
     * @return 最大扭矩绝对值，单位 Nm。
     */
    float GetMaxTorque() const {
        return maxTor_;
    }

    /**
     * @brief 获取关节允许的最大速度。
     * @return 最大速度绝对值，单位 rad/s。
     */
    float GetMaxSpeed() const {
        return maxSpd_;
    }

    /**
     * @brief 查询关节方向是否反转。
     * @return reverse 配置值。
     */
    bool IsReversed() const {
        return reverse_;
    }

    /**
     * @brief 获取关节坐标系下的软件限位。
     * @return 关节坐标系限位，单位 rad。
     */
    Range<float> GetJointLimit() const {
        return reverse_ ? Range<float>{-limit_.max, -limit_.min} : limit_;
    }

protected:
    Motor* motor_{nullptr};  ///< Bus-owned, non-owning motor instance
    float maxCur_;           ///< 最大电流

    /**
     * @brief 验证控制输入均为有限值，失败时先停止电机。
     * @tparam Values 控制输入参数类型。
     * @param values 需要检查的控制输入。
     * @throw std::invalid_argument 当任一输入不是有限值时抛出。
     */
    template <typename... Values>
    void RejectNonFiniteControlInput(const char* mode, Values... values) {
        if ((std::isfinite(values) && ...)) {
            return;
        }
        WarnProtection(mode, "non-finite input; FullBrake");
        Stop(MotorStopMode::FullBrake);
        throw std::invalid_argument("RJoint control input must be finite");
    }

    /**
     * @brief 应用反转方向
     * @param pos 位置（将被修改）
     * @param spd 速度（将被修改）
     * @param torque 扭矩（将被修改）
     */
    void ApplyReverse(float& pos, float& spd, float& torque) {
        if (reverse_) {
            pos = -pos;
            spd = -spd;
            torque = -torque;
        }
    }

    /**
     * @brief 将位置控制目标钳制到带安全余量的软件限位内。
     */
    void ClampPosition(float& pos) {
        pos = ClampControl(pos, limit_.min + kLimitSlack, limit_.max - kLimitSlack, "PosControl",
                           "position");
    }

    bool ShouldHoldPVTAtLimit(float current_pos, float target_pos, float target_spd,
                              float target_effort) const {
        const float lower = limit_.min + kLimitSlack;
        const float upper = limit_.max - kLimitSlack;
        if (current_pos <= lower) {
            return target_pos < current_pos || target_spd < 0.0f || target_effort < 0.0f;
        }
        if (current_pos >= upper) {
            return target_pos > current_pos || target_spd > 0.0f || target_effort > 0.0f;
        }
        return false;
    }

    bool ShouldHoldDynamicAtLimit(float current_pos, float command) const {
        const float lower = limit_.min + kLimitSlack;
        const float upper = limit_.max - kLimitSlack;
        if (current_pos <= lower) {
            return command <= 0.0f;
        }
        if (current_pos >= upper) {
            return command >= 0.0f;
        }
        return false;
    }

    /// @brief 按当前位置发位置保持命令；限位保护日志点由调用方记录
    void SendLimitHold(float current_pos, int feedback) {
        PosControl(reverse_ ? -current_pos : current_pos, maxSpd_, maxCur_, feedback);
    }

    /** @brief 控制前读取位置；反馈异常时先尝试制动再传播原始异常。 */
    float ReadControlMotorPosition(int feedback) {
        try {
            return ReadMotorPosition();
        } catch (...) {
            WarnProtection("feedback", "position read failed; FullBrake");
            try {
                Stop(MotorStopMode::FullBrake, 0.0f, feedback);
            } catch (...) {
                // Preserve the original feedback error even if transport also fails to stop.
            }
            throw;
        }
    }

    /**
     * @brief 读取底层电机位置。
     * @param read_cache true 时优先读取电机状态缓存，false 时直接读取电机参数。
     * @param life_cycle_deduction 读取状态缓存时扣减的生命周期值。
     * @return 电机坐标系下的位置，单位 rad。
     */
    float ReadMotorPosition(bool read_cache = true, int life_cycle_deduction = 0) {
        if (!read_cache) {
            return motor_->GetParameter<MotorParameter::Position>();
        }
        auto status = motor_->GetStatus(life_cycle_deduction);
        if (!status || std::isnan(status->position)) {
            return motor_->GetParameter<MotorParameter::Position>();
        }
        return status->position;
    }
};

}  // namespace encos
