// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

#pragma once

#include <algorithm>
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

#include "BaseJoint.h"
#include "JsonSchemaGenerator.h"

namespace encos {

template <typename... JointTypes>
class JointManager;

/**
 * @brief 可无限旋转的单电机连续关节。
 *
 * ContinuousJoint 直接继承 BaseJoint，不具备软件位置限位，也不提供位置控制、
 * PVT 控制或限位校准操作。速度、电流和扭矩控制仍执行幅值保护；状态位置会依据
 * 电机 PVT 位置范围展开回绕，从而跨越反馈上下限后保持连续。
 */
class ContinuousJoint : public BaseJoint {
    friend class JointManager<>;

public:
    /**
     * @brief ContinuousJoint 配置结构体（不包含位置限位）。
     */
    struct Config {
        // clang-format off
        GENERATE_CONFIG(ContinuousJoint,
                        ((motorParams, MotorParams, Complex, true, "Motor initialization parameters"))
                        ((maxSpd, float, Float, true, "Maximum speed"))
                        ((maxTor, float, Float, true, "Maximum torque"))
                        ((reverse, bool, Bool, true, "Reverse direction")))
        // clang-format on
    };

    /**
     * @brief 由配置和预创建 Motor 构造连续关节。
     * @param config 关节配置。
     * @param motor 底层电机实例。
     */
    explicit ContinuousJoint(const Config& config, Motor* motor)
        : BaseJoint{config.maxSpd, config.maxTor, config.reverse}, motor_{motor} {
        if (!motor_) {
            throw std::invalid_argument("ContinuousJoint motor must not be null");
        }
        const auto ranges = motor_->GetPVTRanges();
        if (!std::isfinite(ranges.kt) || ranges.kt <= 0.0f) {
            throw std::invalid_argument("ContinuousJoint motor kt must be positive");
        }
        if (!std::isfinite(ranges.position.min) || !std::isfinite(ranges.position.max) ||
            ranges.position.min >= ranges.position.max) {
            throw std::invalid_argument(
                "ContinuousJoint feedback position range must be finite and increasing");
        }
        const double max_current = static_cast<double>(config.maxTor) / ranges.kt;
        if (!std::isfinite(max_current) || max_current > std::numeric_limits<float>::max()) {
            throw std::invalid_argument("ContinuousJoint maximum current must be finite");
        }
        maxCur_ = static_cast<float>(max_current);
        position_state_ = std::make_shared<PositionState>(static_cast<double>(ranges.position.max) -
                                                          ranges.position.min);
        InstallStatusDispatcher(ranges.kt);
    }

    ~ContinuousJoint() override {
        motor_->SetOnStatus(nullptr);
    }

    void PVTControl([[maybe_unused]] float kp, [[maybe_unused]] float kd,
                    [[maybe_unused]] float pos, [[maybe_unused]] float spd,
                    [[maybe_unused]] float torque) override {
        ThrowUnsupported("PVTControl");
    }

    void PVTControl([[maybe_unused]] float kp, [[maybe_unused]] float kd,
                    [[maybe_unused]] float pos, [[maybe_unused]] float spd,
                    [[maybe_unused]] float torque,
                    [[maybe_unused]] PVTControlMode torque_mode) override {
        ThrowUnsupported("PVTControl");
    }

    void PosControl([[maybe_unused]] float position, [[maybe_unused]] float speed = 0,
                    [[maybe_unused]] float current = 0,
                    [[maybe_unused]] int feedback = 1) override {
        ThrowUnsupported("PosControl");
    }

    /**
     * @brief 执行不带位置限位保护的速度控制。
     */
    void SpdControl(float speed, float current = 0, int feedback = 1) override {
        RejectNonFiniteControlInput("SpdControl", speed, current);
        if (reverse_) {
            speed = -speed;
        }
        LogProtection("SpdControl", "current limit", current > maxCur_);
        if (current <= 0.0f || current > maxCur_) {
            current = maxCur_;
        }
        speed = ClampControl(speed, -maxSpd_, maxSpd_, "SpdControl", "speed");
        motor_->SpdControl<0>(speed, current, feedback);
    }

    /**
     * @brief 执行不带位置限位保护的电流控制。
     */
    void CurControl(float current = 0, int feedback = 1) override {
        RejectNonFiniteControlInput("CurControl", current);
        if (reverse_) {
            current = -current;
        }
        current = ClampControl(current, -maxCur_, maxCur_, "CurControl", "current");
        motor_->CurControl<0>(current, feedback);
    }

    /**
     * @brief 执行不带位置限位保护的扭矩控制。
     */
    void TorControl(float torque = 0, int feedback = 1) override {
        RejectNonFiniteControlInput("TorControl", torque);
        if (reverse_) {
            torque = -torque;
        }
        torque = ClampControl(torque, -maxTor_, maxTor_, "TorControl", "torque");
        motor_->TorControl<0>(torque, feedback);
    }

    void Stop(MotorStopMode mode, float current = 0, int feedback = 1) override {
        const bool finite_current = std::isfinite(current);
        LogProtection("Stop", "non-finite current; using zero", !finite_current);
        if (!finite_current) {
            current = 0.0f;
        }
        motor_->Stop<0>(mode, current, feedback);
    }

    bool SetZero() override {
        const bool result = motor_->ResetZeroPos();
        if (result) {
            ResetPositionState();
        }
        return result;
    }

    void SetLimit([[maybe_unused]] Range<float> limit) override {
        ThrowUnsupported("SetLimit");
    }

    float GotoLimit([[maybe_unused]] int dir, [[maybe_unused]] float spd = 2.0f,
                    [[maybe_unused]] float cur = 2.0f,
                    [[maybe_unused]] std::chrono::milliseconds timeout = std::chrono::milliseconds{
                        5000}) override {
        ThrowUnsupported("GotoLimit");
    }

    bool GotoZero([[maybe_unused]] float offset, [[maybe_unused]] float spd = 2.0f,
                  [[maybe_unused]] float cur = 2.0f,
                  [[maybe_unused]] std::chrono::milliseconds timeout = std::chrono::milliseconds{
                      5000}) override {
        ThrowUnsupported("GotoZero");
    }

    float PosAsOffset([[maybe_unused]] int dir) override {
        ThrowUnsupported("PosAsOffset");
    }

    bool Calibrate([[maybe_unused]] int dir, [[maybe_unused]] float spd = 2.0f,
                   [[maybe_unused]] float cur = 2.0f,
                   [[maybe_unused]] std::chrono::milliseconds timeout = std::chrono::milliseconds{
                       5000}) override {
        ThrowUnsupported("Calibrate");
    }

    /**
     * @brief 获取连续关节状态。
     * @param autoSendCommand 缓存状态不可用时，是否下发内部 PVT 反馈命令。
     * @param lifeCycleDeduction 本次读取扣减的生命周期值。
     * @return 位置已展开为连续值的关节状态；状态不可用时返回 std::nullopt。
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

        std::optional<JointStatus> joint_status;
        {
            std::lock_guard<std::mutex> lock(position_state_->mutex);
            if (!position_state_->awaiting_fresh_feedback) {
                if (position_state_->latest_status) {
                    joint_status = position_state_->latest_status;
                } else {
                    joint_status = MakeJointStatusLocked(*status, *position_state_,
                                                         motor_->GetPVTRanges().kt, reverse_);
                    position_state_->latest_status = joint_status;
                }
            }
        }
        if (!joint_status && autoSendCommand) {
            motor_->PVTControl<0>(0, 1, 0, 0, 0);
        }
        return joint_status;
    }

    void SetOnStatus(std::function<void(const JointStatus&)> onStatus) override {
        std::lock_guard<std::mutex> lock(position_state_->mutex);
        position_state_->on_status = std::move(onStatus);
    }

private:
    struct PositionState {
        explicit PositionState(double feedback_period) : period{feedback_period} {}

        std::mutex mutex;
        double period;
        double last_raw{0.0};
        double continuous{0.0};
        bool initialized{false};
        bool awaiting_fresh_feedback{false};
        std::optional<JointStatus> latest_status;
        std::function<void(const JointStatus&)> on_status;
    };

    [[noreturn]] void ThrowUnsupported(const char* operation) const {
        WarnProtection(operation, "unsupported operation");
        throw JointException(JointError::kContinuousJointUnsupportedOperation,
                             std::string{"ContinuousJoint does not support "} + operation);
    }

    template <typename... Values>
    void RejectNonFiniteControlInput(const char* mode, Values... values) {
        if ((std::isfinite(values) && ...)) {
            return;
        }
        WarnProtection(mode, "non-finite input; FullBrake");
        Stop(MotorStopMode::FullBrake);
        throw std::invalid_argument("ContinuousJoint control input must be finite");
    }

    static JointStatus MakeJointStatusLocked(const MotorStatus& status, PositionState& state,
                                             float kt, bool reverse) {
        const float direction = reverse ? -1.0f : 1.0f;
        JointStatus joint_status{};
        joint_status.pos = UnwrapPositionLocked(state, direction * status.position);
        joint_status.spd = direction * status.speed;
        joint_status.tor = direction * status.current * kt;
        joint_status.mosTemperature = status.mos_temperature;
        joint_status.motorTemperature = status.motor_temperature;
        joint_status.error = status.error;
        return joint_status;
    }

    static float UnwrapPositionLocked(PositionState& state, float raw_position) {
        if (!std::isfinite(raw_position)) {
            return raw_position;
        }
        const double raw = raw_position;
        if (!state.initialized) {
            state.last_raw = raw;
            state.continuous = raw;
            state.initialized = true;
            return raw_position;
        }

        double delta = raw - state.last_raw;
        const double half_period = state.period / 2.0;
        if (delta > half_period) {
            delta -= state.period;
        } else if (delta < -half_period) {
            delta += state.period;
        }
        state.continuous += delta;
        state.last_raw = raw;
        return static_cast<float>(state.continuous);
    }

    void InstallStatusDispatcher(float kt) {
        auto state = position_state_;
        const bool reverse = reverse_;
        motor_->SetOnStatus([state = std::move(state), kt, reverse](const MotorStatus& status) {
            JointStatus joint_status{};
            std::function<void(const JointStatus&)> on_status;
            {
                std::lock_guard<std::mutex> lock(state->mutex);
                state->awaiting_fresh_feedback = false;
                joint_status = MakeJointStatusLocked(status, *state, kt, reverse);
                state->latest_status = joint_status;
                on_status = state->on_status;
            }
            if (on_status) {
                on_status(joint_status);
            }
        });
    }

    void ResetPositionState() {
        std::lock_guard<std::mutex> lock(position_state_->mutex);
        position_state_->last_raw = 0.0;
        position_state_->continuous = 0.0;
        position_state_->initialized = false;
        position_state_->awaiting_fresh_feedback = true;
        position_state_->latest_status.reset();
    }

    Motor* motor_{nullptr};  ///< Bus-owned, non-owning motor instance
    float maxCur_{0.0f};     ///< 最大电流，由 maxTor / kt 计算
    std::shared_ptr<PositionState> position_state_;
};

}  // namespace encos
