// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <variant>

#include "BaseJoint.h"
#include "RJoint/RJoint.h"
#include "RJoint/StribeckModel.h"

namespace encos {

/**
 * @brief 为 RJoint 提供 Stribeck 零阻尼补偿的主动控制包装层。
 */
class ZeroDampingRJoint final : public BaseJoint {
private:
    struct StatusCallbackState {
        std::mutex mutex;
        std::function<void(const JointStatus&)> callback;
        std::atomic<float> retained_compensation{0.0f};
        std::atomic<float> retained_friction_compensation{0.0f};
        std::atomic<float> retained_inertia_compensation{0.0f};
    };

public:
    /**
     * @brief 创建零阻尼包装层并启动 1 kHz 软实时线程。
     * @param joint 被包装的 RJoint。
     * @param config Stribeck 控制参数。
     */
    ZeroDampingRJoint(std::shared_ptr<RJoint> joint, StribeckConfig config)
        : BaseJoint{RequireJoint(joint).GetJointLimit(), RequireJoint(joint).GetMaxSpeed(),
                    RequireJoint(joint).GetMaxTorque(), RequireJoint(joint).IsReversed()},
          joint_{std::move(joint)},
          model_{config},
          inertia_model_{config.inertia, config.inertia_compensation_ratio,
                         config.max_inertia_compensation_torque, config.velocity_filter_cutoff},
          max_compensation_torque_{config.max_compensation_torque},
          worker_{[this] {
              RunWorker();
          }} {}

    ZeroDampingRJoint(const ZeroDampingRJoint&) = delete;
    ZeroDampingRJoint& operator=(const ZeroDampingRJoint&) = delete;

    /**
     * @brief 停止线程并对底层关节执行全刹车。
     */
    ~ZeroDampingRJoint() override {
        {
            std::lock_guard<std::mutex> registration_lock(callback_registration_mutex_);
            {
                std::lock_guard<std::mutex> callback_lock(callback_state_->mutex);
                callback_state_->callback = nullptr;
            }
            try {
                joint_->SetOnStatus(nullptr);
            } catch (...) {}
        }
        shutdown_.store(true, std::memory_order_release);
        wakeup_.notify_all();
        if (worker_.joinable()) {
            worker_.join();
        }
        try {
            joint_->Stop(MotorStopMode::FullBrake);
        } catch (...) {}
    }

    void PVTControl(float kp, float kd, float pos, float spd, float torque) override {
        RejectNonFiniteControlInput(kp, kd, pos, spd, torque);
        StoreCommand(PVTCommand{kp, kd, pos, spd, torque});
    }

    void PosControl(float position, float speed = 0, float current = 0, int feedback = 1) override {
        RejectNonFiniteControlInput(position, speed, current);
        StoreCommand(PositionCommand{position, speed, current, feedback});
    }

    void SpdControl(float speed, float current = 0, int feedback = 1) override {
        RejectNonFiniteControlInput(speed, current);
        StoreCommand(SpeedCommand{speed, current, feedback});
    }

    void CurControl(float current = 0, int feedback = 1) override {
        RejectNonFiniteControlInput(current);
        StoreCommand(CurrentCommand{current, feedback});
    }

    void TorControl(float torque = 0, int feedback = 1) override {
        RejectNonFiniteControlInput(torque);
        StoreCommand(TorqueCommand{torque, feedback});
    }

    void Stop(MotorStopMode mode, float current = 0, int feedback = 1) override {
        LatchStop();
        std::lock_guard<std::mutex> lock(operation_mutex_);
        joint_->Stop(mode, current, feedback);
    }

    float PosAsOffset(int dir) override {
        ThrowIfFaulted();
        LatchStop();
        std::lock_guard<std::mutex> lock(operation_mutex_);
        return joint_->PosAsOffset(dir);
    }

    /**
     * @warning 关节运动过程中不得将 autoSendCommand 设为 true；底层反馈命令可能改变运动状态。
     */
    std::optional<JointStatus> GetStatus(bool autoSendCommand = false,
                                         int lifeCycleDeduction = 1) override {
        auto status = joint_->GetStatus(autoSendCommand, lifeCycleDeduction);
        if (status) {
            status->tor -= callback_state_->retained_compensation.load(std::memory_order_relaxed);
        }
        return status;
    }

    void SetOnStatus(std::function<void(const JointStatus&)> onStatus) override {
        std::lock_guard<std::mutex> registration_lock(callback_registration_mutex_);
        bool enabled = false;
        {
            std::lock_guard<std::mutex> lock(callback_state_->mutex);
            callback_state_->callback = std::move(onStatus);
            enabled = static_cast<bool>(callback_state_->callback);
        }
        if (!enabled) {
            joint_->SetOnStatus(nullptr);
            return;
        }
        const auto state = callback_state_;
        joint_->SetOnStatus([state](const JointStatus& status) {
            std::function<void(const JointStatus&)> callback;
            {
                std::lock_guard<std::mutex> lock(state->mutex);
                callback = state->callback;
            }
            if (!callback) {
                return;
            }
            auto adjusted = status;
            adjusted.tor -= state->retained_compensation.load(std::memory_order_relaxed);
            callback(adjusted);
        });
    }

    void SetLimit(Range<float> limit) override {
        std::lock_guard<std::mutex> lock(operation_mutex_);
        joint_->SetLimit(limit);
        SetValidatedLimit(limit);
    }

    /**
     * @brief 获取控制线程累计的周期超限次数。
     */
    std::uint64_t DeadlineOverrunCount() const {
        return deadline_overrun_count_.load(std::memory_order_relaxed);
    }

    /**
     * @brief 获取最近一次实际保留在底层命令中的摩擦与惯量总补偿力矩。
     * @return 关节坐标系下的补偿力矩，单位 Nm。
     */
    float RetainedCompensationTorque() const noexcept {
        return callback_state_->retained_compensation.load(std::memory_order_relaxed);
    }

    /**
     * @brief 获取最近一次实际保留在底层命令中的摩擦补偿力矩。
     */
    float RetainedFrictionCompensationTorque() const noexcept {
        return callback_state_->retained_friction_compensation.load(std::memory_order_relaxed);
    }

    /**
     * @brief 获取最近一次实际保留在底层命令中的惯量补偿力矩。
     */
    float RetainedInertiaCompensationTorque() const noexcept {
        return callback_state_->retained_inertia_compensation.load(std::memory_order_relaxed);
    }

private:
    using Clock = std::chrono::steady_clock;
    static constexpr auto kControlPeriod = std::chrono::milliseconds{1};
    static constexpr auto kCommandTimeout = std::chrono::milliseconds{500};
    static constexpr float kSamplePeriodSeconds = 0.001F;

    class InertiaCompensationModel {
    public:
        InertiaCompensationModel(float inertia, float compensation_ratio,
                                 float max_compensation_torque, float filter_cutoff)
            : compensated_inertia_{inertia * compensation_ratio},
              max_compensation_torque_{max_compensation_torque} {
            constexpr float kPi = 3.14159265358979323846F;
            const float rc = 1.0F / (2.0F * kPi * filter_cutoff);
            filter_alpha_ = kSamplePeriodSeconds / (rc + kSamplePeriodSeconds);
        }

        float Update(float raw_speed) {
            if (!initialized_) {
                filtered_speed_ = raw_speed;
                previous_speed_ = raw_speed;
                initialized_ = true;
                return 0.0F;
            }
            filtered_speed_ += filter_alpha_ * (raw_speed - filtered_speed_);
            const float raw_acceleration =
                (filtered_speed_ - previous_speed_) / kSamplePeriodSeconds;
            previous_speed_ = filtered_speed_;
            filtered_acceleration_ += filter_alpha_ * (raw_acceleration - filtered_acceleration_);
            return std::clamp(compensated_inertia_ * filtered_acceleration_,
                              -max_compensation_torque_, max_compensation_torque_);
        }

        void Reset() {
            filtered_speed_ = 0.0F;
            previous_speed_ = 0.0F;
            filtered_acceleration_ = 0.0F;
            initialized_ = false;
        }

    private:
        float compensated_inertia_;
        float max_compensation_torque_;
        float filter_alpha_{0.0F};
        float filtered_speed_{0.0F};
        float previous_speed_{0.0F};
        float filtered_acceleration_{0.0F};
        bool initialized_{false};
    };

    struct Compensation {
        float friction;
        float inertia;
        float total;
    };

    struct PVTCommand {
        float kp;
        float kd;
        float pos;
        float spd;
        float torque;
    };

    struct PositionCommand {
        float position;
        float speed;
        float current;
        int feedback;
    };

    struct SpeedCommand {
        float speed;
        float current;
        int feedback;
    };

    struct CurrentCommand {
        float current;
        int feedback;
    };

    struct TorqueCommand {
        float torque;
        int feedback;
    };

    using Command = std::variant<std::monostate, PVTCommand, PositionCommand, SpeedCommand,
                                 CurrentCommand, TorqueCommand>;

    struct CommandSnapshot {
        Command command;
        Clock::time_point last_update;
        std::uint64_t generation;
        bool active;
    };

    struct CombinedEffort {
        float command;
        float retained_compensation;
    };

    static RJoint& RequireJoint(const std::shared_ptr<RJoint>& joint) {
        if (!joint) {
            throw std::invalid_argument("ZeroDampingRJoint joint must not be null");
        }
        return *joint;
    }

    static CombinedEffort CombineEffort(float user, float compensation, float limit) {
        const float saturated_user = std::clamp(user, -limit, limit);
        const float combined = std::clamp(saturated_user + compensation, -limit, limit);
        return CombinedEffort{combined, combined - saturated_user};
    }

    template <typename... Values>
    static void RejectNonFiniteControlInput(Values... values) {
        if (!(std::isfinite(values) && ...)) {
            throw std::invalid_argument("ZeroDampingRJoint control input must be finite");
        }
    }

    template <typename T>
    void StoreCommand(T command) {
        ThrowIfFaulted();
        std::unique_lock<std::mutex> lock(command_mutex_);
        if (faulted_.load(std::memory_order_acquire)) {
            lock.unlock();
            ThrowIfFaulted();
        }
        const bool was_stopped = stopped_;
        command_ = std::move(command);
        last_command_time_ = Clock::now();
        has_first_command_ = true;
        stopped_ = false;
        ++command_generation_;
        if (was_stopped) {
            reset_model_.store(true, std::memory_order_release);
        }
    }

    void LatchStop() {
        std::lock_guard<std::mutex> lock(command_mutex_);
        stopped_ = true;
        command_ = std::monostate{};
        ++command_generation_;
        ClearRetainedCompensation();
        reset_model_.store(true, std::memory_order_release);
    }

    void ThrowIfFaulted() const {
        std::exception_ptr fault;
        {
            std::lock_guard<std::mutex> lock(fault_mutex_);
            fault = fault_;
        }
        if (!fault) {
            return;
        }
        try {
            std::rethrow_exception(fault);
        } catch (const std::exception& error) {
            throw std::runtime_error(std::string{"ZeroDampingRJoint fault: "} + error.what());
        } catch (...) {
            throw std::runtime_error("ZeroDampingRJoint fault: unknown error");
        }
    }

    CommandSnapshot SnapshotCommand() const {
        std::lock_guard<std::mutex> lock(command_mutex_);
        return CommandSnapshot{
            command_, last_command_time_, command_generation_,
            has_first_command_ && !stopped_ && !faulted_.load(std::memory_order_acquire)};
    }

    bool SnapshotIsCurrent(const CommandSnapshot& snapshot) const {
        std::lock_guard<std::mutex> lock(command_mutex_);
        return snapshot.generation == command_generation_ && has_first_command_ && !stopped_ &&
               !faulted_.load(std::memory_order_acquire);
    }

    static std::optional<float> FreshEffort(const Command& command, float kt) {
        return std::visit(
            [kt](const auto& value) -> std::optional<float> {
                using T = std::decay_t<decltype(value)>;
                if constexpr (std::is_same_v<T, PVTCommand>) {
                    return value.torque;
                } else if constexpr (std::is_same_v<T, CurrentCommand>) {
                    return value.current * kt;
                } else if constexpr (std::is_same_v<T, TorqueCommand>) {
                    return value.torque;
                }
                return std::nullopt;
            },
            command);
    }

    bool EffortAllowed(const std::optional<JointStatus>& status, float effort) const {
        if (!status || effort == 0.0f) {
            return true;
        }
        const auto limit = joint_->GetJointLimit();
        if (status->pos < limit.min + kLimitSlack && effort < 0.0f) {
            return false;
        }
        if (status->pos > limit.max - kLimitSlack && effort > 0.0f) {
            return false;
        }
        return true;
    }

    float CurrentControlTorqueLimit() const {
        return std::min(joint_->GetMaxTorque(),
                        joint_->GetMaxCurrent() * joint_->GetTorqueConstant());
    }

    void DispatchTorqueAsCurrent(float torque, int feedback) const {
        joint_->CurControl(torque / joint_->GetTorqueConstant(), feedback);
    }

    Compensation ComputeCompensation(std::optional<float> raw_position, float raw_speed,
                                     std::optional<float> effort) {
        const float friction = raw_position ? model_.Update(*raw_position, raw_speed, effort)
                                            : model_.Update(raw_speed, effort);
        const float inertia = inertia_model_.Update(raw_speed);
        const float requested = friction + inertia;
        const float total =
            std::clamp(requested, -max_compensation_torque_, max_compensation_torque_);
        if (requested == 0.0F) {
            return Compensation{0.0F, 0.0F, 0.0F};
        }
        const float scale = total / requested;
        return Compensation{friction * scale, inertia * scale, total};
    }

    void StoreRetainedCompensation(const Compensation& compensation, float retained_total) {
        float friction = 0.0F;
        float inertia = 0.0F;
        if (compensation.total != 0.0F) {
            const float scale = retained_total / compensation.total;
            friction = compensation.friction * scale;
            inertia = compensation.inertia * scale;
        }
        callback_state_->retained_friction_compensation.store(friction, std::memory_order_relaxed);
        callback_state_->retained_inertia_compensation.store(inertia, std::memory_order_relaxed);
        callback_state_->retained_compensation.store(retained_total, std::memory_order_relaxed);
    }

    void ClearRetainedCompensation() {
        StoreRetainedCompensation(Compensation{0.0F, 0.0F, 0.0F}, 0.0F);
    }

    void DispatchOnce(const CommandSnapshot& snapshot, Clock::time_point now) {
        std::lock_guard<std::mutex> lock(operation_mutex_);
        if (!SnapshotIsCurrent(snapshot)) {
            return;
        }
        if (reset_model_.exchange(false, std::memory_order_acq_rel)) {
            model_.Reset();
            inertia_model_.Reset();
        }
        const auto status = joint_->GetStatus(false, 0);
        const float speed = status && std::isfinite(status->spd) ? status->spd : 0.0f;
        const bool timed_out = now - snapshot.last_update >= kCommandTimeout;
        const auto effort =
            timed_out ? std::nullopt : FreshEffort(snapshot.command, joint_->GetTorqueConstant());
        const auto position =
            status && std::isfinite(status->pos) ? std::optional<float>{status->pos} : std::nullopt;
        const auto compensation = ComputeCompensation(position, speed, effort);

        if (timed_out) {
            const auto combined =
                CombineEffort(0.0F, compensation.total, CurrentControlTorqueLimit());
            const bool allowed = EffortAllowed(status, combined.command);
            StoreRetainedCompensation(compensation,
                                      allowed ? combined.retained_compensation : 0.0F);
            DispatchTorqueAsCurrent(allowed ? combined.command : 0.0F, 1);
            return;
        }

        std::visit(
            [this, &status, compensation](const auto& value) {
                using T = std::decay_t<decltype(value)>;
                if constexpr (std::is_same_v<T, PVTCommand>) {
                    const auto combined =
                        CombineEffort(value.torque, compensation.total, joint_->GetMaxTorque());
                    const bool allowed = EffortAllowed(status, combined.command);
                    StoreRetainedCompensation(compensation,
                                              allowed ? combined.retained_compensation : 0.0F);
                    joint_->PVTControl(value.kp, value.kd, value.pos, value.spd,
                                       allowed ? combined.command : 0.0f);
                } else if constexpr (std::is_same_v<T, PositionCommand>) {
                    ClearRetainedCompensation();
                    joint_->PosControl(value.position, value.speed, value.current, value.feedback);
                } else if constexpr (std::is_same_v<T, SpeedCommand>) {
                    ClearRetainedCompensation();
                    joint_->SpdControl(value.speed, value.current, value.feedback);
                } else if constexpr (std::is_same_v<T, CurrentCommand>) {
                    const float kt = joint_->GetTorqueConstant();
                    const auto combined = CombineEffort(value.current, compensation.total / kt,
                                                        joint_->GetMaxCurrent());
                    const bool allowed = EffortAllowed(status, combined.command);
                    StoreRetainedCompensation(compensation,
                                              allowed ? combined.retained_compensation * kt : 0.0F);
                    joint_->CurControl(allowed ? combined.command : 0.0f, value.feedback);
                } else if constexpr (std::is_same_v<T, TorqueCommand>) {
                    const auto combined = CombineEffort(value.torque, compensation.total,
                                                        CurrentControlTorqueLimit());
                    const bool allowed = EffortAllowed(status, combined.command);
                    StoreRetainedCompensation(compensation,
                                              allowed ? combined.retained_compensation : 0.0F);
                    DispatchTorqueAsCurrent(allowed ? combined.command : 0.0F, value.feedback);
                }
            },
            snapshot.command);
    }

    void RecordFault(std::exception_ptr fault) noexcept {
        {
            std::lock_guard<std::mutex> lock(fault_mutex_);
            if (!fault_) {
                fault_ = std::move(fault);
            }
        }
        faulted_.store(true, std::memory_order_release);
        LatchStop();
        try {
            std::lock_guard<std::mutex> lock(operation_mutex_);
            joint_->Stop(MotorStopMode::FullBrake);
        } catch (...) {}
    }

    void RunWorker() noexcept {
        auto next = Clock::now();
        while (!shutdown_.load(std::memory_order_acquire)) {
            next += kControlPeriod;
            const auto snapshot = SnapshotCommand();
            if (snapshot.active) {
                try {
                    DispatchOnce(snapshot, Clock::now());
                } catch (...) {
                    RecordFault(std::current_exception());
                    return;
                }
            }

            const auto after = Clock::now();
            if (after > next) {
                deadline_overrun_count_.fetch_add(1, std::memory_order_relaxed);
                do {
                    next += kControlPeriod;
                } while (next <= after);
            }
            std::unique_lock<std::mutex> lock(wakeup_mutex_);
            wakeup_.wait_until(lock, next, [this] {
                return shutdown_.load(std::memory_order_acquire);
            });
        }
    }

    std::shared_ptr<RJoint> joint_;
    StribeckModel model_;
    InertiaCompensationModel inertia_model_;
    float max_compensation_torque_;

    mutable std::mutex command_mutex_;
    Command command_;
    Clock::time_point last_command_time_{};
    bool has_first_command_{false};
    bool stopped_{false};
    std::uint64_t command_generation_{0};

    std::mutex operation_mutex_;
    std::atomic<bool> reset_model_{false};

    mutable std::mutex fault_mutex_;
    std::exception_ptr fault_;
    std::atomic<bool> faulted_{false};

    std::mutex callback_registration_mutex_;
    std::shared_ptr<StatusCallbackState> callback_state_{std::make_shared<StatusCallbackState>()};

    std::atomic<bool> shutdown_{false};
    std::mutex wakeup_mutex_;
    std::condition_variable wakeup_;
    std::atomic<std::uint64_t> deadline_overrun_count_{0};
    std::thread worker_;
};

}  // namespace encos
