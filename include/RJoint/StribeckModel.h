// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace encos {

/**
 * @brief Stribeck 摩擦补偿参数。
 */
struct StribeckConfig {
    float static_friction;
    float coulomb_friction;
    float viscous_coefficient;
    float stribeck_velocity;
    float velocity_filter_cutoff;
    float start_velocity;
    float stop_velocity;
    float effort_direction_threshold;
    float max_compensation_torque;
    float static_friction_compensation_ratio;
    float position_intent_threshold;
    float inertia{0.0f};
    float inertia_compensation_ratio{0.0f};
    float max_inertia_compensation_torque{0.0f};

    /**
     * @brief 从 friction.json 的 control 对象解析配置。
     * @param json 参数 JSON。
     * @return 已校验的 Stribeck 配置。
     */
    static StribeckConfig FromJson(const nlohmann::json& json) {
        StribeckConfig config{
            json.at("staticFriction").get<float>(),
            json.at("coulombFriction").get<float>(),
            json.at("viscousCoefficient").get<float>(),
            json.at("stribeckVelocity").get<float>(),
            json.at("velocityFilterCutoff").get<float>(),
            json.at("startVelocity").get<float>(),
            json.at("stopVelocity").get<float>(),
            json.at("effortDirectionThreshold").get<float>(),
            json.at("maxCompensationTorque").get<float>(),
            json.at("staticFrictionCompensationRatio").get<float>(),
            json.at("positionIntentThreshold").get<float>(),
            json.at("inertia").get<float>(),
            json.at("inertiaCompensationRatio").get<float>(),
            json.at("maxInertiaCompensationTorque").get<float>(),
        };
        config.Validate();
        return config;
    }

    /**
     * @brief 校验物理参数和 1 kHz 采样约束。
     */
    void Validate() const {
        const bool all_finite =
            std::isfinite(static_friction) && std::isfinite(coulomb_friction) &&
            std::isfinite(viscous_coefficient) && std::isfinite(stribeck_velocity) &&
            std::isfinite(velocity_filter_cutoff) && std::isfinite(start_velocity) &&
            std::isfinite(stop_velocity) && std::isfinite(effort_direction_threshold) &&
            std::isfinite(max_compensation_torque) &&
            std::isfinite(static_friction_compensation_ratio) &&
            std::isfinite(position_intent_threshold) && std::isfinite(inertia) &&
            std::isfinite(inertia_compensation_ratio) &&
            std::isfinite(max_inertia_compensation_torque);
        if (!all_finite) {
            throw std::invalid_argument("StribeckConfig values must be finite");
        }
        if (!(static_friction >= coulomb_friction && coulomb_friction >= 0.0f)) {
            throw std::invalid_argument("StribeckConfig requires Fs >= Fc >= 0");
        }
        if (viscous_coefficient < 0.0f) {
            throw std::invalid_argument("StribeckConfig requires B >= 0");
        }
        if (stribeck_velocity <= 0.0f) {
            throw std::invalid_argument("StribeckConfig requires Vs > 0");
        }
        if (!(velocity_filter_cutoff > 0.0f && velocity_filter_cutoff < 500.0f)) {
            throw std::invalid_argument(
                "StribeckConfig velocityFilterCutoff must be between 0 and 500 Hz");
        }
        if (!(stop_velocity >= 0.0f && stop_velocity < start_velocity)) {
            throw std::invalid_argument(
                "StribeckConfig requires 0 <= stopVelocity < startVelocity");
        }
        if (effort_direction_threshold < 0.0f) {
            throw std::invalid_argument(
                "StribeckConfig effortDirectionThreshold must be non-negative");
        }
        if (max_compensation_torque <= 0.0f) {
            throw std::invalid_argument("StribeckConfig maxCompensationTorque must be positive");
        }
        if (!(static_friction_compensation_ratio >= 0.0f &&
              static_friction_compensation_ratio < 1.0f)) {
            throw std::invalid_argument(
                "StribeckConfig staticFrictionCompensationRatio must be in [0, 1)");
        }
        if (position_intent_threshold <= 0.0f) {
            throw std::invalid_argument("StribeckConfig positionIntentThreshold must be positive");
        }
        if (inertia < 0.0f) {
            throw std::invalid_argument("StribeckConfig inertia must be non-negative");
        }
        if (!(inertia_compensation_ratio >= 0.0f && inertia_compensation_ratio < 1.0f)) {
            throw std::invalid_argument(
                "StribeckConfig inertiaCompensationRatio must be in [0, 1)");
        }
        if (max_inertia_compensation_torque < 0.0f ||
            (inertia > 0.0f && inertia_compensation_ratio > 0.0f &&
             max_inertia_compensation_torque == 0.0f)) {
            throw std::invalid_argument(
                "StribeckConfig maxInertiaCompensationTorque must be positive when inertia "
                "compensation is enabled");
        }
        if (max_inertia_compensation_torque > max_compensation_torque) {
            throw std::invalid_argument(
                "StribeckConfig maxInertiaCompensationTorque must not exceed "
                "maxCompensationTorque");
        }
    }
};

/**
 * @brief 带因果速度滤波和滞回的 Stribeck 摩擦模型。
 */
class StribeckModel {
public:
    /**
     * @brief 构造模型。
     * @param config 模型配置。
     * @param sample_period 采样周期，单位 s。
     */
    explicit StribeckModel(StribeckConfig config, float sample_period = 0.001f)
        : config_{config}, sample_period_{sample_period} {
        config_.Validate();
        if (!std::isfinite(sample_period_) || sample_period_ <= 0.0f) {
            throw std::invalid_argument("StribeckModel sample period must be positive");
        }
        constexpr float kPi = 3.14159265358979323846f;
        const float rc = 1.0f / (2.0f * kPi * config_.velocity_filter_cutoff);
        filter_alpha_ = sample_period_ / (rc + sample_period_);
        position_intent_hold_cycles_ =
            std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(0.05f / sample_period_)));
    }

    /**
     * @brief 更新速度状态并计算摩擦补偿力矩。
     * @param raw_speed 原始关节速度，单位 rad/s。
     * @param fresh_effort 新鲜作用力指令，单位 Nm；空值表示不能用指令决定方向。
     * @return 摩擦补偿力矩，单位 Nm。
     */
    float Update(float raw_speed, std::optional<float> fresh_effort) {
        return UpdateImpl(std::nullopt, raw_speed, fresh_effort);
    }

    /**
     * @brief 使用低速位置变化辅助判断静摩擦方向。
     * @param raw_position 原始关节位置，单位 rad。
     * @param raw_speed 原始关节速度，单位 rad/s。
     * @param fresh_effort 新鲜作用力指令，单位 Nm。
     * @return 摩擦补偿力矩，单位 Nm。
     */
    float Update(float raw_position, float raw_speed, std::optional<float> fresh_effort) {
        if (!std::isfinite(raw_position)) {
            throw std::invalid_argument("StribeckModel inputs must be finite");
        }
        return UpdateImpl(raw_position, raw_speed, fresh_effort);
    }

    /**
     * @brief 清空滤波和滞回状态。
     */
    void Reset() {
        filtered_speed_ = 0.0f;
        speed_active_ = false;
        speed_direction_ = 0.0f;
        position_initialized_ = false;
        position_anchor_ = 0.0f;
        previous_position_ = 0.0f;
        position_intent_direction_ = 0.0f;
        position_intent_quiet_cycles_ = 0;
    }

    /**
     * @brief 获取当前滤波速度。
     * @return 滤波速度，单位 rad/s。
     */
    float FilteredSpeed() const {
        return filtered_speed_;
    }

private:
    float UpdateImpl(std::optional<float> raw_position, float raw_speed,
                     std::optional<float> fresh_effort) {
        if (!std::isfinite(raw_speed) ||
            (fresh_effort.has_value() && !std::isfinite(*fresh_effort))) {
            throw std::invalid_argument("StribeckModel inputs must be finite");
        }

        filtered_speed_ += filter_alpha_ * (raw_speed - filtered_speed_);
        const float abs_speed = std::abs(filtered_speed_);
        if (speed_active_) {
            if (abs_speed <= config_.stop_velocity) {
                speed_active_ = false;
                speed_direction_ = 0.0f;
            } else if (filtered_speed_ != 0.0f) {
                speed_direction_ = std::copysign(1.0f, filtered_speed_);
            }
        } else if (abs_speed >= config_.start_velocity) {
            speed_active_ = true;
            speed_direction_ = std::copysign(1.0f, filtered_speed_);
        }

        const bool effort_direction_available =
            !speed_active_ && fresh_effort.has_value() &&
            std::abs(*fresh_effort) >= config_.effort_direction_threshold && *fresh_effort != 0.0f;
        UpdatePositionIntent(raw_position, speed_active_ || effort_direction_available);

        float direction = speed_active_ ? speed_direction_ : 0.0f;
        if (effort_direction_available) {
            direction = std::copysign(1.0f, *fresh_effort);
        } else if (!speed_active_ && position_intent_direction_ != 0.0f) {
            const float torque = position_intent_direction_ * config_.static_friction *
                                 config_.static_friction_compensation_ratio;
            return std::clamp(torque, -config_.max_compensation_torque,
                              config_.max_compensation_torque);
        }
        if (direction == 0.0f) {
            return 0.0f;
        }

        const float normalized_speed = abs_speed / config_.stribeck_velocity;
        const float magnitude =
            config_.coulomb_friction + (config_.static_friction - config_.coulomb_friction) *
                                           std::exp(-(normalized_speed * normalized_speed));
        const float torque = direction * magnitude + config_.viscous_coefficient * filtered_speed_;
        return std::clamp(torque, -config_.max_compensation_torque,
                          config_.max_compensation_torque);
    }

    void UpdatePositionIntent(std::optional<float> raw_position, bool blocked) {
        if (!raw_position.has_value()) {
            position_intent_direction_ = 0.0f;
            return;
        }
        const float position = *raw_position;
        if (!position_initialized_) {
            position_initialized_ = true;
            position_anchor_ = position;
            previous_position_ = position;
            return;
        }
        if (blocked) {
            position_anchor_ = position;
            previous_position_ = position;
            position_intent_direction_ = 0.0f;
            position_intent_quiet_cycles_ = 0;
            return;
        }

        const float step = position - previous_position_;
        if (position_intent_direction_ != 0.0f) {
            if (step * position_intent_direction_ > 0.0f) {
                position_intent_quiet_cycles_ = 0;
            } else if (step * position_intent_direction_ < 0.0f) {
                position_anchor_ = previous_position_;
                position_intent_direction_ = 0.0f;
                position_intent_quiet_cycles_ = 0;
            } else if (++position_intent_quiet_cycles_ >= position_intent_hold_cycles_) {
                position_anchor_ = position;
                position_intent_direction_ = 0.0f;
                position_intent_quiet_cycles_ = 0;
            }
        }
        if (position_intent_direction_ == 0.0f) {
            const float displacement = position - position_anchor_;
            if (std::abs(displacement) >= config_.position_intent_threshold) {
                position_intent_direction_ = std::copysign(1.0f, displacement);
                position_intent_quiet_cycles_ = 0;
            }
        }
        previous_position_ = position;
    }

    StribeckConfig config_;
    float sample_period_;
    float filter_alpha_{0.0f};
    float filtered_speed_{0.0f};
    float speed_direction_{0.0f};
    bool speed_active_{false};
    std::size_t position_intent_hold_cycles_{1};
    std::size_t position_intent_quiet_cycles_{0};
    float position_anchor_{0.0f};
    float previous_position_{0.0f};
    float position_intent_direction_{0.0f};
    bool position_initialized_{false};
};

}  // namespace encos
