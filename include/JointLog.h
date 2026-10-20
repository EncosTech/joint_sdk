// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>

#include "utils/log_writer.h"

namespace encos {

/**
 * @brief JointManager 使用的关节级 CSV+Zstd 日志会话。
 *
 * 每个关节分别拥有 command/status 文件，保护告警则写入一个跨关节的文件。
 */
class JointLogContext {
public:
    explicit JointLogContext(const std::filesystem::path& directory)
        : directory_(directory),
          protection_writer_(
              (directory / "joint_protection").string(),
              std::array<std::string, 9>{"timestamp_ns", "joint", "mode", "scope", "reason",
                                         "state", "field", "requested", "applied"}) {}

    JointLogContext(const JointLogContext&) = delete;
    JointLogContext& operator=(const JointLogContext&) = delete;

    /** @brief 为关节创建独立的 command/status 文件。 */
    void RegisterJoint(std::string_view joint) {
        const std::string name(joint);
        std::lock_guard<std::mutex> lock(writers_mutex_);
        if (command_writers_.find(name) != command_writers_.end()) {
            return;
        }
        auto command = std::make_shared<CommandWriter>(
            (directory_ / (SafeFileName(name) + "_command")).string(),
            std::array<std::string, 13>{"timestamp_ns", "joint", "type", "kp", "kd", "position",
                                        "speed", "current", "torque", "stop_mode", "brake_enabled",
                                        "feedback", "pvt_mode"});
        auto status = std::make_shared<StatusWriter>(
            (directory_ / (SafeFileName(name) + "_status")).string(),
            std::array<std::string, 8>{"timestamp_ns", "joint", "position", "speed", "torque",
                                       "mos_temperature", "motor_temperature", "error"});
        command_writers_.emplace(name, std::move(command));
        status_writers_.emplace(name, std::move(status));
    }

    void RecordCommand(
        std::string_view joint, std::string_view type, std::optional<float> kp = std::nullopt,
        std::optional<float> kd = std::nullopt, std::optional<float> position = std::nullopt,
        std::optional<float> speed = std::nullopt, std::optional<float> current = std::nullopt,
        std::optional<float> torque = std::nullopt, std::optional<int> stop_mode = std::nullopt,
        std::optional<bool> brake_enabled = std::nullopt,
        std::optional<int> feedback = std::nullopt,
        std::optional<int> pvt_mode = std::nullopt) noexcept {
        std::shared_ptr<CommandWriter> writer;
        {
            std::lock_guard<std::mutex> lock(writers_mutex_);
            const auto it = command_writers_.find(std::string(joint));
            if (it == command_writers_.end()) {
                return;
            }
            writer = it->second;
        }
        try {
            writer->write(Now(), joint, type, kp, kd, position, speed, current, torque, stop_mode,
                          brake_enabled, feedback, pvt_mode);
        } catch (...) {}
    }

    void RecordStatus(std::string_view joint, float position, float speed, float torque,
                      float mos_temperature, float motor_temperature, int error) noexcept {
        std::shared_ptr<StatusWriter> writer;
        {
            std::lock_guard<std::mutex> lock(writers_mutex_);
            const auto it = status_writers_.find(std::string(joint));
            if (it == status_writers_.end()) {
                return;
            }
            writer = it->second;
        }
        try {
            writer->write(Now(), joint, position, speed, torque, mos_temperature, motor_temperature,
                          error);
        } catch (...) {}
    }

    void RecordProtection(std::string_view joint, std::string_view mode, std::string_view scope,
                          std::string_view reason, std::string_view state,
                          std::string_view field = {},
                          std::optional<double> requested = std::nullopt,
                          std::optional<double> applied = std::nullopt) noexcept {
        try {
            protection_writer_.write(Now(), joint, mode, scope, reason, state, field, requested,
                                     applied);
        } catch (...) {}
    }

private:
    using CommandWriter = LogWriter<13>;
    using StatusWriter = LogWriter<8>;

    static std::string SafeFileName(std::string_view value) {
        constexpr char kHexDigits[] = "0123456789ABCDEF";
        std::string encoded;
        encoded.reserve(value.size());
        for (const unsigned char ch : value) {
            const bool safe = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
                              (ch >= '0' && ch <= '9') || ch == '-' || ch == '_' || ch == '.';
            if (safe) {
                encoded.push_back(static_cast<char>(ch));
            } else {
                encoded.push_back('%');
                encoded.push_back(kHexDigits[ch >> 4U]);
                encoded.push_back(kHexDigits[ch & 0x0FU]);
            }
        }
        return encoded.empty() ? "unnamed" : encoded;
    }

    static std::int64_t Now() noexcept {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
            .count();
    }

    std::filesystem::path directory_;
    std::mutex writers_mutex_;
    std::map<std::string, std::shared_ptr<CommandWriter>> command_writers_;
    std::map<std::string, std::shared_ptr<StatusWriter>> status_writers_;
    LogWriter<9> protection_writer_;
};

}  // namespace encos
