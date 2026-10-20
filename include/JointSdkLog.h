// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

#pragma once

#include <string>
#include <utility>

#include "platform/log.h"

namespace encos {

/**
 * @brief joint_sdk 内部日志辅助函数。
 *
 * 不直接依赖 spdlog；在原生构建中通过 motor_driver 的 Logger（可转 spdlog）输出，
 * 在 Emscripten 构建中自动使用 emscripten 控制台后端。这样 joint_sdk 的 header-only
 * 部分在 WASM 编译时无需拉取 spdlog。
 */
class JointSdkLogger {
public:
    /**
     * @brief 获取 joint_sdk 使用的共享日志器。
     * @return LoggerPtr 引用，首次调用时创建名为 joint_sdk 的日志器。
     */
    static LoggerPtr& Get() {
        static LoggerPtr logger = CreateLogger("joint_sdk");
        return logger;
    }
};

/**
 * @brief 写入 trace 级别日志。
 * @tparam Args 格式化参数类型。
 * @param fmt 日志格式字符串。
 * @param args 传递给底层日志器的格式化参数。
 */
template <typename... Args>
inline void JointSdkLogTrace(const std::string& fmt, Args&&... args) {
    JointSdkLogger::Get()->trace(fmt, std::forward<Args>(args)...);
}

/**
 * @brief 写入 debug 级别日志。
 * @tparam Args 格式化参数类型。
 * @param fmt 日志格式字符串。
 * @param args 传递给底层日志器的格式化参数。
 */
template <typename... Args>
inline void JointSdkLogDebug(const std::string& fmt, Args&&... args) {
    JointSdkLogger::Get()->debug(fmt, std::forward<Args>(args)...);
}

/**
 * @brief 写入 info 级别日志。
 * @tparam Args 格式化参数类型。
 * @param fmt 日志格式字符串。
 * @param args 传递给底层日志器的格式化参数。
 */
template <typename... Args>
inline void JointSdkLogInfo(const std::string& fmt, Args&&... args) {
    JointSdkLogger::Get()->info(fmt, std::forward<Args>(args)...);
}

/**
 * @brief 写入 warn 级别日志。
 * @tparam Args 格式化参数类型。
 * @param fmt 日志格式字符串。
 * @param args 传递给底层日志器的格式化参数。
 */
template <typename... Args>
inline void JointSdkLogWarn(const std::string& fmt, Args&&... args) {
    JointSdkLogger::Get()->warn(fmt, std::forward<Args>(args)...);
}

/**
 * @brief 写入 error 级别日志。
 * @tparam Args 格式化参数类型。
 * @param fmt 日志格式字符串。
 * @param args 传递给底层日志器的格式化参数。
 */
template <typename... Args>
inline void JointSdkLogError(const std::string& fmt, Args&&... args) {
    JointSdkLogger::Get()->error(fmt, std::forward<Args>(args)...);
}

/**
 * @brief 写入 critical 级别日志。
 * @tparam Args 格式化参数类型。
 * @param fmt 日志格式字符串。
 * @param args 传递给底层日志器的格式化参数。
 */
template <typename... Args>
inline void JointSdkLogCritical(const std::string& fmt, Args&&... args) {
    JointSdkLogger::Get()->critical(fmt, std::forward<Args>(args)...);
}

/**
 * @brief 电平型保护日志点在进入保护时可附带的诊断信息。
 *
 * 只在「进入保护」那一条日志里使用；退出保护时不输出这些字段。
 * 各字段均为轻量可选值，不分配内存，字符串需在本次调用期间保持有效。
 */
struct ProtectionLogDetail {
    const char* field{nullptr};  ///< 数值字段名，非空时输出 `reason=<reason> <field>` 与请求/实际值
    double requested{0.0};  ///< 请求值（限幅前的原始命令）
    double applied{0.0};    ///< 实际生效值（限幅后的值）
};

/**
 * @brief 软件保护日志点：一个错误点对应一个标志位。
 *
 * 限位保持、命令限幅、反馈回退这类保护条件一旦成立，通常会在每个控制周期持续成立；
 * 逐周期输出同一告警会淹没其它错误日志。本类只保存标志位，由调用方把条件每次调用
 * 的结果交给 Update()：
 *
 * - `false -> true`（进入保护）返回 Entered，调用方输出一条 enter 日志；
 * - `true -> false`（退出保护）返回 Exited，调用方输出一条 exit 日志；
 * - 状态不变返回 None，不输出任何日志。
 *
 * 进入/退出日志的统一格式见 LogProtectionEnter() / LogProtectionExit()。
 */
class ProtectionLogPoint {
public:
    /** @brief 本次 Update 引起的状态跳变。 */
    enum class Transition {
        None,     ///< 状态未变
        Entered,  ///< 进入保护
        Exited,   ///< 退出保护
    };

    /**
     * @brief 更新标志位。
     * @param active 本次判断该保护是否成立。
     * @return 状态跳变类型；None 表示无需输出日志。
     */
    Transition Update(bool active) {
        if (active == active_) {
            return Transition::None;
        }
        active_ = active;
        return active ? Transition::Entered : Transition::Exited;
    }

    /**
     * @brief 查询该保护当前是否成立。
     * @return true 表示上一次 Update 传入的条件成立。
     */
    bool IsActive() const {
        return active_;
    }

private:
    bool active_{false};  ///< 保护标志位
};

/**
 * @brief 判断保护日志点是否带作用域。
 * @param scope 作用域名，可为空。
 * @return true 表示需要输出 scope 字段。
 */
inline bool HasProtectionScope(const char* scope) {
    return scope != nullptr && scope[0] != '\0';
}

/**
 * @brief 输出「进入保护」日志（warn 级别）。
 * @param logger 保护日志器。
 * @param joint_name 关节名。
 * @param mode 触发保护的控制模式。
 * @param reason 保护原因。
 * @param scope 作用域名（如 roll/pitch/motor1），空表示关节级保护点；仅用于区分同名保护点。
 * @param detail 附加诊断信息；field 非空时一并输出限幅的请求值与实际值。
 */
inline void LogProtectionEnter(const LoggerPtr& logger, const std::string& joint_name,
                               const char* mode, const char* reason, const char* scope,
                               const ProtectionLogDetail& detail = {}) {
    const bool scoped = HasProtectionScope(scope);
    if (detail.field != nullptr) {
        if (scoped) {
            logger->warn(
                "Joint protection: joint={}, mode={}, reason={} {}, requested={}, applied={}, "
                "scope={}, state=enter",
                joint_name, mode, reason, detail.field, detail.requested, detail.applied, scope);
            return;
        }
        logger->warn(
            "Joint protection: joint={}, mode={}, reason={} {}, requested={}, applied={}, "
            "state=enter",
            joint_name, mode, reason, detail.field, detail.requested, detail.applied);
        return;
    }
    if (scoped) {
        logger->warn("Joint protection: joint={}, mode={}, reason={}, scope={}, state=enter",
                     joint_name, mode, reason, scope);
        return;
    }
    logger->warn("Joint protection: joint={}, mode={}, reason={}, state=enter", joint_name, mode,
                 reason);
}

/**
 * @brief 输出「退出保护」日志（info 级别），格式对所有保护点统一。
 * @param logger 保护日志器。
 * @param joint_name 关节名。
 * @param mode 触发保护的控制模式。
 * @param reason 保护原因。
 * @param scope 作用域名，需与进入保护时一致；空表示关节级保护点。
 */
inline void LogProtectionExit(const LoggerPtr& logger, const std::string& joint_name,
                              const char* mode, const char* reason, const char* scope = nullptr) {
    if (HasProtectionScope(scope)) {
        logger->info("Joint protection: joint={}, mode={}, reason={}, scope={}, state=exit",
                     joint_name, mode, reason, scope);
        return;
    }
    logger->info("Joint protection: joint={}, mode={}, reason={}, state=exit", joint_name, mode,
                 reason);
}

}  // namespace encos
