// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>

#include <nlohmann/json.hpp>

#include "JointSdkLog.h"
#include "adapter/base_adapter.h"
#include "bus/bus.h"
#include "encos_motor.h"
#include "motor/motor.h"
#include "motor/types.h"

namespace encos {

/** @brief 耦合关节 PVT 解算与下发模式。 */
enum class PVTControlMode : int {
    MotorPVT = 0,       ///< 映射关节 PVT 后由电机执行位置与速度闭环。
    HostTorque = 1,     ///< 上位机计算 Kp、Kd 和前馈力矩，映射后下发 TorControl。
    HostKpMotorKd = 2,  ///< 上位机计算 Kp 和前馈力矩，电机执行映射后的 Kd 闭环。
};

constexpr bool IsValidPVTControlMode(PVTControlMode mode) {
    return mode == PVTControlMode::MotorPVT || mode == PVTControlMode::HostTorque ||
           mode == PVTControlMode::HostKpMotorKd;
}

template <typename T>
struct AlwaysFalse : std::false_type {};

/**
 * @brief 自动电机初始化参数（空结构体）
 */
struct MotorInitAuto {};

/**
 * @brief 基于型号的电机初始化参数
 */
struct MotorInitModel {
    MotorModel model;  ///< 电机型号
};

/**
 * @brief 基于参数范围的电机初始化参数
 */
struct MotorInitParams {
    MotorPVTRanges ranges;  ///< PVT参数范围
};

/**
 * @brief 电机初始化参数变体类型
 */
using MotorInit = std::variant<MotorInitAuto, MotorInitModel, MotorInitParams>;

/**
 * @brief 适配器引用
 *
 * 整数按适配器建立顺序索引解析，字符串按适配器名称解析；两者的取值类型决定使用
 * 哪一份索引，不做跨索引回退。
 */
using AdapterReference = std::variant<std::uint32_t, std::string>;

/**
 * @brief 电机参数配置结构体
 */
struct MotorParams {
    AdapterReference adapterId;  ///< 适配器引用：整数为建立顺序索引，字符串为适配器名称
    int busId;                   ///< 总线ID
    int motorId;                 ///< 电机ID
    MotorInit initParam;       ///< 初始化参数
    std::uint16_t slaveId{0};  ///< 从站ID，缺省0
    bool canfd{false};         ///< 是否对该电机启用 CAN FD 帧

    /** @brief 适配器引用是否为建立顺序索引（整数）形式。 */
    bool AdapterIdIsIndex() const {
        return std::holds_alternative<std::uint32_t>(adapterId);
    }

    /**
     * @brief 适配器引用的字符串形式
     *
     * 索引形式转换为十进制字符串，名称形式原样返回。用于日志文件名、双电机配对键
     * 等只需要稳定标识的场景。
     * @return 适配器引用的字符串形式。
     */
    std::string AdapterIdString() const {
        if (const auto* index = std::get_if<std::uint32_t>(&adapterId)) {
            return std::to_string(*index);
        }
        return std::get<std::string>(adapterId);
    }

    /**
     * @brief 合成总线索引
     *
     * 底层适配器以 `slaveId << 16 | busId` 作为总线索引；slaveId 为 0 时结果与
     * 直接使用 busId 相同。
     * @return 合成后的总线索引。
     */
    int BusIndex() const {
        return static_cast<int>((static_cast<std::uint32_t>(slaveId) << 16U) |
                                static_cast<std::uint32_t>(busId));
    }
};

/**
 * @brief 关节限位缓冲区（弧度）
 *
 * 用于在到达硬限位前停止运动，值为2度对应的弧度值
 */
constexpr float kLimitSlack = 2.f / 180.f * 3.14159265358979323846f;

/**
 * @brief 关节运行时错误码
 */
enum class JointError {
    kDoubleCoupleJointNotPaired,      ///< 双电机关节未配对即尝试控制
    kDoubleCoupleJointDuplicatePair,  ///< 双电机关节重复配对
    kDoubleCoupleJointSameMotor,      ///< 双电机关节配置中两台电机为同一物理电机
    kDoubleCoupleJointPending,  ///< 双电机关节配置已存储，等待第二个关节完成配对
    kDoubleCoupleJointDuplicateAxis,        ///< 双电机关节配对中两个关节轴重复
    kDoubleCoupleJointCalibrationMismatch,  ///< 双电机关节配对中两份标定不一致
    kDoubleCoupleJointCalibrationFailed,    ///< 双电机关节求解器标定失败
    kDoubleCoupleJointSolveFailed,          ///< 双电机关节运动学解算失败
    kDoubleCoupleJointUnsupportedControl,   ///< 双电机关节不支持该控制模式
    kDoubleCoupleJointZeroPoseNotSet,  ///< 兼容保留：旧版双电机关节零姿态门禁错误
    kDoubleCoupleJointMotorLimitMismatch,  ///< 双电机关节配对中两份电机限位不一致
    kContinuousJointUnsupportedOperation = 11,  ///< 连续关节不支持位置或限位相关操作
};

/**
 * @brief 关节运行时异常
 */
class JointException : public std::runtime_error {
public:
    /**
     * @brief 构造关节异常。
     * @param error 结构化关节错误码。
     * @param msg 面向日志或调用方的错误说明。
     */
    JointException(JointError error, const std::string& msg)
        : std::runtime_error(msg), error_(error) {}

    /**
     * @brief 获取结构化关节错误码。
     * @return 构造异常时传入的 JointError。
     */
    JointError Error() const {
        return error_;
    }

private:
    JointError error_;
};

/**
 * @brief 关节状态结构体
 *
 * 所有字段均为关节坐标系（joint coords），已应用方向反转。
 */
struct JointStatus {
    float pos;               ///< 位置（rad，关节坐标系）
    float spd;               ///< 速度（rad/s，关节坐标系）
    float tor;               ///< 扭矩（Nm，关节坐标系）
    float mosTemperature;    ///< MOS管温度（°C）
    float motorTemperature;  ///< 电机温度（°C）
    MotorError error;        ///< 电机错误状态
};

/**
 * @brief 关节基类
 *
 * 提供关节控制的抽象接口，包括位置、速度、电流、扭矩控制以及校准功能。
 *
 * ### 坐标系约定
 * 本类所有公开接口（位置、速度、扭矩等参数）均使用**关节坐标系**（joint coords）。
 * 若构造时 `reverse=true`，实现层负责在关节坐标系与电机坐标系之间进行透明转换，
 * 调用方无需关心底层电机的实际方向。
 *
 * - `GetStatus()` 返回的位置/速度/扭矩均为关节坐标系
 * - `PosControl`、`SpdControl`、`PVTControl` 等控制函数的输入均为关节坐标系
 * - `GotoLimit` 返回的偏置为关节坐标系，可直接传入 `GotoZero`
 */
class BaseJoint {
public:
    /**
     * @brief 构造函数
     * @param limit 位置限位范围（关节坐标系，rad）
     * @param maxSpd 最大速度（rad/s）
     * @param maxTor 最大扭矩（Nm）
     * @param reverse 是否反转方向。为 true 时，关节坐标系与电机坐标系方向相反，
     *                所有公开接口仍使用关节坐标系
     */
    BaseJoint(Range<float> limit, float maxSpd, float maxTor, bool reverse)
        : BaseJoint{maxSpd, maxTor, reverse} {
        SetValidatedLimit(limit);
    }

protected:
    /**
     * @brief 构造不具备软件位置限位的关节基类。
     * @param maxSpd 最大速度（rad/s）。
     * @param maxTor 最大扭矩（Nm）。
     * @param reverse 是否反转方向。
     */
    BaseJoint(float maxSpd, float maxTor, bool reverse) : reverse_{reverse} {
        if (!std::isfinite(maxSpd) || maxSpd <= 0.0f) {
            throw std::invalid_argument("Joint max speed must be finite and positive");
        }
        if (!std::isfinite(maxTor) || maxTor <= 0.0f) {
            throw std::invalid_argument("Joint max torque must be finite and positive");
        }
        maxSpd_ = maxSpd;
        maxTor_ = maxTor;
    }

public:
    /**
     * @brief 析构关节基类。
     */
    virtual ~BaseJoint() = default;

    /** @brief 设置由 JointManager 提供的保护日志器和关节名称。 */
    void SetProtectionLogContext(LoggerPtr logger, std::string name) {
        if (!logger) {
            throw std::invalid_argument("Joint protection logger must not be null");
        }
        protection_logger_ = std::move(logger);
        protection_name_ = std::move(name);
    }

    const std::string& GetProtectionName() const {
        return protection_name_;
    }

    /**
     * @brief PVT（位置-速度-扭矩）控制
     *
     * 所有参数均为关节坐标系。
     * @param kp 位置比例增益
     * @param kd 速度微分增益
     * @param pos 目标位置（rad，关节坐标系）
     * @param spd 目标速度（rad/s，关节坐标系）
     * @param torque 目标扭矩（Nm，关节坐标系）
     */
    virtual void PVTControl(float kp, float kd, float pos, float spd, float torque) = 0;

    /**
     * @brief 带上位机力矩模式选择的 PVT 控制。
     *
     * 默认实现保持普通关节类型的既有行为；支持上位机力矩模式的关节类型可重载本接口。
     * @param kp 位置比例增益。
     * @param kd 速度微分增益。
     * @param pos 目标位置（rad，关节坐标系）。
     * @param spd 目标速度（rad/s，关节坐标系）。
     * @param torque 前馈扭矩（Nm，关节坐标系）。
     * @param torque_mode 耦合关节的解算模式；普通旋转关节忽略此模式选择。
     */
    virtual void PVTControl(float kp, float kd, float pos, float spd, float torque,
                            PVTControlMode torque_mode) {
        if (!IsValidPVTControlMode(torque_mode)) {
            throw std::invalid_argument("Invalid PVT control mode");
        }
        PVTControl(kp, kd, pos, spd, torque);
    }

    /**
     * @brief 位置控制
     * @param position 目标位置（rad，关节坐标系）
     * @param speed 最大速度（rad/s；0 表示使用关节最大速度）
     * @param current 最大电流（A；0 表示使用该关节类型的默认电流策略）
     * @param feedback 反馈模式
     */
    virtual void PosControl(float position, float speed = 0, float current = 0,
                            int feedback = 1) = 0;

    /**
     * @brief 速度控制
     * @param speed 目标速度（rad/s，关节坐标系）
     * @param current 最大电流（A；0 表示使用该关节类型的默认电流策略）
     * @param feedback 反馈模式
     */
    virtual void SpdControl(float speed, float current = 0, int feedback = 1) = 0;

    /**
     * @brief 电流控制
     * @param current 目标电流
     * @param feedback 反馈模式
     */
    virtual void CurControl(float current = 0, int feedback = 1) = 0;

    /**
     * @brief 扭矩控制
     * @param torque 目标扭矩（Nm，关节坐标系）
     * @param feedback 反馈模式
     */
    virtual void TorControl(float torque = 0, int feedback = 1) = 0;

    /**
     * @brief 停止电机
     * @param mode 停止模式
     * @param current 最大电流
     * @param feedback 反馈模式
     */
    virtual void Stop(MotorStopMode mode, float current = 0, int feedback = 1) = 0;

    /**
     * @brief 将当前位置设置为电机零点
     * @return 成功返回 true
     */
    virtual bool SetZero() = 0;

    /**
     * @brief 驱动关节运动至限位
     *
     * 返回值为**关节坐标系**的偏置，可直接传入 `GotoZero`，
     * 也可与 `GetStatus().pos` 等关节坐标系量配合使用。
     * @param dir 移动方向（关节坐标系；1 或 -1）
     * @param spd 移动速度（rad/s）
     * @param cur 最大电流（A）
     * @param timeout 超时时间
     * @return 零点相对当前零点的偏置值（rad，关节坐标系），失败返回 NaN
     */
    virtual float GotoLimit(int dir, float spd = 2.f, float cur = 2.f,
                            std::chrono::milliseconds timeout = std::chrono::milliseconds{
                                5000}) = 0;

    /**
     * @brief 驱动关节运动至零点并重置零位
     *
     * `offset` 使用**关节坐标系**，与 `GotoLimit` 返回值及 `GetStatus().pos` 单位一致，
     * 外部可直接将 `GetStatus().pos` 作为偏置传入。
     * @param offset 零点相对当前零点的偏置值（rad，关节坐标系），通常由 `GotoLimit` 返回
     * @param spd 移动速度（rad/s）
     * @param cur 最大电流（A）
     * @param timeout 超时时间
     * @return 成功返回 true
     */
    virtual bool GotoZero(float offset, float spd = 2.f, float cur = 2.f,
                          std::chrono::milliseconds timeout = std::chrono::milliseconds{5000}) = 0;

    /**
     * @brief 将当前位置转换为偏置值（不移动电机）
     *
     * 假设关节已运动至指定方向的限位，直接通过当前位置减去对应限位值计算偏置。
     * 返回值与 `GotoLimit` 相同，可直接传入 `GotoZero`。
     * @param dir 方向（关节坐标系；1 或 -1），1 对应 max 限位，-1 对应 min 限位
     * @return 零点偏置值（rad，关节坐标系），如位置不可用则返回 NaN
     */
    virtual float PosAsOffset(int dir) = 0;

    /**
     * @brief 校准关节限位（依次调用 GotoLimit 和 GotoZero）
     * @param dir 校准方向（关节坐标系；1 或 -1）
     * @param spd 校准速度（rad/s）
     * @param cur 校准电流（A）
     * @param timeout 每个阶段的超时时间
     * @return 校准是否成功
     */
    virtual bool Calibrate(int dir, float spd = 2.f, float cur = 2.f,
                           std::chrono::milliseconds timeout = std::chrono::milliseconds{5000}) = 0;

    /**
     * @brief 获取关节状态
     * @param autoSendCommand 为 true 时，若缓存状态不可用则发送一次反馈命令，本次仍返回空
     * @param lifeCycleDeduction 本次读取扣减的生命周期值，<=0 时不扣减
     * @return 关节状态，如果不可用则返回空
     * @warning 关节运动过程中不得将 autoSendCommand 设为 true；缓存状态不可用时发送的
     *          反馈命令可能改变关节运动状态。
     */
    virtual std::optional<JointStatus> GetStatus(bool autoSendCommand = false,
                                                 int lifeCycleDeduction = 1) = 0;

    /**
     * @brief 注册关节状态回调
     *
     * 每当底层电机接收到新状态数据时，回调将被调用。
     * 传入 nullptr 可取消注册。
     * @param onStatus 回调函数，参数为最新的 JointStatus（已应用坐标反转）
     */
    virtual void SetOnStatus(std::function<void(const JointStatus&)> onStatus) = 0;

    /**
     * @brief 设置位置限位
     * @param limit 新的限位范围（关节坐标系）
     */
    virtual void SetLimit(Range<float> limit) = 0;

protected:
    /**
     * @brief 验证关节位置限位是否有限且有序。
     * @param limit 待验证的位置限位范围。
     * @throw std::invalid_argument 当限位包含非有限值或 min > max 时抛出。
     */
    static void ValidateLimit(Range<float> limit) {
        ValidateOrderedRange(limit);
        if (limit.max - limit.min < 2.0f * kLimitSlack) {
            throw std::invalid_argument("Joint limit must contain the configured safety slack");
        }
    }

    /**
     * @brief 验证普通范围是否有限且有序，不附加关节安全余量要求。
     * @param range 待验证的范围。
     * @throw std::invalid_argument 当范围包含非有限值或 min > max 时抛出。
     */
    static void ValidateOrderedRange(Range<float> range) {
        if (!std::isfinite(range.min) || !std::isfinite(range.max) || range.min > range.max) {
            throw std::invalid_argument("Range must be finite and ordered");
        }
    }

    /**
     * @brief 验证并保存关节位置限位。
     * @param limit 关节坐标系下的新限位；reverse 为 true 时内部转换为反向坐标存储。
     * @throw std::invalid_argument 当限位无效时抛出。
     */
    void SetValidatedLimit(Range<float> limit) {
        ValidateLimit(limit);
        if (reverse_) {
            limit_ = Range<float>{-limit.max, -limit.min};
        } else {
            limit_ = limit;
        }
    }

    /**
     * @brief 输出一次性保护告警：每次调用都输出。
     * @param mode 触发保护的控制模式。
     * @param reason 保护原因。
     * @note 仅用于「抛异常的一次性故障点」——这类故障会终止本次控制流，逐次留痕更有价值。
     *       条件会在控制循环中持续成立的点请改用 LogProtection()。
     */
    void WarnProtection(const char* mode, const char* reason) const {
        protection_logger_->warn("Joint protection: joint={}, mode={}, reason={}", protection_name_,
                                 mode, reason);
    }

    /**
     * @brief 电平型保护告警：只在进入保护（warn）与退出保护（info）时输出。
     * @param mode 触发保护的控制模式。
     * @param reason 保护原因；同时作为该错误点标志位的键，必须是静态字符串字面量。
     * @param active 本次判断该保护是否成立。
     * @param detail 进入保护时附带的诊断信息。
     * @param scope 作用域名（roll/pitch/motor1/motor2），空表示关节级保护点。双轴关节的
     *        两条轴、两台电机在同一帧里会各自调用同一个辅助函数，同名保护点若不加作用域
     *        会互相清除标志位；作用域只影响标志位键，不影响日志文本。
     * @return true 表示本次输出了日志。
     */
    bool LogProtection(const char* mode, const char* reason, bool active,
                       const ProtectionLogDetail& detail = {}, const char* scope = nullptr) const {
        const auto transition = UpdateProtection(scope, reason, active);
        if (transition == ProtectionLogPoint::Transition::None) {
            return false;
        }
        if (transition == ProtectionLogPoint::Transition::Entered) {
            LogProtectionEnter(protection_logger_, protection_name_, mode, reason, scope, detail);
        } else {
            LogProtectionExit(protection_logger_, protection_name_, mode, reason, scope);
        }
        return true;
    }

    /**
     * @brief 电平型保护告警，进入保护时使用调用方给定的消息格式。
     * @tparam Args 进入保护消息的格式化参数类型。
     * @param mode 触发保护的控制模式。
     * @param reason 保护原因；同时作为该错误点标志位的键，必须是静态字符串字面量。
     * @param active 本次判断该保护是否成立。
     * @param fmt 进入保护时的格式串，需自带 state=enter 字段。
     * @param args fmt 的格式化参数；只在进入保护时求值。
     * @return true 表示本次输出了日志。
     */
    template <typename... Args>
    bool LogProtectionMessage(const char* mode, const char* reason, bool active, const char* fmt,
                              Args&&... args) const {
        const auto transition = UpdateProtection(nullptr, reason, active);
        if (transition == ProtectionLogPoint::Transition::None) {
            return false;
        }
        if (transition == ProtectionLogPoint::Transition::Entered) {
            protection_logger_->warn(fmt, std::forward<Args>(args)...);
        } else {
            LogProtectionExit(protection_logger_, protection_name_, mode, reason);
        }
        return true;
    }

    float ClampControl(float value, float lower, float upper, const char* mode, const char* field,
                       const char* scope = nullptr) const {
        const float result = std::clamp(value, lower, upper);
        LogProtection(mode, field, result != value, ProtectionLogDetail{field, value, result},
                      scope);
        return result;
    }

    /**
     * @brief 取错误点标志位并更新。
     * @param scope 作用域名，可为空。
     * @param reason 保护原因；必须是静态字符串字面量，与 scope 一起构成标志位键。
     * @param active 本次判断该保护是否成立。
     * @return 状态跳变类型；None 表示状态未变、无需输出日志。
     * @note 标志位表可能被多个线程触及（例如 ZeroDampingRJoint 的工作线程与用户线程的
     *       GetStatus），因此用互斥量保护；锁内只做标志位更新，日志在锁外输出。
     */
    ProtectionLogPoint::Transition UpdateProtection(const char* scope, const char* reason,
                                                    bool active) const {
        const std::lock_guard<std::mutex> lock(protection_points_mutex_);
        const std::pair<std::string_view, std::string_view> key{
            scope != nullptr ? std::string_view{scope} : std::string_view{},
            std::string_view{reason}};
        return protection_points_[key].Update(active);
    }

    LoggerPtr protection_logger_{JointSdkLogger::Get()};
    std::string protection_name_{"unmanaged"};
    /// 每个关节每个错误点一个标志位：键为（作用域，保护原因），两者均为静态字面量；
    /// 由 protection_points_mutex_ 保护
    mutable std::map<std::pair<std::string_view, std::string_view>, ProtectionLogPoint>
        protection_points_;
    mutable std::mutex protection_points_mutex_;
    Range<float> limit_{};  ///< 位置限位范围；无限位关节不使用
    float maxSpd_{0.0f};    ///< 最大速度
    float maxTor_{0.0f};    ///< 最大扭矩
    bool reverse_;          ///< 是否反转方向
};

using BaseJointPtr = std::shared_ptr<BaseJoint>;

/// 适配器名称到实例的映射类型
using AdapterMap = std::map<std::string, BaseAdapterPtr>;

/**
 * @brief Range<T> 的 JSON 反序列化函数
 * @tparam T 数值类型
 * @param j 包含 min/max 字段的 JSON 对象。
 * @param r 输出的范围对象。
 */
template <typename T>
void from_json(const nlohmann::json& j, Range<T>& r) {
    j.at("min").get_to(r.min);
    j.at("max").get_to(r.max);
}

/**
 * @brief MotorInitModel 的 JSON 反序列化函数
 * @param j 包含 model 字段的 JSON 对象。
 * @param m 输出的电机型号初始化参数。
 */
inline void from_json(const nlohmann::json& j, MotorInitModel& m) {
    m.model = StringToMotorModel(j.at("model").get<std::string>());
}

/**
 * @brief MotorInitParams 的 JSON 反序列化函数
 * @param j 包含 ranges 字段的 JSON 对象。
 * @param p 输出的自定义 PVT 范围初始化参数。
 */
inline void from_json(const nlohmann::json& j, MotorInitParams& p) {
    auto rangesJson = j.at("ranges");
    rangesJson.at("kp").get_to(p.ranges.kp);
    rangesJson.at("kd").get_to(p.ranges.kd);
    rangesJson.at("position").get_to(p.ranges.position);
    rangesJson.at("speed").get_to(p.ranges.speed);
    rangesJson.at("torque").get_to(p.ranges.torque);
    rangesJson.at("current").get_to(p.ranges.current);
    rangesJson.at("kt").get_to(p.ranges.kt);
}

/**
 * @brief MotorInit 变体类型的 JSON 反序列化函数
 * @param j 电机初始化 JSON 对象；空对象表示自动初始化。
 * @param i 输出的初始化参数变体。
 * @throw nlohmann::json::type_error 当 j 不是对象时抛出。
 */
inline void from_json(const nlohmann::json& j, MotorInit& i) {
    if (!j.is_object()) {
        throw nlohmann::json::type_error::create(302, "MotorInit must be a JSON object", &j);
    }
    if (j.contains("ranges")) {
        i = j.get<MotorInitParams>();
    } else if (j.contains("model")) {
        i = j.get<MotorInitModel>();
    } else {
        i = MotorInitAuto{};
    }
}

/**
 * @brief MotorParams 的 JSON 反序列化函数
 * @param j 包含 adapterId、busId、motorId、initParam 字段的 JSON 对象；
 *          adapterId 为字符串时按适配器名称解析，为非负整数时按建立顺序索引解析；
 *          slaveId、canfd 为可缺省字段，缺失时保持默认值 0 与 false。
 * @param p 输出的电机参数配置。
 * @throw nlohmann::json::type_error 当 adapterId 既不是字符串也不是整数时抛出。
 * @throw std::invalid_argument 当 adapterId 为负数或超出 uint32 范围时抛出。
 */
inline void from_json(const nlohmann::json& j, MotorParams& p) {
    const auto& adapter_ref = j.at("adapterId");
    if (adapter_ref.is_string()) {
        p.adapterId = adapter_ref.get<std::string>();
    } else if (adapter_ref.is_number_unsigned()) {
        const auto index = adapter_ref.get<std::uint64_t>();
        if (index > std::numeric_limits<std::uint32_t>::max()) {
            throw std::invalid_argument("MotorParams adapterId index out of range: " +
                                        std::to_string(index));
        }
        p.adapterId = static_cast<std::uint32_t>(index);
    } else if (adapter_ref.is_number_integer()) {
        const auto index = adapter_ref.get<std::int64_t>();
        if (index < 0 ||
            index > static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max())) {
            throw std::invalid_argument("MotorParams adapterId index out of range: " +
                                        std::to_string(index));
        }
        p.adapterId = static_cast<std::uint32_t>(index);
    } else {
        throw nlohmann::json::type_error::create(
            302, "MotorParams adapterId must be a string name or an integer index", &adapter_ref);
    }
    j.at("busId").get_to(p.busId);
    j.at("motorId").get_to(p.motorId);
    j.at("initParam").get_to(p.initParam);
    if (j.contains("slaveId")) {
        j.at("slaveId").get_to(p.slaveId);
    }
    if (j.contains("canfd")) {
        j.at("canfd").get_to(p.canfd);
    }
}

}  // namespace encos
