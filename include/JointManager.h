// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include "BaseJoint.h"
#include "DoubleCoupleJoint/DoubleCoupleJoint.h"
#include "RJoint/ContinuousJoint.h"
#include "RJoint/RJoint.h"
#include "adapter/fake_adapter_control.h"
#include "bus/bus.h"
#include "encos/encos_driver.h"
#include "encos_motor.h"
#include "external/json.hpp"

namespace encos {

/**
 * @brief 关节管理器基类
 *
 * 管理适配器和关节实例，提供关节的创建、获取和控制功能
 */
template <typename... JointTypes>
class JointManager;

/**
 * @brief 解析适配器创建配置 JSON
 *
 * 接受形如 {"type": "EthercatIGH", "name": "0", "extend": {...}} 的对象。
 * @param j 适配器创建配置。
 * @param type 输出的适配器类型。
 * @param name 输出的适配器名称。
 * @param extend 输出的适配器扩展配置；字段缺省或为 null 时输出空对象。
 * @throw std::invalid_argument 当 j 不是对象、缺少 type/name、三者类型错误或 extend
 *        不是对象时抛出。
 */
inline void ParseAdapterDefinition(const nlohmann::json& j, std::string& type, std::string& name,
                                   nlohmann::json& extend) {
    if (!j.is_object()) {
        throw std::invalid_argument("Adapter JSON must be an object");
    }
    const auto it_type = j.find("type");
    if (it_type == j.end() || !it_type->is_string()) {
        throw std::invalid_argument("Adapter JSON must contain a string 'type' field");
    }
    const auto it_name = j.find("name");
    if (it_name == j.end() || !it_name->is_string()) {
        throw std::invalid_argument("Adapter JSON must contain a string 'name' field");
    }
    type = it_type->get<std::string>();
    name = it_name->get<std::string>();
    const auto it_extend = j.find("extend");
    if (it_extend == j.end() || it_extend->is_null()) {
        extend = nlohmann::json::object();
        return;
    }
    if (!it_extend->is_object()) {
        throw std::invalid_argument("Adapter JSON 'extend' must be an object");
    }
    extend = *it_extend;
}

/**
 * @brief 解析适配器创建配置 JSON（不关心扩展配置的重载）
 * @param j 适配器创建配置。
 * @param type 输出的适配器类型。
 * @param name 输出的适配器名称。
 * @throw std::invalid_argument 语义同四参数重载。
 */
inline void ParseAdapterDefinition(const nlohmann::json& j, std::string& type, std::string& name) {
    nlohmann::json extend;
    ParseAdapterDefinition(j, type, name, extend);
}

/**
 * @brief 关节管理器特化（空参数包）
 */
template <>
class JointManager<> {
public:
    /**
     * @brief 创建适配器
     * @param adapterType 适配器类型
     * @param adapterName 适配器名称
     * @throw std::invalid_argument 当指定类型的适配器无法创建时抛出。
     */
    void CreateAdapter(const std::string& adapterType, const std::string& adapterName) {
        CreateAdapter(adapterType, adapterName, nlohmann::json::object());
    }

    /**
     * @brief 创建适配器并传入扩展配置
     * @param adapterType 适配器类型
     * @param adapterName 适配器名称
     * @param extend 适配器扩展配置，字段语义由插件自行定义；空对象表示不传扩展配置。
     * @throw std::invalid_argument 当指定类型的适配器无法创建时抛出。
     * @throw std::runtime_error 当 extend 不是 JSON 对象、插件缺失或插件构造失败时抛出。
     *
     * 适配器身份只由 adapterName 决定：
     * - 名字未创建过：按 extend 创建适配器，只有导出 `MakeAdapterV2` 的插件会收到该配置；
     * - 名字已存在且 extend 为空：保持既有幂等语义，直接返回已登记的实例；
     * - 名字已存在且 extend 非空：仍回到驱动器，由它把 extend 中的 `buses` 条目经
     *   `BaseAdapter::UpdateBusExtend` 增量应用到既有实例（顶层公共参数无法重新应用，
     *   会被忽略并记录诊断），既有实例的登记不变。
     */
    void CreateAdapter(const std::string& adapterType, const std::string& adapterName,
                       const nlohmann::json& extend) {
        const bool already_registered = adapters_.find(adapterName) != adapters_.end();
        if (already_registered && extend.empty()) {
            return;
        }
        auto adapter = MakeAdapter(adapterType, adapterName, adapterName, log_level_, extend);
        if (!adapter) {
            throw std::invalid_argument("Cannot create adapter: " + adapterName);
        }
        if (already_registered) {
            // 既有实例已在注册表中，本次调用只为把 extend 增量应用到它
            return;
        }
        TrackAdapter(adapterName, std::move(adapter));
    }

    /**
     * @brief 按 JSON 配置创建适配器
     * @param adapterJson 形如 {"type": "EthercatIGH", "name": "0", "extend": {...}} 的对象
     * @throw std::invalid_argument 当 JSON 非法或指定类型的适配器无法创建时抛出。
     * @throw std::runtime_error 当 extend 不是 JSON 对象、插件缺失或插件构造失败时抛出。
     */
    void CreateAdapter(const nlohmann::json& adapterJson) {
        std::string adapterType;
        std::string adapterName;
        nlohmann::json extend;
        ParseAdapterDefinition(adapterJson, adapterType, adapterName, extend);
        CreateAdapter(adapterType, adapterName, extend);
    }

    /**
     * @brief 设置关节保护日志和后续创建适配器使用的日志级别
     *
     * 已创建的适配器不受影响。默认 SDK 日志器是共享的，可通过 SetLogger 单独指定。
     * @param level 日志级别
     */
    void SetLogLevel(LogLevel level) {
        log_level_ = level;
        logger_->SetLevel(level);
    }

    /** @brief 获取保护日志器（默认 SDK 共享日志器）；所有关节保护 warning 使用此日志器。 */
    LoggerPtr GetLogger() const {
        return logger_;
    }

    /** @brief 为已有及后续关节设置保护日志器。 */
    void SetLogger(LoggerPtr logger) {
        if (!logger)
            throw std::invalid_argument("JointManager logger must not be null");
        logger_ = std::move(logger);
        logger_->SetLevel(log_level_);
        for (const auto& [name, joint] : joints_) {
            (void) name;
            joint->SetProtectionLogContext(logger_, joint->GetProtectionName());
        }
    }

    /**
     * @brief 查询当前创建适配器时使用的日志级别
     * @return 当前日志级别
     */
    LogLevel GetLogLevel() const {
        return log_level_;
    }

    /**
     * @brief 为当前及后续创建的关节启用电机与关节级状态/命令文件日志
     *
     * 单适配器时日志基础名为 s<slaveId>b<busId>m<motorId>；多适配器时增加
     * <adapterName>_ 前缀。底层 driver 会分别追加 _status.csv.zstd 和
     * _command.csv.zstd 后缀；每个关节还会创建独立的 <jointName>_command.csv.zstd 和
     * <jointName>_status.csv.zstd，所有关节保护告警写入 joint_protection.csv.zstd。
     * @param path 日志文件输出目录；目录不存在时自动创建。
     */
    void EnableLog(const std::filesystem::path& path) {
        if (path.empty()) {
            throw std::invalid_argument("Log path must not be empty");
        }
        const auto absolute_path = std::filesystem::absolute(path).lexically_normal();
        std::filesystem::create_directories(absolute_path);
        if (!std::filesystem::is_directory(absolute_path)) {
            throw std::invalid_argument("Log path is not a directory: " + absolute_path.string());
        }

        auto joint_log = std::make_shared<JointLogContext>(absolute_path);
        log_path_ = absolute_path;
        joint_log_context_ = std::move(joint_log);
        for (const auto& [name, joint] : joints_) {
            joint->SetJointLogContext(joint_log_context_, name);
        }
        EnableTrackedMotorLogs(adapters_.size() > 1);
    }

    /**
     * @brief 刷新并关闭当前管理器全部关节电机的文件日志
     *
     * 底层 Motor 的异常直接向调用方传播。
     */
    void DisableLog() {
        for (const auto& [_, motor] : motors_) {
            motor->DisableLog();
        }
        for (const auto& [_, joint] : joints_) {
            joint->SetJointLogContext(nullptr, {});
        }
        joint_log_context_.reset();
        log_path_.clear();
    }

    /**
     * @brief 查询当前管理器是否已为全部关节电机启用文件日志
     * @return 存在 Motor 且全部 Motor 均处于记录状态时返回 true。
     */
    bool IsLogged() const {
        if (motors_.empty()) {
            return false;
        }
        for (const auto& [_, motor] : motors_) {
            if (!motor->IsLogged()) {
                return false;
            }
        }
        return static_cast<bool>(joint_log_context_);
    }

    /**
     * @brief 根据 MotorParams 和适配器映射创建电机实例
     *
     * 从传入的适配器映射中查找 adapter → bus → motor。
     * @param motor_params 电机参数配置
     * @param adapters 适配器名称到实例的映射
     * @return 创建或取得的电机实例。
     * @throw std::invalid_argument 当适配器、总线或电机不存在时抛出。
     */
    static Motor* CreateMotor(const MotorParams& motor_params, const AdapterMap& adapters) {
        return CreateMotor(motor_params, adapters, AdapterMap{});
    }

    /**
     * @brief 根据 MotorParams 和两份适配器索引创建电机实例
     *
     * MotorParams::adapterId 的取值类型决定查找哪份索引：整数查建立顺序 id，
     * 字符串查适配器名称。
     * @param motor_params 电机参数配置
     * @param adapters 适配器名称到实例的映射
     * @param adapters_by_id 适配器建立顺序 id 到实例的映射
     * @return 创建或取得的电机实例。
     * @throw std::invalid_argument 当适配器、总线或电机不存在时抛出。
     */
    static Motor* CreateMotor(const MotorParams& motor_params, const AdapterMap& adapters,
                              const AdapterMap& adapters_by_id) {
        const auto* adapter = LookupAdapter(motor_params.adapterId, adapters, adapters_by_id);
        if (!adapter) {
            throw std::invalid_argument(AdapterLookupError(motor_params));
        }
        if (!*adapter) {
            throw std::invalid_argument("Adapter is null: " + motor_params.AdapterIdString());
        }
        const auto bus_index = motor_params.BusIndex();
        auto bus = (*adapter)->GetBus(bus_index);
        if (!bus) {
            throw std::invalid_argument("Cannot find bus: " + std::to_string(bus_index));
        }
        auto motor = std::visit(
            [&bus, &motor_params](auto&& arg) -> Motor* {
                using T = std::decay_t<decltype(arg)>;
                if constexpr (std::is_same_v<T, MotorInitAuto>) {
                    return RawMotor(bus->GetMotor(motor_params.motorId));
                } else if constexpr (std::is_same_v<T, MotorInitModel>) {
                    return RawMotor(bus->GetMotor(motor_params.motorId, arg.model));
                } else if constexpr (std::is_same_v<T, MotorInitParams>) {
                    return RawMotor(bus->GetMotor(motor_params.motorId, arg.ranges));
                } else {
                    static_assert(AlwaysFalse<T>::value, "Unsupported motor init type");
                    return nullptr;
                }
            },
            motor_params.initParam);
        if (!motor) {
            throw std::invalid_argument("Cannot find motor: " +
                                        std::to_string(motor_params.motorId));
        }
        if (motor_params.canfd) {
            motor->EnableCanFd();
        }
        return motor;
    }

private:
    template <typename MotorHandle>
    static Motor* RawMotor(MotorHandle&& motor) {
        if constexpr (std::is_pointer_v<std::decay_t<MotorHandle>>) {
            return motor;
        } else {
            return motor.get();
        }
    }

public:
    /**
     * @brief 全局启用所有关节电机的 CAN FD 帧标志
     *
     * 对已创建的所有关节电机以及后续新创建的电机均生效。
     */
    void EnableJointMotorCanfd() {
        motor_canfd_override_ = true;
        motor_canfd_enabled_ = true;
        for (auto& [_, joint] : joints_) {
            SetJointMotorCanFd(joint, true);
        }
    }

    /**
     * @brief 全局禁用所有关节电机的 CAN FD 帧标志
     *
     * 对已创建的所有关节电机以及后续新创建的电机均生效。
     */
    void DisableJointMotorCanfd() {
        motor_canfd_override_ = true;
        motor_canfd_enabled_ = false;
        for (auto& [_, joint] : joints_) {
            SetJointMotorCanFd(joint, false);
        }
    }

    /**
     * @brief 查询当前全局 CAN FD 是否已启用
     *
     * 仅表示全局开关自身的状态，不代表每台电机的实际帧标志；未被显式调用过时
     * 电机按各自 MotorParams::canfd 决定。
     * @return true 表示已启用
     */
    bool IsJointMotorCanfd() const {
        return motor_canfd_enabled_;
    }

    /**
     * @brief 设置当前管理器下所有 Fake 适配器的位置模拟随机误差开关
     *
     * 非 Fake 适配器会被自动跳过。
     * @param enabled true 表示启用随机误差，false 表示禁用。
     */
    void EnableFakePositionError(bool enabled = true) {
        for (auto& [_, adapter] : adapters_) {
            if (!adapter) {
                continue;
            }
            auto fake_control = adapter->GetFakeAdapterControl();
            if (fake_control) {
                fake_control->EnablePositionError(enabled);
            }
        }
    }

    /**
     * @brief 禁用当前管理器下所有 Fake 适配器的位置模拟随机误差
     */
    void DisableFakePositionError() {
        EnableFakePositionError(false);
    }

    /**
     * @brief 添加关节
     * @tparam JointType 关节类型
     * @param jointName 关节名称
     * @param config 关节配置
     * @return 创建的关节实例
     * @throw std::invalid_argument 如果关节名称重复
     * @throw JointException 如果双电机关节配对异常
     */
    template <typename JointType>
    BaseJointPtr AddJoint(const std::string& jointName, const typename JointType::Config& config) {
        if (joints_.find(jointName) != joints_.end()) {
            throw std::invalid_argument("Joint with name '" + jointName + "' already exists");
        }
        if constexpr (std::is_base_of_v<DoubleCoupleJoint, JointType>) {
            auto key = DoubleCoupleJoint::MakePairKey(config.motorParams1, config.motorParams2);
            if (key.empty()) {
                throw JointException(JointError::kDoubleCoupleJointSameMotor,
                                     "DoubleCoupleJoint config: motorParams1 and motorParams2 "
                                     "refer to the same physical motor");
            }

            if (active_pairs_.find(key) != active_pairs_.end()) {
                throw JointException(JointError::kDoubleCoupleJointDuplicatePair,
                                     "DoubleCoupleJoint pair already exists for key: " + key);
            }

            auto it = pending_pairs_.find(key);
            if (it == pending_pairs_.end()) {
                auto slave = DoubleCoupleJoint::CreateJoint(config);
                slave->SetProtectionLogContext(logger_, jointName);
                ConfigureJointLog(slave, jointName);
                joints_[jointName] = slave;
                pending_pairs_[key] = std::static_pointer_cast<SlaveJoint>(slave);
                return slave;
            }

            auto pendingSlave = it->second;
            DoubleCoupleJoint::ValidatePair(config, pendingSlave);
            auto motor1 = CreateMotor(config.motorParams1, adapters_, adapters_by_id_);
            auto motor2 = CreateMotor(config.motorParams2, adapters_, adapters_by_id_);
            ApplyMotorCanFd(motor1, config.motorParams1.canfd);
            ApplyMotorCanFd(motor2, config.motorParams2.canfd);
            auto master = DoubleCoupleJoint::CreateJoint(config, motor1, motor2, pendingSlave);
            master->SetProtectionLogContext(logger_,
                                            jointName + "/" + pendingSlave->GetProtectionName());
            ConfigureJointLog(master, jointName);
            RegisterMotor(config.motorParams1, motor1);
            RegisterMotor(config.motorParams2, motor2);
            pending_pairs_.erase(it);
            active_pairs_.insert(key);
            joints_[jointName] = master;
            return master;
        } else {
            auto motor = CreateMotor(config.motorParams, adapters_, adapters_by_id_);
            ApplyMotorCanFd(motor, config.motorParams.canfd);
            auto joint = std::make_shared<JointType>(config, motor);
            joint->SetProtectionLogContext(logger_, jointName);
            ConfigureJointLog(joint, jointName);
            RegisterMotor(config.motorParams, motor);
            joints_[jointName] = joint;
            return joint;
        }
    }

    /**
     * @brief 获取关节
     * @param jointName 关节名称
     * @return 关节实例，如果不存在则返回nullptr
     */
    BaseJointPtr GetJoint(const std::string& jointName) {
        auto jointIt = joints_.find(jointName);
        if (jointIt == joints_.end()) {
            return nullptr;
        }
        return jointIt->second;
    }

    /**
     * @brief 获取所有关节状态
     * @param autoSendCommand 为 true 时，若单个关节缓存状态不可用则发送一次反馈命令，本次忽略该关节
     * @param lifeCycleDeduction 本次读取扣减的生命周期值，<=0 时不扣减
     * @todo 后续实现双关节引用时这里逻辑可能还需要调整
     * @return 关节名称到状态的映射
     * @warning 任一关节运动过程中均不得将 autoSendCommand 设为 true；缓存状态不可用时发送的
     *          反馈命令可能改变关节运动状态。
     */
    std::map<std::string, JointStatus> GetStatus(bool autoSendCommand = false,
                                                 int lifeCycleDeduction = 1) {
        std::map<std::string, JointStatus> res;
        for (auto& [name, joint] : joints_) {
            auto status = joint->GetStatus(autoSendCommand, lifeCycleDeduction);
            if (status) {
                res[name] = *status;
            }
        }
        return res;
    }

    /**
     * @brief 提交所有已创建关节电机所使用总线上的待发送消息。
     *
     * 每条总线在一次调用中只提交一次；尚未创建电机时不执行任何操作。
     */
    void Commit() {
        for (auto* bus : buses_) {
            bus->Commit();
        }
    }

    /**
     * @brief 设置当前管理器已创建关节所使用总线的软同步发送模式
     *
     * @param enabled true 时消息等待 Commit 批量发送；false 时恢复直接发送。
     * 对已登记的总线及后续创建关节时新登记的总线生效，不修改未使用总线。
     * 保留接口名称以兼容已有调用方。
     */
    void SetAdaptersSyncMode(bool enabled) {
        sync_mode_enabled_ = enabled;
        for (auto* bus : buses_) {
            bus->SetSyncMode(enabled);
        }
    }

    /**
     * @brief 设置当前管理器下所有适配器的状态最大生命周期
     * @param maxLifeCycle 最大生命周期，INT_MAX 表示禁用生命周期清理
     */
    void SetAdaptersStatusMaxLifeCycle(int maxLifeCycle) {
        for (auto& [_, adapter] : adapters_) {
            adapter->SetMaxStatusLifeCycle(maxLifeCycle);
        }
    }

    /**
     * @brief 位置控制
     * @param positions 关节名称到目标位置的映射
     */
    void PosControl(const std::map<std::string, float>& positions) {
        for (const auto& [name, pos] : positions) {
            auto jointIt = joints_.find(name);
            if (jointIt == joints_.end()) {
                continue;
            }
            jointIt->second->PosControl(pos);
        }
    }

    /**
     * @brief 为指定关节注册状态回调
     *
     * 每当底层电机接收到新状态数据时，回调将被调用，参数已转换至关节坐标系。
     * 传入 nullptr 可取消注册。
     * @param jointName 关节名称
     * @param onStatus 状态回调函数
     */
    void SetOnStatus(const std::string& jointName,
                     std::function<void(const JointStatus&)> onStatus) {
        auto jointIt = joints_.find(jointName);
        if (jointIt == joints_.end()) {
            return;
        }
        jointIt->second->SetOnStatus(std::move(onStatus));
    }

protected:
    struct MotorUniqueId {
        std::string adapter_id;
        int bus_id;
        int motor_id;

        bool operator<(const MotorUniqueId& other) const {
            return std::tie(adapter_id, bus_id, motor_id) <
                   std::tie(other.adapter_id, other.bus_id, other.motor_id);
        }
    };

    /**
     * @brief 登记适配器实例
     *
     * 同时写入名称索引和建立顺序 id 索引。同名重复登记不会新增 id，但会让两个索引
     * 都指向最新实例。
     * @param name 适配器名称
     * @param adapter 适配器实例
     */
    void TrackAdapter(const std::string& name, BaseAdapterPtr adapter) {
        auto id_it = adapter_id_by_name_.find(name);
        if (id_it == adapter_id_by_name_.end()) {
            id_it = adapter_id_by_name_.emplace(name, std::to_string(next_adapter_id_++)).first;
        }
        adapters_by_id_[id_it->second] = adapter;
        adapters_[name] = std::move(adapter);
    }

    /**
     * @brief 按适配器引用查找适配器实例
     *
     * 引用取值类型决定查找的索引：整数按建立顺序 id 索引，字符串按名称索引，
     * 不做跨索引回退。
     * @param adapter_reference 适配器引用
     * @param adapters 适配器名称索引
     * @param adapters_by_id 适配器建立顺序 id 索引
     * @return 指向命中的适配器指针的指针；未命中返回 nullptr。
     */
    static const BaseAdapterPtr* LookupAdapter(const AdapterReference& adapter_reference,
                                               const AdapterMap& adapters,
                                               const AdapterMap& adapters_by_id) {
        if (const auto* index = std::get_if<std::uint32_t>(&adapter_reference)) {
            const auto by_id = adapters_by_id.find(std::to_string(*index));
            if (by_id != adapters_by_id.end()) {
                return &by_id->second;
            }
            return nullptr;
        }
        const auto by_name = adapters.find(std::get<std::string>(adapter_reference));
        if (by_name != adapters.end()) {
            return &by_name->second;
        }
        return nullptr;
    }

    /**
     * @brief 构造适配器查找失败的错误说明
     * @param motor_params 用于生成说明的电机参数配置。
     * @return 区分索引引用与名称引用的错误说明。
     */
    static std::string AdapterLookupError(const MotorParams& motor_params) {
        if (motor_params.AdapterIdIsIndex()) {
            return "Cannot find adapter index: " + motor_params.AdapterIdString();
        }
        return "Cannot find adapter: " + motor_params.AdapterIdString();
    }

    /**
     * @brief 登记已成功构造关节所使用的 Motor 和 Bus。
     * @param motor_params Motor 的物理地址配置。
     * @param motor 已创建的底层 Motor。
     */
    void RegisterMotor(const MotorParams& motor_params, Motor* motor) {
        const auto* adapter = LookupAdapter(motor_params.adapterId, adapters_, adapters_by_id_);
        if (!adapter || !*adapter) {
            throw std::invalid_argument(AdapterLookupError(motor_params));
        }
        auto* bus = (*adapter)->GetBus(motor_params.BusIndex());
        if (buses_.insert(bus).second) {
            bus->SetSyncMode(sync_mode_enabled_);
        }
        MotorUniqueId id{motor_params.AdapterIdentityKey(), motor_params.BusIndex(),
                         motor_params.motorId};
        const auto [it, inserted] = motors_.emplace(std::move(id), motor);
        if (inserted && !log_path_.empty()) {
            motor->EnableLog(MotorLogBaseName(it->first, adapters_.size() > 1).string());
        }
    }

    static std::string FileNameSafeAdapterId(const std::string& adapter_id) {
        constexpr char kHexDigits[] = "0123456789ABCDEF";
        std::string encoded;
        encoded.reserve(adapter_id.size());
        for (const unsigned char ch : adapter_id) {
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
        return encoded;
    }

    std::filesystem::path MotorLogBaseName(const MotorUniqueId& motor_id,
                                           bool multiple_adapters) const {
        if (motor_id.bus_id < 0) {
            throw std::invalid_argument("Motor log bus id must not be negative");
        }
        const auto raw_bus_id = static_cast<std::uint32_t>(motor_id.bus_id);
        const auto slave_id = (raw_bus_id >> 16U) & 0xFFFFU;
        const auto bus_id = raw_bus_id & 0xFFFFU;
        std::string name;
        if (multiple_adapters) {
            name = FileNameSafeAdapterId(motor_id.adapter_id) + "_";
        }
        name += "s" + std::to_string(slave_id) + "b" + std::to_string(bus_id) + "m" +
                std::to_string(motor_id.motor_id);
        return log_path_ / name;
    }

    void EnableTrackedMotorLogs(bool multiple_adapters) {
        for (const auto& [motor_id, motor] : motors_) {
            motor->EnableLog(MotorLogBaseName(motor_id, multiple_adapters).string());
        }
    }

    void ConfigureJointLog(const BaseJointPtr& joint, const std::string& name) {
        if (joint && joint_log_context_) {
            joint->SetJointLogContext(joint_log_context_, name);
        }
    }

    /**
     * @brief 设置单个电机的 CAN FD 帧标志。
     * @param motor 需要设置的电机实例。
     * @param enabled true 表示启用 CAN FD，false 表示禁用。
     * @throw std::invalid_argument 当 motor 为空时抛出。
     */
    static void SetMotorCanFd(Motor* motor, bool enabled) {
        if (!motor) {
            throw std::invalid_argument("Motor must not be null");
        }
        if (enabled) {
            motor->EnableCanFd();
        } else {
            motor->DisableCanFd();
        }
    }

    /**
     * @brief 按电机自身配置与全局开关决定 CAN FD 帧标志
     *
     * 全局开关被显式调用过时以全局开关为准，否则使用 motorParams.canfd。
     * @param motor 需要设置的电机实例。
     * @param params_canfd 该电机 MotorParams::canfd 的取值。
     */
    void ApplyMotorCanFd(Motor* motor, bool params_canfd) {
        SetMotorCanFd(motor, motor_canfd_override_ ? motor_canfd_enabled_ : params_canfd);
    }

    /**
     * @brief 设置关节所持有电机的 CAN FD 帧标志。
     * @param joint 需要设置的关节实例；为空时不执行任何操作。
     * @param enabled true 表示启用 CAN FD，false 表示禁用。
     */
    static void SetJointMotorCanFd(const BaseJointPtr& joint, bool enabled) {
        if (!joint) {
            return;
        }
        if (auto r_joint = std::dynamic_pointer_cast<RJoint>(joint)) {
            SetMotorCanFd(r_joint->motor_, enabled);
            return;
        }
        if (auto continuous_joint = std::dynamic_pointer_cast<ContinuousJoint>(joint)) {
            SetMotorCanFd(continuous_joint->motor_, enabled);
            return;
        }
        if (auto master_joint = std::dynamic_pointer_cast<MasterJoint>(joint)) {
            SetMotorCanFd(master_joint->motor1_, enabled);
            SetMotorCanFd(master_joint->motor2_, enabled);
        }
    }

    std::map<std::string, BaseAdapterPtr> adapters_;        ///< driver-owned adapter pointers
    std::map<std::string, BaseAdapterPtr> adapters_by_id_;  ///< 建立顺序 id → 适配器实例
    std::map<std::string, std::string> adapter_id_by_name_;  ///< 适配器名 → 建立顺序 id
    std::size_t next_adapter_id_{0};              ///< 下一个适配器的建立顺序 id
    std::map<std::string, BaseJointPtr> joints_;  ///< 关节映射
    std::unordered_set<Bus*> buses_;              ///< 已创建电机使用的非拥有总线指针
    std::map<MotorUniqueId, Motor*> motors_;      ///< 物理地址到非拥有 Motor 指针
    std::unordered_map<std::string, std::shared_ptr<SlaveJoint>>
        pending_pairs_;  ///< 待配对双电机关节（key → SlaveJoint）
    std::unordered_set<std::string> active_pairs_;  ///< 已配对的电机对 key
    bool motor_canfd_enabled_{false};               ///< 全局 CAN FD 标志
    bool motor_canfd_override_{false};  ///< 全局 CAN FD 开关是否已被显式调用过
    bool sync_mode_enabled_{false};     ///< 已使用总线及后续登记总线的软同步模式
    LogLevel log_level_{LogLevel::Info};       ///< 创建适配器时使用的日志级别
    LoggerPtr logger_{JointSdkLogger::Get()};  ///< 关节保护共享日志器
    std::filesystem::path log_path_;           ///< Motor 文件日志输出目录
    std::shared_ptr<JointLogContext> joint_log_context_;  ///< 关节级 CSV 日志会话
};

/**
 * @brief 关节管理器模板类
 *
 * 支持多种关节类型的泛化管理器
 * @tparam JointTypes 支持的关节类型列表
 */
template <typename... JointTypes>
class JointManager : public JointManager<> {
public:
    using JointManager<>::JointManager;
    using JointManager<>::AddJoint;

    /** @brief 默认构造：不预置任何适配器与关节。 */
    JointManager() = default;

    /**
     * @brief 由 JSON 配置构造管理器
     *
     * 先按 `adapters` 数组逐个创建适配器，再按 `joints` 数组逐个创建关节，
     * 两个数组均按给定顺序处理。两个键都可缺省；出现但类型不是数组时抛
     * std::invalid_argument。单个适配器/关节的解析失败语义与 CreateAdapter、
     * AddJoint 一致。
     * @param config 形如 {"adapters": [{"type": ..., "name": ...}], "joints":
     *               [{"name": ..., "type": ..., "config": {...}}]} 的对象。
     * @throw std::invalid_argument 当顶层不是对象、adapters/joints 不是数组、
     *        适配器配置非法、关节类型未知或关节名称重复时抛出。
     * @throw JointException 当底层关节创建或配对失败时抛出。
     */
    explicit JointManager(const nlohmann::json& config) {
        if (!config.is_object()) {
            throw std::invalid_argument("JointManager config must be a JSON object");
        }
        if (const auto it = config.find("adapters"); it != config.end()) {
            if (!it->is_array()) {
                throw std::invalid_argument("JointManager config 'adapters' must be an array");
            }
            for (const auto& adapter_json : *it) {
                CreateAdapter(adapter_json);
            }
        }
        if (const auto it = config.find("joints"); it != config.end()) {
            if (!it->is_array()) {
                throw std::invalid_argument("JointManager config 'joints' must be an array");
            }
            for (const auto& joint_json : *it) {
                AddJoint(joint_json);
            }
        }
    }

    /**
     * @brief 基于JSON添加关节
     * @param j 包含type、name、config字段的JSON对象
     * @return 创建的关节实例
     * @throw std::invalid_argument 如果关节类型未知
     * @throw JointException 如果底层关节创建或配对失败
     */
    BaseJointPtr AddJoint(const nlohmann::json& j) {
        std::string type = j.at("type").get<std::string>();
        std::string name = j.at("name").get<std::string>();
        if (joints_.find(name) != joints_.end()) {
            throw std::invalid_argument("Joint with name '" + name + "' already exists");
        }
        if (!((type == JointTypes::Config::kJointType) || ...)) {
            throw std::invalid_argument("Unknown joint type: " + type);
        }
        const auto& configJson = j.at("config");

        BaseJointPtr result = nullptr;
        ((result = TryAddJoint<JointTypes>(type, name, configJson)) || ...);
        return result;
    }

    /**
     * @brief 获取所有可用的关节类型名称
     * @return 关节类型名称列表
     */
    std::vector<std::string> GetAvailableJointTypes() {
        return {std::string(JointTypes::Config::kJointType)...};
    }

    /**
     * @brief 获取所有关节类型的 JSON Schema
     * @return 关节类型名称到 JSON Schema 字符串的映射
     */
    std::map<std::string, std::string> GetJointSchema() {
        return {
            {std::string(JointTypes::Config::kJointType), JointTypes::Config::ToJsonSchema()}...};
    }

private:
    /**
     * @brief 尝试添加指定类型的关节
     * @tparam JointType 关节类型
     * @param type 类型名称
     * @param name 关节名称
     * @param configJson 配置JSON
     * @return 创建的关节实例，如果类型不匹配则返回nullptr
     */
    template <typename JointType>
    BaseJointPtr TryAddJoint(const std::string& type, const std::string& name,
                             const nlohmann::json& configJson) {
        if (type == JointType::Config::kJointType) {
            auto config = JointType::Config::FromJson(configJson);
            return JointManager<>::AddJoint<JointType>(name, config);
        }
        return nullptr;
    }
};

/// 默认关节管理器类型（支持 RJoint、ContinuousJoint 和 DoubleCoupleJoint）
using DefaultJointManager = JointManager<RJoint, ContinuousJoint, DoubleCoupleJoint>;

}  // namespace encos
