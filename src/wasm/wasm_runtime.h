// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "JointManager.h"
#include "adapter/base_adapter.h"
#include "adapter/fake_adapter_control.h"
#include "encos_motor.h"
#include "external/json.hpp"
#include "platform/log.h"
#include "plugins/adapter/fake/fake_adapter.h"

namespace encos {
namespace wasm {

class WasmJointManager : public DefaultJointManager {
public:
    WasmJointManager() = default;

    void InjectAdapter(const std::string& name, BaseAdapterPtr adapter) {
        TrackAdapter(name, std::move(adapter));
    }

    /**
     * @brief 按 MotorParams 的适配器引用解析已登记的适配器
     * @param reference 适配器引用，整数为建立顺序索引，字符串为名称。
     * @return 命中的适配器；未命中返回空指针。
     */
    BaseAdapterPtr FindAdapter(const AdapterReference& reference) const {
        const auto* adapter = LookupAdapter(reference, adapters_, adapters_by_id_);
        return adapter != nullptr ? *adapter : nullptr;
    }
};

struct JointEntry;

/**
 * @brief ManagerEntry 持有 WasmJointManager 及其适配器/ Fake 资源。
 */
struct ManagerEntry {
    using FakeControlHandle = decltype(std::declval<BaseAdapter*>()->GetFakeAdapterControl());

    bool valid = true;
    std::shared_ptr<WasmJointManager> manager;
    std::string adapter_name;
    std::string adapter_type;
    BaseAdapterPtr adapter;
    FakeAdapter* fake_adapter = nullptr;
    FakeControlHandle fake_control{};
    std::unordered_map<std::string, FakeControlHandle> fake_controls;
    std::vector<std::uint32_t> joint_handles;
    std::unordered_map<std::string, std::uint32_t> joint_handles_by_name;
    std::vector<FakeCommandRecord> command_records;
};

/**
 * @brief JointEntry 是 manager 下属关节的句柄包装。
 */
struct JointEntry {
    bool valid = true;
    std::uint32_t manager_handle = 0;
    std::string joint_name;
    BaseJointPtr joint;
    void (*status_callback)(const char*) = nullptr;
};

/**
 * @brief WASM 层错误码。
 */
enum class WasmErrorCode : int {
    kOk = 0,
    kInvalidArgument = 1,
    kNotFound = 2,
    kAlreadyExists = 3,
    kInternal = 4,
    kJointError = 5,
    kWrongAdapter = 6,
};

/**
 * @brief 最近一次错误状态。
 */
struct ErrorState {
    WasmErrorCode code = WasmErrorCode::kOk;
    std::string message;
    std::optional<JointError> joint_error;
};

/**
 * @brief 单例运行时仓库，持有 manager/joint 句柄与错误状态。
 */
class RuntimeStore {
public:
    static RuntimeStore& Instance();

    std::uint32_t CreateEmptyManager();
    std::uint32_t CreateManager(const std::string& adapter_type, const std::string& adapter_name,
                                const std::string& logger_name, LogLevel log_level);
    void CreateAdapter(std::uint32_t manager_handle, const std::string& adapter_type,
                       const std::string& adapter_name, const std::string& logger_name,
                       LogLevel log_level, const nlohmann::json& extend);
    void DisposeManager(std::uint32_t handle);
    ManagerEntry* GetManager(std::uint32_t handle);
    WasmJointManager* GetManagerObject(std::uint32_t handle);

    std::uint32_t AddJoint(std::uint32_t manager_handle, const nlohmann::json& definition);
    std::uint32_t RegisterJoint(std::uint32_t manager_handle, const std::string& name,
                                BaseJointPtr joint);
    JointEntry* GetJoint(std::uint32_t handle);

    void SetLastError(WasmErrorCode code, const std::string& message,
                      std::optional<JointError> joint_error = std::nullopt);
    ErrorState GetLastError() const;
    void ClearLastError();

    std::string Version() const;

private:
    RuntimeStore() = default;

    std::uint32_t NextHandle();
    void ClearJointCallbacks(JointEntry* entry);

    std::uint32_t next_handle_ = 1;
    std::unordered_map<std::uint32_t, std::unique_ptr<ManagerEntry>> managers_;
    std::unordered_map<std::uint32_t, std::unique_ptr<JointEntry>> joints_;
    ErrorState last_error_{WasmErrorCode::kOk, ""};
};

}  // namespace wasm
}  // namespace encos
