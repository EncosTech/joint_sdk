// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

#include "src/wasm/wasm_runtime.h"

#include <nlohmann/json.hpp>

#include "motor/motor_model_generated.h"

namespace encos {
namespace wasm {

namespace {

/**
 * @brief 取出适配器句柄的裸指针
 *
 * 驱动在不同构建配置下把 BaseAdapterPtr 定义为 shared_ptr 或裸指针，这里统一成
 * 裸指针供 dynamic_cast 使用（与 JointManager<>::RawMotor 处理 Motor 句柄的方式一致）。
 */
template <typename AdapterHandle>
BaseAdapter* RawAdapter(AdapterHandle&& adapter) {
    if constexpr (std::is_pointer_v<std::decay_t<AdapterHandle>>) {
        return adapter;
    } else {
        return adapter.get();
    }
}

void SeedFakeMotor(FakeAdapter* fake, const nlohmann::json& motor_params) {
    if (!fake) {
        return;
    }
    const auto params = motor_params.get<MotorParams>();
    std::visit(
        [&](const auto& init) {
            using Init = std::decay_t<decltype(init)>;
            if constexpr (std::is_same_v<Init, MotorInitModel>) {
                fake->SeedMotor(params.BusIndex(), params.motorId, init.model);
            } else if constexpr (std::is_same_v<Init, MotorInitParams>) {
                auto snapshot = fake->CreateSnapshot(MotorModel::EC_A4310_P2);
                snapshot.ranges = init.ranges;
                snapshot.kt = init.ranges.kt;
                fake->SeedMotor(params.BusIndex(), params.motorId, snapshot);
            }
        },
        params.initParam);
}

void SeedFakeMotors(ManagerEntry* entry, const nlohmann::json& definition) {
    if (!entry || !definition.contains("config")) {
        return;
    }
    const auto& config = definition.at("config");
    for (const char* key : {"motorParams", "motorParams1", "motorParams2"}) {
        if (!config.contains(key)) {
            continue;
        }
        const auto params = config.at(key).get<MotorParams>();
        auto* fake =
            dynamic_cast<FakeAdapter*>(RawAdapter(entry->manager->FindAdapter(params.adapterId)));
        if (fake != nullptr) {
            SeedFakeMotor(fake, config.at(key));
        }
    }
}

constexpr std::size_t kMaxFakeCommandRecords = 100000;

nlohmann::json ToJson(const JointStatus& s) {
    return nlohmann::json{
        {"pos", s.pos},
        {"spd", s.spd},
        {"tor", s.tor},
        {"mosTemperature", s.mosTemperature},
        {"motorTemperature", s.motorTemperature},
        {"error", static_cast<int>(s.error)},
    };
}

std::string JointErrorToString(JointError error) {
    switch (error) {
        case JointError::kDoubleCoupleJointNotPaired:
            return "DoubleCoupleJointNotPaired";
        case JointError::kDoubleCoupleJointDuplicatePair:
            return "DoubleCoupleJointDuplicatePair";
        case JointError::kDoubleCoupleJointSameMotor:
            return "DoubleCoupleJointSameMotor";
        case JointError::kDoubleCoupleJointPending:
            return "DoubleCoupleJointPending";
        case JointError::kDoubleCoupleJointDuplicateAxis:
            return "DoubleCoupleJointDuplicateAxis";
        case JointError::kDoubleCoupleJointCalibrationMismatch:
            return "DoubleCoupleJointCalibrationMismatch";
        case JointError::kDoubleCoupleJointCalibrationFailed:
            return "DoubleCoupleJointCalibrationFailed";
        case JointError::kDoubleCoupleJointSolveFailed:
            return "DoubleCoupleJointSolveFailed";
        case JointError::kDoubleCoupleJointUnsupportedControl:
            return "DoubleCoupleJointUnsupportedControl";
        case JointError::kDoubleCoupleJointZeroPoseNotSet:
            return "DoubleCoupleJointZeroPoseNotSet";
        case JointError::kDoubleCoupleJointMotorLimitMismatch:
            return "DoubleCoupleJointMotorLimitMismatch";
        case JointError::kContinuousJointUnsupportedOperation:
            return "ContinuousJointUnsupportedOperation";
    }
    return "Unknown";
}

}  // namespace

RuntimeStore& RuntimeStore::Instance() {
    static RuntimeStore store;
    return store;
}

std::uint32_t RuntimeStore::NextHandle() {
    return next_handle_++;
}

void RuntimeStore::ClearJointCallbacks(JointEntry* entry) {
    if (!entry || !entry->joint) {
        return;
    }
    entry->joint->SetOnStatus(nullptr);
    entry->status_callback = nullptr;
}

std::uint32_t RuntimeStore::CreateManager(const std::string& adapter_type,
                                          const std::string& adapter_name,
                                          const std::string& logger_name, LogLevel log_level) {
    const auto handle = CreateEmptyManager();
    CreateAdapter(handle, adapter_type, adapter_name, logger_name, log_level);
    return handle;
}

std::uint32_t RuntimeStore::CreateEmptyManager() {
    auto entry = std::make_unique<ManagerEntry>();
    entry->manager = std::make_shared<WasmJointManager>();

    const auto handle = NextHandle();
    managers_[handle] = std::move(entry);
    return handle;
}

void RuntimeStore::CreateAdapter(std::uint32_t manager_handle, const std::string& adapter_type,
                                 const std::string& adapter_name, const std::string& logger_name,
                                 LogLevel log_level) {
    auto* entry = GetManager(manager_handle);
    if (!entry || !entry->manager) {
        throw std::invalid_argument("Invalid manager handle");
    }
    auto adapter = MakeAdapter(adapter_type, adapter_name, logger_name, log_level);
    if (!adapter) {
        throw std::runtime_error("MakeAdapter returned null for " + adapter_type);
    }

    entry->manager->InjectAdapter(adapter_name, adapter);

    entry->adapter_name = adapter_name;
    entry->adapter_type = adapter_type;
    entry->adapter = adapter;

    if (adapter_type == "Fake") {
        entry->fake_adapter = dynamic_cast<FakeAdapter*>(adapter);
        entry->fake_control = adapter->GetFakeAdapterControl();
        if (entry->fake_control) {
            entry->fake_controls[adapter_name] = entry->fake_control;
            entry->fake_control->EnableAutoCreateMotor();
            auto* raw = entry;
            entry->fake_control->SetDecodedCommandObserver([raw](const FakeCommandRecord& record) {
                if (raw->command_records.size() < kMaxFakeCommandRecords) {
                    raw->command_records.push_back(record);
                }
            });
        }
    }
}

void RuntimeStore::DisposeManager(std::uint32_t handle) {
    auto it = managers_.find(handle);
    if (it == managers_.end()) {
        return;
    }
    auto* entry = it->second.get();
    entry->valid = false;

    for (const auto joint_handle : entry->joint_handles) {
        auto jt = joints_.find(joint_handle);
        if (jt != joints_.end()) {
            ClearJointCallbacks(jt->second.get());
            jt->second->valid = false;
            joints_.erase(jt);
        }
    }
    entry->joint_handles.clear();

    if (entry->fake_control) {
        entry->fake_control->ClearDecodedCommandObserver();
    }
    for (auto& [_, fake_control] : entry->fake_controls) {
        if (fake_control && fake_control != entry->fake_control) {
            fake_control->ClearDecodedCommandObserver();
        }
    }

    managers_.erase(it);
}

ManagerEntry* RuntimeStore::GetManager(std::uint32_t handle) {
    auto it = managers_.find(handle);
    if (it == managers_.end() || !it->second->valid) {
        return nullptr;
    }
    return it->second.get();
}

WasmJointManager* RuntimeStore::GetManagerObject(std::uint32_t handle) {
    auto* entry = GetManager(handle);
    if (!entry) {
        return nullptr;
    }
    return entry->manager.get();
}

std::uint32_t RuntimeStore::AddJoint(std::uint32_t manager_handle,
                                     const nlohmann::json& definition) {
    auto* manager = GetManagerObject(manager_handle);
    if (!manager) {
        throw std::invalid_argument("Invalid manager handle");
    }

    auto joint = manager->AddJoint(definition);
    if (!joint) {
        throw std::runtime_error("AddJoint returned null");
    }

    auto* entry = GetManager(manager_handle);
    SeedFakeMotors(entry, definition);
    const auto name = definition.at("name").get<std::string>();
    return RegisterJoint(manager_handle, name, std::move(joint));
}

std::uint32_t RuntimeStore::RegisterJoint(std::uint32_t manager_handle, const std::string& name,
                                          BaseJointPtr joint) {
    auto* manager_entry = GetManager(manager_handle);
    if (!manager_entry) {
        throw std::invalid_argument("Invalid manager handle");
    }
    if (const auto it = manager_entry->joint_handles_by_name.find(name);
        it != manager_entry->joint_handles_by_name.end()) {
        if (GetJoint(it->second)) {
            return it->second;
        }
        manager_entry->joint_handles_by_name.erase(it);
    }

    auto entry = std::make_unique<JointEntry>();
    entry->manager_handle = manager_handle;
    entry->joint_name = name;
    entry->joint = std::move(joint);

    const auto handle = NextHandle();
    joints_[handle] = std::move(entry);
    manager_entry->joint_handles.push_back(handle);
    manager_entry->joint_handles_by_name[name] = handle;
    return handle;
}

JointEntry* RuntimeStore::GetJoint(std::uint32_t handle) {
    auto it = joints_.find(handle);
    if (it == joints_.end() || !it->second->valid) {
        return nullptr;
    }
    return it->second.get();
}

void RuntimeStore::SetLastError(WasmErrorCode code, const std::string& message,
                                std::optional<JointError> joint_error) {
    last_error_ = ErrorState{code, message, joint_error};
}

ErrorState RuntimeStore::GetLastError() const {
    return last_error_;
}

void RuntimeStore::ClearLastError() {
    last_error_ = ErrorState{WasmErrorCode::kOk, ""};
}

std::string RuntimeStore::Version() const {
#ifdef JOINT_SDK_VERSION
    return JOINT_SDK_VERSION;
#else
    return "0.0.0";
#endif
}

}  // namespace wasm
}  // namespace encos
