// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <emscripten.h>
#include <variant>

#include "motor/types.h"
#include "src/wasm/wasm_runtime.h"

namespace encos {
namespace wasm {

namespace {

// Number field ids for decoded Fake command payloads.
constexpr int kPVTFieldKp = 0;
constexpr int kPVTFieldKd = 1;
constexpr int kPVTFieldPosition = 2;
constexpr int kPVTFieldSpeed = 3;
constexpr int kPVTFieldTorque = 4;

constexpr int kPosFieldPosition = 10;
constexpr int kPosFieldSpeed = 11;
constexpr int kPosFieldCurrent = 12;

constexpr int kSpdFieldSpeed = 20;
constexpr int kSpdFieldCurrent = 21;

constexpr int kCurFieldCurrent = 30;
constexpr int kTorFieldTorque = 40;
constexpr int kStopFieldCurrent = 50;

// Int field ids.
constexpr int kIntFieldFeedbackType = 0;
constexpr int kIntFieldStopMode = 1;
constexpr int kIntFieldRawFrameFlags = 2;

int ToInt(WasmErrorCode code) {
    return static_cast<int>(code);
}

void SetOk() {
    RuntimeStore::Instance().ClearLastError();
}

void SetError(WasmErrorCode code, const std::string& message,
              std::optional<JointError> joint_error = std::nullopt) {
    RuntimeStore::Instance().SetLastError(code, message, joint_error);
}

int CatchException(const std::string& context) {
    try {
        throw;
    } catch (const JointException& ex) {
        SetError(WasmErrorCode::kJointError, context + ": " + ex.what(), ex.Error());
    } catch (const nlohmann::json::exception& ex) {
        SetError(WasmErrorCode::kInvalidArgument, context + ": " + ex.what());
    } catch (const std::invalid_argument& ex) {
        SetError(WasmErrorCode::kInvalidArgument, context + ": " + ex.what());
    } catch (const std::exception& ex) {
        SetError(WasmErrorCode::kInternal, context + ": " + ex.what());
    } catch (...) {
        SetError(WasmErrorCode::kInternal, context + ": unknown exception");
    }
    return ToInt(RuntimeStore::Instance().GetLastError().code);
}

template <typename T>
int RequirePointer(T* ptr, const std::string& name) {
    if (ptr == nullptr) {
        SetError(WasmErrorCode::kInvalidArgument, name + " must not be null");
        return ToInt(WasmErrorCode::kInvalidArgument);
    }
    return ToInt(WasmErrorCode::kOk);
}

char* DupString(const std::string& s) {
    return strdup(s.c_str());
}

nlohmann::json JointSchemaToJson(const std::map<std::string, std::string>& schemas) {
    nlohmann::json obj = nlohmann::json::object();
    for (const auto& [type, schema_str] : schemas) {
        obj[type] = nlohmann::json::parse(schema_str);
    }
    return obj;
}

nlohmann::json StatusMapToJson(const std::map<std::string, JointStatus>& statuses) {
    nlohmann::json obj = nlohmann::json::object();
    for (const auto& [name, status] : statuses) {
        obj[name] = nlohmann::json{
            {"pos", status.pos},
            {"spd", status.spd},
            {"tor", status.tor},
            {"mosTemperature", status.mosTemperature},
            {"motorTemperature", status.motorTemperature},
            {"error", static_cast<int>(status.error)},
        };
    }
    return obj;
}

ManagerEntry::FakeControlHandle ResolveFakeControl(ManagerEntry* entry) {
    if (!entry) {
        return {};
    }
    return entry->fake_control;
}

int GetCommandNumberField(const FakeCommandRecord& record, int field_id, double* out_value) {
    switch (record.kind) {
        case FakeCommandKind::PVTControl: {
            const auto& p = std::get<FakePVTControlPayload>(record.payload);
            switch (field_id) {
                case kPVTFieldKp:
                    *out_value = p.kp;
                    return ToInt(WasmErrorCode::kOk);
                case kPVTFieldKd:
                    *out_value = p.kd;
                    return ToInt(WasmErrorCode::kOk);
                case kPVTFieldPosition:
                    *out_value = p.position;
                    return ToInt(WasmErrorCode::kOk);
                case kPVTFieldSpeed:
                    *out_value = p.speed;
                    return ToInt(WasmErrorCode::kOk);
                case kPVTFieldTorque:
                    *out_value = p.torque;
                    return ToInt(WasmErrorCode::kOk);
            }
            break;
        }
        case FakeCommandKind::PosControl: {
            const auto& p = std::get<FakePosControlPayload>(record.payload);
            switch (field_id) {
                case kPosFieldPosition:
                    *out_value = p.position;
                    return ToInt(WasmErrorCode::kOk);
                case kPosFieldSpeed:
                    *out_value = p.speed;
                    return ToInt(WasmErrorCode::kOk);
                case kPosFieldCurrent:
                    *out_value = p.current;
                    return ToInt(WasmErrorCode::kOk);
            }
            break;
        }
        case FakeCommandKind::SpdControl: {
            const auto& p = std::get<FakeSpdControlPayload>(record.payload);
            switch (field_id) {
                case kSpdFieldSpeed:
                    *out_value = p.speed;
                    return ToInt(WasmErrorCode::kOk);
                case kSpdFieldCurrent:
                    *out_value = p.current;
                    return ToInt(WasmErrorCode::kOk);
            }
            break;
        }
        case FakeCommandKind::CurControl: {
            const auto& p = std::get<FakeCurControlPayload>(record.payload);
            if (field_id == kCurFieldCurrent) {
                *out_value = p.current;
                return ToInt(WasmErrorCode::kOk);
            }
            break;
        }
        case FakeCommandKind::TorControl: {
            const auto& p = std::get<FakeTorControlPayload>(record.payload);
            if (field_id == kTorFieldTorque) {
                *out_value = p.torque;
                return ToInt(WasmErrorCode::kOk);
            }
            break;
        }
        case FakeCommandKind::Stop: {
            const auto& p = std::get<FakeStopPayload>(record.payload);
            if (field_id == kStopFieldCurrent) {
                *out_value = p.current;
                return ToInt(WasmErrorCode::kOk);
            }
            break;
        }
        default:
            break;
    }
    return ToInt(WasmErrorCode::kInvalidArgument);
}

int GetCommandIntField(const FakeCommandRecord& record, int field_id, int* out_value) {
    switch (field_id) {
        case kIntFieldRawFrameFlags:
            *out_value = record.raw_frame_flags;
            return ToInt(WasmErrorCode::kOk);
    }

    switch (record.kind) {
        case FakeCommandKind::PVTControl:
            break;  // PVT has no feedback_type in payload.
        case FakeCommandKind::PosControl: {
            const auto& p = std::get<FakePosControlPayload>(record.payload);
            if (field_id == kIntFieldFeedbackType) {
                *out_value = p.feedback_type;
                return ToInt(WasmErrorCode::kOk);
            }
            break;
        }
        case FakeCommandKind::SpdControl: {
            const auto& p = std::get<FakeSpdControlPayload>(record.payload);
            if (field_id == kIntFieldFeedbackType) {
                *out_value = p.feedback_type;
                return ToInt(WasmErrorCode::kOk);
            }
            break;
        }
        case FakeCommandKind::CurControl: {
            const auto& p = std::get<FakeCurControlPayload>(record.payload);
            if (field_id == kIntFieldFeedbackType) {
                *out_value = p.feedback_type;
                return ToInt(WasmErrorCode::kOk);
            }
            break;
        }
        case FakeCommandKind::TorControl: {
            const auto& p = std::get<FakeTorControlPayload>(record.payload);
            if (field_id == kIntFieldFeedbackType) {
                *out_value = p.feedback_type;
                return ToInt(WasmErrorCode::kOk);
            }
            break;
        }
        case FakeCommandKind::Stop: {
            const auto& p = std::get<FakeStopPayload>(record.payload);
            if (field_id == kIntFieldFeedbackType) {
                *out_value = p.feedback_type;
                return ToInt(WasmErrorCode::kOk);
            }
            if (field_id == kIntFieldStopMode) {
                *out_value = static_cast<int>(p.mode);
                return ToInt(WasmErrorCode::kOk);
            }
            break;
        }
        default:
            break;
    }
    return ToInt(WasmErrorCode::kInvalidArgument);
}

}  // namespace

}  // namespace wasm
}  // namespace encos

using encos::wasm::ManagerEntry;
using encos::wasm::RuntimeStore;
using encos::wasm::WasmErrorCode;

extern "C" {

EMSCRIPTEN_KEEPALIVE
int joint_sdk_get_last_error_code() {
    return encos::wasm::ToInt(RuntimeStore::Instance().GetLastError().code);
}

EMSCRIPTEN_KEEPALIVE
char* joint_sdk_get_last_error_message() {
    return encos::wasm::DupString(RuntimeStore::Instance().GetLastError().message);
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_get_last_joint_error() {
    const auto error = RuntimeStore::Instance().GetLastError().joint_error;
    return error.has_value() ? static_cast<int>(*error) : -1;
}

EMSCRIPTEN_KEEPALIVE
void joint_sdk_clear_last_error() {
    RuntimeStore::Instance().ClearLastError();
}

EMSCRIPTEN_KEEPALIVE
char* joint_sdk_get_version() {
    return encos::wasm::DupString(RuntimeStore::Instance().Version());
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_create_manager(const char* adapter_type, const char* adapter_name,
                             const char* logger_name, int log_level, std::uint32_t* out_manager) {
    try {
        if (encos::wasm::RequirePointer(adapter_type, "adapter_type") != 0 ||
            encos::wasm::RequirePointer(adapter_name, "adapter_name") != 0 ||
            encos::wasm::RequirePointer(out_manager, "out_manager") != 0) {
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        const auto handle = RuntimeStore::Instance().CreateManager(
            adapter_type, adapter_name, logger_name ? logger_name : adapter_name,
            static_cast<encos::LogLevel>(log_level));
        *out_manager = handle;
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_create_manager");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_create_empty_manager(std::uint32_t* out_manager) {
    try {
        if (encos::wasm::RequirePointer(out_manager, "out_manager") != 0) {
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        *out_manager = RuntimeStore::Instance().CreateEmptyManager();
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_create_empty_manager");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_manager_create_adapter(std::uint32_t manager, const char* adapter_type,
                                     const char* adapter_name, const char* logger_name,
                                     int log_level) {
    try {
        if (encos::wasm::RequirePointer(adapter_type, "adapter_type") != 0 ||
            encos::wasm::RequirePointer(adapter_name, "adapter_name") != 0) {
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        RuntimeStore::Instance().CreateAdapter(manager, adapter_type, adapter_name,
                                               logger_name ? logger_name : adapter_name,
                                               static_cast<encos::LogLevel>(log_level));
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_manager_create_adapter");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_manager_create_adapter_json(std::uint32_t manager, const char* adapter_json) {
    try {
        if (encos::wasm::RequirePointer(adapter_json, "adapter_json") != 0) {
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        const auto definition = nlohmann::json::parse(adapter_json);
        std::string adapter_type;
        std::string adapter_name;
        encos::ParseAdapterDefinition(definition, adapter_type, adapter_name);
        RuntimeStore::Instance().CreateAdapter(manager, adapter_type, adapter_name, adapter_name,
                                               encos::LogLevel::Info);
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_manager_create_adapter_json");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_dispose_manager(std::uint32_t manager) {
    try {
        RuntimeStore::Instance().DisposeManager(manager);
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_dispose_manager");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_manager_add_joint_json(std::uint32_t manager, const char* json,
                                     std::uint32_t* out_joint) {
    try {
        if (encos::wasm::RequirePointer(json, "json") != 0 ||
            encos::wasm::RequirePointer(out_joint, "out_joint") != 0) {
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        const auto definition = nlohmann::json::parse(json);
        const auto handle = RuntimeStore::Instance().AddJoint(manager, definition);
        *out_joint = handle;
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_manager_add_joint_json");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_manager_get_joint(std::uint32_t manager, const char* name, std::uint32_t* out_joint) {
    try {
        if (encos::wasm::RequirePointer(name, "name") != 0 ||
            encos::wasm::RequirePointer(out_joint, "out_joint") != 0) {
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        auto* mgr = RuntimeStore::Instance().GetManagerObject(manager);
        if (!mgr) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid manager handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        auto joint = mgr->GetJoint(name);
        if (!joint) {
            encos::wasm::SetError(WasmErrorCode::kNotFound,
                                  std::string("Joint not found: ") + name);
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        const auto handle = RuntimeStore::Instance().RegisterJoint(manager, name, std::move(joint));
        *out_joint = handle;
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_manager_get_joint");
    }
}

EMSCRIPTEN_KEEPALIVE
char* joint_sdk_manager_get_joint_schema_json(std::uint32_t manager) {
    try {
        auto* mgr = RuntimeStore::Instance().GetManagerObject(manager);
        if (!mgr) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid manager handle");
            return encos::wasm::DupString("null");
        }
        return encos::wasm::DupString(encos::wasm::JointSchemaToJson(mgr->GetJointSchema()).dump());
    } catch (...) {
        encos::wasm::CatchException("joint_sdk_manager_get_joint_schema_json");
        return encos::wasm::DupString("null");
    }
}

EMSCRIPTEN_KEEPALIVE
char* joint_sdk_manager_get_available_joint_types_json(std::uint32_t manager) {
    try {
        auto* mgr = RuntimeStore::Instance().GetManagerObject(manager);
        if (!mgr) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid manager handle");
            return encos::wasm::DupString("null");
        }
        const auto types = mgr->GetAvailableJointTypes();
        return encos::wasm::DupString(nlohmann::json(types).dump());
    } catch (...) {
        encos::wasm::CatchException("joint_sdk_manager_get_available_joint_types_json");
        return encos::wasm::DupString("null");
    }
}

EMSCRIPTEN_KEEPALIVE
char* joint_sdk_manager_get_status_json(std::uint32_t manager, int auto_send,
                                        int life_cycle_deduction) {
    try {
        auto* mgr = RuntimeStore::Instance().GetManagerObject(manager);
        if (!mgr) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid manager handle");
            return encos::wasm::DupString("null");
        }
        const auto statuses = mgr->GetStatus(auto_send != 0, life_cycle_deduction);
        return encos::wasm::DupString(encos::wasm::StatusMapToJson(statuses).dump());
    } catch (...) {
        encos::wasm::CatchException("joint_sdk_manager_get_status_json");
        return encos::wasm::DupString("null");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_manager_enable_joint_motor_canfd(std::uint32_t manager) {
    try {
        auto* mgr = RuntimeStore::Instance().GetManagerObject(manager);
        if (!mgr) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid manager handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        mgr->EnableJointMotorCanfd();
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_manager_enable_joint_motor_canfd");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_manager_disable_joint_motor_canfd(std::uint32_t manager) {
    try {
        auto* mgr = RuntimeStore::Instance().GetManagerObject(manager);
        if (!mgr) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid manager handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        mgr->DisableJointMotorCanfd();
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_manager_disable_joint_motor_canfd");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_manager_is_joint_motor_canfd(std::uint32_t manager, int* out_enabled) {
    try {
        if (encos::wasm::RequirePointer(out_enabled, "out_enabled") != 0) {
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        auto* mgr = RuntimeStore::Instance().GetManagerObject(manager);
        if (!mgr) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid manager handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        *out_enabled = mgr->IsJointMotorCanfd() ? 1 : 0;
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_manager_is_joint_motor_canfd");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_joint_pvt_control(std::uint32_t joint, float kp, float kd, float pos, float spd,
                                float torque, int torque_mode) {
    try {
        auto* entry = RuntimeStore::Instance().GetJoint(joint);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid joint handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        const auto mode = static_cast<encos::PVTControlMode>(torque_mode);
        if (!encos::IsValidPVTControlMode(mode)) {
            encos::wasm::SetError(WasmErrorCode::kInvalidArgument, "Invalid PVT control mode");
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        entry->joint->PVTControl(kp, kd, pos, spd, torque, mode);
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_joint_pvt_control");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_joint_pos_control(std::uint32_t joint, float position, float speed, float current,
                                int feedback) {
    try {
        auto* entry = RuntimeStore::Instance().GetJoint(joint);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid joint handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        entry->joint->PosControl(position, speed, current, feedback);
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_joint_pos_control");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_joint_spd_control(std::uint32_t joint, float speed, float current, int feedback) {
    try {
        auto* entry = RuntimeStore::Instance().GetJoint(joint);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid joint handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        entry->joint->SpdControl(speed, current, feedback);
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_joint_spd_control");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_joint_cur_control(std::uint32_t joint, float current, int feedback) {
    try {
        auto* entry = RuntimeStore::Instance().GetJoint(joint);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid joint handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        entry->joint->CurControl(current, feedback);
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_joint_cur_control");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_joint_tor_control(std::uint32_t joint, float torque, int feedback) {
    try {
        auto* entry = RuntimeStore::Instance().GetJoint(joint);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid joint handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        entry->joint->TorControl(torque, feedback);
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_joint_tor_control");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_joint_stop(std::uint32_t joint, int mode, float current, int feedback) {
    try {
        auto* entry = RuntimeStore::Instance().GetJoint(joint);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid joint handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        entry->joint->Stop(static_cast<encos::MotorStopMode>(mode), current, feedback);
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_joint_stop");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_joint_set_zero(std::uint32_t joint, int* out_ok) {
    try {
        if (encos::wasm::RequirePointer(out_ok, "out_ok") != 0) {
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        auto* entry = RuntimeStore::Instance().GetJoint(joint);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid joint handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        *out_ok = entry->joint->SetZero() ? 1 : 0;
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_joint_set_zero");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_joint_goto_limit(std::uint32_t joint, int dir, float speed, float current,
                               int timeout_ms, float* out_offset) {
    try {
        if (encos::wasm::RequirePointer(out_offset, "out_offset") != 0) {
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        auto* entry = RuntimeStore::Instance().GetJoint(joint);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid joint handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        *out_offset =
            entry->joint->GotoLimit(dir, speed, current, std::chrono::milliseconds(timeout_ms));
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_joint_goto_limit");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_joint_goto_zero(std::uint32_t joint, float offset, float speed, float current,
                              int timeout_ms, int* out_ok) {
    try {
        if (encos::wasm::RequirePointer(out_ok, "out_ok") != 0) {
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        auto* entry = RuntimeStore::Instance().GetJoint(joint);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid joint handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        *out_ok =
            entry->joint->GotoZero(offset, speed, current, std::chrono::milliseconds(timeout_ms))
                ? 1
                : 0;
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_joint_goto_zero");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_joint_pos_as_offset(std::uint32_t joint, int dir, float* out_offset) {
    try {
        if (encos::wasm::RequirePointer(out_offset, "out_offset") != 0) {
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        auto* entry = RuntimeStore::Instance().GetJoint(joint);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid joint handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        *out_offset = entry->joint->PosAsOffset(dir);
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_joint_pos_as_offset");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_joint_calibrate(std::uint32_t joint, int dir, float speed, float current,
                              int timeout_ms, int* out_ok) {
    try {
        if (encos::wasm::RequirePointer(out_ok, "out_ok") != 0) {
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        auto* entry = RuntimeStore::Instance().GetJoint(joint);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid joint handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        *out_ok =
            entry->joint->Calibrate(dir, speed, current, std::chrono::milliseconds(timeout_ms)) ? 1
                                                                                                : 0;
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_joint_calibrate");
    }
}

EMSCRIPTEN_KEEPALIVE
char* joint_sdk_joint_get_status_json(std::uint32_t joint, int auto_send,
                                      int life_cycle_deduction) {
    try {
        auto* entry = RuntimeStore::Instance().GetJoint(joint);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid joint handle");
            return encos::wasm::DupString("null");
        }
        const auto status = entry->joint->GetStatus(auto_send != 0, life_cycle_deduction);
        if (!status) {
            encos::wasm::SetOk();
            return encos::wasm::DupString("null");
        }
        encos::wasm::SetOk();
        return encos::wasm::DupString(encos::wasm::StatusMapToJson({{"", *status}})[""].dump());
    } catch (...) {
        encos::wasm::CatchException("joint_sdk_joint_get_status_json");
        return encos::wasm::DupString("null");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_joint_set_limit(std::uint32_t joint, float min, float max) {
    try {
        auto* entry = RuntimeStore::Instance().GetJoint(joint);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid joint handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        entry->joint->SetLimit(encos::Range<float>{min, max});
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_joint_set_limit");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_joint_set_status_callback(std::uint32_t joint_handle, void (*callback)(const char*)) {
    try {
        auto* entry = RuntimeStore::Instance().GetJoint(joint_handle);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid joint handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        if (callback == nullptr) {
            entry->joint->SetOnStatus(nullptr);
            entry->status_callback = nullptr;
            encos::wasm::SetOk();
            return encos::wasm::ToInt(WasmErrorCode::kOk);
        }
        entry->status_callback = callback;
        entry->joint->SetOnStatus([entry](const encos::JointStatus& status) {
            if (!entry->status_callback) {
                return;
            }
            const auto j = nlohmann::json{
                {"pos", status.pos},
                {"spd", status.spd},
                {"tor", status.tor},
                {"mosTemperature", status.mosTemperature},
                {"motorTemperature", status.motorTemperature},
                {"error", static_cast<int>(status.error)},
            };
            const auto str = j.dump();
            entry->status_callback(str.c_str());
        });
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_joint_set_status_callback");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_joint_clear_status_callback(std::uint32_t joint_handle) {
    return joint_sdk_joint_set_status_callback(joint_handle, nullptr);
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_fake_set_reply_mode(std::uint32_t manager, int mode) {
    try {
        auto* entry = RuntimeStore::Instance().GetManager(manager);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid manager handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        if (!entry->fake_control) {
            encos::wasm::SetError(WasmErrorCode::kWrongAdapter,
                                  "Manager was not created with adapterType 'Fake'");
            return encos::wasm::ToInt(WasmErrorCode::kWrongAdapter);
        }
        entry->fake_control->SetReplyMode(static_cast<encos::FakeReplyMode>(mode));
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_fake_set_reply_mode");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_fake_enable_position_error(std::uint32_t manager, int enabled) {
    try {
        auto* entry = RuntimeStore::Instance().GetManager(manager);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid manager handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        if (!entry->fake_control) {
            encos::wasm::SetError(WasmErrorCode::kWrongAdapter,
                                  "Manager was not created with adapterType 'Fake'");
            return encos::wasm::ToInt(WasmErrorCode::kWrongAdapter);
        }
        entry->manager->EnableFakePositionError(enabled != 0);
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_fake_enable_position_error");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_fake_clear_command_records(std::uint32_t manager) {
    try {
        auto* entry = RuntimeStore::Instance().GetManager(manager);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid manager handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        entry->command_records.clear();
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_fake_clear_command_records");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_fake_get_command_count(std::uint32_t manager, int* out_count) {
    try {
        if (encos::wasm::RequirePointer(out_count, "out_count") != 0) {
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        auto* entry = RuntimeStore::Instance().GetManager(manager);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid manager handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        *out_count = static_cast<int>(entry->command_records.size());
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_fake_get_command_count");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_fake_get_command_kind(std::uint32_t manager, int index, int* out_kind) {
    try {
        if (encos::wasm::RequirePointer(out_kind, "out_kind") != 0) {
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        auto* entry = RuntimeStore::Instance().GetManager(manager);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid manager handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        if (index < 0 || static_cast<std::size_t>(index) >= entry->command_records.size()) {
            encos::wasm::SetError(WasmErrorCode::kInvalidArgument, "Command index out of range");
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        *out_kind = static_cast<int>(entry->command_records[index].kind);
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_fake_get_command_kind");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_fake_get_command_bus_index(std::uint32_t manager, int index, int* out_bus) {
    try {
        if (encos::wasm::RequirePointer(out_bus, "out_bus") != 0) {
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        auto* entry = RuntimeStore::Instance().GetManager(manager);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid manager handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        if (index < 0 || static_cast<std::size_t>(index) >= entry->command_records.size()) {
            encos::wasm::SetError(WasmErrorCode::kInvalidArgument, "Command index out of range");
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        *out_bus = entry->command_records[index].bus_idx;
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_fake_get_command_bus_index");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_fake_get_command_motor_index(std::uint32_t manager, int index, int* out_motor) {
    try {
        if (encos::wasm::RequirePointer(out_motor, "out_motor") != 0) {
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        auto* entry = RuntimeStore::Instance().GetManager(manager);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid manager handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        if (index < 0 || static_cast<std::size_t>(index) >= entry->command_records.size()) {
            encos::wasm::SetError(WasmErrorCode::kInvalidArgument, "Command index out of range");
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        *out_motor = entry->command_records[index].motor_idx;
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_fake_get_command_motor_index");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_fake_get_command_number_field(std::uint32_t manager, int index, int field_id,
                                            double* out_value) {
    try {
        if (encos::wasm::RequirePointer(out_value, "out_value") != 0) {
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        auto* entry = RuntimeStore::Instance().GetManager(manager);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid manager handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        if (index < 0 || static_cast<std::size_t>(index) >= entry->command_records.size()) {
            encos::wasm::SetError(WasmErrorCode::kInvalidArgument, "Command index out of range");
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        const int code =
            encos::wasm::GetCommandNumberField(entry->command_records[index], field_id, out_value);
        if (code != encos::wasm::ToInt(WasmErrorCode::kOk)) {
            encos::wasm::SetError(WasmErrorCode::kInvalidArgument,
                                  "Invalid number field id for command kind");
        }
        return code;
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_fake_get_command_number_field");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_fake_get_command_int_field(std::uint32_t manager, int index, int field_id,
                                         int* out_value) {
    try {
        if (encos::wasm::RequirePointer(out_value, "out_value") != 0) {
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        auto* entry = RuntimeStore::Instance().GetManager(manager);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid manager handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        if (index < 0 || static_cast<std::size_t>(index) >= entry->command_records.size()) {
            encos::wasm::SetError(WasmErrorCode::kInvalidArgument, "Command index out of range");
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        const int code =
            encos::wasm::GetCommandIntField(entry->command_records[index], field_id, out_value);
        if (code != encos::wasm::ToInt(WasmErrorCode::kOk)) {
            encos::wasm::SetError(WasmErrorCode::kInvalidArgument,
                                  "Invalid int field id for command kind");
        }
        return code;
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_fake_get_command_int_field");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_fake_inject_feedback(std::uint32_t manager, int bus_id, int motor_id,
                                   const double* values_ptr, int values_count, const int* meta_ptr,
                                   int meta_count) {
    try {
        if (encos::wasm::RequirePointer(values_ptr, "values_ptr") != 0 ||
            encos::wasm::RequirePointer(meta_ptr, "meta_ptr") != 0) {
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        if (values_count < 5 || meta_count < 1) {
            encos::wasm::SetError(WasmErrorCode::kInvalidArgument,
                                  "feedback buffers are too small");
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        auto* entry = RuntimeStore::Instance().GetManager(manager);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid manager handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        auto fake = encos::wasm::ResolveFakeControl(entry);
        if (!fake) {
            encos::wasm::SetError(WasmErrorCode::kWrongAdapter,
                                  "Manager was not created with adapterType 'Fake'");
            return encos::wasm::ToInt(WasmErrorCode::kWrongAdapter);
        }
        if (!entry->fake_adapter) {
            encos::wasm::SetError(WasmErrorCode::kWrongAdapter,
                                  "Fake adapter implementation is unavailable");
            return encos::wasm::ToInt(WasmErrorCode::kWrongAdapter);
        }
        encos::MotorStatus status{};
        status.position = static_cast<float>(values_ptr[0]);
        status.speed = static_cast<float>(values_ptr[1]);
        status.current = static_cast<float>(values_ptr[2]);
        status.motor_temperature = static_cast<float>(values_ptr[3]);
        status.mos_temperature = static_cast<float>(values_ptr[4]);
        status.error = static_cast<encos::MotorError>(meta_ptr[0]);
        entry->fake_adapter->InjectMessage(
            entry->fake_adapter->MakeFeedbackMessage(bus_id, motor_id, status, 1));
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_fake_inject_feedback");
    }
}

EMSCRIPTEN_KEEPALIVE
int joint_sdk_fake_get_motor_snapshot(std::uint32_t manager, int bus_id, int motor_id,
                                      double* values_ptr, int values_count, int* meta_ptr,
                                      int meta_count) {
    try {
        if (encos::wasm::RequirePointer(values_ptr, "values_ptr") != 0 ||
            encos::wasm::RequirePointer(meta_ptr, "meta_ptr") != 0) {
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        if (values_count < 21 || meta_count < 6) {
            encos::wasm::SetError(WasmErrorCode::kInvalidArgument,
                                  "snapshot buffers are too small");
            return encos::wasm::ToInt(WasmErrorCode::kInvalidArgument);
        }
        auto* entry = RuntimeStore::Instance().GetManager(manager);
        if (!entry) {
            encos::wasm::SetError(WasmErrorCode::kNotFound, "Invalid manager handle");
            return encos::wasm::ToInt(WasmErrorCode::kNotFound);
        }
        auto fake = encos::wasm::ResolveFakeControl(entry);
        if (!fake) {
            encos::wasm::SetError(WasmErrorCode::kWrongAdapter,
                                  "Manager was not created with adapterType 'Fake'");
            return encos::wasm::ToInt(WasmErrorCode::kWrongAdapter);
        }
        const auto snapshot = fake->GetMotorSnapshot(bus_id, motor_id);
        values_ptr[0] = snapshot.ranges.kp.min;
        values_ptr[1] = snapshot.ranges.kp.max;
        values_ptr[2] = snapshot.ranges.kd.min;
        values_ptr[3] = snapshot.ranges.kd.max;
        values_ptr[4] = snapshot.ranges.position.min;
        values_ptr[5] = snapshot.ranges.position.max;
        values_ptr[6] = snapshot.ranges.speed.min;
        values_ptr[7] = snapshot.ranges.speed.max;
        values_ptr[8] = snapshot.ranges.torque.min;
        values_ptr[9] = snapshot.ranges.torque.max;
        values_ptr[10] = snapshot.ranges.current.min;
        values_ptr[11] = snapshot.ranges.current.max;
        values_ptr[12] = snapshot.ranges.kt;
        values_ptr[13] = snapshot.position_rad;
        values_ptr[14] = snapshot.speed_rad_s;
        values_ptr[15] = snapshot.current_a;
        values_ptr[16] = snapshot.torque_nm;
        values_ptr[17] = snapshot.motor_temp_c;
        values_ptr[18] = snapshot.mos_temp_c;
        values_ptr[19] = snapshot.acceleration;
        values_ptr[20] = snapshot.kt;
        meta_ptr[0] = static_cast<int>(snapshot.model);
        meta_ptr[1] = static_cast<int>(snapshot.can_timeout_ms);
        meta_ptr[2] = static_cast<int>(snapshot.reply_frame_flags);
        meta_ptr[3] = static_cast<int>(snapshot.communication_mode);
        meta_ptr[4] = static_cast<int>(snapshot.error);
        meta_ptr[5] = snapshot.brake_enabled ? 1 : 0;
        encos::wasm::SetOk();
        return encos::wasm::ToInt(WasmErrorCode::kOk);
    } catch (...) {
        return encos::wasm::CatchException("joint_sdk_fake_get_motor_snapshot");
    }
}

EMSCRIPTEN_KEEPALIVE
void joint_sdk_free_string(char* ptr) {
    if (ptr) {
        std::free(ptr);
    }
}

}  // extern "C"
