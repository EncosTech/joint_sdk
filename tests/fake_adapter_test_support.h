// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

#pragma once

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "adapter/base_adapter.h"
#include "adapter/fake_adapter_control.h"
#include "encos_motor.h"
#include "motor/motor.h"
#include "motor/types.h"

namespace encos {

class FakeAdapterHarness {
public:
    explicit FakeAdapterHarness(const std::string& adapter_name)
        : adapter_{MakeAdapter("Fake", adapter_name, adapter_name)} {
        if (!adapter_) {
            throw std::runtime_error("Cannot create Fake adapter: " + adapter_name);
        }
        control_ = adapter_->GetFakeAdapterControl();
        if (!control_) {
            throw std::runtime_error("Fake adapter control is unavailable: " + adapter_name);
        }
        control_->EnableAutoCreateMotor();
        control_->SetReplyMode(FakeReplyMode::Manual);
        control_->SetDecodedCommandObserver([this](const FakeCommandRecord& record) {
            std::lock_guard<std::mutex> lock(command_records_mutex_);
            command_records_.push_back(record);
        });
    }

    BaseAdapterPtr Adapter() const {
        return adapter_;
    }

    FakeAdapterControl& Control() const {
        return *control_;
    }

    void SetReplyMode(FakeReplyMode mode) {
        control_->SetReplyMode(mode);
    }

    const std::vector<FakeCommandRecord>& GetCommandRecords() const {
        return command_records_;
    }

    std::vector<FakeCommandRecord> SnapshotCommandRecords() const {
        std::lock_guard<std::mutex> lock(command_records_mutex_);
        return command_records_;
    }

    void ClearCommandRecords() {
        std::lock_guard<std::mutex> lock(command_records_mutex_);
        command_records_.clear();
    }

private:
    BaseAdapterPtr adapter_;
    FakeAdapterControl* control_{nullptr};
    mutable std::mutex command_records_mutex_;
    std::vector<FakeCommandRecord> command_records_;
};

inline std::shared_ptr<FakeAdapterHarness> CreateFakeAdapterHarness(
    const std::string& adapter_name) {
    return std::make_shared<FakeAdapterHarness>(adapter_name);
}

inline void SyncFakeRangeSnapshot(FakeAdapterHarness& adapter, Motor* motor,
                                  const MotorPVTRanges& ranges) {
    adapter.SetReplyMode(FakeReplyMode::Automatic);
    (void) motor->SetPVTPosRange(ranges.position);
    (void) motor->SetPVTSpdRange(ranges.speed);
    (void) motor->SetPVTTorRange(ranges.torque);
    (void) motor->SetPVTCurRange(ranges.current);
    adapter.SetReplyMode(FakeReplyMode::Manual);
    adapter.ClearCommandRecords();
}

inline void EmitFakeFeedback(FakeAdapterHarness& adapter, Motor* motor, float position,
                             float speed = 0.0f, float torque = 0.0f) {
    adapter.SetReplyMode(FakeReplyMode::Automatic);
    motor->PVTControl<0>(0.0f, 1.0f, position, speed, torque);
    adapter.SetReplyMode(FakeReplyMode::Manual);
}

}  // namespace encos
