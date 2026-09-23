// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "JointManager.h"
#include "RJoint/RJoint.h"
#include "RJoint/StribeckModel.h"
#include "RJoint/ZeroDampingRJoint.h"
#include "fake_adapter_test_support.h"

using namespace encos;

namespace {

using namespace std::chrono_literals;

struct RJointFixture {
    std::shared_ptr<FakeAdapterHarness> adapter;
    std::shared_ptr<RJoint> joint;
    Motor* motor{nullptr};
};

MotorPVTRanges CreateMotorRanges() {
    MotorPVTRanges ranges{};
    ranges.kp = Range<float>{0.0f, 500.0f};
    ranges.kd = Range<float>{0.0f, 5.0f};
    ranges.position = Range<float>{-12.5f, 12.5f};
    ranges.speed = Range<float>{-18.0f, 18.0f};
    ranges.torque = Range<float>{-30.0f, 30.0f};
    ranges.current = Range<float>{-30.0f, 30.0f};
    ranges.kt = 0.5f;
    return ranges;
}

RJointFixture CreateFixture(const std::string& adapter_name, bool reverse = false) {
    auto adapter = CreateFakeAdapterHarness(adapter_name);
    const auto ranges = CreateMotorRanges();
    RJoint::Config config{};
    config.motorParams = MotorParams{"adapter1", 0, 1, MotorInitParams{ranges}};
    config.limit = Range<float>{-1.0f, 2.0f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    config.reverse = reverse;

    AdapterMap adapters{{"adapter1", adapter->Adapter()}};
    auto motor = JointManager<>::CreateMotor(config.motorParams, adapters);
    SyncFakeRangeSnapshot(*adapter, motor, ranges);
    auto joint = std::make_shared<RJoint>(config, motor);
    adapter->ClearCommandRecords();
    return {adapter, joint, motor};
}

class FaultingRJoint final : public RJoint {
public:
    explicit FaultingRJoint(Motor* motor) : RJoint{MakeConfig(), motor} {}

    void CurControl(float, int) override {
        current_calls.fetch_add(1, std::memory_order_relaxed);
        throw std::runtime_error("injected current failure");
    }

    void Stop(MotorStopMode, float, int) override {
        stop_calls.fetch_add(1, std::memory_order_relaxed);
    }

    bool SetZero() override {
        synchronous_calls.fetch_add(1, std::memory_order_relaxed);
        return true;
    }

    std::atomic<int> current_calls{0};
    std::atomic<int> stop_calls{0};
    std::atomic<int> synchronous_calls{0};

private:
    static Config MakeConfig() {
        Config config{};
        config.motorParams = MotorParams{"adapter1", 0, 1, MotorInitParams{CreateMotorRanges()}};
        config.limit = Range<float>{-1.0f, 2.0f};
        config.maxSpd = 2.0f;
        config.maxTor = 3.0f;
        config.reverse = false;
        return config;
    }
};

StribeckConfig ValidStribeckConfig() {
    return StribeckConfig{1.2f,  0.8f, 0.04f, 0.12f,  100.0f, 0.01f, 0.005f,
                          0.05f, 2.0f, 0.8f,  0.001f, 0.2f,   0.3f,  0.15f};
}

bool WaitUntil(const std::function<bool()>& condition, std::chrono::milliseconds timeout = 100ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (condition()) {
            return true;
        }
        std::this_thread::sleep_for(1ms);
    }
    return condition();
}

std::vector<FakeCommandRecord> CommandsOfKind(const FakeAdapterHarness& adapter,
                                              FakeCommandKind kind) {
    auto records = adapter.SnapshotCommandRecords();
    records.erase(std::remove_if(records.begin(), records.end(),
                                 [kind](const auto& record) {
                                     return record.kind != kind;
                                 }),
                  records.end());
    return records;
}

TEST(RJointRuntimeInfoTest, ExposesJointCoordinateLimitsAndEffortRanges) {
    auto fixture = CreateFixture("runtime_info", true);

    EXPECT_FLOAT_EQ(fixture.joint->GetTorqueConstant(), 0.5f);
    EXPECT_FLOAT_EQ(fixture.joint->GetMaxCurrent(), 6.0f);
    EXPECT_FLOAT_EQ(fixture.joint->GetMaxTorque(), 3.0f);
    EXPECT_FLOAT_EQ(fixture.joint->GetMaxSpeed(), 2.0f);
    EXPECT_TRUE(fixture.joint->IsReversed());
    const auto limit = fixture.joint->GetJointLimit();
    EXPECT_FLOAT_EQ(limit.min, -1.0f);
    EXPECT_FLOAT_EQ(limit.max, 2.0f);
}

TEST(StribeckConfigTest, ParsesControlJson) {
    const auto config = StribeckConfig::FromJson(nlohmann::json{
        {"staticFriction", 1.2},
        {"coulombFriction", 0.8},
        {"viscousCoefficient", 0.04},
        {"stribeckVelocity", 0.12},
        {"velocityFilterCutoff", 30.0},
        {"startVelocity", 0.01},
        {"stopVelocity", 0.005},
        {"effortDirectionThreshold", 0.05},
        {"maxCompensationTorque", 2.0},
        {"staticFrictionCompensationRatio", 0.8},
        {"positionIntentThreshold", 0.001},
        {"inertia", 0.2},
        {"inertiaCompensationRatio", 0.3},
        {"maxInertiaCompensationTorque", 0.15},
    });

    EXPECT_FLOAT_EQ(config.static_friction, 1.2f);
    EXPECT_FLOAT_EQ(config.coulomb_friction, 0.8f);
    EXPECT_FLOAT_EQ(config.stop_velocity, 0.005f);
    EXPECT_FLOAT_EQ(config.static_friction_compensation_ratio, 0.8f);
    EXPECT_FLOAT_EQ(config.position_intent_threshold, 0.001f);
    EXPECT_FLOAT_EQ(config.inertia, 0.2f);
    EXPECT_FLOAT_EQ(config.inertia_compensation_ratio, 0.3f);
    EXPECT_FLOAT_EQ(config.max_inertia_compensation_torque, 0.15f);
}

TEST(StribeckConfigTest, RejectsInvalidPhysicalRanges) {
    auto config = ValidStribeckConfig();
    config.static_friction = 0.7f;
    config.coulomb_friction = 0.8f;
    EXPECT_THROW(config.Validate(), std::invalid_argument);

    config = ValidStribeckConfig();
    config.stop_velocity = config.start_velocity;
    EXPECT_THROW(config.Validate(), std::invalid_argument);

    config = ValidStribeckConfig();
    config.velocity_filter_cutoff = 500.0f;
    EXPECT_THROW(config.Validate(), std::invalid_argument);

    config = ValidStribeckConfig();
    config.inertia_compensation_ratio = 1.0f;
    EXPECT_THROW(config.Validate(), std::invalid_argument);

    config = ValidStribeckConfig();
    config.max_inertia_compensation_torque = 0.0f;
    EXPECT_THROW(config.Validate(), std::invalid_argument);

    config = ValidStribeckConfig();
    config.static_friction_compensation_ratio = 1.0f;
    EXPECT_THROW(config.Validate(), std::invalid_argument);

    config = ValidStribeckConfig();
    config.position_intent_threshold = 0.0f;
    EXPECT_THROW(config.Validate(), std::invalid_argument);
}

TEST(StribeckModelTest, ComputesSymmetricFriction) {
    StribeckModel model(ValidStribeckConfig());
    float positive = 0.0f;
    for (int i = 0; i < 20; ++i) {
        positive = model.Update(0.2f, std::nullopt);
    }
    model.Reset();
    float negative = 0.0f;
    for (int i = 0; i < 20; ++i) {
        negative = model.Update(-0.2f, std::nullopt);
    }

    EXPECT_GT(positive, 0.0f);
    EXPECT_NEAR(negative, -positive, 1.0e-5f);
}

TEST(StribeckModelTest, FiltersRawSpeedBeforeCompensation) {
    auto config = ValidStribeckConfig();
    config.velocity_filter_cutoff = 10.0f;
    config.start_velocity = 0.001f;
    config.stop_velocity = 0.0005f;
    StribeckModel model(config);

    (void) model.Update(1.0f, std::nullopt);

    EXPECT_GT(model.FilteredSpeed(), 0.0f);
    EXPECT_LT(model.FilteredSpeed(), 1.0f);
}

TEST(StribeckModelTest, UsesEffortPositionIntentAndMeasuredSpeedAfterReset) {
    StribeckModel model(ValidStribeckConfig());
    EXPECT_FLOAT_EQ(model.Update(0.0f, std::nullopt), 0.0f);
    EXPECT_GT(model.Update(0.0f, 0.2f), 0.0f);

    model.Reset();
    EXPECT_FLOAT_EQ(model.Update(0.0f, std::nullopt), 0.0f);
    float compensation = 0.0f;
    for (int i = 0; i < 20; ++i) {
        compensation = model.Update(0.1f, std::nullopt);
    }
    EXPECT_GT(compensation, 0.0f);

    model.Reset();
    EXPECT_FLOAT_EQ(model.Update(0.0f, 0.0f, std::nullopt), 0.0f);
    EXPECT_FLOAT_EQ(model.Update(0.0005f, 0.0f, std::nullopt), 0.0f);
    EXPECT_NEAR(model.Update(0.0011f, 0.0f, std::nullopt), 0.96f, 1.0e-5f);
    for (int i = 0; i < 60; ++i) {
        compensation = model.Update(0.0011f, 0.0f, std::nullopt);
    }
    EXPECT_FLOAT_EQ(compensation, 0.0f);

    model.Reset();
    (void) model.Update(0.0f, 0.0f, std::nullopt);
    EXPECT_NEAR(model.Update(0.0011f, 0.0f, std::nullopt), 0.96f, 1.0e-5f);
    for (int i = 0; i < 20; ++i) {
        compensation = model.Update(0.01f, 0.2f, std::nullopt);
    }
    EXPECT_LT(compensation, 0.9f);
}

TEST(StribeckModelTest, UsesVelocityHysteresis) {
    auto config = ValidStribeckConfig();
    config.velocity_filter_cutoff = 499.0f;
    StribeckModel model(config);

    EXPECT_FLOAT_EQ(model.Update(0.007f, std::nullopt), 0.0f);
    for (int i = 0; i < 4; ++i) {
        (void) model.Update(0.03f, std::nullopt);
    }
    EXPECT_GT(model.Update(0.007f, std::nullopt), 0.0f);
    for (int i = 0; i < 8; ++i) {
        (void) model.Update(0.0f, std::nullopt);
    }
    EXPECT_FLOAT_EQ(model.Update(0.0f, std::nullopt), 0.0f);
}

static_assert(std::is_base_of_v<BaseJoint, ZeroDampingRJoint>);

TEST(ZeroDampingRJointTest, RejectsNullRJoint) {
    EXPECT_THROW(ZeroDampingRJoint(nullptr, ValidStribeckConfig()), std::invalid_argument);
}

TEST(ZeroDampingRJointTest, SendsNothingBeforeFirstCommand) {
    auto fixture = CreateFixture("zero_damping_idle");
    ZeroDampingRJoint wrapper(fixture.joint, ValidStribeckConfig());

    std::this_thread::sleep_for(5ms);

    EXPECT_TRUE(fixture.adapter->SnapshotCommandRecords().empty());
}

TEST(ZeroDampingRJointTest, RoutesTorqueControlThroughCurrentAndAddsCompensation) {
    auto fixture = CreateFixture("zero_damping_torque");
    EmitFakeFeedback(*fixture.adapter, fixture.motor, 0.0f, 0.0f, 0.0f);
    fixture.adapter->ClearCommandRecords();
    ZeroDampingRJoint wrapper(fixture.joint, ValidStribeckConfig());

    wrapper.TorControl(0.4f);

    ASSERT_TRUE(WaitUntil([&] {
        return !CommandsOfKind(*fixture.adapter, FakeCommandKind::CurControl).empty();
    }));
    const auto commands = CommandsOfKind(*fixture.adapter, FakeCommandKind::CurControl);
    const auto current = std::get<FakeCurControlPayload>(commands.back().payload).current;
    EXPECT_NEAR(current, 3.2f, 0.02f);
}

TEST(ZeroDampingRJointTest, AddsCompensationToPvtFeedforward) {
    auto fixture = CreateFixture("zero_damping_pvt");
    EmitFakeFeedback(*fixture.adapter, fixture.motor, 0.0f, 0.0f, 0.0f);
    fixture.adapter->ClearCommandRecords();
    ZeroDampingRJoint wrapper(fixture.joint, ValidStribeckConfig());

    wrapper.PVTControl(10.0f, 1.0f, 0.2f, 0.0f, 0.3f);

    ASSERT_TRUE(WaitUntil([&] {
        return !CommandsOfKind(*fixture.adapter, FakeCommandKind::PVTControl).empty();
    }));
    const auto commands = CommandsOfKind(*fixture.adapter, FakeCommandKind::PVTControl);
    const auto payload = std::get<FakePVTControlPayload>(commands.back().payload);
    EXPECT_NEAR(payload.position, 0.2f, 0.005f);
    EXPECT_NEAR(payload.torque, 1.5f, 0.02f);
}

TEST(ZeroDampingRJointTest, ConvertsCompensationToCurrent) {
    auto fixture = CreateFixture("zero_damping_current");
    EmitFakeFeedback(*fixture.adapter, fixture.motor, 0.0f, 0.0f, 0.0f);
    fixture.adapter->ClearCommandRecords();
    ZeroDampingRJoint wrapper(fixture.joint, ValidStribeckConfig());

    wrapper.CurControl(0.4f);

    ASSERT_TRUE(WaitUntil([&] {
        return !CommandsOfKind(*fixture.adapter, FakeCommandKind::CurControl).empty();
    }));
    const auto commands = CommandsOfKind(*fixture.adapter, FakeCommandKind::CurControl);
    const auto current = std::get<FakeCurControlPayload>(commands.back().payload).current;
    EXPECT_NEAR(current, 2.8f, 0.02f);
}

TEST(ZeroDampingRJointTest, ReplaysPositionWithoutCompensation) {
    auto fixture = CreateFixture("zero_damping_position");
    ZeroDampingRJoint wrapper(fixture.joint, ValidStribeckConfig());

    wrapper.PosControl(0.2f, 0.7f, 0.8f, 1);

    ASSERT_TRUE(WaitUntil([&] {
        return !CommandsOfKind(*fixture.adapter, FakeCommandKind::PosControl).empty();
    }));
    const auto commands = CommandsOfKind(*fixture.adapter, FakeCommandKind::PosControl);
    const auto payload = std::get<FakePosControlPayload>(commands.back().payload);
    EXPECT_NEAR(payload.position, 0.2f, 0.005f);
    EXPECT_NEAR(payload.speed, 0.7f, 0.02f);
    EXPECT_NEAR(payload.current, 0.8f, 0.005f);
}

TEST(ZeroDampingRJointTest, ReplaysSpeedWithoutCompensation) {
    auto fixture = CreateFixture("zero_damping_speed");
    EmitFakeFeedback(*fixture.adapter, fixture.motor, 0.0f, 0.0f, 0.0f);
    fixture.adapter->ClearCommandRecords();
    ZeroDampingRJoint wrapper(fixture.joint, ValidStribeckConfig());

    wrapper.SpdControl(0.6f, 0.8f, 1);

    ASSERT_TRUE(WaitUntil([&] {
        return !CommandsOfKind(*fixture.adapter, FakeCommandKind::SpdControl).empty();
    }));
    const auto commands = CommandsOfKind(*fixture.adapter, FakeCommandKind::SpdControl);
    const auto payload = std::get<FakeSpdControlPayload>(commands.back().payload);
    EXPECT_NEAR(payload.speed, 0.6f, 0.005f);
    EXPECT_NEAR(payload.current, 0.8f, 0.005f);
}

TEST(ZeroDampingRJointTest, TimesOutToCompensationOnly) {
    auto fixture = CreateFixture("zero_damping_timeout");
    EmitFakeFeedback(*fixture.adapter, fixture.motor, 0.0f, 0.2f, 0.0f);
    fixture.adapter->ClearCommandRecords();
    ZeroDampingRJoint wrapper(fixture.joint, ValidStribeckConfig());
    wrapper.TorControl(0.4f);

    ASSERT_TRUE(WaitUntil([&] {
        return CommandsOfKind(*fixture.adapter, FakeCommandKind::CurControl).size() >= 5;
    }));
    const auto fresh_commands = CommandsOfKind(*fixture.adapter, FakeCommandKind::CurControl);
    const float fresh_current =
        std::get<FakeCurControlPayload>(fresh_commands.back().payload).current;

    std::this_thread::sleep_for(520ms);

    const auto timed_out_commands = CommandsOfKind(*fixture.adapter, FakeCommandKind::CurControl);
    ASSERT_GT(timed_out_commands.size(), fresh_commands.size());
    const float timed_out_current =
        std::get<FakeCurControlPayload>(timed_out_commands.back().payload).current;
    EXPECT_NEAR(fresh_current - timed_out_current, 0.8f, 0.07f);
    EXPECT_GT(timed_out_current, 0.0f);
}

TEST(ZeroDampingRJointTest, TimeoutDropsHistoricalDirectionAndMeasuredSpeedRestartsIt) {
    auto fixture = CreateFixture("zero_damping_timeout_direction");
    EmitFakeFeedback(*fixture.adapter, fixture.motor, 0.0f, 0.0f, 0.0f);
    fixture.adapter->ClearCommandRecords();
    ZeroDampingRJoint wrapper(fixture.joint, ValidStribeckConfig());
    wrapper.TorControl(0.4f);

    ASSERT_TRUE(WaitUntil([&] {
        const auto commands = CommandsOfKind(*fixture.adapter, FakeCommandKind::CurControl);
        return !commands.empty() &&
               std::get<FakeCurControlPayload>(commands.back().payload).current > 2.0f;
    }));

    std::this_thread::sleep_for(520ms);
    ASSERT_TRUE(WaitUntil([&] {
        const auto commands = CommandsOfKind(*fixture.adapter, FakeCommandKind::CurControl);
        return !commands.empty() &&
               std::abs(std::get<FakeCurControlPayload>(commands.back().payload).current) < 0.01f;
    }));

    EmitFakeFeedback(*fixture.adapter, fixture.motor, 0.0f, 0.2f, 0.0f);
    ASSERT_TRUE(WaitUntil([&] {
        const auto commands = CommandsOfKind(*fixture.adapter, FakeCommandKind::CurControl);
        return !commands.empty() &&
               std::get<FakeCurControlPayload>(commands.back().payload).current > 1.0f;
    }));
}

TEST(ZeroDampingRJointTest, StopRemainsLatchedPastTimeout) {
    auto fixture = CreateFixture("zero_damping_stop_latch");
    EmitFakeFeedback(*fixture.adapter, fixture.motor, 0.0f, 0.0f, 0.0f);
    fixture.adapter->ClearCommandRecords();
    ZeroDampingRJoint wrapper(fixture.joint, ValidStribeckConfig());
    wrapper.TorControl(0.4f);
    ASSERT_TRUE(WaitUntil([&] {
        return CommandsOfKind(*fixture.adapter, FakeCommandKind::CurControl).size() >= 2;
    }));

    wrapper.Stop(MotorStopMode::FullBrake);
    fixture.adapter->ClearCommandRecords();
    std::this_thread::sleep_for(520ms);

    EXPECT_TRUE(fixture.adapter->SnapshotCommandRecords().empty());
}

TEST(ZeroDampingRJointTest, SubtractsRetainedCompensationFromStatus) {
    auto fixture = CreateFixture("zero_damping_status");
    EmitFakeFeedback(*fixture.adapter, fixture.motor, 0.0f, 0.0f, 0.0f);
    fixture.adapter->ClearCommandRecords();
    fixture.adapter->SetReplyMode(FakeReplyMode::Automatic);
    ZeroDampingRJoint wrapper(fixture.joint, ValidStribeckConfig());
    wrapper.CurControl(0.4f);
    ASSERT_TRUE(WaitUntil([&] {
        return CommandsOfKind(*fixture.adapter, FakeCommandKind::CurControl).size() >= 10;
    }));

    const auto current_commands = CommandsOfKind(*fixture.adapter, FakeCommandKind::CurControl);
    ASSERT_FALSE(current_commands.empty());
    const float commanded_current =
        std::get<FakeCurControlPayload>(current_commands.back().payload).current;
    const auto raw_status = fixture.joint->GetStatus(false, 0);
    const auto status = wrapper.GetStatus(false, 0);

    ASSERT_TRUE(raw_status.has_value());
    ASSERT_TRUE(status.has_value());
    EXPECT_NEAR(raw_status->tor, commanded_current * fixture.joint->GetTorqueConstant(), 0.03f);
    EXPECT_NEAR(status->tor, 0.2f, 0.03f);
}

TEST(ZeroDampingRJointTest, RejectsNonFiniteControlInputSynchronously) {
    auto fixture = CreateFixture("zero_damping_nonfinite");
    ZeroDampingRJoint wrapper(fixture.joint, ValidStribeckConfig());

    EXPECT_THROW(wrapper.TorControl(std::numeric_limits<float>::quiet_NaN()),
                 std::invalid_argument);
    EXPECT_TRUE(CommandsOfKind(*fixture.adapter, FakeCommandKind::CurControl).empty());
}

TEST(ZeroDampingRJointTest, SubtractsOnlyCompensationRetainedAfterCurrentSaturation) {
    auto fixture = CreateFixture("zero_damping_saturation");
    EmitFakeFeedback(*fixture.adapter, fixture.motor, 0.0f, 0.0f, 0.0f);
    fixture.adapter->ClearCommandRecords();
    fixture.adapter->SetReplyMode(FakeReplyMode::Automatic);
    ZeroDampingRJoint wrapper(fixture.joint, ValidStribeckConfig());
    wrapper.CurControl(5.8f);
    ASSERT_TRUE(WaitUntil([&] {
        return CommandsOfKind(*fixture.adapter, FakeCommandKind::CurControl).size() >= 5;
    }));

    const auto commands = CommandsOfKind(*fixture.adapter, FakeCommandKind::CurControl);
    const float current = std::get<FakeCurControlPayload>(commands.back().payload).current;
    const auto status = wrapper.GetStatus(false, 0);

    EXPECT_NEAR(current, 6.0f, 0.02f);
    ASSERT_TRUE(status.has_value());
    EXPECT_NEAR(status->tor, 5.8f * fixture.joint->GetTorqueConstant(), 0.03f);
}

TEST(ZeroDampingRJointTest, AdjustsTorqueInStatusCallback) {
    auto fixture = CreateFixture("zero_damping_callback");
    EmitFakeFeedback(*fixture.adapter, fixture.motor, 0.0f, 0.0f, 0.0f);
    fixture.adapter->ClearCommandRecords();
    fixture.adapter->SetReplyMode(FakeReplyMode::Automatic);
    ZeroDampingRJoint wrapper(fixture.joint, ValidStribeckConfig());
    std::atomic<int> callback_count{0};
    std::atomic<float> callback_torque{0.0f};
    wrapper.SetOnStatus([&](const JointStatus& status) {
        callback_torque.store(status.tor, std::memory_order_relaxed);
        callback_count.fetch_add(1, std::memory_order_relaxed);
    });

    wrapper.CurControl(0.4f);

    ASSERT_TRUE(WaitUntil([&] {
        return callback_count.load(std::memory_order_relaxed) >= 5;
    }));
    EXPECT_NEAR(callback_torque.load(std::memory_order_relaxed), 0.2f, 0.03f);
    wrapper.SetOnStatus(nullptr);
}

TEST(ZeroDampingRJointTest, LatchesWorkerFaultAndRejectsFurtherCommands) {
    auto fixture = CreateFixture("zero_damping_fault");
    EmitFakeFeedback(*fixture.adapter, fixture.motor, 0.0f, 0.0f, 0.0f);
    auto faulting_joint = std::make_shared<FaultingRJoint>(fixture.motor);
    ZeroDampingRJoint wrapper(faulting_joint, ValidStribeckConfig());

    wrapper.TorControl(0.4f);

    ASSERT_TRUE(WaitUntil([&] {
        return faulting_joint->stop_calls.load(std::memory_order_relaxed) > 0;
    }));
    EXPECT_EQ(faulting_joint->current_calls.load(std::memory_order_relaxed), 1);
    EXPECT_THROW(wrapper.TorControl(0.2f), std::runtime_error);
    EXPECT_THROW((void) wrapper.SetZero(), std::runtime_error);
    EXPECT_EQ(faulting_joint->synchronous_calls.load(std::memory_order_relaxed), 0);
    std::this_thread::sleep_for(5ms);
    EXPECT_EQ(faulting_joint->current_calls.load(std::memory_order_relaxed), 1);
}

}  // namespace
