// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

#include "external/json.hpp"
#if ENCOS_ENABLE_SPDLOG
#include <sstream>

#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>
#endif

#include "BaseJoint.h"
#include "DoubleCoupleJoint/DoubleCoupleJoint.h"
#include "JointManager.h"
#include "RJoint/ContinuousJoint.h"
#include "RJoint/RJoint.h"
#include "fake_adapter_test_support.h"

using namespace encos;

namespace {

class TemporaryDirectory {
public:
    TemporaryDirectory() {
        static std::atomic<std::uint64_t> next_id{0};
        std::random_device random;
        for (int attempt = 0; attempt < 10; ++attempt) {
            path_ = std::filesystem::temp_directory_path() /
                    ("joint_manager_log_test_" + std::to_string(random()) + "_" +
                     std::to_string(next_id.fetch_add(1)));
            if (std::filesystem::create_directory(path_)) {
                return;
            }
        }
        throw std::runtime_error("Cannot create unique JointManager log test directory");
    }

    ~TemporaryDirectory() {
        std::error_code error;
        std::filesystem::remove_all(path_, error);
    }

    const std::filesystem::path& Path() const {
        return path_;
    }

private:
    std::filesystem::path path_;
};

class ScopedCurrentPath {
public:
    explicit ScopedCurrentPath(const std::filesystem::path& path)
        : original_path_{std::filesystem::current_path()} {
        std::filesystem::current_path(path);
    }

    ~ScopedCurrentPath() {
        std::error_code error;
        std::filesystem::current_path(original_path_, error);
    }

private:
    std::filesystem::path original_path_;
};

couple_joint::CalibrationInput CreateValidCalibrationInput() {
    couple_joint::CalibrationInput calibration;
    calibration.left_joint_center = {-35.462080740, 29.491263548, 58.546474855};
    calibration.right_joint_center = {-35.462080740, -29.491263548, 58.546474855};
    calibration.left_planar_endpoint = {-35.5, 11.61629736, 56.42889465};
    calibration.right_planar_endpoint = {-35.5, -11.61629736, 56.42889465};
    calibration.roll_center = {0.0, 0.0, 0.0};
    calibration.pitch_center = {0.0, 0.0, 0.0};
    calibration.left_spatial_endpoint = {-36.6, 26.0, -5.0};
    calibration.right_spatial_endpoint = {-36.6, -26.0, -5.0};
    calibration.rotate_direction = {0.0, 0.0, 0.0};
    calibration.use_rotate_direction = false;
    calibration.rotation_order = couple_joint::RotationOrder::PitchThenRoll;
    return calibration;
}

couple_joint::CalibrationInput CreateH130AnkleCalibrationInput() {
    couple_joint::CalibrationInput calibration;
    calibration.left_joint_center = {-27.5, 0.0, 260.0};
    calibration.right_joint_center = {-27.5, 0.0, 200.0};
    calibration.left_planar_endpoint = {-27.5, 17.497758355829838, 266.00653415215356};
    calibration.right_planar_endpoint = {-27.5, -17.495529250415565, 206.0130238855216};
    calibration.roll_center = {0.0, 0.0, -30.0};
    calibration.pitch_center = {0.0, 0.0, 0.0};
    calibration.left_spatial_endpoint = {-24.76, 12.0, 3.48};
    calibration.right_spatial_endpoint = {-24.76, -12.0, 3.48};
    calibration.rotate_direction = {-1.0, 0.0, 0.0};
    calibration.use_rotate_direction = true;
    calibration.rotation_order = couple_joint::RotationOrder::PitchThenRoll;
    return calibration;
}

DoubleCoupleJoint::Config CreateDoubleCoupleJointConfig(DoubleCoupleJointAxis axis) {
    DoubleCoupleJoint::Config config{};
    config.motorParams1 = MotorParams{"adapter1", 0, 1, MotorInitAuto{}};
    config.motorParams2 = MotorParams{"adapter1", 0, 2, MotorInitAuto{}};
    config.axis = axis;
    config.calibration = CreateValidCalibrationInput();
    config.limit = Range<float>{-3.14f, 3.14f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    config.reverse = false;
    return config;
}

DoubleCoupleJoint::Config CreateH130AnkleDoubleCoupleJointConfig(DoubleCoupleJointAxis axis,
                                                                 MotorProperties ranges) {
    DoubleCoupleJoint::Config config{};
    config.motorParams1 = MotorParams{"adapter1", 0, 5, MotorInitParams{ranges}};
    config.motorParams2 = MotorParams{"adapter1", 0, 6, MotorInitParams{ranges}};
    config.axis = axis;
    config.calibration = CreateH130AnkleCalibrationInput();
    config.motorLimit1 = Range<float>{-1.2217304763960306f, 1.2217304763960306f};
    config.motorLimit2 = Range<float>{-1.2217304763960306f, 1.2217304763960306f};
    config.reverse = false;
    if (axis == DoubleCoupleJointAxis::Pitch) {
        config.limit = Range<float>{-0.8726646f, 0.5235988f};
        config.maxSpd = 27.25f;
        config.maxTor = 80.0f;
    } else {
        config.limit = Range<float>{-0.2617994f, 0.2617994f};
        config.maxSpd = 57.37f;
        config.maxTor = 38.0f;
    }
    return config;
}

MotorProperties CreateMotorRanges(float position_limit = 12.5f, float speed_limit = 18.0f,
                                  float torque_limit = 30.0f, float current_limit = 30.0f) {
    MotorProperties ranges{};
    ranges.kp = Range<float>{0.0f, 500.0f};
    ranges.kd = Range<float>{0.0f, 5.0f};
    ranges.position = Range<float>{-position_limit, position_limit};
    ranges.speed = Range<float>{-speed_limit, speed_limit};
    ranges.torque = Range<float>{-torque_limit, torque_limit};
    ranges.current = Range<float>{-current_limit, current_limit};
    ranges.kt = 1.0f;
    return ranges;
}

DoubleCoupleJoint::Config CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis axis,
                                                                  MotorProperties ranges) {
    auto config = CreateDoubleCoupleJointConfig(axis);
    config.motorParams1.initParam = MotorInitParams{ranges};
    config.motorParams2.initParam = MotorInitParams{ranges};
    return config;
}

nlohmann::json CreateDoubleCoupleJointConfigJson(const std::string& axis) {
    return nlohmann::json{
        {"motorParams1",
         {{"adapterId", "adapter1"},
          {"busId", 0},
          {"motorId", 1},
          {"initParam", nlohmann::json::object()}}},
        {"motorParams2",
         {{"adapterId", "adapter1"},
          {"busId", 0},
          {"motorId", 2},
          {"initParam", nlohmann::json::object()}}},
        {"axis", axis},
        {"calibration",
         {{"leftJointCenter", {{"x", -35.462080740}, {"y", 29.491263548}, {"z", 58.546474855}}},
          {"rightJointCenter", {{"x", -35.462080740}, {"y", -29.491263548}, {"z", 58.546474855}}},
          {"leftPlanarEndpoint", {{"x", -35.5}, {"y", 11.61629736}, {"z", 56.42889465}}},
          {"rightPlanarEndpoint", {{"x", -35.5}, {"y", -11.61629736}, {"z", 56.42889465}}},
          {"rollCenter", {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}}},
          {"pitchCenter", {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}}},
          {"leftSpatialEndpoint", {{"x", -36.6}, {"y", 26.0}, {"z", -5.0}}},
          {"rightSpatialEndpoint", {{"x", -36.6}, {"y", -26.0}, {"z", -5.0}}},
          {"rotationOrder", "PitchThenRoll"}}},
        {"limit", {{"min", -3.14}, {"max", 3.14}}},
        {"maxSpd", 2.0},
        {"maxTor", 3.0},
        {"reverse", false},
    };
}

struct PairedDoubleCoupleFixture {
    std::shared_ptr<FakeAdapterHarness> adapter;
    std::shared_ptr<MasterJoint> master;
    std::shared_ptr<SlaveJoint> slave;
    Motor* motor1{nullptr};
    Motor* motor2{nullptr};
};

struct RJointFixture {
    std::shared_ptr<FakeAdapterHarness> adapter;
    std::shared_ptr<RJoint> joint;
    Motor* motor{nullptr};
};

struct ContinuousJointFixture {
    std::shared_ptr<FakeAdapterHarness> adapter;
    std::shared_ptr<ContinuousJoint> joint;
    Motor* motor{nullptr};
};

struct ExpectedMotorPositions {
    float motor1{0.0f};
    float motor2{0.0f};
};

struct ExpectedMotorValues {
    float motor1{0.0f};
    float motor2{0.0f};
};

struct ExpectedMotorCurrents {
    float motor1{0.0f};
    float motor2{0.0f};
};

PairedDoubleCoupleFixture CreatePairedDoubleCoupleFixture(
    const std::string& adapter_name, MotorProperties ranges, bool master_reverse = false,
    bool slave_reverse = false, std::optional<Range<float>> motor_limit1 = std::nullopt,
    std::optional<Range<float>> motor_limit2 = std::nullopt,
    std::optional<Range<float>> master_limit = std::nullopt,
    std::optional<Range<float>> slave_limit = std::nullopt,
    std::optional<couple_joint::CalibrationInput> calibration = std::nullopt) {
    auto adapter = CreateFakeAdapterHarness(adapter_name);
    AdapterMap adapters{{"adapter1", adapter->Adapter()}};

    auto slave_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Pitch, ranges);
    auto master_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Roll, ranges);
    slave_config.reverse = slave_reverse;
    master_config.reverse = master_reverse;
    if (calibration) {
        slave_config.calibration = *calibration;
        master_config.calibration = *calibration;
    }
    if (slave_limit) {
        slave_config.limit = *slave_limit;
    }
    if (master_limit) {
        master_config.limit = *master_limit;
    }
    slave_config.motorLimit1 = motor_limit1;
    slave_config.motorLimit2 = motor_limit2;
    master_config.motorLimit1 = motor_limit1;
    master_config.motorLimit2 = motor_limit2;
    auto slave = std::static_pointer_cast<SlaveJoint>(DoubleCoupleJoint::CreateJoint(slave_config));
    auto motor1 = JointManager<>::CreateMotor(master_config.motorParams1, adapters);
    auto motor2 = JointManager<>::CreateMotor(master_config.motorParams2, adapters);
    SyncFakeRangeSnapshot(*adapter, motor1, ranges);
    SyncFakeRangeSnapshot(*adapter, motor2, ranges);
    adapter->SetReplyMode(FakeReplyMode::Automatic);
    auto master = std::static_pointer_cast<MasterJoint>(
        DoubleCoupleJoint::CreateJoint(master_config, motor1, motor2, slave));

    adapter->SetReplyMode(FakeReplyMode::Manual);
    adapter->ClearCommandRecords();
    return PairedDoubleCoupleFixture{adapter, master, slave, motor1, motor2};
}

PairedDoubleCoupleFixture CreateH130AnklePairedDoubleCoupleFixture(const std::string& adapter_name,
                                                                   MotorProperties ranges) {
    auto adapter = CreateFakeAdapterHarness(adapter_name);
    AdapterMap adapters{{"adapter1", adapter->Adapter()}};

    auto slave_config =
        CreateH130AnkleDoubleCoupleJointConfig(DoubleCoupleJointAxis::Pitch, ranges);
    auto master_config =
        CreateH130AnkleDoubleCoupleJointConfig(DoubleCoupleJointAxis::Roll, ranges);
    auto slave = std::static_pointer_cast<SlaveJoint>(DoubleCoupleJoint::CreateJoint(slave_config));
    auto motor1 = JointManager<>::CreateMotor(master_config.motorParams1, adapters);
    auto motor2 = JointManager<>::CreateMotor(master_config.motorParams2, adapters);
    SyncFakeRangeSnapshot(*adapter, motor1, ranges);
    SyncFakeRangeSnapshot(*adapter, motor2, ranges);
    adapter->SetReplyMode(FakeReplyMode::Automatic);
    auto master = std::static_pointer_cast<MasterJoint>(
        DoubleCoupleJoint::CreateJoint(master_config, motor1, motor2, slave));

    adapter->SetReplyMode(FakeReplyMode::Manual);
    adapter->ClearCommandRecords();
    return PairedDoubleCoupleFixture{adapter, master, slave, motor1, motor2};
}

RJointFixture CreateRJointFixture(const std::string& adapter_name, MotorProperties ranges,
                                  bool reverse = false) {
    auto adapter = CreateFakeAdapterHarness(adapter_name);
    AdapterMap adapters{{"adapter1", adapter->Adapter()}};

    RJoint::Config config{};
    config.motorParams = MotorParams{"adapter1", 0, 1, MotorInitParams{ranges}};
    config.limit = Range<float>{-1.0f, 1.0f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    config.reverse = reverse;

    auto motor = JointManager<>::CreateMotor(config.motorParams, adapters);
    SyncFakeRangeSnapshot(*adapter, motor, ranges);
    auto joint = std::make_shared<RJoint>(config, motor);
    adapter->ClearCommandRecords();
    return RJointFixture{adapter, joint, motor};
}

ContinuousJointFixture CreateContinuousJointFixture(const std::string& adapter_name,
                                                    MotorProperties ranges, bool reverse = false) {
    auto adapter = CreateFakeAdapterHarness(adapter_name);
    AdapterMap adapters{{"adapter1", adapter->Adapter()}};

    ContinuousJoint::Config config{};
    config.motorParams = MotorParams{"adapter1", 0, 1, MotorInitParams{ranges}};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    config.reverse = reverse;

    auto motor = JointManager<>::CreateMotor(config.motorParams, adapters);
    SyncFakeRangeSnapshot(*adapter, motor, ranges);
    auto joint = std::make_shared<ContinuousJoint>(config, motor);
    adapter->ClearCommandRecords();
    return ContinuousJointFixture{adapter, joint, motor};
}

nlohmann::json CreateRJointConfigJson(uint32_t motor_id = 1) {
    return nlohmann::json{
        {"motorParams",
         {{"adapterId", "adapter1"},
          {"busId", 0},
          {"motorId", motor_id},
          {"initParam",
           {{"ranges",
             {{"kp", {{"min", 0.0}, {"max", 500.0}}},
              {"kd", {{"min", 0.0}, {"max", 5.0}}},
              {"position", {{"min", -12.5}, {"max", 12.5}}},
              {"speed", {{"min", -18.0}, {"max", 18.0}}},
              {"torque", {{"min", -30.0}, {"max", 30.0}}},
              {"current", {{"min", -30.0}, {"max", 30.0}}},
              {"kt", 1.0}}}}}}},
        {"limit", {{"min", -1.0}, {"max", 1.0}}},
        {"maxSpd", 2.0},
        {"maxTor", 3.0},
        {"reverse", false},
    };
}

nlohmann::json CreateContinuousJointConfigJson(uint32_t motor_id = 1) {
    auto config = CreateRJointConfigJson(motor_id);
    config.erase("limit");
    return config;
}

float DecodePvtPosition(const FakeCommandRecord& command,
                        [[maybe_unused]] const MotorProperties& ranges) {
    EXPECT_EQ(command.kind, FakeCommandKind::PVTControl);
    return std::get<FakePVTControlPayload>(command.payload).position;
}

float DecodePvtSpeed(const FakeCommandRecord& command,
                     [[maybe_unused]] const MotorProperties& ranges) {
    EXPECT_EQ(command.kind, FakeCommandKind::PVTControl);
    return std::get<FakePVTControlPayload>(command.payload).speed;
}

float DecodePvtTorque(const FakeCommandRecord& command,
                      [[maybe_unused]] const MotorProperties& ranges) {
    EXPECT_EQ(command.kind, FakeCommandKind::PVTControl);
    return std::get<FakePVTControlPayload>(command.payload).torque;
}

float DecodePvtKp(const FakeCommandRecord& command) {
    EXPECT_EQ(command.kind, FakeCommandKind::PVTControl);
    return std::get<FakePVTControlPayload>(command.payload).kp;
}

float DecodePvtKd(const FakeCommandRecord& command) {
    EXPECT_EQ(command.kind, FakeCommandKind::PVTControl);
    return std::get<FakePVTControlPayload>(command.payload).kd;
}

float DecodeSpeedCommand(const FakeCommandRecord& command) {
    EXPECT_EQ(command.kind, FakeCommandKind::SpdControl);
    return std::get<FakeSpdControlPayload>(command.payload).speed;
}

float DecodePosSpeed(const FakeCommandRecord& command) {
    EXPECT_EQ(command.kind, FakeCommandKind::PosControl);
    return std::get<FakePosControlPayload>(command.payload).speed;
}

float DecodePosPosition(const FakeCommandRecord& command) {
    EXPECT_EQ(command.kind, FakeCommandKind::PosControl);
    return std::get<FakePosControlPayload>(command.payload).position;
}

float DecodeCurrentCommand(const FakeCommandRecord& command) {
    if (command.kind == FakeCommandKind::Stop) {
        return std::get<FakeStopPayload>(command.payload).current;
    }
    EXPECT_EQ(command.kind, FakeCommandKind::CurControl);
    return std::get<FakeCurControlPayload>(command.payload).current;
}

float DecodeTorqueCommand(const FakeCommandRecord& command) {
    EXPECT_EQ(command.kind, FakeCommandKind::TorControl);
    return std::get<FakeTorControlPayload>(command.payload).torque;
}

const FakeCommandRecord* FindSentMessage(const std::vector<FakeCommandRecord>& messages,
                                         uint32_t motor_id) {
    const auto it = std::find_if(messages.begin(), messages.end(), [motor_id](const auto& msg) {
        return msg.motor_idx == static_cast<int>(motor_id);
    });
    return it == messages.end() ? nullptr : &(*it);
}

/**
 * @brief 断言所有命令的 CAN FD 帧标志一致
 *
 * 电机在构造期会发一帧固件版本查询（0x32），它早于用例设置 CAN FD，因此不计入断言。
 */
void ExpectAllMessagesUseCanFd(const std::vector<FakeCommandRecord>& messages, bool expected) {
    bool checked_any = false;
    for (const auto& message : messages) {
        if (message.kind == FakeCommandKind::GetParameter) {
            const auto* payload = std::get_if<FakeGetParameterPayload>(&message.payload);
            if (payload != nullptr && payload->parameter == MotorParameter::FirmwareVersion) {
                continue;
            }
        }
        EXPECT_EQ(CanFrameFlagsUseCanFd(message.raw_frame_flags), expected);
        checked_any = true;
    }
    EXPECT_TRUE(checked_any);
}

void ExpectStopMessagesWithZeroCurrent(const std::vector<FakeCommandRecord>& messages,
                                       size_t expected_count) {
    ASSERT_EQ(messages.size(), expected_count);
    for (const auto& message : messages) {
        EXPECT_EQ(message.kind, FakeCommandKind::Stop);
        EXPECT_NEAR(DecodeCurrentCommand(message), 0.0f, 0.001f);
    }
}

void ExpectSpeedCommands(const std::vector<FakeCommandRecord>& messages, float expected_motor1,
                         float expected_motor2, float tolerance = 0.002f) {
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    EXPECT_EQ(motor1_message->kind, FakeCommandKind::SpdControl);
    EXPECT_EQ(motor2_message->kind, FakeCommandKind::SpdControl);
    EXPECT_NEAR(DecodeSpeedCommand(*motor1_message), expected_motor1, tolerance);
    EXPECT_NEAR(DecodeSpeedCommand(*motor2_message), expected_motor2, tolerance);
}

void ExpectTorqueCommands(const std::vector<FakeCommandRecord>& messages, float expected_motor1,
                          float expected_motor2, float tolerance = 0.02f) {
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    EXPECT_EQ(motor1_message->kind, FakeCommandKind::TorControl);
    EXPECT_EQ(motor2_message->kind, FakeCommandKind::TorControl);
    EXPECT_NEAR(DecodeTorqueCommand(*motor1_message), expected_motor1, tolerance);
    EXPECT_NEAR(DecodeTorqueCommand(*motor2_message), expected_motor2, tolerance);
}

void ExpectCurrentCommands(const std::vector<FakeCommandRecord>& messages, float expected_motor1,
                           float expected_motor2, float tolerance = 0.02f) {
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    EXPECT_EQ(motor1_message->kind, FakeCommandKind::CurControl);
    EXPECT_EQ(motor2_message->kind, FakeCommandKind::CurControl);
    EXPECT_NEAR(DecodeCurrentCommand(*motor1_message), expected_motor1, tolerance);
    EXPECT_NEAR(DecodeCurrentCommand(*motor2_message), expected_motor2, tolerance);
}

void ExpectPositionHoldMessage(const FakeCommandRecord& message, float expected_position,
                               float expected_speed, float expected_current,
                               int expected_feedback) {
    ASSERT_EQ(message.kind, FakeCommandKind::PosControl);
    const auto& payload = std::get<FakePosControlPayload>(message.payload);
    EXPECT_NEAR(payload.position, expected_position, 0.001f);
    EXPECT_NEAR(payload.speed, expected_speed, 0.01f);
    EXPECT_NEAR(payload.current, expected_current, 0.1f);
    EXPECT_EQ(payload.feedback_type, expected_feedback);
}

void ExpectPvtTorqueCommands(const std::vector<FakeCommandRecord>& messages, float torque1,
                             float torque2, const MotorProperties& ranges) {
    ASSERT_EQ(messages.size(), 2u);
    for (int id : {1, 2}) {
        const auto* message = FindSentMessage(messages, id);
        ASSERT_NE(message, nullptr);
        ASSERT_EQ(message->kind, FakeCommandKind::PVTControl);
        EXPECT_FLOAT_EQ(DecodePvtKp(*message), 0.0f);
        EXPECT_NEAR(DecodePvtTorque(*message, ranges), id == 1 ? torque1 : torque2, 0.025f);
    }
}

void ExpectPositionHoldMessages(const std::vector<FakeCommandRecord>& messages,
                                const ExpectedMotorPositions& expected_positions,
                                const MotorProperties& ranges, int expected_feedback = 1) {
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    const float current = std::min(3.0f / ranges.kt, ranges.current.max);
    ExpectPositionHoldMessage(*motor1_message, expected_positions.motor1, ranges.speed.max, current,
                              expected_feedback);
    ExpectPositionHoldMessage(*motor2_message, expected_positions.motor2, ranges.speed.max, current,
                              expected_feedback);
}

void EmitDoubleCoupleFeedback(PairedDoubleCoupleFixture& fixture,
                              const ExpectedMotorPositions& positions, float speed = 0.0f,
                              float torque = 0.0f) {
    EmitFakeFeedback(*fixture.adapter, fixture.motor1, positions.motor1, speed, torque);
    EmitFakeFeedback(*fixture.adapter, fixture.motor2, positions.motor2, speed, torque);
    fixture.adapter->ClearCommandRecords();
}

void EmitMotor1Feedback(PairedDoubleCoupleFixture& fixture, float position, float speed = 0.0f,
                        float torque = 0.0f) {
    EmitFakeFeedback(*fixture.adapter, fixture.motor1, position, speed, torque);
    fixture.adapter->ClearCommandRecords();
}

void EmitMotor2Feedback(PairedDoubleCoupleFixture& fixture, float position, float speed = 0.0f,
                        float torque = 0.0f) {
    EmitFakeFeedback(*fixture.adapter, fixture.motor2, position, speed, torque);
    fixture.adapter->ClearCommandRecords();
}

class TestableJointManager : public JointManager<> {
public:
    using JointManager<>::AddJoint;

    void RegisterAdapter(const std::string& name, BaseAdapterPtr adapter) {
        adapters_[name] = adapter;
    }

    std::size_t TrackedMotorCount() const {
        return motors_.size();
    }

    BaseAdapterPtr Adapter(const std::string& name) const {
        return adapters_.at(name);
    }

    const AdapterMap& AdaptersByName() const {
        return adapters_;
    }

    const AdapterMap& AdaptersById() const {
        return adapters_by_id_;
    }

    void TrackTestAdapter(const std::string& name, BaseAdapterPtr adapter) {
        TrackAdapter(name, adapter);
    }
};

class TestableDefaultJointManager : public DefaultJointManager {
public:
    using DefaultJointManager::AddJoint;
    using DefaultJointManager::DefaultJointManager;

    void RegisterAdapter(const std::string& name, BaseAdapterPtr adapter) {
        adapters_[name] = adapter;
    }

    BaseAdapterPtr Adapter(const std::string& name) const {
        return adapters_.at(name);
    }
};

ExpectedMotorPositions CalculateExpectedMotorPositions(double roll, double pitch) {
    couple_joint::MechanismSolver solver(CreateValidCalibrationInput());
    const auto result = solver.SolveInverse(roll, pitch);
    EXPECT_TRUE(result.status == couple_joint::SolveStatus::Success ||
                result.status == couple_joint::SolveStatus::NearSingular);
    const auto state = solver.GetState();
    const auto params = solver.GetParameters();
    return ExpectedMotorPositions{static_cast<float>(state.theta1 - params.initial_theta1),
                                  static_cast<float>(state.theta2 - params.initial_theta2)};
}

void ExpectNearSingularPose(double roll, double pitch) {
    couple_joint::MechanismSolver solver(CreateValidCalibrationInput());
    const auto result = solver.SolveInverse(roll, pitch);
    ASSERT_EQ(result.status, couple_joint::SolveStatus::NearSingular);
    EXPECT_TRUE(result.transmission.near_singular);
    EXPECT_FALSE(result.transmission.singular_fu);
    EXPECT_FALSE(result.transmission.singular_fq);
    EXPECT_FALSE(result.transmission.singular_j);
}

std::optional<double> ScaleForRange(double value, const Range<float>& range) {
    constexpr double kRangeTolerance = 1.0e-6;
    if (!std::isfinite(value)) {
        return std::nullopt;
    }
    if (value >= range.min - kRangeTolerance && value <= range.max + kRangeTolerance) {
        return 1.0;
    }
    if (value > range.max) {
        return value > 0.0 && range.max >= 0.0f
                   ? std::optional<double>{std::clamp(static_cast<double>(range.max) / value, 0.0,
                                                      1.0)}
                   : std::nullopt;
    }
    return value < 0.0 && range.min <= 0.0f
               ? std::optional<double>{std::clamp(static_cast<double>(range.min) / value, 0.0, 1.0)}
               : std::nullopt;
}

ExpectedMotorValues ApplyExpectedUniformScale(ExpectedMotorValues expected,
                                              const Range<float>& range) {
    const auto scale1 = ScaleForRange(expected.motor1, range);
    const auto scale2 = ScaleForRange(expected.motor2, range);
    EXPECT_TRUE(scale1.has_value());
    EXPECT_TRUE(scale2.has_value());
    const double scale = std::min(scale1.value_or(0.0), scale2.value_or(0.0));
    expected.motor1 = static_cast<float>(expected.motor1 * scale);
    expected.motor2 = static_cast<float>(expected.motor2 * scale);
    return expected;
}

ExpectedMotorValues CalculateExpectedMotorSpeeds(DoubleCoupleJointAxis axis, double roll,
                                                 double pitch, float speed,
                                                 const MotorProperties& ranges) {
    couple_joint::MechanismSolver solver(CreateValidCalibrationInput());
    const auto inverse_result = solver.SolveInverse(roll, pitch);
    EXPECT_TRUE(inverse_result.status == couple_joint::SolveStatus::Success ||
                inverse_result.status == couple_joint::SolveStatus::NearSingular);

    const auto velocity = axis == DoubleCoupleJointAxis::Roll
                              ? solver.SolveInverseVelocity(speed, 0.0)
                              : solver.SolveInverseVelocity(0.0, speed);
    EXPECT_TRUE(velocity.status == couple_joint::SolveStatus::Success ||
                velocity.status == couple_joint::SolveStatus::NearSingular);

    return ApplyExpectedUniformScale(ExpectedMotorValues{static_cast<float>(velocity.theta1),
                                                         static_cast<float>(velocity.theta2)},
                                     ranges.speed);
}

ExpectedMotorValues CalculateExpectedCombinedMotorSpeeds(double roll, double pitch,
                                                         float roll_speed, float pitch_speed,
                                                         const MotorProperties& ranges) {
    couple_joint::MechanismSolver solver(CreateValidCalibrationInput());
    const auto inverse_result = solver.SolveInverse(roll, pitch);
    EXPECT_TRUE(inverse_result.status == couple_joint::SolveStatus::Success ||
                inverse_result.status == couple_joint::SolveStatus::NearSingular);
    const auto velocity = solver.SolveInverseVelocity(roll_speed, pitch_speed);
    EXPECT_TRUE(velocity.status == couple_joint::SolveStatus::Success ||
                velocity.status == couple_joint::SolveStatus::NearSingular);
    return ApplyExpectedUniformScale(ExpectedMotorValues{static_cast<float>(velocity.theta1),
                                                         static_cast<float>(velocity.theta2)},
                                     ranges.speed);
}

ExpectedMotorValues CalculateExpectedMotorTorques(DoubleCoupleJointAxis axis, double roll,
                                                  double pitch, float torque,
                                                  const MotorProperties& ranges) {
    couple_joint::MechanismSolver solver(CreateValidCalibrationInput());
    const auto inverse_result = solver.SolveInverse(roll, pitch);
    EXPECT_TRUE(inverse_result.status == couple_joint::SolveStatus::Success ||
                inverse_result.status == couple_joint::SolveStatus::NearSingular);

    const auto torque_result = axis == DoubleCoupleJointAxis::Roll
                                   ? solver.SolveInverseTorque(torque, 0.0)
                                   : solver.SolveInverseTorque(0.0, torque);
    EXPECT_TRUE(torque_result.status == couple_joint::SolveStatus::Success ||
                torque_result.status == couple_joint::SolveStatus::NearSingular);

    return ApplyExpectedUniformScale(ExpectedMotorValues{static_cast<float>(torque_result.theta1),
                                                         static_cast<float>(torque_result.theta2)},
                                     ranges.torque);
}

ExpectedMotorValues CalculateExpectedMotorGains(double roll, double pitch, float roll_gain,
                                                float pitch_gain) {
    couple_joint::MechanismSolver solver(CreateValidCalibrationInput());
    const auto inverse_result = solver.SolveInverse(roll, pitch);
    EXPECT_TRUE(inverse_result.status == couple_joint::SolveStatus::Success ||
                inverse_result.status == couple_joint::SolveStatus::NearSingular);
    const auto& jacobian = inverse_result.transmission.velocity_jacobian;
    return ExpectedMotorValues{
        static_cast<float>(jacobian.m00 * jacobian.m00 * roll_gain +
                           jacobian.m10 * jacobian.m10 * pitch_gain),
        static_cast<float>(jacobian.m01 * jacobian.m01 * roll_gain +
                           jacobian.m11 * jacobian.m11 * pitch_gain),
    };
}

ExpectedMotorValues CalculateExpectedCombinedMotorTorques(double roll, double pitch,
                                                          float roll_torque, float pitch_torque,
                                                          const MotorProperties& ranges) {
    couple_joint::MechanismSolver solver(CreateValidCalibrationInput());
    const auto inverse_result = solver.SolveInverse(roll, pitch);
    EXPECT_TRUE(inverse_result.status == couple_joint::SolveStatus::Success ||
                inverse_result.status == couple_joint::SolveStatus::NearSingular);
    const auto torque_result = solver.SolveInverseTorque(roll_torque, pitch_torque);
    EXPECT_TRUE(torque_result.status == couple_joint::SolveStatus::Success ||
                torque_result.status == couple_joint::SolveStatus::NearSingular);
    return ApplyExpectedUniformScale(ExpectedMotorValues{static_cast<float>(torque_result.theta1),
                                                         static_cast<float>(torque_result.theta2)},
                                     ranges.torque);
}

ExpectedMotorCurrents CalculateExpectedMotorCurrents(DoubleCoupleJointAxis axis, double roll,
                                                     double pitch, float current,
                                                     const MotorProperties& ranges) {
    couple_joint::MechanismSolver solver(CreateValidCalibrationInput());
    const auto inverse_result = solver.SolveInverse(roll, pitch);
    EXPECT_TRUE(inverse_result.status == couple_joint::SolveStatus::Success ||
                inverse_result.status == couple_joint::SolveStatus::NearSingular);

    const double unit_axis_torque = std::copysign(1.0, static_cast<double>(current));
    const auto torque_result = axis == DoubleCoupleJointAxis::Roll
                                   ? solver.SolveInverseTorque(unit_axis_torque, 0.0)
                                   : solver.SolveInverseTorque(0.0, unit_axis_torque);
    EXPECT_TRUE(torque_result.status == couple_joint::SolveStatus::Success ||
                torque_result.status == couple_joint::SolveStatus::NearSingular);

    const double weight1 = torque_result.theta1 / ranges.kt;
    const double weight2 = torque_result.theta2 / ranges.kt;
    const double weight_sum = std::abs(weight1) + std::abs(weight2);
    ExpectedMotorCurrents expected{static_cast<float>(weight1 * std::abs(current) / weight_sum),
                                   static_cast<float>(weight2 * std::abs(current) / weight_sum)};

    const auto scale1 = ScaleForRange(expected.motor1, ranges.current);
    const auto scale2 = ScaleForRange(expected.motor2, ranges.current);
    EXPECT_TRUE(scale1.has_value());
    EXPECT_TRUE(scale2.has_value());
    const double scale = std::min(scale1.value_or(0.0), scale2.value_or(0.0));
    expected.motor1 = static_cast<float>(expected.motor1 * scale);
    expected.motor2 = static_cast<float>(expected.motor2 * scale);
    return expected;
}

bool JsonArrayContainsString(const nlohmann::json& array, const std::string& value) {
    return std::find(array.begin(), array.end(), value) != array.end();
}

struct OptionalMacroConfigOwner {
    struct Config {
        // clang-format off
        GENERATE_CONFIG(OptionalMacroTestJoint,
                        ((requiredValue, int, Integer, true, "Required value"))
                        ((optionalValue, std::optional<int>, Integer, false, "Optional value")))
        // clang-format on
    };
};

}  // namespace

// ============================================================
// JointError & JointException
// ============================================================

TEST(JointErrorTest, EnumValues) {
    EXPECT_EQ(static_cast<int>(JointError::kDoubleCoupleJointNotPaired), 0);
    EXPECT_EQ(static_cast<int>(JointError::kDoubleCoupleJointDuplicatePair), 1);
    EXPECT_EQ(static_cast<int>(JointError::kDoubleCoupleJointSameMotor), 2);
    EXPECT_EQ(static_cast<int>(JointError::kDoubleCoupleJointPending), 3);
    EXPECT_EQ(static_cast<int>(JointError::kContinuousJointUnsupportedOperation), 11);
}

TEST(JointExceptionTest, What) {
    JointException ex(JointError::kDoubleCoupleJointNotPaired, "not paired yet");
    EXPECT_STREQ(ex.what(), "not paired yet");
}

TEST(JointExceptionTest, Error) {
    JointException ex(JointError::kDoubleCoupleJointSolveFailed, "solve failed");
    EXPECT_EQ(ex.Error(), JointError::kDoubleCoupleJointSolveFailed);
}

TEST(JointExceptionTest, InheritsRuntimeError) {
    JointException ex(JointError::kDoubleCoupleJointDuplicatePair, "dup");
    EXPECT_NE(dynamic_cast<std::runtime_error*>(&ex), nullptr);
}

TEST(JointExceptionTest, CaughtAsStdException) {
    try {
        throw JointException(JointError::kDoubleCoupleJointSameMotor, "same motor");
    } catch (const std::exception& e) {
        EXPECT_STREQ(e.what(), "same motor");
        return;
    }
    FAIL() << "JointException should be caught as std::exception";
}

// ============================================================
// MakePairKey
// ============================================================

class MakePairKeyTest : public ::testing::Test {
protected:
    MotorParams a{"adapter1", 0, 1, MotorInitAuto{}};
    MotorParams b{"adapter1", 0, 2, MotorInitAuto{}};
    MotorParams c{"adapter1", 0, 3, MotorInitAuto{}};
    MotorParams sameAsA{"adapter1", 0, 1, MotorInitModel{MotorModel::EC_A2806_P2}};
};

TEST_F(MakePairKeyTest, OrderNowMatters) {
    auto k1 = DoubleCoupleJoint::MakePairKey(a, b);
    auto k2 = DoubleCoupleJoint::MakePairKey(b, a);
    EXPECT_NE(k1, k2);  // 顺序不再自动排序，两次调用中电机顺序必须一致
    EXPECT_FALSE(k1.empty());
    EXPECT_FALSE(k2.empty());
}

TEST_F(MakePairKeyTest, ConsistentOrderProducesSameKey) {
    auto k1 = DoubleCoupleJoint::MakePairKey(a, b);
    auto k2 = DoubleCoupleJoint::MakePairKey(a, b);
    EXPECT_EQ(k1, k2);
}

TEST_F(MakePairKeyTest, SamePhysicalMotorReturnsEmpty) {
    auto k = DoubleCoupleJoint::MakePairKey(a, sameAsA);
    EXPECT_TRUE(k.empty());
}

TEST_F(MakePairKeyTest, DifferentPairsProduceDifferentKeys) {
    auto k1 = DoubleCoupleJoint::MakePairKey(a, b);
    auto k2 = DoubleCoupleJoint::MakePairKey(a, c);
    EXPECT_NE(k1, k2);
}

TEST_F(MakePairKeyTest, DifferentAdapter) {
    MotorParams otherAdapter{"adapter2", 0, 1, MotorInitAuto{}};
    auto k1 = DoubleCoupleJoint::MakePairKey(a, b);
    auto k2 = DoubleCoupleJoint::MakePairKey(a, otherAdapter);
    EXPECT_NE(k1, k2);
}

TEST_F(MakePairKeyTest, DifferentBus) {
    MotorParams otherBus{"adapter1", 1, 1, MotorInitAuto{}};
    auto k1 = DoubleCoupleJoint::MakePairKey(a, b);
    auto k2 = DoubleCoupleJoint::MakePairKey(a, otherBus);
    EXPECT_NE(k1, k2);
}

TEST_F(MakePairKeyTest, DifferentSlaveIdOnSameBus) {
    MotorParams slaveZero{"adapter1", 1, 4, MotorInitAuto{}};
    MotorParams slaveThree = slaveZero;
    slaveThree.slaveId = 3;

    // 同 adapter/bus/motor 但 slaveId 不同 => 不是同一台物理电机
    EXPECT_FALSE(DoubleCoupleJoint::MakePairKey(slaveZero, slaveThree).empty());

    auto k1 = DoubleCoupleJoint::MakePairKey(a, slaveZero);
    auto k2 = DoubleCoupleJoint::MakePairKey(a, slaveThree);
    EXPECT_NE(k1, k2);
}

TEST_F(MakePairKeyTest, ExplicitSlaveIdMatchesHandEncodedBusId) {
    MotorParams encoded{"adapter1", (3 << 16) | 4, 7, MotorInitAuto{}};
    MotorParams explicitIds{"adapter1", 4, 7, MotorInitAuto{}};
    explicitIds.slaveId = 3;

    EXPECT_TRUE(DoubleCoupleJoint::MakePairKey(encoded, explicitIds).empty());
}

// ============================================================
// JointManager type system
// ============================================================

TEST(JointManagerTest, DefaultJointManagerType) {
    using Mgr = JointManager<RJoint, ContinuousJoint, DoubleCoupleJoint>;
    EXPECT_TRUE((std::is_same_v<Mgr, DefaultJointManager>) );
}

TEST(JointManagerTest, DefaultConstructible) {
    EXPECT_TRUE(std::is_default_constructible_v<DefaultJointManager>);
}

TEST(JointManagerTest, GetAvailableJointTypes) {
    DefaultJointManager mgr;
    auto types = mgr.GetAvailableJointTypes();
    EXPECT_EQ(types.size(), 3u);

    bool hasRJoint = false, hasContinuous = false, hasDM = false;
    for (const auto& t : types) {
        if (t == "RJoint")
            hasRJoint = true;
        if (t == "ContinuousJoint")
            hasContinuous = true;
        if (t == "DoubleCoupleJoint")
            hasDM = true;
    }
    EXPECT_TRUE(hasRJoint);
    EXPECT_TRUE(hasContinuous);
    EXPECT_TRUE(hasDM);
}

TEST(JointManagerTest, GetJointSchema) {
    DefaultJointManager mgr;
    auto schemas = mgr.GetJointSchema();
    EXPECT_EQ(schemas.size(), 3u);
    EXPECT_FALSE(schemas["RJoint"].empty());
    EXPECT_FALSE(schemas["ContinuousJoint"].empty());
    EXPECT_FALSE(schemas["DoubleCoupleJoint"].empty());
}

TEST(JointManagerTest, GetJointReturnsNullForMissing) {
    DefaultJointManager mgr;
    EXPECT_EQ(mgr.GetJoint("nonexistent"), nullptr);
}

TEST(JointManagerTest, DefaultManagerExposesConfigAddJointTemplate) {
    DefaultJointManager manager;
    RJoint::Config config{};
    config.motorParams = MotorParams{"missing_adapter", 0, 1, MotorInitAuto{}};
    config.limit = Range<float>{-1.0f, 1.0f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    config.reverse = false;

    EXPECT_THROW((void) manager.AddJoint<RJoint>("r_joint", config), std::invalid_argument);
}

TEST(JointManagerTest, JsonDuplicateNameChecksBeforeConfigParsing) {
    TestableDefaultJointManager manager;
    auto adapter = CreateFakeAdapterHarness("json_duplicate_priority");
    manager.RegisterAdapter("adapter1", adapter->Adapter());

    nlohmann::json first{
        {"type", "RJoint"}, {"name", "same_name"}, {"config", CreateRJointConfigJson()}};
    ASSERT_NE(manager.AddJoint(first), nullptr);

    nlohmann::json duplicate{{"type", "RJoint"},
                             {"name", "same_name"},
                             {"config", {{"motorParams", "invalid_config_shape"}}}};

    try {
        (void) manager.AddJoint(duplicate);
    } catch (const std::invalid_argument& ex) {
        EXPECT_NE(std::string(ex.what()).find("already exists"), std::string::npos);
        return;
    } catch (...) {
        FAIL() << "Duplicate name should not be masked by config parsing errors";
    }
    FAIL() << "Expected duplicate name std::invalid_argument";
}

TEST(JointManagerTest, JsonUnknownTypeDoesNotRequireConfig) {
    DefaultJointManager manager;
    nlohmann::json unknown{{"type", "UnknownJoint"}, {"name", "unknown_without_config"}};

    try {
        (void) manager.AddJoint(unknown);
    } catch (const std::invalid_argument& ex) {
        EXPECT_NE(std::string(ex.what()).find("Unknown joint type"), std::string::npos);
        return;
    } catch (...) {
        FAIL() << "Unknown type should not be masked by config access or parsing errors";
    }
    FAIL() << "Expected unknown type std::invalid_argument";
}

TEST(JointManagerTest, AddsContinuousJointFromJsonWithoutPositionLimit) {
    TestableDefaultJointManager manager;
    auto adapter = CreateFakeAdapterHarness("continuous_json_manager");
    manager.RegisterAdapter("adapter1", adapter->Adapter());
    const nlohmann::json definition{{"type", "ContinuousJoint"},
                                    {"name", "continuous"},
                                    {"config", CreateContinuousJointConfigJson()}};

    const auto joint = manager.AddJoint(definition);

    EXPECT_NE(std::dynamic_pointer_cast<ContinuousJoint>(joint), nullptr);
}

TEST(JointManagerTest, CreatesAdapterFromJsonDefinition) {
    TestableJointManager manager;

    manager.CreateAdapter(nlohmann::json{{"type", "Fake"}, {"name", "json_adapter"}});

    EXPECT_NE(manager.Adapter("json_adapter"), nullptr);
    // 同名适配器重复创建为幂等操作
    EXPECT_NO_THROW(
        manager.CreateAdapter(nlohmann::json{{"type", "Fake"}, {"name", "json_adapter"}}));
}

TEST(JointManagerTest, CreateAdapterFromJsonRejectsInvalidDefinitions) {
    TestableJointManager manager;

    EXPECT_THROW(manager.CreateAdapter(nlohmann::json::array()), std::invalid_argument);
    EXPECT_THROW(manager.CreateAdapter(nlohmann::json("Fake")), std::invalid_argument);
    EXPECT_THROW(manager.CreateAdapter(nlohmann::json{{"name", "missing_type"}}),
                 std::invalid_argument);
    EXPECT_THROW(manager.CreateAdapter(nlohmann::json{{"type", "Fake"}}), std::invalid_argument);
    EXPECT_THROW(manager.CreateAdapter(nlohmann::json{{"type", 1}, {"name", "number_type"}}),
                 std::invalid_argument);
    EXPECT_THROW(manager.CreateAdapter(nlohmann::json{{"type", "Fake"}, {"name", nullptr}}),
                 std::invalid_argument);
    // 未知类型的适配器由插件加载器抛 std::runtime_error，向上原样传播
    EXPECT_THROW(manager.CreateAdapter(nlohmann::json{{"type", "NoSuchPlugin"}, {"name", "x"}}),
                 std::runtime_error);
}

TEST(JointManagerTest, CreateAdapterAcceptsExtendConfiguration) {
    TestableJointManager manager;

    // Fake 插件未导出 MakeAdapterV2，仍按内置默认参数创建成功
    ASSERT_NO_THROW(manager.CreateAdapter("Fake", "extend_adapter",
                                          nlohmann::json{{"buses", nlohmann::json::array()}}));
    EXPECT_NE(manager.Adapter("extend_adapter"), nullptr);
    const auto first = manager.Adapter("extend_adapter");

    // 空对象等价于不带扩展配置：保持幂等，不替换已登记实例
    ASSERT_NO_THROW(manager.CreateAdapter("Fake", "extend_adapter", nlohmann::json::object()));
    EXPECT_EQ(manager.Adapter("extend_adapter"), first);

    // extend 非对象由驱动器拒绝：创建新适配器时同样会先校验，说明 extend 确实被转发
    EXPECT_THROW(manager.CreateAdapter("Fake", "extend_bad_value", nlohmann::json("not_object")),
                 std::runtime_error);
    EXPECT_EQ(manager.AdaptersByName().count("extend_bad_value"), 0u);
}

TEST(JointManagerTest, CreateAdapterFromJsonParsesExtend) {
    TestableJointManager manager;

    manager.CreateAdapter(nlohmann::json{{"type", "Fake"},
                                         {"name", "json_extend"},
                                         {"extend", {{"buses", nlohmann::json::array()}}}});
    EXPECT_NE(manager.Adapter("json_extend"), nullptr);

    // extend 缺省或为 null 视为空对象
    EXPECT_NO_THROW(
        manager.CreateAdapter(nlohmann::json{{"type", "Fake"}, {"name", "json_without_extend"}}));
    EXPECT_NO_THROW(manager.CreateAdapter(
        nlohmann::json{{"type", "Fake"}, {"name", "json_null_extend"}, {"extend", nullptr}}));

    // extend 必须是对象
    EXPECT_THROW(manager.CreateAdapter(nlohmann::json{
                     {"type", "Fake"}, {"name", "json_bad_extend"}, {"extend", "not_object"}}),
                 std::invalid_argument);
    EXPECT_EQ(manager.AdaptersByName().count("json_bad_extend"), 0u);
}

TEST(JointManagerTest, CreateAdapterReappliesExtendToExistingAdapter) {
    TestableJointManager manager;
    manager.CreateAdapter("Fake", "extend_existing");
    const auto existing = manager.Adapter("extend_existing");
    ASSERT_NE(existing, nullptr);

    // 名字已存在且 extend 非空时仍回到驱动器：驱动器把非对象 extend 判为非法并抛
    // runtime_error。若 SDK 在此处提前返回，这次调用不会抛，以此确认 extend 被转发。
    EXPECT_THROW(manager.CreateAdapter("Fake", "extend_existing", nlohmann::json("not_object")),
                 std::runtime_error);
    // 既有实例仍按原样登记，没有被替换或清除
    EXPECT_EQ(manager.Adapter("extend_existing"), existing);
    EXPECT_EQ(manager.AdaptersById().size(), 1u);
}

// ============================================================
// 适配器建立顺序 id 索引
// ============================================================

TEST(JointManagerAdapterLookupTest, IntegerAdapterIdUsesIndexOnly) {
    auto index_adapter = CreateFakeAdapterHarness("lookup_by_index");
    auto name_adapter = CreateFakeAdapterHarness("lookup_by_name");
    // 名称索引里恰好也有一个叫 "0" 的适配器；整数引用必须走 id 索引
    const AdapterMap by_name{{"0", name_adapter->Adapter()}};
    const AdapterMap by_id{{"0", index_adapter->Adapter()}};

    auto motor = JointManager<>::CreateMotor(
        MotorParams{std::uint32_t{0}, 0, 1, MotorInitParams{CreateMotorRanges()}}, by_name, by_id);

    EXPECT_EQ(motor, index_adapter->Adapter()->GetBus(0)->GetMotor(1));
}

TEST(JointManagerAdapterLookupTest, StringAdapterIdUsesNameIndexOnly) {
    auto index_adapter = CreateFakeAdapterHarness("lookup_name_expects_name");
    auto name_adapter = CreateFakeAdapterHarness("lookup_name_expects_index");
    // 与上一条镜像：同一个 "0" 在 id 索引里指向另一个适配器，字符串引用必须走名称索引
    const AdapterMap by_name{{"0", name_adapter->Adapter()}};
    const AdapterMap by_id{{"0", index_adapter->Adapter()}};

    auto motor = JointManager<>::CreateMotor(
        MotorParams{"0", 0, 1, MotorInitParams{CreateMotorRanges()}}, by_name, by_id);

    EXPECT_EQ(motor, name_adapter->Adapter()->GetBus(0)->GetMotor(1));
}

TEST(JointManagerAdapterLookupTest, DoesNotFallBackToTheOtherIndex) {
    auto adapter = CreateFakeAdapterHarness("lookup_no_fallback");
    const AdapterMap by_name{{"adapter1", adapter->Adapter()}};
    // id 索引里只有 "0"，名称索引里只有 "adapter1"
    const AdapterMap by_id{{"0", adapter->Adapter()}};

    // 名称引用命中名称索引，整数引用命中 id 索引
    EXPECT_NO_THROW(JointManager<>::CreateMotor(
        MotorParams{"adapter1", 0, 1, MotorInitParams{CreateMotorRanges()}}, by_name, by_id));
    EXPECT_NO_THROW(JointManager<>::CreateMotor(
        MotorParams{std::uint32_t{0}, 0, 2, MotorInitParams{CreateMotorRanges()}}, by_name, by_id));
    // 整数引用不回退名称索引，字符串引用不回退 id 索引
    EXPECT_THROW(JointManager<>::CreateMotor(MotorParams{std::uint32_t{1}, 0, 3, MotorInitAuto{}},
                                             by_name, by_id),
                 std::invalid_argument);
    EXPECT_THROW(
        JointManager<>::CreateMotor(MotorParams{"0", 0, 4, MotorInitAuto{}}, by_name, by_id),
        std::invalid_argument);
    // 两份索引都没有该引用时仍然报找不到适配器
    EXPECT_THROW(
        JointManager<>::CreateMotor(MotorParams{"missing", 0, 5, MotorInitAuto{}}, by_name, by_id),
        std::invalid_argument);
    EXPECT_THROW(JointManager<>::CreateMotor(MotorParams{std::uint32_t{9}, 0, 6, MotorInitAuto{}},
                                             by_name, by_id),
                 std::invalid_argument);
}

TEST(JointManagerAdapterLookupTest, AssignsCreationOrderIdsInCreationOrder) {
    TestableJointManager manager;
    manager.CreateAdapter("Fake", "order_first");
    manager.CreateAdapter("Fake", "order_second");
    // 同名重复创建不新增 id
    manager.CreateAdapter("Fake", "order_first");

    auto first = manager.Adapter("order_first");
    auto second = manager.Adapter("order_second");
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    ASSERT_EQ(manager.AdaptersById().size(), 2u);
    EXPECT_EQ(manager.AdaptersById().at("0"), first);
    EXPECT_EQ(manager.AdaptersById().at("1"), second);

    const auto ranges = CreateMotorRanges();
    auto motor_by_index0 =
        JointManager<>::CreateMotor(MotorParams{std::uint32_t{0}, 0, 1, MotorInitParams{ranges}},
                                    manager.AdaptersByName(), manager.AdaptersById());
    auto motor_by_index1 =
        JointManager<>::CreateMotor(MotorParams{std::uint32_t{1}, 0, 2, MotorInitParams{ranges}},
                                    manager.AdaptersByName(), manager.AdaptersById());
    auto motor_by_name =
        JointManager<>::CreateMotor(MotorParams{"order_second", 0, 3, MotorInitParams{ranges}},
                                    manager.AdaptersByName(), manager.AdaptersById());

    EXPECT_EQ(motor_by_index0, first->GetBus(0)->GetMotor(1));
    EXPECT_EQ(motor_by_index1, second->GetBus(0)->GetMotor(2));
    EXPECT_EQ(motor_by_name, second->GetBus(0)->GetMotor(3));
}

TEST(JointManagerAdapterLookupTest, AddJointResolvesAdapterIdFromCreationOrder) {
    TestableJointManager manager;
    auto first = CreateFakeAdapterHarness("id_lookup_first");
    auto second = CreateFakeAdapterHarness("id_lookup_second");
    manager.TrackTestAdapter("id_lookup_first", first->Adapter());    // id "0"
    manager.TrackTestAdapter("id_lookup_second", second->Adapter());  // id "1"

    RJoint::Config config{};
    // 整数引用 1 → 建立顺序第二个适配器
    config.motorParams = MotorParams{std::uint32_t{1}, 0, 2, MotorInitParams{CreateMotorRanges()}};
    config.limit = Range<float>{-1.0f, 1.0f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    config.reverse = false;
    auto joint = manager.AddJoint<RJoint>("by_id_joint", config);

    first->ClearCommandRecords();
    second->ClearCommandRecords();
    joint->PosControl(0.1f);

    EXPECT_TRUE(first->GetCommandRecords().empty());
    EXPECT_FALSE(second->GetCommandRecords().empty());
}

// ============================================================
// JSON 配置构造函数
// ============================================================

TEST(JointManagerJsonConfigTest, BuildsAdaptersThenJointsInOrder) {
    auto by_index_config = CreateRJointConfigJson(1);
    // 整数引用按建立顺序索引解析适配器
    by_index_config["motorParams"]["adapterId"] = 0;
    // 老配置里残留的 adapterType 键不属于 schema，被静默忽略
    by_index_config["motorParams"]["adapterType"] = "Fake";
    auto by_name_config = CreateRJointConfigJson(2);
    by_name_config["motorParams"]["adapterId"] = "cfg_adapter";

    const nlohmann::json config{
        {"adapters", nlohmann::json::array({{{"type", "Fake"}, {"name", "cfg_adapter"}}})},
        {"joints",
         nlohmann::json::array(
             {{{"name", "cfg_by_index"}, {"type", "RJoint"}, {"config", by_index_config}},
              {{"name", "cfg_by_name"}, {"type", "RJoint"}, {"config", by_name_config}}})},
    };

    DefaultJointManager manager(config);

    // 关节能建出来即证明适配器先于关节建立，且整数索引与名称两种引用都可解析
    ASSERT_NE(manager.GetJoint("cfg_by_index"), nullptr);
    ASSERT_NE(manager.GetJoint("cfg_by_name"), nullptr);
}

TEST(JointManagerJsonConfigTest, AcceptsEmptyAndPartialConfig) {
    DefaultJointManager empty(nlohmann::json::object());
    EXPECT_EQ(empty.GetJoint("any"), nullptr);

    DefaultJointManager only_adapters(nlohmann::json{
        {"adapters", nlohmann::json::array({{{"type", "Fake"}, {"name", "solo"}}})}});
    EXPECT_EQ(only_adapters.GetJoint("any"), nullptr);

    DefaultJointManager only_joints(nlohmann::json{{"joints", nlohmann::json::array()}});
    EXPECT_EQ(only_joints.GetJoint("any"), nullptr);
}

TEST(JointManagerJsonConfigTest, RejectsMalformedConfig) {
    EXPECT_THROW((void) DefaultJointManager(nlohmann::json::array()), std::invalid_argument);
    EXPECT_THROW(DefaultJointManager(nlohmann::json("RJoint")), std::invalid_argument);
    EXPECT_THROW(DefaultJointManager(nlohmann::json{{"adapters", 1}}), std::invalid_argument);
    EXPECT_THROW(DefaultJointManager(nlohmann::json{{"joints", "x"}}), std::invalid_argument);
    // 适配器配置非法
    EXPECT_THROW(DefaultJointManager(
                     nlohmann::json{{"adapters", nlohmann::json::array({{{{"type", "Fake"}}}})}}),
                 std::invalid_argument);
    // 未知关节类型
    EXPECT_THROW(DefaultJointManager(nlohmann::json{
                     {"joints",
                      nlohmann::json::array({{{"name", "x"}, {"type", "Nope"}, {"config", {}}}})}}),
                 std::invalid_argument);
    // 关节缺少 name
    EXPECT_THROW(DefaultJointManager(nlohmann::json{
                     {"joints", nlohmann::json::array({{{"type", "RJoint"}, {"config", {}}}})}}),
                 nlohmann::json::out_of_range);
    // 关节引用了不存在的适配器
    EXPECT_THROW(DefaultJointManager(nlohmann::json{
                     {"joints", nlohmann::json::array({{{"name", "no_adapter"},
                                                        {"type", "RJoint"},
                                                        {"config", CreateRJointConfigJson()}}})}}),
                 std::invalid_argument);
}

TEST(JointManagerJsonConfigTest, RejectsDuplicateJointNamesWithinConfig) {
    auto config_json = CreateRJointConfigJson();

    EXPECT_THROW(
        DefaultJointManager(nlohmann::json{
            {"adapters", nlohmann::json::array({{{"type", "Fake"}, {"name", "dup"}}})},
            {"joints", nlohmann::json::array(
                           {{{"name", "dup_joint"}, {"type", "RJoint"}, {"config", config_json}},
                            {{"name", "dup_joint"},
                             {"type", "ContinuousJoint"},
                             {"config", config_json}}})}}),
        std::invalid_argument);
}

TEST(JointManagerCanFdTest, TogglesExistingRJointMotors) {
    TestableJointManager manager;
    auto adapter = CreateFakeAdapterHarness("canfd_existing_rjoint");
    manager.RegisterAdapter("adapter1", adapter->Adapter());

    RJoint::Config config{};
    config.motorParams = MotorParams{"adapter1", 0, 1, MotorInitParams{CreateMotorRanges()}};
    config.limit = Range<float>{-1.0f, 1.0f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    config.reverse = false;

    auto joint = manager.AddJoint<RJoint>("r_joint", config);
    EXPECT_FALSE(manager.IsJointMotorCanfd());

    joint->PosControl(0.1f);
    ExpectAllMessagesUseCanFd(adapter->GetCommandRecords(), false);

    adapter->ClearCommandRecords();
    manager.EnableJointMotorCanfd();
    EXPECT_TRUE(manager.IsJointMotorCanfd());
    joint->PosControl(0.2f);
    ExpectAllMessagesUseCanFd(adapter->GetCommandRecords(), true);

    adapter->ClearCommandRecords();
    manager.DisableJointMotorCanfd();
    EXPECT_FALSE(manager.IsJointMotorCanfd());
    joint->PosControl(0.3f);
    ExpectAllMessagesUseCanFd(adapter->GetCommandRecords(), false);
}

TEST(JointManagerCanFdTest, AppliesCurrentCanFdStateToFutureRJointMotors) {
    TestableJointManager manager;
    auto adapter = CreateFakeAdapterHarness("canfd_future_rjoint");
    manager.RegisterAdapter("adapter1", adapter->Adapter());
    auto ranges = CreateMotorRanges();

    RJoint::Config fd_config{};
    fd_config.motorParams = MotorParams{"adapter1", 0, 1, MotorInitParams{ranges}};
    fd_config.limit = Range<float>{-1.0f, 1.0f};
    fd_config.maxSpd = 2.0f;
    fd_config.maxTor = 3.0f;
    fd_config.reverse = false;

    manager.EnableJointMotorCanfd();
    auto fd_joint = manager.AddJoint<RJoint>("fd_joint", fd_config);
    fd_joint->PosControl(0.1f);
    ExpectAllMessagesUseCanFd(adapter->GetCommandRecords(), true);

    adapter->ClearCommandRecords();
    manager.DisableJointMotorCanfd();
    RJoint::Config classic_config = fd_config;
    classic_config.motorParams.motorId = 2;
    auto classic_joint = manager.AddJoint<RJoint>("classic_joint", classic_config);
    classic_joint->PosControl(0.2f);
    ExpectAllMessagesUseCanFd(adapter->GetCommandRecords(), false);
}

TEST(JointManagerCanFdTest, AppliesPerMotorCanFdFromParams) {
    TestableJointManager manager;
    auto adapter = CreateFakeAdapterHarness("canfd_params_rjoint");
    manager.RegisterAdapter("adapter1", adapter->Adapter());

    RJoint::Config config{};
    config.motorParams = MotorParams{"adapter1", 0, 1, MotorInitParams{CreateMotorRanges()}};
    config.motorParams.canfd = true;
    config.limit = Range<float>{-1.0f, 1.0f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    config.reverse = false;

    auto joint = manager.AddJoint<RJoint>("params_fd_joint", config);
    EXPECT_FALSE(manager.IsJointMotorCanfd());

    joint->PosControl(0.1f);
    ExpectAllMessagesUseCanFd(adapter->GetCommandRecords(), true);

    // 全局开关被显式调用后压过逐电机配置
    adapter->ClearCommandRecords();
    manager.DisableJointMotorCanfd();
    joint->PosControl(0.2f);
    ExpectAllMessagesUseCanFd(adapter->GetCommandRecords(), false);
}

TEST(JointManagerCanFdTest, ExplicitGlobalSwitchOverridesMotorParams) {
    TestableJointManager manager;
    auto adapter = CreateFakeAdapterHarness("canfd_global_override");
    manager.RegisterAdapter("adapter1", adapter->Adapter());

    RJoint::Config config{};
    config.motorParams = MotorParams{"adapter1", 0, 1, MotorInitParams{CreateMotorRanges()}};
    config.limit = Range<float>{-1.0f, 1.0f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    config.reverse = false;

    // canfd 缺省为 false，但全局开关已显式打开时仍按 CAN FD 发送
    manager.EnableJointMotorCanfd();
    auto joint = manager.AddJoint<RJoint>("global_fd_joint", config);

    joint->PosControl(0.1f);
    ExpectAllMessagesUseCanFd(adapter->GetCommandRecords(), true);
}

TEST(JointManagerCanFdTest, DoubleCoupleJointAppliesPerMotorCanFd) {
    TestableJointManager manager;
    auto adapter = CreateFakeAdapterHarness("canfd_params_double_couple");
    manager.RegisterAdapter("adapter1", adapter->Adapter());
    auto ranges = CreateMotorRanges();

    auto slave_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Pitch, ranges);
    auto master_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Roll, ranges);
    slave_config.motorParams1.canfd = true;
    master_config.motorParams1.canfd = true;

    auto slave = manager.AddJoint<DoubleCoupleJoint>("params_pitch_pending", slave_config);
    ASSERT_NE(std::dynamic_pointer_cast<SlaveJoint>(slave), nullptr);
    adapter->SetReplyMode(FakeReplyMode::Automatic);
    auto master = manager.AddJoint<DoubleCoupleJoint>("params_roll_master", master_config);

    adapter->ClearCommandRecords();
    master->Stop(MotorStopMode::FullBrake);
    ASSERT_EQ(adapter->GetCommandRecords().size(), 2u);

    bool motor1_uses_canfd = false;
    bool motor2_uses_canfd = false;
    for (const auto& record : adapter->GetCommandRecords()) {
        if (record.motor_idx == 1) {
            motor1_uses_canfd = CanFrameFlagsUseCanFd(record.raw_frame_flags);
        } else if (record.motor_idx == 2) {
            motor2_uses_canfd = CanFrameFlagsUseCanFd(record.raw_frame_flags);
        }
    }
    EXPECT_TRUE(motor1_uses_canfd);
    EXPECT_FALSE(motor2_uses_canfd);
}

TEST(JointManagerCanFdTest, PendingSlaveHasNoMotorAndMasterInheritsState) {
    TestableJointManager manager;
    auto adapter = CreateFakeAdapterHarness("canfd_double_couple");
    manager.RegisterAdapter("adapter1", adapter->Adapter());
    auto ranges = CreateMotorRanges();

    auto slave_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Pitch, ranges);
    auto master_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Roll, ranges);

    auto slave = manager.AddJoint<DoubleCoupleJoint>("pitch_pending", slave_config);
    EXPECT_NE(std::dynamic_pointer_cast<SlaveJoint>(slave), nullptr);

    EXPECT_NO_THROW(manager.EnableJointMotorCanfd());
    adapter->SetReplyMode(FakeReplyMode::Automatic);
    auto master = manager.AddJoint<DoubleCoupleJoint>("roll_master", master_config);
    adapter->ClearCommandRecords();
    master->Stop(MotorStopMode::FullBrake);
    ASSERT_EQ(adapter->GetCommandRecords().size(), 2u);
    ExpectAllMessagesUseCanFd(adapter->GetCommandRecords(), true);

    adapter->ClearCommandRecords();
    manager.DisableJointMotorCanfd();
    master->Stop(MotorStopMode::FullBrake);
    ASSERT_EQ(adapter->GetCommandRecords().size(), 2u);
    ExpectAllMessagesUseCanFd(adapter->GetCommandRecords(), false);
}

TEST(JointManagerLogTest, UsesSlaveBusMotorNameForSingleAdapter) {
    TemporaryDirectory directory;
    TestableJointManager manager;
    auto adapter = CreateFakeAdapterHarness("single_log_adapter");
    manager.RegisterAdapter("adapter1", adapter->Adapter());

    constexpr int kEncodedBusId = (3 << 16) | 4;
    RJoint::Config config{};
    config.motorParams =
        MotorParams{"adapter1", kEncodedBusId, 5, MotorInitParams{CreateMotorRanges()}};
    config.limit = Range<float>{-1.0f, 1.0f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    config.reverse = false;
    manager.AddJoint<RJoint>("logged_joint", config);

    manager.EnableLog(directory.Path());
    EXPECT_TRUE(manager.IsLogged());
    auto motor = adapter->Adapter()->GetBus(kEncodedBusId)->GetMotor(5);
    EXPECT_TRUE(motor->IsLogged());

    motor->DisableLog();
    EXPECT_FALSE(manager.IsLogged());
    manager.EnableLog(directory.Path());
    EXPECT_TRUE(manager.IsLogged());

    manager.DisableLog();
    EXPECT_FALSE(manager.IsLogged());
    EXPECT_FALSE(motor->IsLogged());
    EXPECT_TRUE(std::filesystem::exists(directory.Path() / "s3b4m5_status.csv.zstd"));
    EXPECT_TRUE(std::filesystem::exists(directory.Path() / "s3b4m5_command.csv.zstd"));
}

TEST(JointManagerLogTest, UsesSlaveIdAndBusIdFromMotorParamsInFileName) {
    TemporaryDirectory directory;
    TestableJointManager manager;
    auto adapter = CreateFakeAdapterHarness("slave_id_log_adapter");
    manager.RegisterAdapter("adapter1", adapter->Adapter());

    RJoint::Config config{};
    config.motorParams = MotorParams{"adapter1", 4, 5, MotorInitParams{CreateMotorRanges()}};
    config.motorParams.slaveId = 3;
    config.limit = Range<float>{-1.0f, 1.0f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    config.reverse = false;
    manager.AddJoint<RJoint>("slave_id_joint", config);

    manager.EnableLog(directory.Path());
    manager.DisableLog();

    EXPECT_TRUE(std::filesystem::exists(directory.Path() / "s3b4m5_status.csv.zstd"));
    EXPECT_TRUE(std::filesystem::exists(directory.Path() / "s3b4m5_command.csv.zstd"));
    // 电机挂在合成 bus 索引上
    EXPECT_NE(adapter->Adapter()->GetBus((3 << 16) | 4), nullptr);
}

TEST(JointManagerLogTest, PrefixesNamesWhenManagerHasMultipleAdapters) {
    TemporaryDirectory directory;
    TestableJointManager manager;
    auto left_adapter = CreateFakeAdapterHarness("left_log_adapter");
    auto right_adapter = CreateFakeAdapterHarness("right_log_adapter");
    manager.RegisterAdapter("left", left_adapter->Adapter());
    manager.RegisterAdapter("right", right_adapter->Adapter());

    RJoint::Config config{};
    config.motorParams = MotorParams{"left", 0, 1, MotorInitParams{CreateMotorRanges()}};
    config.limit = Range<float>{-1.0f, 1.0f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    config.reverse = false;
    manager.AddJoint<RJoint>("left_joint", config);
    config.motorParams.adapterId = "right";
    manager.AddJoint<RJoint>("right_joint", config);

    manager.EnableLog(directory.Path());
    manager.DisableLog();

    EXPECT_TRUE(
        std::filesystem::exists(directory.Path() / "name%3A4%3Aleft_s0b0m1_status.csv.zstd"));
    EXPECT_TRUE(
        std::filesystem::exists(directory.Path() / "name%3A4%3Aleft_s0b0m1_command.csv.zstd"));
    EXPECT_TRUE(
        std::filesystem::exists(directory.Path() / "name%3A5%3Aright_s0b0m1_status.csv.zstd"));
    EXPECT_TRUE(
        std::filesystem::exists(directory.Path() / "name%3A5%3Aright_s0b0m1_command.csv.zstd"));
}

TEST(JointManagerLogTest, AppliesEnabledLoggingToFutureMotors) {
    TemporaryDirectory directory;
    TestableJointManager manager;
    auto adapter = CreateFakeAdapterHarness("future_log_adapter");
    manager.RegisterAdapter("adapter1", adapter->Adapter());
    manager.EnableLog(directory.Path());
    EXPECT_FALSE(manager.IsLogged());

    RJoint::Config config{};
    config.motorParams = MotorParams{"adapter1", 2, 7, MotorInitParams{CreateMotorRanges()}};
    config.limit = Range<float>{-1.0f, 1.0f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    config.reverse = false;
    manager.AddJoint<RJoint>("future_logged_joint", config);

    EXPECT_TRUE(manager.IsLogged());
    EXPECT_TRUE(adapter->Adapter()->GetBus(2)->GetMotor(7)->IsLogged());
    manager.DisableLog();
    EXPECT_TRUE(std::filesystem::exists(directory.Path() / "s0b2m7_status.csv.zstd"));
    EXPECT_TRUE(std::filesystem::exists(directory.Path() / "s0b2m7_command.csv.zstd"));
}

TEST(JointManagerLogTest, TracksSharedPhysicalMotorOnlyOnce) {
    TemporaryDirectory directory;
    TestableJointManager manager;
    auto adapter = CreateFakeAdapterHarness("shared_motor_log_adapter");
    manager.RegisterAdapter("adapter1", adapter->Adapter());

    RJoint::Config config{};
    config.motorParams = MotorParams{"adapter1", 0, 3, MotorInitParams{CreateMotorRanges()}};
    config.limit = Range<float>{-1.0f, 1.0f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    config.reverse = false;
    manager.AddJoint<RJoint>("first_joint", config);
    manager.AddJoint<RJoint>("second_joint", config);

    EXPECT_EQ(manager.TrackedMotorCount(), 1u);
    manager.EnableLog(directory.Path());
    manager.DisableLog();
    EXPECT_TRUE(std::filesystem::exists(directory.Path() / "s0b0m3_status.csv.zstd"));
    EXPECT_TRUE(std::filesystem::exists(directory.Path() / "s0b0m3_command.csv.zstd"));
}

TEST(JointManagerLogTest, FailedJointConstructionDoesNotTrackOrLogMotor) {
    TemporaryDirectory directory;
    TestableJointManager manager;
    auto adapter = CreateFakeAdapterHarness("failed_joint_log_adapter");
    manager.RegisterAdapter("adapter1", adapter->Adapter());
    manager.EnableLog(directory.Path());

    RJoint::Config config{};
    config.motorParams = MotorParams{"adapter1", 0, 9, MotorInitParams{CreateMotorRanges()}};
    config.limit = Range<float>{1.0f, -1.0f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    config.reverse = false;

    EXPECT_THROW(manager.AddJoint<RJoint>("invalid_joint", config), std::invalid_argument);
    EXPECT_EQ(manager.TrackedMotorCount(), 0u);
    EXPECT_FALSE(manager.IsLogged());
    EXPECT_FALSE(adapter->Adapter()->GetBus(0)->GetMotor(9)->IsLogged());
    manager.DisableLog();
    EXPECT_FALSE(std::filesystem::exists(directory.Path() / "s0b0m9_status.csv.zstd"));
}

TEST(JointManagerLogTest, EscapesAdapterNameInMultiAdapterFileName) {
    TemporaryDirectory directory;
    TestableJointManager manager;
    auto unsafe_adapter = CreateFakeAdapterHarness("unsafe_log_adapter");
    auto other_adapter = CreateFakeAdapterHarness("other_log_adapter");
    manager.RegisterAdapter("left/right", unsafe_adapter->Adapter());
    manager.RegisterAdapter("other", other_adapter->Adapter());

    RJoint::Config config{};
    config.motorParams = MotorParams{"left/right", 0, 1, MotorInitParams{CreateMotorRanges()}};
    config.limit = Range<float>{-1.0f, 1.0f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    config.reverse = false;
    manager.AddJoint<RJoint>("safe_path_joint", config);

    manager.EnableLog(directory.Path());
    manager.DisableLog();
    EXPECT_TRUE(std::filesystem::exists(directory.Path() /
                                        "name%3A10%3Aleft%2Fright_s0b0m1_status.csv.zstd"));
    EXPECT_TRUE(std::filesystem::exists(directory.Path() /
                                        "name%3A10%3Aleft%2Fright_s0b0m1_command.csv.zstd"));
}

TEST(JointManagerLogTest, KeepsDistinctRawBusIdsDistinctInFileNames) {
    TemporaryDirectory directory;
    TestableJointManager manager;
    auto adapter = CreateFakeAdapterHarness("distinct_bus_log_adapter");
    manager.RegisterAdapter("adapter1", adapter->Adapter());

    RJoint::Config config{};
    config.motorParams = MotorParams{"adapter1", 4, 1, MotorInitParams{CreateMotorRanges()}};
    config.limit = Range<float>{-1.0f, 1.0f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    config.reverse = false;
    manager.AddJoint<RJoint>("bus_4_joint", config);
    config.motorParams.busId = 0x104;
    manager.AddJoint<RJoint>("bus_260_joint", config);

    manager.EnableLog(directory.Path());
    manager.DisableLog();
    EXPECT_TRUE(std::filesystem::exists(directory.Path() / "s0b4m1_status.csv.zstd"));
    EXPECT_TRUE(std::filesystem::exists(directory.Path() / "s0b260m1_status.csv.zstd"));
}

TEST(JointManagerLogTest, DoesNotRenameActiveLogsWhenSecondAdapterIsCreated) {
    TemporaryDirectory directory;
    TestableJointManager manager;
    manager.CreateAdapter("Fake", "first");
    auto first_control = manager.Adapter("first")->GetFakeAdapterControl();
    ASSERT_NE(first_control, nullptr);
    first_control->EnableAutoCreateMotor();

    RJoint::Config config{};
    config.motorParams = MotorParams{"first", 0, 2, MotorInitParams{CreateMotorRanges()}};
    config.limit = Range<float>{-1.0f, 1.0f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    config.reverse = false;
    manager.AddJoint<RJoint>("first_joint", config);
    manager.EnableLog(directory.Path());

    manager.CreateAdapter("Fake", "second");
    auto second_control = manager.Adapter("second")->GetFakeAdapterControl();
    ASSERT_NE(second_control, nullptr);
    second_control->EnableAutoCreateMotor();
    EXPECT_TRUE(manager.IsLogged());

    config.motorParams.adapterId = "second";
    config.motorParams.motorId = 3;
    manager.AddJoint<RJoint>("second_joint", config);
    manager.DisableLog();
    EXPECT_TRUE(std::filesystem::exists(directory.Path() / "s0b0m2_status.csv.zstd"));
    EXPECT_TRUE(std::filesystem::exists(directory.Path() / "s0b0m2_command.csv.zstd"));
    EXPECT_FALSE(
        std::filesystem::exists(directory.Path() / "name%3A5%3Afirst_s0b0m2_status.csv.zstd"));
    EXPECT_TRUE(
        std::filesystem::exists(directory.Path() / "name%3A6%3Asecond_s0b0m3_status.csv.zstd"));
    EXPECT_TRUE(
        std::filesystem::exists(directory.Path() / "name%3A6%3Asecond_s0b0m3_command.csv.zstd"));
}

TEST(JointManagerLogTest, ResolvesRelativePathBeforeFutureMotorCreation) {
    TemporaryDirectory directory;
    const auto original_working_directory = directory.Path() / "original";
    const auto later_working_directory = directory.Path() / "later";
    std::filesystem::create_directories(original_working_directory);
    std::filesystem::create_directories(later_working_directory);

    {
        ScopedCurrentPath current_path(original_working_directory);
        TestableJointManager manager;
        auto adapter = CreateFakeAdapterHarness("relative_path_log_adapter");
        manager.RegisterAdapter("adapter1", adapter->Adapter());
        manager.EnableLog("logs");
        std::filesystem::current_path(later_working_directory);

        RJoint::Config config{};
        config.motorParams = MotorParams{"adapter1", 0, 4, MotorInitParams{CreateMotorRanges()}};
        config.limit = Range<float>{-1.0f, 1.0f};
        config.maxSpd = 2.0f;
        config.maxTor = 3.0f;
        config.reverse = false;
        manager.AddJoint<RJoint>("relative_path_joint", config);
        manager.DisableLog();
    }

    EXPECT_TRUE(
        std::filesystem::exists(original_working_directory / "logs/s0b0m4_status.csv.zstd"));
    EXPECT_FALSE(std::filesystem::exists(later_working_directory / "logs/s0b0m4_status.csv.zstd"));
}

TEST(JointManagerCommitTest, FlushesQueuedCommandsForEachUsedBus) {
    TestableJointManager manager;
    auto adapter = CreateFakeAdapterHarness("manager_commit");
    manager.RegisterAdapter("adapter1", adapter->Adapter());

    const auto ranges = CreateMotorRanges();
    RJoint::Config first_config{};
    first_config.motorParams = MotorParams{"adapter1", 0, 1, MotorInitParams{ranges}};
    first_config.limit = Range<float>{-1.0f, 1.0f};
    first_config.maxSpd = 2.0f;
    first_config.maxTor = 3.0f;

    auto second_config = first_config;
    second_config.motorParams = MotorParams{"adapter1", 1, 2, MotorInitParams{ranges}};

    auto first_joint = manager.AddJoint<RJoint>("first_joint", first_config);
    auto second_joint = manager.AddJoint<RJoint>("second_joint", second_config);
    adapter->Adapter()->GetBus(0)->SetSyncMode(true);
    adapter->Adapter()->GetBus(1)->SetSyncMode(true);
    adapter->ClearCommandRecords();

    first_joint->PosControl(0.1f);
    second_joint->PosControl(0.2f);
    EXPECT_TRUE(adapter->GetCommandRecords().empty());

    manager.Commit();

    EXPECT_EQ(adapter->GetCommandRecords().size(), 2u);
}

TEST(JointManagerSyncTest, OnlyUsedBusesFollowManagerSyncMode) {
    TestableJointManager manager;
    auto adapter = CreateFakeAdapterHarness("manager_used_bus_sync");
    manager.CreateAdapter("Fake", "manager_used_bus_sync");
    const auto ranges = CreateMotorRanges();
    RJoint::Config config{};
    config.motorParams = MotorParams{"manager_used_bus_sync", 0, 1, MotorInitParams{ranges}};
    config.limit = {-1.0f, 1.0f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    auto joint = manager.AddJoint<RJoint>("used", config);
    auto* unused = adapter->Adapter()->GetBus(1)->GetMotor(2, ranges);

    manager.SetAdaptersSyncMode(true);
    adapter->ClearCommandRecords();
    joint->PosControl(0.1f);
    unused->PosControl<0>(0.2f, 1.0f, 0.0f);
    ASSERT_EQ(adapter->SnapshotCommandRecords().size(), 1U);
    manager.Commit();
    EXPECT_EQ(adapter->SnapshotCommandRecords().size(), 2U);

    // Disabling the manager must not change an independently synchronized bus.
    adapter->Adapter()->GetBus(1)->SetSyncMode(true);
    manager.SetAdaptersSyncMode(false);
    adapter->ClearCommandRecords();
    joint->PosControl(0.1f);
    unused->PosControl<0>(0.2f, 1.0f, 0.0f);
    ASSERT_EQ(adapter->SnapshotCommandRecords().size(), 1U);
    adapter->Adapter()->GetBus(1)->Commit();
    EXPECT_EQ(adapter->SnapshotCommandRecords().size(), 2U);
}

TEST(JointManagerSyncTest, FutureBusesInheritModeWithoutChangingUnusedAdapters) {
    TestableJointManager manager;
    manager.SetAdaptersSyncMode(true);
    auto adapter = CreateFakeAdapterHarness("manager_future_bus_sync");
    manager.CreateAdapter("Fake", "manager_future_bus_sync");
    auto other = CreateFakeAdapterHarness("manager_unused_adapter_sync");
    manager.CreateAdapter("Fake", "manager_unused_adapter_sync");
    const auto ranges = CreateMotorRanges();
    auto* unused = other->Adapter()->GetBus(0)->GetMotor(1, ranges);
    other->ClearCommandRecords();
    unused->PosControl<0>(0.1f, 1.0f, 0.0f);
    ASSERT_EQ(other->SnapshotCommandRecords().size(), 1U);

    RJoint::Config config{};
    config.motorParams = MotorParams{"manager_future_bus_sync", 0, 1, MotorInitParams{ranges}};
    config.limit = {-1.0f, 1.0f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    auto first = manager.AddJoint<RJoint>("first", config);
    config.motorParams.busId = 1;
    auto second = manager.AddJoint<RJoint>("second", config);
    adapter->ClearCommandRecords();
    first->PosControl(0.1f);
    second->PosControl(0.2f);
    EXPECT_TRUE(adapter->SnapshotCommandRecords().empty());
    manager.Commit();
    EXPECT_EQ(adapter->SnapshotCommandRecords().size(), 2U);
}

TEST(JointManagerLifetimeTest, RetainedJointRemainsUsableAfterManagerDestruction) {
    auto adapter = CreateFakeAdapterHarness("manager_lifetime_retained_joint");
    BaseJointPtr joint;

    {
        TestableJointManager manager;
        manager.RegisterAdapter("adapter1", adapter->Adapter());

        RJoint::Config config{};
        config.motorParams = MotorParams{"adapter1", 0, 1, MotorInitParams{CreateMotorRanges()}};
        config.limit = Range<float>{-1.0f, 1.0f};
        config.maxSpd = 2.0f;
        config.maxTor = 3.0f;
        joint = manager.AddJoint<RJoint>("r_joint", config);
    }

    adapter->ClearCommandRecords();
    joint->PosControl(0.2f);

    EXPECT_EQ(adapter->GetCommandRecords().size(), 1u);
}

TEST(JointManagerLifetimeTest, DestroyingOneManagerDoesNotInvalidateSharedAdapter) {
    const std::string adapter_name = "manager_lifetime_shared_adapter";
    auto adapter = CreateFakeAdapterHarness(adapter_name);
    auto first_manager = std::make_unique<TestableJointManager>();
    TestableJointManager second_manager;
    first_manager->CreateAdapter("Fake", adapter_name);
    second_manager.CreateAdapter("Fake", adapter_name);

    RJoint::Config first_config{};
    first_config.motorParams =
        MotorParams{adapter_name, 0, 1, MotorInitParams{CreateMotorRanges()}};
    first_config.limit = Range<float>{-1.0f, 1.0f};
    first_config.maxSpd = 2.0f;
    first_config.maxTor = 3.0f;
    auto second_config = first_config;
    second_config.motorParams.motorId = 2;

    (void) first_manager->AddJoint<RJoint>("first_joint", first_config);
    auto second_joint = second_manager.AddJoint<RJoint>("second_joint", second_config);
    first_manager.reset();

    adapter->ClearCommandRecords();
    second_joint->PosControl(0.2f);

    EXPECT_EQ(adapter->GetCommandRecords().size(), 1u);
}

TEST(JointManagerFakeTest, TogglesFakePositionErrorOnRegisteredAdapters) {
    TestableJointManager manager;
    auto adapter = CreateFakeAdapterHarness("fake_position_error");
    manager.RegisterAdapter("adapter1", adapter->Adapter());

    auto ranges = CreateMotorRanges();
    RJoint::Config config{};
    config.motorParams = MotorParams{"adapter1", 0, 1, MotorInitParams{ranges}};
    config.limit = Range<float>{-1.0f, 1.0f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    config.reverse = false;

    manager.EnableFakePositionError(false);
    auto joint = manager.AddJoint<RJoint>("r_joint", config);
    adapter->SetReplyMode(FakeReplyMode::Automatic);

    joint->SpdControl(2.0f);
    const auto t1 = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    joint->SpdControl(1.0f);
    const auto t2 = std::chrono::steady_clock::now();

    const float elapsed = std::chrono::duration<float>(t2 - t1).count();
    const auto snapshot = adapter->Control().GetMotorSnapshot(0, 1);
    EXPECT_NEAR(snapshot.position_rad, 2.0f * elapsed, 0.02f);
}

TEST(JointManagerStatusLifetimeTest, PairedStatusIsAvailableForBothAxes) {
    auto fixture = CreatePairedDoubleCoupleFixture("paired_status_lifetime", CreateMotorRanges());
    fixture.adapter->SetReplyMode(FakeReplyMode::Automatic);
    fixture.master->PosControl(0.1f, 1.0f, 0.5f);
    fixture.master->PosControl(0.1f, 1.0f, 0.5f);
    ASSERT_TRUE(fixture.slave->GetStatus(false, 1).has_value());
    EXPECT_TRUE(fixture.master->GetStatus(false, 1).has_value());
}

// ============================================================
// Config structures
// ============================================================

TEST(ConfigTest, RJointFieldsExist) {
    RJoint::Config c{};
    EXPECT_TRUE((std::is_same_v<decltype(c.motorParams), MotorParams>) );
    EXPECT_TRUE((std::is_same_v<decltype(c.limit), Range<float>>) );
    EXPECT_TRUE((std::is_same_v<decltype(c.maxSpd), float>) );
    EXPECT_TRUE((std::is_same_v<decltype(c.maxTor), float>) );
    EXPECT_TRUE((std::is_same_v<decltype(c.reverse), bool>) );
}

TEST(ConfigTest, ContinuousJointFieldsExcludePositionLimit) {
    ContinuousJoint::Config c{};
    EXPECT_TRUE((std::is_same_v<decltype(c.motorParams), MotorParams>) );
    EXPECT_TRUE((std::is_same_v<decltype(c.maxSpd), float>) );
    EXPECT_TRUE((std::is_same_v<decltype(c.maxTor), float>) );
    EXPECT_TRUE((std::is_same_v<decltype(c.reverse), bool>) );

    const auto schema = nlohmann::json::parse(ContinuousJoint::Config::ToJsonSchema());
    EXPECT_FALSE(schema["properties"].contains("limit"));
    EXPECT_TRUE(JsonArrayContainsString(schema["required"], "motorParams"));
    EXPECT_TRUE(JsonArrayContainsString(schema["required"], "maxSpd"));
    EXPECT_TRUE(JsonArrayContainsString(schema["required"], "maxTor"));
    EXPECT_TRUE(JsonArrayContainsString(schema["required"], "reverse"));
}

TEST(ConfigTest, DoubleCoupleJointFieldsExist) {
    DoubleCoupleJoint::Config c{};
    EXPECT_TRUE((std::is_same_v<decltype(c.motorParams1), MotorParams>) );
    EXPECT_TRUE((std::is_same_v<decltype(c.motorParams2), MotorParams>) );
    EXPECT_TRUE((std::is_same_v<decltype(c.axis), DoubleCoupleJointAxis>) );
    EXPECT_TRUE((std::is_same_v<decltype(c.calibration), couple_joint::CalibrationInput>) );
    EXPECT_TRUE((std::is_same_v<decltype(c.motorLimit1), std::optional<Range<float>>>) );
    EXPECT_TRUE((std::is_same_v<decltype(c.motorLimit2), std::optional<Range<float>>>) );
    EXPECT_TRUE((std::is_same_v<decltype(c.limit), Range<float>>) );
    EXPECT_TRUE((std::is_same_v<decltype(c.maxSpd), float>) );
    EXPECT_TRUE((std::is_same_v<decltype(c.maxTor), float>) );
    EXPECT_TRUE((std::is_same_v<decltype(c.reverse), bool>) );
}

TEST(ConfigTest, MacroOptionalFieldMayBeMissingFromJson) {
    const auto config =
        OptionalMacroConfigOwner::Config::FromJson(nlohmann::json{{"requiredValue", 7}});

    EXPECT_EQ(config.requiredValue, 7);
    EXPECT_FALSE(config.optionalValue.has_value());
}

TEST(ConfigTest, MacroOptionalFieldParsesWhenPresent) {
    const auto config = OptionalMacroConfigOwner::Config::FromJson(
        nlohmann::json{{"requiredValue", 7}, {"optionalValue", 9}});

    EXPECT_EQ(config.requiredValue, 7);
    ASSERT_TRUE(config.optionalValue.has_value());
    EXPECT_EQ(*config.optionalValue, 9);
}

TEST(ConfigTest, MacroOptionalFieldAppearsInSchemaButIsNotRequired) {
    const auto schema = nlohmann::json::parse(OptionalMacroConfigOwner::Config::ToJsonSchema());

    EXPECT_TRUE(schema["properties"].contains("optionalValue"));
    const auto& required = schema["required"];
    EXPECT_NE(std::find(required.begin(), required.end(), "requiredValue"), required.end());
    EXPECT_EQ(std::find(required.begin(), required.end(), "optionalValue"), required.end());
}

TEST(ConfigTest, MotorInitRejectsNonObjectJson) {
    const std::vector<nlohmann::json> invalid_values{nullptr, true,   1,
                                                     1.5,     "auto", nlohmann::json::array()};

    for (const auto& value : invalid_values) {
        EXPECT_THROW((void) value.get<MotorInit>(), nlohmann::json::type_error) << value.dump();
    }
}

TEST(ConfigTest, RJointFromJsonRejectsScalarInitParam) {
    auto json = CreateRJointConfigJson();
    json["motorParams"]["initParam"] = "not-an-object";

    EXPECT_THROW((void) RJoint::Config::FromJson(json), nlohmann::json::type_error);
}

TEST(ConfigTest, DoubleCoupleJointParsesAxisAndCalibrationFromJson) {
    auto config = DoubleCoupleJoint::Config::FromJson(CreateDoubleCoupleJointConfigJson("roll"));
    EXPECT_EQ(config.axis, DoubleCoupleJointAxis::Roll);
    EXPECT_EQ(config.calibration.rotation_order, couple_joint::RotationOrder::PitchThenRoll);
    EXPECT_NEAR(config.calibration.left_joint_center.y, 29.491263548, 1.0e-9);
    EXPECT_FALSE(config.calibration.use_rotate_direction);
}

TEST(ConfigTest, DoubleCoupleJointParsesMissingMotorLimitsAsNullopt) {
    auto config = DoubleCoupleJoint::Config::FromJson(CreateDoubleCoupleJointConfigJson("roll"));

    EXPECT_FALSE(config.motorLimit1.has_value());
    EXPECT_FALSE(config.motorLimit2.has_value());
}

TEST(ConfigTest, DoubleCoupleJointParsesOptionalMotorLimitsFromJson) {
    auto json = CreateDoubleCoupleJointConfigJson("roll");
    json["motorLimit1"] = {{"min", -1.2}, {"max", 1.3}};
    json["motorLimit2"] = {{"min", -0.7}, {"max", 0.8}};

    auto config = DoubleCoupleJoint::Config::FromJson(json);

    ASSERT_TRUE(config.motorLimit1.has_value());
    ASSERT_TRUE(config.motorLimit2.has_value());
    EXPECT_NEAR(config.motorLimit1->min, -1.2f, 1.0e-6f);
    EXPECT_NEAR(config.motorLimit1->max, 1.3f, 1.0e-6f);
    EXPECT_NEAR(config.motorLimit2->min, -0.7f, 1.0e-6f);
    EXPECT_NEAR(config.motorLimit2->max, 0.8f, 1.0e-6f);
}

TEST(ConfigTest, DoubleCoupleJointSchemaContainsOptionalMotorLimits) {
    const auto schema = nlohmann::json::parse(DoubleCoupleJoint::Config::ToJsonSchema());

    ASSERT_TRUE(schema["properties"].contains("motorLimit1"));
    ASSERT_TRUE(schema["properties"].contains("motorLimit2"));
    EXPECT_EQ(schema["properties"]["motorLimit1"]["$ref"], "#/definitions/Range");
    EXPECT_EQ(schema["properties"]["motorLimit2"]["$ref"], "#/definitions/Range");

    const auto& required = schema["required"];
    EXPECT_EQ(std::find(required.begin(), required.end(), "motorLimit1"), required.end());
    EXPECT_EQ(std::find(required.begin(), required.end(), "motorLimit2"), required.end());
}

TEST(ConfigTest, GeneratedSchemasAreParseableJson) {
    EXPECT_NO_THROW({
        const auto schema = nlohmann::json::parse(RJoint::Config::ToJsonSchema());
        (void) schema;
    });
    EXPECT_NO_THROW({
        const auto schema = nlohmann::json::parse(ContinuousJoint::Config::ToJsonSchema());
        (void) schema;
    });
    EXPECT_NO_THROW({
        const auto schema = nlohmann::json::parse(DoubleCoupleJoint::Config::ToJsonSchema());
        (void) schema;
    });
}

TEST(ConfigTest, RJointSchemaRequiredFieldsAreComplete) {
    const auto schema = nlohmann::json::parse(RJoint::Config::ToJsonSchema());
    const auto& required = schema["required"];

    EXPECT_TRUE(JsonArrayContainsString(required, "motorParams"));
    EXPECT_TRUE(JsonArrayContainsString(required, "limit"));
    EXPECT_TRUE(JsonArrayContainsString(required, "maxSpd"));
    EXPECT_TRUE(JsonArrayContainsString(required, "maxTor"));
    EXPECT_TRUE(JsonArrayContainsString(required, "reverse"));
}

TEST(ConfigTest, DoubleCoupleSchemaRequiredFieldsAreComplete) {
    const auto schema = nlohmann::json::parse(DoubleCoupleJoint::Config::ToJsonSchema());
    const auto& required = schema["required"];

    EXPECT_TRUE(JsonArrayContainsString(required, "motorParams1"));
    EXPECT_TRUE(JsonArrayContainsString(required, "motorParams2"));
    EXPECT_TRUE(JsonArrayContainsString(required, "axis"));
    EXPECT_TRUE(JsonArrayContainsString(required, "calibration"));
    EXPECT_TRUE(JsonArrayContainsString(required, "limit"));
    EXPECT_TRUE(JsonArrayContainsString(required, "maxSpd"));
    EXPECT_TRUE(JsonArrayContainsString(required, "maxTor"));
    EXPECT_TRUE(JsonArrayContainsString(required, "reverse"));
}

TEST(ConfigTest, MotorParamsSchemaKeepsInitParamOneOfContract) {
    const auto schema = nlohmann::json::parse(RJoint::Config::ToJsonSchema());
    const auto& init_param =
        schema["definitions"]["MotorParams"]["properties"]["initParam"]["oneOf"];

    ASSERT_EQ(init_param.size(), 3u);
    EXPECT_EQ(init_param[0]["additionalProperties"], false);
    EXPECT_TRUE(JsonArrayContainsString(init_param[1]["required"], "model"));
    EXPECT_TRUE(JsonArrayContainsString(init_param[2]["required"], "ranges"));
}

TEST(ConfigTest, MotorParamsParsesSlaveIdAndCanFdDefaults) {
    const auto config = RJoint::Config::FromJson(CreateRJointConfigJson());

    EXPECT_EQ(config.motorParams.slaveId, 0u);
    EXPECT_FALSE(config.motorParams.canfd);
    EXPECT_EQ(config.motorParams.BusIndex(), config.motorParams.busId);
}

TEST(ConfigTest, MotorParamsParsesExplicitSlaveIdAndCanFd) {
    auto json = CreateRJointConfigJson();
    json["motorParams"]["slaveId"] = 3;
    json["motorParams"]["busId"] = 4;
    json["motorParams"]["canfd"] = true;

    const auto config = RJoint::Config::FromJson(json);

    EXPECT_EQ(config.motorParams.slaveId, 3u);
    EXPECT_EQ(config.motorParams.busId, 4);
    EXPECT_TRUE(config.motorParams.canfd);
    EXPECT_EQ(config.motorParams.BusIndex(), (3 << 16) | 4);
}

TEST(ConfigTest, MotorParamsParsesAdapterReferenceByJsonType) {
    // 字符串 → 名称引用
    const auto by_name = RJoint::Config::FromJson(CreateRJointConfigJson());
    EXPECT_FALSE(by_name.motorParams.AdapterIdIsIndex());
    EXPECT_EQ(by_name.motorParams.AdapterIdString(), "adapter1");

    // 非负整数 → 建立顺序索引引用
    auto json = CreateRJointConfigJson();
    json["motorParams"]["adapterId"] = 2;
    const auto by_index = RJoint::Config::FromJson(json);
    EXPECT_TRUE(by_index.motorParams.AdapterIdIsIndex());
    EXPECT_EQ(by_index.motorParams.AdapterIdString(), "2");

    // 0 也是合法索引
    json["motorParams"]["adapterId"] = 0;
    const auto by_zero = RJoint::Config::FromJson(json);
    EXPECT_TRUE(by_zero.motorParams.AdapterIdIsIndex());
    EXPECT_EQ(by_zero.motorParams.AdapterIdString(), "0");
}

TEST(ConfigTest, MotorParamsRejectsInvalidAdapterReference) {
    // 非字符串也非整数 → JSON 类型异常
    const std::vector<nlohmann::json> invalid_references{
        nlohmann::json(true), nlohmann::json(1.5), nlohmann::json(nullptr), nlohmann::json::array(),
        nlohmann::json::object()};
    for (const auto& invalid : invalid_references) {
        auto json = CreateRJointConfigJson();
        json["motorParams"]["adapterId"] = invalid;
        EXPECT_THROW((void) RJoint::Config::FromJson(json), nlohmann::json::type_error);
    }

    // 负数或超出 uint32 → 取值非法
    auto negative_json = CreateRJointConfigJson();
    negative_json["motorParams"]["adapterId"] = -1;
    EXPECT_THROW((void) RJoint::Config::FromJson(negative_json), std::invalid_argument);

    auto overflow_json = CreateRJointConfigJson();
    overflow_json["motorParams"]["adapterId"] = std::uint64_t{1} << 40U;
    EXPECT_THROW((void) RJoint::Config::FromJson(overflow_json), std::invalid_argument);
}

TEST(ConfigTest, MotorParamsRejectsNonBooleanCanFd) {
    auto json = CreateRJointConfigJson();
    json["motorParams"]["canfd"] = 1;
    EXPECT_THROW((void) RJoint::Config::FromJson(json), nlohmann::json::type_error);

    json = CreateRJointConfigJson();
    json["motorParams"]["canfd"] = "true";
    EXPECT_THROW((void) RJoint::Config::FromJson(json), nlohmann::json::type_error);
}

TEST(ConfigTest, MotorParamsSchemaMatchesParsableKeys) {
    const auto schema = nlohmann::json::parse(RJoint::Config::ToJsonSchema());
    const auto& motor_params = schema["definitions"]["MotorParams"];

    std::vector<std::string> properties;
    for (auto it = motor_params["properties"].begin(); it != motor_params["properties"].end();
         ++it) {
        properties.push_back(it.key());
    }
    std::sort(properties.begin(), properties.end());
    // 与 from_json(MotorParams) 实际读取的键逐键一致
    const std::vector<std::string> expected_properties{"adapterId", "busId",   "canfd",
                                                       "initParam", "motorId", "slaveId"};
    EXPECT_EQ(properties, expected_properties);

    auto required = motor_params["required"].get<std::vector<std::string>>();
    std::sort(required.begin(), required.end());
    const std::vector<std::string> expected_required{"adapterId", "busId", "initParam", "motorId"};
    EXPECT_EQ(required, expected_required);

    EXPECT_EQ(motor_params["properties"]["slaveId"]["type"], "integer");
    EXPECT_EQ(motor_params["properties"]["canfd"]["type"], "boolean");
    // adapterId 同时接受名称字符串与建立顺序索引
    const std::vector<std::string> expected_adapter_id_types{"string", "integer"};
    EXPECT_EQ(motor_params["properties"]["adapterId"]["type"], expected_adapter_id_types);
}

TEST(ConfigTest, CalibrationSchemaRequiresAllPointFieldsAndRotationOrder) {
    const auto schema = nlohmann::json::parse(DoubleCoupleJoint::Config::ToJsonSchema());
    const auto& required = schema["definitions"]["CalibrationInput"]["required"];

    EXPECT_TRUE(JsonArrayContainsString(required, "leftJointCenter"));
    EXPECT_TRUE(JsonArrayContainsString(required, "rightJointCenter"));
    EXPECT_TRUE(JsonArrayContainsString(required, "leftPlanarEndpoint"));
    EXPECT_TRUE(JsonArrayContainsString(required, "rightPlanarEndpoint"));
    EXPECT_TRUE(JsonArrayContainsString(required, "rollCenter"));
    EXPECT_TRUE(JsonArrayContainsString(required, "pitchCenter"));
    EXPECT_TRUE(JsonArrayContainsString(required, "leftSpatialEndpoint"));
    EXPECT_TRUE(JsonArrayContainsString(required, "rightSpatialEndpoint"));
    EXPECT_TRUE(JsonArrayContainsString(required, "rotationOrder"));
    EXPECT_FALSE(JsonArrayContainsString(required, "rotateDirection"));
}

TEST(ConfigTest, DoubleCoupleJointParsesOptionalRotateDirectionFromJson) {
    auto json = CreateDoubleCoupleJointConfigJson("roll");
    json["calibration"]["rotateDirection"] = {{"x", 0.0}, {"y", 0.0}, {"z", 1.0}};

    auto config = DoubleCoupleJoint::Config::FromJson(json);

    EXPECT_TRUE(config.calibration.use_rotate_direction);
    EXPECT_NEAR(config.calibration.rotate_direction.z, 1.0, 1.0e-9);
}

TEST(ConfigTest, DoubleCoupleJointRequiresRotationOrderFromJson) {
    auto json = CreateDoubleCoupleJointConfigJson("roll");
    json["calibration"].erase("rotationOrder");

    EXPECT_THROW((void) DoubleCoupleJoint::Config::FromJson(json), nlohmann::json::out_of_range);
}

TEST(ConfigTest, DoubleCoupleJointParsesNumericRotationOrderFromJson) {
    auto json = CreateDoubleCoupleJointConfigJson("pitch");
    json["calibration"]["rotationOrder"] = 1;

    auto config = DoubleCoupleJoint::Config::FromJson(json);

    EXPECT_EQ(config.axis, DoubleCoupleJointAxis::Pitch);
    EXPECT_EQ(config.calibration.rotation_order, couple_joint::RotationOrder::RollThenPitch);
}

TEST(ConfigTest, DoubleCoupleJointCalibrationIgnoresRotateDirectionWhenUnused) {
    auto lhs = CreateValidCalibrationInput();
    auto rhs = lhs;
    lhs.use_rotate_direction = false;
    rhs.use_rotate_direction = false;
    lhs.rotate_direction = {1.0, 0.0, 0.0};
    rhs.rotate_direction = {0.0, 1.0, 0.0};

    EXPECT_TRUE(DoubleCoupleJoint::CalibrationEqual(lhs, rhs));
}

TEST(ConfigTest, DoubleCoupleJointCalibrationComparesRotateDirectionWhenUsed) {
    auto lhs = CreateValidCalibrationInput();
    auto rhs = lhs;
    lhs.use_rotate_direction = true;
    rhs.use_rotate_direction = true;
    lhs.rotate_direction = {1.0, 0.0, 0.0};
    rhs.rotate_direction = {0.0, 1.0, 0.0};

    EXPECT_FALSE(DoubleCoupleJoint::CalibrationEqual(lhs, rhs));
}

TEST(ConfigTest, DoubleCoupleJointSchemaContainsAxisAndCalibrationInput) {
    const auto schema = DoubleCoupleJoint::Config::ToJsonSchema();
    EXPECT_NE(schema.find("\"axis\""), std::string::npos);
    EXPECT_NE(schema.find("\"Roll\""), std::string::npos);
    EXPECT_NE(schema.find("\"Pitch\""), std::string::npos);
    EXPECT_NE(schema.find("CalibrationInput"), std::string::npos);
    EXPECT_NE(schema.find("rotateDirection"), std::string::npos);
    EXPECT_EQ(schema.find("useRotateDirection"), std::string::npos);
    EXPECT_NE(schema.find("\"limit\""), std::string::npos);
}

TEST(ConfigTest, RJointSchemaDoesNotContainCalibrationInput) {
    const auto schema = RJoint::Config::ToJsonSchema();
    EXPECT_EQ(schema.find("CalibrationInput"), std::string::npos);
}

TEST(RJointTest, RejectsNonPositiveTorqueConstant) {
    auto adapter = CreateFakeAdapterHarness("rjoint_invalid_kt_adapter");
    AdapterMap adapters{{"adapter1", adapter->Adapter()}};
    auto ranges = CreateMotorRanges();
    ranges.kt = 0.0f;

    RJoint::Config config{};
    config.motorParams = MotorParams{"adapter1", 0, 1, MotorInitParams{ranges}};
    config.limit = Range<float>{-1.0f, 1.0f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    config.reverse = false;

    EXPECT_THROW(
        {
            auto motor = JointManager<>::CreateMotor(config.motorParams, adapters);
            RJoint joint(config, motor);
            (void) joint;
        },
        std::invalid_argument);
}

TEST(RJointTest, RejectsNullMotor) {
    RJoint::Config config{};
    config.motorParams = MotorParams{"adapter1", 0, 1, MotorInitAuto{}};
    config.limit = Range<float>{-1.0f, 1.0f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    config.reverse = false;

    EXPECT_THROW(
        {
            RJoint joint(config, nullptr);
            (void) joint;
        },
        std::invalid_argument);
}

TEST(RJointTest, RejectsInvalidRuntimeLimits) {
    const auto ranges = CreateMotorRanges();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const std::vector<std::pair<std::string, RJoint::Config>> cases = {
        {"reversed_limit", RJoint::Config{MotorParams{"adapter1", 0, 1, MotorInitParams{ranges}},
                                          Range<float>{1.0f, -1.0f}, 2.0f, 3.0f, false}},
        {"nan_limit", RJoint::Config{MotorParams{"adapter1", 0, 1, MotorInitParams{ranges}},
                                     Range<float>{-1.0f, nan}, 2.0f, 3.0f, false}},
        {"zero_max_speed", RJoint::Config{MotorParams{"adapter1", 0, 1, MotorInitParams{ranges}},
                                          Range<float>{-1.0f, 1.0f}, 0.0f, 3.0f, false}},
        {"negative_max_torque",
         RJoint::Config{MotorParams{"adapter1", 0, 1, MotorInitParams{ranges}},
                        Range<float>{-1.0f, 1.0f}, 2.0f, -3.0f, false}},
    };

    for (const auto& [name, config] : cases) {
        auto adapter = CreateFakeAdapterHarness("rjoint_invalid_runtime_" + name);
        AdapterMap adapters{{"adapter1", adapter->Adapter()}};
        auto motor = JointManager<>::CreateMotor(config.motorParams, adapters);

        EXPECT_THROW(
            {
                RJoint joint(config, motor);
                (void) joint;
            },
            std::invalid_argument)
            << name;
    }
}

TEST(RJointTest, SetLimitRejectsInvalidRange) {
    auto fixture = CreateRJointFixture("rjoint_invalid_set_limit", CreateMotorRanges());

    EXPECT_THROW(fixture.joint->SetLimit(Range<float>{1.0f, -1.0f}), std::invalid_argument);
    EXPECT_THROW(
        fixture.joint->SetLimit(Range<float>{-1.0f, std::numeric_limits<float>::quiet_NaN()}),
        std::invalid_argument);
}

TEST(RJointTest, RejectsLimitNarrowerThanSafetySlack) {
    const auto ranges = CreateMotorRanges();
    auto adapter = CreateFakeAdapterHarness("rjoint_narrow_limit");
    RJoint::Config config{MotorParams{"adapter1", 0, 1, MotorInitParams{ranges}},
                          Range<float>{0.0f, 0.01f}, 2.0f, 3.0f, false};
    auto motor = JointManager<>::CreateMotor(
        config.motorParams, AdapterMap{{config.motorParams.AdapterIdString(), adapter->Adapter()}});
    EXPECT_THROW((void) RJoint(config, motor), std::invalid_argument);
}

TEST(RJointTest, SetLimitRejectsRangeNarrowerThanSafetySlack) {
    auto fixture = CreateRJointFixture("rjoint_narrow_set_limit", CreateMotorRanges());
    EXPECT_THROW(fixture.joint->SetLimit(Range<float>{0.0f, 0.01f}), std::invalid_argument);
}

TEST(RJointTest, RejectsNonFiniteControlInputsWithStop) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const auto ranges = CreateMotorRanges();
    const std::vector<std::pair<std::string, std::function<void(RJoint&)>>> cases{
        {"pvt_kp",
         [nan](RJoint& joint) {
             joint.PVTControl(nan, 1.0f, 0.0f, 0.0f, 0.0f);
         }},
        {"pvt_pos",
         [nan](RJoint& joint) {
             joint.PVTControl(1.0f, 1.0f, nan, 0.0f, 0.0f);
         }},
        {"pos",
         [nan](RJoint& joint) {
             joint.PosControl(nan, 1.0f, 0.5f);
         }},
        {"speed_current",
         [nan](RJoint& joint) {
             joint.SpdControl(0.5f, nan);
         }},
        {"current",
         [inf](RJoint& joint) {
             joint.CurControl(inf);
         }},
        {"torque",
         [nan](RJoint& joint) {
             joint.TorControl(nan);
         }},
    };

    for (const auto& [name, action] : cases) {
        auto fixture = CreateRJointFixture("rjoint_nonfinite_" + name, ranges);

        EXPECT_THROW(action(*fixture.joint), std::invalid_argument) << name;
        ExpectStopMessagesWithZeroCurrent(fixture.adapter->GetCommandRecords(), 1u);
    }
}

TEST(RJointTest, StopSanitizesNonFiniteCurrent) {
    auto fixture = CreateRJointFixture("rjoint_stop_nonfinite_current", CreateMotorRanges());

    fixture.joint->Stop(MotorStopMode::DynamicBrake, std::numeric_limits<float>::infinity(), 1);

    ExpectStopMessagesWithZeroCurrent(fixture.adapter->GetCommandRecords(), 1u);
}

TEST(RJointTest, FeedbackTimeoutBrakesBeforePropagatingError) {
    const std::vector<std::function<void(RJoint&)>> controls{
        [](RJoint& joint) {
            joint.PVTControl(1, 1, 0, 0, 0);
        },
        [](RJoint& joint) {
            joint.SpdControl(1);
        },
        [](RJoint& joint) {
            joint.CurControl(1);
        },
        [](RJoint& joint) {
            joint.TorControl(1);
        },
    };
    for (std::size_t i = 0; i < controls.size(); ++i) {
        auto fixture = CreateRJointFixture("rjoint_feedback_timeout_" + std::to_string(i),
                                           CreateMotorRanges());
        fixture.motor->SetMaxStatusLifeCycle(1);
        EmitFakeFeedback(*fixture.adapter, fixture.motor, 0.0f);
        fixture.joint->SpdControl(1);
        fixture.motor->GetStatus(1000);
        ASSERT_FALSE(fixture.motor->GetStatus(0).has_value());
        fixture.adapter->SetReplyMode(FakeReplyMode::Manual);
        fixture.adapter->ClearCommandRecords();
        EXPECT_ANY_THROW(controls[i](*fixture.joint));
        const auto records = fixture.adapter->GetCommandRecords();
        ASSERT_FALSE(records.empty());
        ASSERT_EQ(records.back().kind, FakeCommandKind::Stop);
        EXPECT_EQ(std::get<FakeStopPayload>(records.back().payload).mode, MotorStopMode::FullBrake);
    }
}

TEST(DoubleCoupleJointSafetyTest, ClippedHoldMustRespectVirtualJointLimit) {
    const auto ranges = CreateMotorRanges();
    const auto actual = CalculateExpectedMotorPositions(0.049, 0.0);
    auto fixture = CreatePairedDoubleCoupleFixture(
        "clipped_hold_virtual_limit", ranges, false, false,
        Range<float>{-3.0f, actual.motor1 + 0.001f}, std::nullopt, Range<float>{-0.2f, 0.05f});
    EmitDoubleCoupleFeedback(fixture, actual);
    fixture.master->PVTControl(10, 1, 0.1f, 0, 0);
    fixture.slave->PVTControl(10, 1, 0, 0, 0);
    ExpectStopMessagesWithZeroCurrent(fixture.adapter->GetCommandRecords(), 2u);
}

TEST(DoubleCoupleJointSafetyTest, CurrentRespectsJointTorqueLimit) {
    for (bool slave : {false, true}) {
        for (float current : {-20.0f, 20.0f}) {
            auto fixture = CreatePairedDoubleCoupleFixture(
                "current_joint_limit_" + std::to_string(slave) + std::to_string(current),
                CreateMotorRanges());
            // 动态类控制按合帧下发：本轴命令 + 另一轴命令各一次
            if (slave) {
                fixture.slave->CurControl(current);
                EXPECT_THROW(fixture.master->CurControl(0.0f), JointException);
            } else {
                fixture.master->CurControl(current);
                EXPECT_THROW(fixture.slave->CurControl(0.0f), JointException);
            }
            const auto records = fixture.adapter->GetCommandRecords();
            ASSERT_EQ(records.size(), 2u);
            for (const auto& record : records)
                EXPECT_EQ(record.kind, FakeCommandKind::PosControl);
        }
    }
}

TEST(DoubleCoupleJointPositionControlTest, PairedPositionPreservesBothAxisConstraints) {
    for (bool reverse_order : {false, true}) {
        auto fixture = CreatePairedDoubleCoupleFixture(
            "paired_position_constraints_" + std::to_string(reverse_order), CreateMotorRanges());
        if (reverse_order)
            fixture.slave->PosControl(-0.03f, 2, 3);
        fixture.master->PosControl(0.1f, 0.01f, 0.1f);
        if (!reverse_order)
            fixture.slave->PosControl(-0.03f, 2, 3);
        const auto records = fixture.adapter->GetCommandRecords();
        ASSERT_EQ(records.size(), 2u);
        couple_joint::MechanismSolver solver(CreateValidCalibrationInput());
        solver.SolveInverse(0.1, -0.03);
        const auto velocity = solver.SolveInverseVelocity(0.01, -0.003);
        for (uint32_t id : {1u, 2u}) {
            const auto* message = FindSentMessage(records, id);
            ASSERT_NE(message, nullptr);
            ASSERT_EQ(message->kind, FakeCommandKind::PosControl);
            EXPECT_LE(DecodePosSpeed(*message),
                      std::abs(id == 1 ? velocity.theta1 : velocity.theta2) + 0.006);
            EXPECT_LE(std::get<FakePosControlPayload>(message->payload).current, 0.101f);
        }
        const auto& J = velocity.transmission.velocity_jacobian;
        const double speed1 = DecodePosSpeed(*FindSentMessage(records, 1));
        const double speed2 = DecodePosSpeed(*FindSentMessage(records, 2));
        // A single motor cap may quantize to zero at this low requested speed.
        EXPECT_GT(speed1 + speed2, 0.001);
        EXPECT_LE(std::abs(J.m00) * speed1 + std::abs(J.m01) * speed2, 0.016);
        EXPECT_LE(std::abs(J.m10) * speed1 + std::abs(J.m11) * speed2, 2.006);
    }
}

TEST(DoubleCoupleJointSafetyTest, CurrentTorqueLimitUsesFeedbackPoseKtAndReverse) {
    auto ranges = CreateMotorRanges();
    ranges.kt = 0.7f;
    auto fixture = CreatePairedDoubleCoupleFixture("current_pose_kt_reverse", ranges, true, true);
    EmitDoubleCoupleFeedback(fixture, CalculateExpectedMotorPositions(0.04, -0.02));
    ASSERT_TRUE(fixture.master->GetStatus(false, 0).has_value());
    fixture.adapter->ClearCommandRecords();
    fixture.slave->CurControl(20);
    EXPECT_THROW(fixture.master->CurControl(0.0f), JointException);
    ExpectPositionHoldMessages(fixture.adapter->GetCommandRecords(),
                               CalculateExpectedMotorPositions(0.04, -0.02), ranges);
}

TEST(DoubleCoupleJointPositionControlTest, StopCancelsPendingPositionBatch) {
    auto fixture =
        CreatePairedDoubleCoupleFixture("stop_cancels_position_batch", CreateMotorRanges());
    fixture.master->PosControl(0.1f);
    fixture.slave->Stop(MotorStopMode::FullBrake);
    fixture.adapter->ClearCommandRecords();
    fixture.slave->PosControl(-0.03f);
    EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());
    fixture.slave->PosControl(-0.03f);
    const auto records = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(records.size(), 2u);
    const auto expected = CalculateExpectedMotorPositions(0, -0.03);
    EXPECT_NEAR(DecodePosPosition(*FindSentMessage(records, 1)), expected.motor1, 0.002f);
    EXPECT_NEAR(DecodePosPosition(*FindSentMessage(records, 2)), expected.motor2, 0.002f);
}

TEST(RJointCoordinateTest, ReversePvtConvertsPositionSpeedAndTorqueToMotorFrame) {
    auto fixture = CreateRJointFixture("rjoint_reverse_pvt", CreateMotorRanges(), true);
    EmitFakeFeedback(*fixture.adapter, fixture.motor, 0.0f, 0.0f, 0.0f);
    fixture.adapter->ClearCommandRecords();

    fixture.joint->PVTControl(10.0f, 1.0f, 0.2f, 0.3f, 0.4f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 1u);
    EXPECT_NEAR(DecodePvtPosition(messages.front(), CreateMotorRanges()), -0.2f, 0.005f);
    EXPECT_NEAR(DecodePvtSpeed(messages.front(), CreateMotorRanges()), -0.3f, 0.005f);
    EXPECT_NEAR(DecodePvtTorque(messages.front(), CreateMotorRanges()), -0.4f, 0.005f);
}

TEST(RJointPVTControlTest, TargetPositionIsNotClampedBeforeActualPositionEntersLimitZone) {
    const auto ranges = CreateMotorRanges();

    for (const bool torque_mode : {false, true}) {
        auto fixture = CreateRJointFixture(
            torque_mode ? "rjoint_unclamped_target_torque_mode" : "rjoint_unclamped_target",
            ranges);
        EmitFakeFeedback(*fixture.adapter, fixture.motor, 0.0f, 0.0f, 0.0f);
        fixture.adapter->ClearCommandRecords();

        BaseJoint& joint = *fixture.joint;
        joint.PVTControl(10.0f, 1.0f, 1.5f, 0.0f, 0.0f,
                         (torque_mode ? PVTControlMode::HostKpMotorKd : PVTControlMode::MotorPVT));

        const auto& messages = fixture.adapter->GetCommandRecords();
        ASSERT_EQ(messages.size(), 1u);
        EXPECT_EQ(messages.front().kind, FakeCommandKind::PVTControl);
        EXPECT_NEAR(DecodePvtPosition(messages.front(), ranges), 1.5f, 0.005f);
    }
}

TEST(RJointPVTControlTest, ActualPositionInLimitZoneStopsOnlyOutwardPvt) {
    const auto ranges = CreateMotorRanges();

    for (const bool torque_mode : {false, true}) {
        auto outward = CreateRJointFixture(
            torque_mode ? "rjoint_actual_limit_torque_mode" : "rjoint_actual_limit", ranges);
        EmitFakeFeedback(*outward.adapter, outward.motor, 0.99f, 0.0f, 0.0f);
        outward.adapter->ClearCommandRecords();
        BaseJoint& outward_joint = *outward.joint;
        outward_joint.PVTControl(
            10.0f, 1.0f, 1.5f, 0.0f, 0.0f,
            (torque_mode ? PVTControlMode::HostKpMotorKd : PVTControlMode::MotorPVT));
        ASSERT_EQ(outward.adapter->GetCommandRecords().size(), 1u);
        EXPECT_EQ(outward.adapter->GetCommandRecords().front().kind, FakeCommandKind::PosControl);

        auto inward = CreateRJointFixture(
            torque_mode ? "rjoint_limit_exit_torque_mode" : "rjoint_limit_exit", ranges);
        EmitFakeFeedback(*inward.adapter, inward.motor, 0.99f, 0.0f, 0.0f);
        inward.adapter->ClearCommandRecords();
        BaseJoint& inward_joint = *inward.joint;
        inward_joint.PVTControl(
            10.0f, 1.0f, 0.0f, 0.0f, 0.0f,
            (torque_mode ? PVTControlMode::HostKpMotorKd : PVTControlMode::MotorPVT));
        ASSERT_EQ(inward.adapter->GetCommandRecords().size(), 1u);
        EXPECT_EQ(inward.adapter->GetCommandRecords().front().kind, FakeCommandKind::PVTControl);
    }
}

TEST(RJointPVTControlTest, ReversePvtUsesTransformedAsymmetricLimitZone) {
    const auto ranges = CreateMotorRanges();
    auto adapter = CreateFakeAdapterHarness("rjoint_reverse_asymmetric_limit");
    AdapterMap adapters{{"adapter1", adapter->Adapter()}};
    RJoint::Config config{MotorParams{"adapter1", 0, 1, MotorInitParams{ranges}},
                          Range<float>{-0.1f, 0.05f}, 2.0f, 3.0f, true};
    auto motor = JointManager<>::CreateMotor(config.motorParams, adapters);
    SyncFakeRangeSnapshot(*adapter, motor, ranges);
    RJoint joint(config, motor);
    EmitFakeFeedback(*adapter, motor, -0.045f, 0.0f, 0.0f);
    adapter->ClearCommandRecords();

    joint.PVTControl(10.0f, 1.0f, 0.08f, 0.0f, 0.0f);

    ASSERT_EQ(adapter->GetCommandRecords().size(), 1u);
    EXPECT_EQ(adapter->GetCommandRecords().front().kind, FakeCommandKind::PosControl);
}

TEST(RJointPVTControlTest, TargetCloserToRangeIsAllowedEvenIfStillInsideLimitZone) {
    const auto ranges = CreateMotorRanges();
    for (const bool torque_mode : {false, true}) {
        auto fixture = CreateRJointFixture(
            torque_mode ? "rjoint_partial_limit_exit_torque_mode" : "rjoint_partial_limit_exit",
            ranges);
        EmitFakeFeedback(*fixture.adapter, fixture.motor, 1.1f, 0.0f, 0.0f);
        fixture.adapter->ClearCommandRecords();

        BaseJoint& joint = *fixture.joint;
        joint.PVTControl(10.0f, 1.0f, 1.0f, 0.0f, 0.0f,
                         (torque_mode ? PVTControlMode::HostKpMotorKd : PVTControlMode::MotorPVT));

        ASSERT_EQ(fixture.adapter->GetCommandRecords().size(), 1u);
        EXPECT_EQ(fixture.adapter->GetCommandRecords().front().kind, FakeCommandKind::PVTControl);
    }
}

TEST(RJointDynamicLimitTest, ZeroDynamicCommandsOutsideLimitHoldInsideSlack) {
    const auto ranges = CreateMotorRanges();
    const std::vector<std::pair<std::string, std::function<void(RJoint&)>>> commands{
        {"speed",
         [](RJoint& joint) {
             joint.SpdControl(0.0f);
         }},
        {"current",
         [](RJoint& joint) {
             joint.CurControl(0.0f);
         }},
        {"torque",
         [](RJoint& joint) {
             joint.TorControl(0.0f);
         }},
    };
    for (const auto& [name, command] : commands) {
        auto fixture = CreateRJointFixture("rjoint_zero_outside_" + name, ranges);
        EmitFakeFeedback(*fixture.adapter, fixture.motor, 1.1f, 0.0f, 0.0f);
        fixture.adapter->ClearCommandRecords();

        command(*fixture.joint);

        ASSERT_EQ(fixture.adapter->GetCommandRecords().size(), 1u) << name;
        const auto& message = fixture.adapter->GetCommandRecords().front();
        ASSERT_EQ(message.kind, FakeCommandKind::PosControl) << name;
        EXPECT_NEAR(DecodePosPosition(message), 1.0f - kLimitSlack, 0.001f) << name;
    }
}

TEST(RJointCoordinateTest, ReverseStatusConvertsMotorFrameToJointFrame) {
    auto fixture = CreateRJointFixture("rjoint_reverse_status", CreateMotorRanges(), true);
    EmitFakeFeedback(*fixture.adapter, fixture.motor, -0.2f, -0.3f, -0.4f);

    const auto status = fixture.joint->GetStatus(false, 0);

    ASSERT_TRUE(status.has_value());
    EXPECT_NEAR(status->pos, 0.2f, 0.005f);
    EXPECT_NEAR(status->spd, 0.3f, 0.005f);
    EXPECT_NEAR(status->tor, 0.4f, 0.005f);
}

TEST(RJointCoordinateTest, ReverseSetOnStatusConvertsMotorFrameToJointFrame) {
    auto fixture = CreateRJointFixture("rjoint_reverse_callback", CreateMotorRanges(), true);
    bool callback_called = false;
    JointStatus callback_status{};
    fixture.joint->SetOnStatus([&](const JointStatus& status) {
        callback_called = true;
        callback_status = status;
    });

    EmitFakeFeedback(*fixture.adapter, fixture.motor, -0.2f, -0.3f, -0.4f);

    ASSERT_TRUE(callback_called);
    EXPECT_NEAR(callback_status.pos, 0.2f, 0.005f);
    EXPECT_NEAR(callback_status.spd, 0.3f, 0.005f);
    EXPECT_NEAR(callback_status.tor, 0.4f, 0.005f);
}

TEST(RJointCoordinateTest, ReversePosControlClampsInJointCoordinates) {
    auto fixture = CreateRJointFixture("rjoint_reverse_pos_clamp", CreateMotorRanges(), true);

    fixture.joint->PosControl(2.0f, 1.0f, 0.5f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 1u);
    EXPECT_NEAR(DecodePosPosition(messages.front()), -1.0f + kLimitSlack, 0.001f);
}

TEST(ContinuousJointTest, RejectsPositionAndLimitOperationsWithoutSendingCommands) {
    auto fixture = CreateContinuousJointFixture("continuous_unsupported", CreateMotorRanges());

    const std::vector<std::function<void()>> operations{
        [&] {
            fixture.joint->PVTControl(1.0f, 1.0f, 0.0f, 0.0f, 0.0f);
        },
        [&] {
            fixture.joint->PVTControl(1.0f, 1.0f, 0.0f, 0.0f, 0.0f, PVTControlMode::HostKpMotorKd);
        },
        [&] {
            fixture.joint->PosControl(0.0f);
        },
        [&] {
            fixture.joint->SetLimit(Range<float>{-1.0f, 1.0f});
        },
        [&] {
            (void) fixture.joint->PosAsOffset(1);
        },
    };

    for (const auto& operation : operations) {
        try {
            operation();
            FAIL() << "Expected ContinuousJoint unsupported-operation error";
        } catch (const JointException& ex) {
            EXPECT_EQ(ex.Error(), JointError::kContinuousJointUnsupportedOperation);
        }
    }
    EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());
}

TEST(ContinuousJointTest, DerivesDirectlyFromBaseJointInsteadOfRJoint) {
    EXPECT_TRUE((std::is_base_of_v<BaseJoint, ContinuousJoint>) );
    EXPECT_FALSE((std::is_base_of_v<RJoint, ContinuousJoint>) );
}

TEST(ContinuousJointTest, RejectsTorqueConstantThatWouldOverflowCurrentLimit) {
    auto ranges = CreateMotorRanges();
    ranges.kt = std::numeric_limits<float>::denorm_min();

    EXPECT_THROW((void) CreateContinuousJointFixture("continuous_overflow_kt", ranges),
                 std::invalid_argument);
}

TEST(ContinuousJointTest, DynamicControlsIgnorePositionBoundaryAndRetainMagnitudeLimits) {
    auto fixture = CreateContinuousJointFixture("continuous_dynamic", CreateMotorRanges());
    EmitFakeFeedback(*fixture.adapter, fixture.motor, 12.4f);
    fixture.adapter->ClearCommandRecords();

    fixture.joint->SpdControl(10.0f, 100.0f, 1);
    fixture.joint->CurControl(100.0f, 1);
    fixture.joint->TorControl(100.0f, 1);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 3u);
    EXPECT_EQ(messages[0].kind, FakeCommandKind::SpdControl);
    EXPECT_NEAR(DecodeSpeedCommand(messages[0]), 2.0f, 0.005f);
    EXPECT_EQ(messages[1].kind, FakeCommandKind::CurControl);
    EXPECT_NEAR(DecodeCurrentCommand(messages[1]), 3.0f, 0.005f);
    EXPECT_EQ(messages[2].kind, FakeCommandKind::TorControl);
    EXPECT_NEAR(DecodeTorqueCommand(messages[2]), 3.0f, 0.005f);
}

TEST(ContinuousJointTest, PullStatusUnwrapsForwardAndBackwardCrossings) {
    auto forward = CreateContinuousJointFixture("continuous_forward", CreateMotorRanges());
    EmitFakeFeedback(*forward.adapter, forward.motor, 12.4f);
    ASSERT_TRUE(forward.joint->GetStatus(false, 0).has_value());
    EmitFakeFeedback(*forward.adapter, forward.motor, -12.4f);
    const auto forward_status = forward.joint->GetStatus(false, 0);
    ASSERT_TRUE(forward_status.has_value());
    EXPECT_NEAR(forward_status->pos, 12.6f, 0.02f);

    auto backward = CreateContinuousJointFixture("continuous_backward", CreateMotorRanges());
    EmitFakeFeedback(*backward.adapter, backward.motor, -12.4f);
    ASSERT_TRUE(backward.joint->GetStatus(false, 0).has_value());
    EmitFakeFeedback(*backward.adapter, backward.motor, 12.4f);
    const auto backward_status = backward.joint->GetStatus(false, 0);
    ASSERT_TRUE(backward_status.has_value());
    EXPECT_NEAR(backward_status->pos, -12.6f, 0.02f);
}

TEST(ContinuousJointTest, UsesConfiguredMotorPositionRangeAsUnwrapPeriod) {
    auto ranges = CreateMotorRanges();
    ranges.position = Range<float>{-5.0f, 5.0f};
    auto fixture = CreateContinuousJointFixture("continuous_custom_period", ranges);

    EmitFakeFeedback(*fixture.adapter, fixture.motor, 4.9f);
    ASSERT_TRUE(fixture.joint->GetStatus(false, 0).has_value());
    EmitFakeFeedback(*fixture.adapter, fixture.motor, -4.9f);
    const auto status = fixture.joint->GetStatus(false, 0);

    ASSERT_TRUE(status.has_value());
    EXPECT_NEAR(status->pos, 5.1f, 0.02f);
}

TEST(ContinuousJointTest, CallbackAndPullStatusShareUnwrappedPosition) {
    auto fixture = CreateContinuousJointFixture("continuous_callback", CreateMotorRanges(), true);
    std::vector<float> callback_positions;
    fixture.joint->SetOnStatus([&](const JointStatus& status) {
        callback_positions.push_back(status.pos);
    });

    EmitFakeFeedback(*fixture.adapter, fixture.motor, 12.4f);
    auto first_pull = fixture.joint->GetStatus(false, 0);
    EmitFakeFeedback(*fixture.adapter, fixture.motor, -12.4f);
    auto second_pull = fixture.joint->GetStatus(false, 0);

    ASSERT_EQ(callback_positions.size(), 2u);
    ASSERT_TRUE(first_pull.has_value());
    ASSERT_TRUE(second_pull.has_value());
    EXPECT_NEAR(callback_positions[0], -12.4f, 0.02f);
    EXPECT_NEAR(first_pull->pos, callback_positions[0], 0.001f);
    EXPECT_NEAR(callback_positions[1], -12.6f, 0.02f);
    EXPECT_NEAR(second_pull->pos, callback_positions[1], 0.001f);
}

TEST(ContinuousJointTest, AutoSendUsesInternalPvtFeedbackCommand) {
    auto fixture = CreateContinuousJointFixture("continuous_auto_send", CreateMotorRanges());

    EXPECT_FALSE(fixture.joint->GetStatus(true, 0).has_value());

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 1u);
    EXPECT_EQ(messages.front().kind, FakeCommandKind::PVTControl);
}

TEST(DoubleCoupleJointPairingTest, RejectsInvalidTorqueConstantBeforeCreatingMaster) {
    const std::vector<float> invalid_kts{0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN(),
                                         std::numeric_limits<float>::infinity()};

    for (size_t index = 0; index < invalid_kts.size(); ++index) {
        auto adapter =
            CreateFakeAdapterHarness("double_couple_invalid_kt_" + std::to_string(index));
        AdapterMap adapters{{"adapter1", adapter->Adapter()}};
        const auto valid_ranges = CreateMotorRanges();
        auto invalid_ranges = valid_ranges;
        invalid_ranges.kt = invalid_kts[index];

        auto slave_config =
            CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Pitch, valid_ranges);
        auto master_config =
            CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Roll, valid_ranges);
        if (index % 2 == 0) {
            master_config.motorParams1.initParam = MotorInitParams{invalid_ranges};
        } else {
            master_config.motorParams2.initParam = MotorInitParams{invalid_ranges};
        }

        auto slave =
            std::static_pointer_cast<SlaveJoint>(DoubleCoupleJoint::CreateJoint(slave_config));
        auto motor1 = JointManager<>::CreateMotor(master_config.motorParams1, adapters);
        auto motor2 = JointManager<>::CreateMotor(master_config.motorParams2, adapters);
        EXPECT_THROW(
            { (void) DoubleCoupleJoint::CreateJoint(master_config, motor1, motor2, slave); },
            std::invalid_argument)
            << "invalid kt index " << index;
    }
}

TEST(DoubleCoupleJointPairingTest, RejectsPartialMotorInjection) {
    auto adapter = CreateFakeAdapterHarness("partial_motor_injection");
    AdapterMap adapters{{"adapter1", adapter->Adapter()}};
    auto ranges = CreateMotorRanges();
    auto slave_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Pitch, ranges);
    auto master_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Roll, ranges);
    auto slave = std::static_pointer_cast<SlaveJoint>(DoubleCoupleJoint::CreateJoint(slave_config));
    auto motor1 = JointManager<>::CreateMotor(master_config.motorParams1, adapters);

    EXPECT_THROW((void) DoubleCoupleJoint::CreateJoint(master_config, motor1, nullptr, slave),
                 std::invalid_argument);
}

TEST(DoubleCoupleJointPairingTest, DirectFactoryRejectsNullSlaveBeforeDereference) {
    auto adapter = CreateFakeAdapterHarness("direct_factory_null_slave");
    AdapterMap adapters{{"adapter1", adapter->Adapter()}};
    auto ranges = CreateMotorRanges();
    auto master_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Roll, ranges);
    auto motor1 = JointManager<>::CreateMotor(master_config.motorParams1, adapters);
    auto motor2 = JointManager<>::CreateMotor(master_config.motorParams2, adapters);

    try {
        (void) DoubleCoupleJoint::CreateJoint(master_config, motor1, motor2, nullptr);
    } catch (const JointException& ex) {
        EXPECT_EQ(ex.Error(), JointError::kDoubleCoupleJointNotPaired);
        return;
    }
    FAIL() << "Expected kDoubleCoupleJointNotPaired";
}

TEST(DoubleCoupleJointPairingTest, DirectFactoryRejectsDuplicateAxis) {
    auto adapter = CreateFakeAdapterHarness("direct_factory_duplicate_axis");
    AdapterMap adapters{{"adapter1", adapter->Adapter()}};
    auto ranges = CreateMotorRanges();
    auto slave_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Roll, ranges);
    auto master_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Roll, ranges);
    auto slave = std::static_pointer_cast<SlaveJoint>(DoubleCoupleJoint::CreateJoint(slave_config));
    auto motor1 = JointManager<>::CreateMotor(master_config.motorParams1, adapters);
    auto motor2 = JointManager<>::CreateMotor(master_config.motorParams2, adapters);

    try {
        (void) DoubleCoupleJoint::CreateJoint(master_config, motor1, motor2, slave);
    } catch (const JointException& ex) {
        EXPECT_EQ(ex.Error(), JointError::kDoubleCoupleJointDuplicateAxis);
        return;
    }
    FAIL() << "Expected kDoubleCoupleJointDuplicateAxis";
}

TEST(DoubleCoupleJointPairingTest, RejectsConflictingPairMotorLimits) {
    auto adapter = CreateFakeAdapterHarness("motor_limit_conflict");
    AdapterMap adapters{{"adapter1", adapter->Adapter()}};
    auto ranges = CreateMotorRanges();
    auto slave_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Pitch, ranges);
    auto master_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Roll, ranges);
    slave_config.motorLimit1 = Range<float>{-0.4f, 0.4f};
    master_config.motorLimit1 = Range<float>{-0.5f, 0.5f};

    auto slave = std::static_pointer_cast<SlaveJoint>(DoubleCoupleJoint::CreateJoint(slave_config));
    auto motor1 = JointManager<>::CreateMotor(master_config.motorParams1, adapters);
    auto motor2 = JointManager<>::CreateMotor(master_config.motorParams2, adapters);

    try {
        (void) DoubleCoupleJoint::CreateJoint(master_config, motor1, motor2, slave);
    } catch (const JointException& ex) {
        EXPECT_EQ(ex.Error(), JointError::kDoubleCoupleJointMotorLimitMismatch);
        return;
    }
    FAIL() << "Expected motor limit mismatch";
}

TEST(DoubleCoupleJointPairingTest, AcceptsSingleSidedPairMotorLimit) {
    auto adapter = CreateFakeAdapterHarness("single_sided_motor_limit");
    AdapterMap adapters{{"adapter1", adapter->Adapter()}};
    auto ranges = CreateMotorRanges();
    auto slave_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Pitch, ranges);
    auto master_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Roll, ranges);
    slave_config.motorLimit2 = Range<float>{-0.3f, 0.3f};

    auto slave = std::static_pointer_cast<SlaveJoint>(DoubleCoupleJoint::CreateJoint(slave_config));
    auto motor1 = JointManager<>::CreateMotor(master_config.motorParams1, adapters);
    auto motor2 = JointManager<>::CreateMotor(master_config.motorParams2, adapters);

    adapter->SetReplyMode(FakeReplyMode::Automatic);
    EXPECT_NO_THROW((void) DoubleCoupleJoint::CreateJoint(master_config, motor1, motor2, slave));
}

TEST(DoubleCoupleJointPairingTest, AcceptsMatchingPairMotorLimits) {
    auto adapter = CreateFakeAdapterHarness("matching_motor_limits");
    AdapterMap adapters{{"adapter1", adapter->Adapter()}};
    auto ranges = CreateMotorRanges();
    auto slave_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Pitch, ranges);
    auto master_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Roll, ranges);
    slave_config.motorLimit1 = Range<float>{-0.4f, 0.4f};
    master_config.motorLimit1 = Range<float>{-0.4f, 0.4f};

    auto slave = std::static_pointer_cast<SlaveJoint>(DoubleCoupleJoint::CreateJoint(slave_config));
    auto motor1 = JointManager<>::CreateMotor(master_config.motorParams1, adapters);
    auto motor2 = JointManager<>::CreateMotor(master_config.motorParams2, adapters);

    adapter->SetReplyMode(FakeReplyMode::Automatic);
    EXPECT_NO_THROW((void) DoubleCoupleJoint::CreateJoint(master_config, motor1, motor2, slave));
}

TEST(DoubleCoupleJointPairingTest, RejectsInvalidMotorLimitRange) {
    auto adapter = CreateFakeAdapterHarness("invalid_motor_limit_range");
    AdapterMap adapters{{"adapter1", adapter->Adapter()}};
    auto ranges = CreateMotorRanges();
    auto slave_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Pitch, ranges);
    auto master_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Roll, ranges);
    master_config.motorLimit1 = Range<float>{0.3f, -0.3f};

    auto slave = std::static_pointer_cast<SlaveJoint>(DoubleCoupleJoint::CreateJoint(slave_config));
    auto motor1 = JointManager<>::CreateMotor(master_config.motorParams1, adapters);
    auto motor2 = JointManager<>::CreateMotor(master_config.motorParams2, adapters);

    EXPECT_THROW((void) DoubleCoupleJoint::CreateJoint(master_config, motor1, motor2, slave),
                 std::invalid_argument);
}

TEST(DoubleCoupleJointPairingTest, ManagerRejectsDuplicateAxisBeforeCreatingMotors) {
    JointManager<> manager;
    auto ranges = CreateMotorRanges();
    auto roll_config = CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Roll, ranges);
    auto duplicate_config = roll_config;

    manager.AddJoint<DoubleCoupleJoint>("roll_pending", roll_config);

    try {
        (void) manager.AddJoint<DoubleCoupleJoint>("roll_duplicate", duplicate_config);
    } catch (const JointException& ex) {
        EXPECT_EQ(ex.Error(), JointError::kDoubleCoupleJointDuplicateAxis);
        return;
    }
    FAIL() << "Expected duplicate axis JointException";
}

TEST(DoubleCoupleJointPairingTest, ManagerRejectsCalibrationMismatchBeforeCreatingMotors) {
    JointManager<> manager;
    auto ranges = CreateMotorRanges();
    auto roll_config = CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Roll, ranges);
    auto pitch_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Pitch, ranges);
    pitch_config.calibration.left_joint_center.x += 0.01;

    manager.AddJoint<DoubleCoupleJoint>("roll_pending", roll_config);

    try {
        (void) manager.AddJoint<DoubleCoupleJoint>("pitch_mismatch", pitch_config);
    } catch (const JointException& ex) {
        EXPECT_EQ(ex.Error(), JointError::kDoubleCoupleJointCalibrationMismatch);
        return;
    }
    FAIL() << "Expected calibration mismatch JointException";
}

TEST(DoubleCoupleJointPairingTest, RejectsInvalidCalibrationBeforeCreatingMotors) {
    auto adapter = CreateFakeAdapterHarness("invalid_calib");
    AdapterMap adapters{{"adapter1", adapter->Adapter()}};
    auto ranges = CreateMotorRanges();
    auto roll_config = CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Roll, ranges);
    auto pitch_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Pitch, ranges);
    roll_config.calibration.right_joint_center = roll_config.calibration.left_joint_center;
    pitch_config.calibration = roll_config.calibration;
    auto slave = std::static_pointer_cast<SlaveJoint>(DoubleCoupleJoint::CreateJoint(roll_config));

    auto motor1 = JointManager<>::CreateMotor(pitch_config.motorParams1, adapters);
    auto motor2 = JointManager<>::CreateMotor(pitch_config.motorParams2, adapters);

    try {
        (void) DoubleCoupleJoint::CreateJoint(pitch_config, motor1, motor2, slave);
    } catch (const JointException& ex) {
        EXPECT_EQ(ex.Error(), JointError::kDoubleCoupleJointCalibrationFailed);
        return;
    }
    FAIL() << "Expected calibration failure JointException";
}

TEST(SlaveJointTest, PendingUnsupportedPosAsOffsetDoesNotThrowNotPaired) {
    auto slave_config = CreateDoubleCoupleJointConfig(DoubleCoupleJointAxis::Pitch);
    auto slave = std::static_pointer_cast<SlaveJoint>(DoubleCoupleJoint::CreateJoint(slave_config));

    EXPECT_TRUE(std::isnan(slave->PosAsOffset(1)));

    try {
        slave->Stop(MotorStopMode::FullBrake);
    } catch (const JointException& ex) {
        EXPECT_EQ(ex.Error(), JointError::kDoubleCoupleJointNotPaired);
        return;
    }
    FAIL() << "Unsupported helpers must not bind a pending SlaveJoint";
}

TEST(DoubleCoupleJointPairingTest, MasterInitializesPoseFromCurrentMotorPositions) {
    auto adapter = CreateFakeAdapterHarness("initial_pose_from_motor_positions");
    AdapterMap adapters{{"adapter1", adapter->Adapter()}};
    const auto ranges = CreateMotorRanges();
    auto slave_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Pitch, ranges);
    auto master_config =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Roll, ranges);
    auto slave = std::static_pointer_cast<SlaveJoint>(DoubleCoupleJoint::CreateJoint(slave_config));
    auto motor1 = JointManager<>::CreateMotor(master_config.motorParams1, adapters);
    auto motor2 = JointManager<>::CreateMotor(master_config.motorParams2, adapters);
    SyncFakeRangeSnapshot(*adapter, motor1, ranges);
    SyncFakeRangeSnapshot(*adapter, motor2, ranges);

    const auto initial_positions = CalculateExpectedMotorPositions(0.04, -0.03);
    adapter->SetReplyMode(FakeReplyMode::Automatic);
    motor1->PVTControl<0>(0.0f, 1.0f, initial_positions.motor1, 0.0f, 0.0f);
    motor2->PVTControl<0>(0.0f, 1.0f, initial_positions.motor2, 0.0f, 0.0f);
    adapter->ClearCommandRecords();

    auto master = std::static_pointer_cast<MasterJoint>(
        DoubleCoupleJoint::CreateJoint(master_config, motor1, motor2, slave));

    const auto& creation_messages = adapter->GetCommandRecords();
    const auto position_parameter_reads = std::count_if(
        creation_messages.begin(), creation_messages.end(), [](const FakeCommandRecord& record) {
            if (record.kind != FakeCommandKind::GetParameter) {
                return false;
            }
            const auto* payload = std::get_if<FakeGetParameterPayload>(&record.payload);
            return payload && payload->parameter == MotorParameter::Position;
        });
    EXPECT_EQ(position_parameter_reads, 2);
    EXPECT_TRUE(std::none_of(creation_messages.begin(), creation_messages.end(),
                             [](const FakeCommandRecord& record) {
                                 return record.kind == FakeCommandKind::ResetZeroPos;
                             }));

    adapter->SetReplyMode(FakeReplyMode::Manual);
    adapter->ClearCommandRecords();
    master->PVTControl(10.0f, 1.0f, 0.05f, 0.3f, 0.2f);
    master->PVTControl(10.0f, 1.0f, 0.05f, 0.3f, 0.2f);

    const auto& control_messages = adapter->GetCommandRecords();
    ASSERT_EQ(control_messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(control_messages, 1);
    const auto* motor2_message = FindSentMessage(control_messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    const auto expected = CalculateExpectedMotorPositions(0.05, -0.03);
    EXPECT_NEAR(DecodePvtPosition(*motor1_message, ranges), expected.motor1, 0.002f);
    EXPECT_NEAR(DecodePvtPosition(*motor2_message, ranges), expected.motor2, 0.002f);
}

TEST(DoubleCoupleJointPVTControlTest, RollPvtSolvesBothMotorPositions) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("pvt_roll_adapter", ranges);

    fixture.master->PVTControl(10.0f, 1.0f, 0.05f, 0.3f, 0.2f);
    fixture.master->PVTControl(10.0f, 1.0f, 0.05f, 0.3f, 0.2f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    const auto expected = CalculateExpectedMotorPositions(0.05, 0.0);
    EXPECT_NEAR(DecodePvtPosition(*motor1_message, ranges), expected.motor1, 0.002f);
    EXPECT_NEAR(DecodePvtPosition(*motor2_message, ranges), expected.motor2, 0.002f);
}

TEST(DoubleCoupleJointPVTControlTest, SlavePitchPvtUsesCachedRollTarget) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("pvt_cached_roll_adapter", ranges);

    fixture.master->PVTControl(10.0f, 1.0f, 0.05f, 0.3f, 0.2f);
    fixture.adapter->ClearCommandRecords();
    fixture.slave->PVTControl(10.0f, 1.0f, -0.04f, 0.2f, 0.1f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    const auto expected = CalculateExpectedMotorPositions(0.05, -0.04);
    EXPECT_NEAR(DecodePvtPosition(*motor1_message, ranges), expected.motor1, 0.002f);
    EXPECT_NEAR(DecodePvtPosition(*motor2_message, ranges), expected.motor2, 0.002f);
}

TEST(DoubleCoupleJointPVTControlTest, BatchesTwoAxisPvtControlsIntoFinalMotorCommand) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("pvt_batched_axes_adapter", ranges);

    fixture.master->PVTControl(10.0f, 1.0f, 0.05f, 0.3f, 0.2f);
    EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());

    fixture.slave->PVTControl(11.0f, 1.5f, -0.04f, 0.0f, 0.0f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    const auto expected = CalculateExpectedMotorPositions(0.05, -0.04);
    const auto expected_speeds = CalculateExpectedCombinedMotorSpeeds(0.0, 0.0, 0.3f, 0.0f, ranges);
    const auto expected_torques =
        CalculateExpectedCombinedMotorTorques(0.0, 0.0, 0.2f, 0.0f, ranges);
    EXPECT_NEAR(DecodePvtPosition(*motor1_message, ranges), expected.motor1, 0.002f);
    EXPECT_NEAR(DecodePvtPosition(*motor2_message, ranges), expected.motor2, 0.002f);
    EXPECT_NEAR(DecodePvtSpeed(*motor1_message, ranges), expected_speeds.motor1, 0.006f);
    EXPECT_NEAR(DecodePvtSpeed(*motor2_message, ranges), expected_speeds.motor2, 0.006f);
    EXPECT_NEAR(DecodePvtTorque(*motor1_message, ranges), expected_torques.motor1, 0.01f);
    EXPECT_NEAR(DecodePvtTorque(*motor2_message, ranges), expected_torques.motor2, 0.01f);
}

TEST(DoubleCoupleJointPVTControlTest, ConvertsAndCombinesJointGainsForBothMotors) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("pvt_combined_gains_adapter", ranges);
    constexpr double kActualRoll = 0.2;
    constexpr double kActualPitch = -0.15;
    EmitDoubleCoupleFeedback(fixture, CalculateExpectedMotorPositions(kActualRoll, kActualPitch));

    fixture.master->PVTControl(12.0f, 8.0f, 0.05f, 0.0f, 0.0f, PVTControlMode::MotorPVT);
    fixture.slave->PVTControl(21.0f, 13.0f, -0.04f, 0.0f, 0.0f, PVTControlMode::MotorPVT);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    const auto expected_kp = CalculateExpectedMotorGains(kActualRoll, kActualPitch, 12.0f, 21.0f);
    const auto expected_kd = CalculateExpectedMotorGains(kActualRoll, kActualPitch, 8.0f, 13.0f);
    EXPECT_NEAR(DecodePvtKp(*motor1_message), expected_kp.motor1, 0.15f);
    EXPECT_NEAR(DecodePvtKp(*motor2_message), expected_kp.motor2, 0.15f);
    EXPECT_NEAR(DecodePvtKd(*motor1_message), expected_kd.motor1, 0.35f);
    EXPECT_NEAR(DecodePvtKd(*motor2_message), expected_kd.motor2, 0.35f);
}

TEST(DoubleCoupleJointPVTControlTest, MapsCombinedSpeedAndTorqueAtActualFeedbackPose) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("pvt_actual_differential_pose_adapter", ranges);
    constexpr double kActualRoll = 0.2;
    constexpr double kActualPitch = -0.15;
    EmitDoubleCoupleFeedback(fixture, CalculateExpectedMotorPositions(kActualRoll, kActualPitch));

    fixture.master->PVTControl(12.0f, 0.8f, 0.05f, 0.3f, 0.4f);
    fixture.slave->PVTControl(21.0f, 1.3f, -0.04f, -0.2f, -0.1f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    const auto expected_speeds =
        CalculateExpectedCombinedMotorSpeeds(kActualRoll, kActualPitch, 0.3f, -0.2f, ranges);
    const auto expected_torques =
        CalculateExpectedCombinedMotorTorques(kActualRoll, kActualPitch, 0.4f, -0.1f, ranges);
    EXPECT_NEAR(DecodePvtSpeed(*motor1_message, ranges), expected_speeds.motor1, 0.006f);
    EXPECT_NEAR(DecodePvtSpeed(*motor2_message, ranges), expected_speeds.motor2, 0.006f);
    EXPECT_NEAR(DecodePvtTorque(*motor1_message, ranges), expected_torques.motor1, 0.01f);
    EXPECT_NEAR(DecodePvtTorque(*motor2_message, ranges), expected_torques.motor2, 0.01f);
}

TEST(DoubleCoupleJointPVTControlTest, FeedbackDoesNotOverwritePendingFirstAxisTarget) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("pvt_pending_feedback_adapter", ranges);

    fixture.master->PVTControl(10.0f, 1.0f, 0.05f, 0.0f, 0.0f);
    EmitDoubleCoupleFeedback(fixture, CalculateExpectedMotorPositions(0.0, 0.0));
    ASSERT_TRUE(fixture.master->GetStatus(false, 0).has_value());
    fixture.slave->PVTControl(11.0f, 1.5f, -0.04f, 0.0f, 0.0f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    const auto expected = CalculateExpectedMotorPositions(0.05, -0.04);
    EXPECT_NEAR(DecodePvtPosition(*motor1_message, ranges), expected.motor1, 0.002f);
    EXPECT_NEAR(DecodePvtPosition(*motor2_message, ranges), expected.motor2, 0.002f);
}

TEST(DoubleCoupleJointPVTControlTest, TorqueModeComputesPositionTorqueAndDelegatesDamping) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("pvt_host_torque_adapter", ranges);
    constexpr float kActualRoll = 0.01f;
    constexpr float kActualPitch = -0.02f;
    const auto actual_positions = CalculateExpectedMotorPositions(kActualRoll, kActualPitch);
    EmitDoubleCoupleFeedback(fixture, actual_positions);

    BaseJoint& master = *fixture.master;
    BaseJoint& slave = *fixture.slave;
    master.PVTControl(10.0f, 0.5f, 0.05f, 0.2f, 0.3f, PVTControlMode::HostKpMotorKd);
    slave.PVTControl(20.0f, 0.75f, -0.04f, -0.1f, -0.2f, PVTControlMode::HostKpMotorKd);

    constexpr float kExpectedRollTorque = 0.7f;
    constexpr float kExpectedPitchTorque = -0.6f;
    const auto expected = CalculateExpectedCombinedMotorTorques(
        kActualRoll, kActualPitch, kExpectedRollTorque, kExpectedPitchTorque, ranges);
    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto kd = CalculateExpectedMotorGains(kActualRoll, kActualPitch, 0.5f, 0.75f);
    const auto damping = CalculateExpectedCombinedMotorTorques(kActualRoll, kActualPitch,
                                                               0.5f * 0.2f, 0.75f * -0.1f, ranges);
    for (int id : {1, 2}) {
        const auto* message = FindSentMessage(messages, id);
        ASSERT_NE(message, nullptr);
        ASSERT_EQ(message->kind, FakeCommandKind::PVTControl);
        EXPECT_FLOAT_EQ(DecodePvtKp(*message), 0.0f);
        EXPECT_NEAR(DecodePvtKd(*message), std::max(0.1f, id == 1 ? kd.motor1 : kd.motor2), 0.01f);
        EXPECT_NEAR(DecodePvtSpeed(*message, ranges),
                    (id == 1 ? damping.motor1 : damping.motor2) /
                        std::max(0.1f, id == 1 ? kd.motor1 : kd.motor2),
                    0.01f);
        EXPECT_NEAR(DecodePvtTorque(*message, ranges), id == 1 ? expected.motor1 : expected.motor2,
                    0.02f);
    }
}

TEST(DoubleCoupleJointPVTControlTest, HostKpMotorKdMatchesFullJointLayerDamping) {
    constexpr double kActualRoll = 0.2;
    constexpr double kActualPitch = -0.15;
    constexpr float kMotor1Speed = 0.3f;
    constexpr float kMotor2Speed = -0.2f;
    constexpr float kRollKp = 12.0f;
    constexpr float kPitchKp = 21.0f;
    constexpr float kRollKd = 0.8f;
    constexpr float kPitchKd = 1.3f;
    constexpr float kTargetRoll = 0.05f;
    constexpr float kTargetPitch = -0.04f;
    constexpr float kTargetRollSpeed = 0.3f;
    constexpr float kTargetPitchSpeed = -0.2f;
    const auto ranges = CreateMotorRanges();

    for (const auto mode : {PVTControlMode::HostKpMotorKd}) {
        auto fixture = CreatePairedDoubleCoupleFixture(
            "pvt_new_coupled_mode_" + std::to_string(static_cast<int>(mode)), ranges);
        const auto actual_positions = CalculateExpectedMotorPositions(kActualRoll, kActualPitch);
        EmitMotor1Feedback(fixture, actual_positions.motor1, kMotor1Speed);
        EmitMotor2Feedback(fixture, actual_positions.motor2, kMotor2Speed);

        fixture.master->PVTControl(kRollKp, kRollKd, kTargetRoll, kTargetRollSpeed, 0.0f, mode);
        fixture.slave->PVTControl(kPitchKp, kPitchKd, kTargetPitch, kTargetPitchSpeed, 0.0f, mode);

        const auto& messages = fixture.adapter->GetCommandRecords();
        ASSERT_EQ(messages.size(), 2u);
        const auto* motor1_message = FindSentMessage(messages, 1);
        const auto* motor2_message = FindSentMessage(messages, 2);
        ASSERT_NE(motor1_message, nullptr);
        ASSERT_NE(motor2_message, nullptr);
        ASSERT_EQ(motor1_message->kind, FakeCommandKind::PVTControl);
        ASSERT_EQ(motor2_message->kind, FakeCommandKind::PVTControl);

        couple_joint::MechanismSolver solver(CreateValidCalibrationInput());
        ASSERT_TRUE(solver.SolveInverse(kActualRoll, kActualPitch).status ==
                        couple_joint::SolveStatus::Success ||
                    solver.GetTransmission().near_singular);
        const auto actual_velocity = solver.SolveForwardVelocity(kMotor1Speed, kMotor2Speed);
        const auto& jacobian = actual_velocity.transmission.velocity_jacobian;
        const float motor1_kd =
            std::max(0.1f, static_cast<float>(jacobian.m00 * jacobian.m00 * kRollKd +
                                              jacobian.m10 * jacobian.m10 * kPitchKd));
        const float motor2_kd =
            std::max(0.1f, static_cast<float>(jacobian.m01 * jacobian.m01 * kRollKd +
                                              jacobian.m11 * jacobian.m11 * kPitchKd));
        const auto kp_torque = solver.SolveInverseTorque(kRollKp * (kTargetRoll - kActualRoll),
                                                         kPitchKp * (kTargetPitch - kActualPitch));
        const auto kd_torque =
            solver.SolveInverseTorque(kRollKd * (kTargetRollSpeed - actual_velocity.roll),
                                      kPitchKd * (kTargetPitchSpeed - actual_velocity.pitch));
        const float expected_speed1 =
            kMotor1Speed + static_cast<float>(kd_torque.theta1 / motor1_kd);
        const float expected_speed2 =
            kMotor2Speed + static_cast<float>(kd_torque.theta2 / motor2_kd);
        EXPECT_NEAR(DecodePvtSpeed(*motor1_message, ranges), expected_speed1, 0.02f);
        EXPECT_NEAR(DecodePvtSpeed(*motor2_message, ranges), expected_speed2, 0.02f);
        EXPECT_FLOAT_EQ(DecodePvtKp(*motor1_message), 0.0f);
        EXPECT_FLOAT_EQ(DecodePvtKp(*motor2_message), 0.0f);
        EXPECT_NEAR(DecodePvtKd(*motor1_message), motor1_kd, 0.01f);
        EXPECT_NEAR(DecodePvtKd(*motor2_message), motor2_kd, 0.01f);
        EXPECT_NEAR(DecodePvtTorque(*motor1_message, ranges), kp_torque.theta1, 0.02f);
        EXPECT_NEAR(DecodePvtTorque(*motor2_message, ranges), kp_torque.theta2, 0.02f);
        EXPECT_NEAR(DecodePvtPosition(*motor1_message, ranges), actual_positions.motor1, 0.01f);
        EXPECT_NEAR(DecodePvtPosition(*motor2_message, ranges), actual_positions.motor2, 0.01f);
    }
}

TEST(DoubleCoupleJointPVTControlTest, HostKpMotorKdRejectsCompensatedSpeedOutsideProtocolRange) {
    auto ranges = CreateMotorRanges();
    ranges.speed = {-0.1f, 0.1f};
    auto fixture = CreatePairedDoubleCoupleFixture("coupled_damping_speed_limit", ranges);
    const auto actual = CalculateExpectedMotorPositions(0, 0);
    EmitDoubleCoupleFeedback(fixture, actual);
    fixture.master->PVTControl(0, 1, 0, 1, 0, PVTControlMode::HostKpMotorKd);
    EXPECT_THROW(fixture.slave->PVTControl(0, 1, 0, 0, 0, PVTControlMode::HostKpMotorKd),
                 JointException);
    ExpectPositionHoldMessages(fixture.adapter->GetCommandRecords(), actual, ranges);
}

TEST(DoubleCoupleJointPVTControlTest, HostTorqueComputesBothFeedbackTermsAndSendsTorqueFrames) {
    for (bool reverse : {false, true}) {
        const auto ranges = CreateMotorRanges();
        auto fixture = CreatePairedDoubleCoupleFixture("host_full_pd_" + std::to_string(reverse),
                                                       ranges, reverse, reverse);
        const auto positions = CalculateExpectedMotorPositions(0.01, -0.02);
        EmitMotor1Feedback(fixture, positions.motor1, 0.3f);
        EmitMotor2Feedback(fixture, positions.motor2, -0.2f);
        const auto status1 = fixture.motor1->GetStatus(0).value();
        const auto status2 = fixture.motor2->GetStatus(0).value();
        couple_joint::MechanismSolver solver(CreateValidCalibrationInput());
        const auto params = solver.GetParameters();
        solver.SolveForward(status1.position + params.initial_theta1,
                            status2.position + params.initial_theta2);
        const auto actual = solver.GetState();
        const auto velocity = solver.SolveForwardVelocity(status1.speed, status2.speed);
        const float sign = reverse ? -1.0f : 1.0f;
        const auto expected = solver.SolveInverseTorque(
            std::clamp(
                10 * (sign * 0.05 - actual.roll) + 0.5 * (sign * 0.2 - velocity.roll) + sign * 0.3,
                -3.0, 3.0),
            std::clamp(20 * (sign * -0.04 - actual.pitch) + 0.75 * (sign * -0.1 - velocity.pitch) +
                           sign * -0.2,
                       -3.0, 3.0));
        fixture.adapter->ClearCommandRecords();
        fixture.master->PVTControl(10, 0.5, 0.05, 0.2, 0.3, PVTControlMode::HostTorque);
        EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());
        fixture.slave->PVTControl(20, 0.75, -0.04, -0.1, -0.2, PVTControlMode::HostTorque);
        ExpectTorqueCommands(fixture.adapter->GetCommandRecords(), expected.theta1,
                             expected.theta2);
    }
}

TEST(DoubleCoupleJointPVTControlTest, HostTorqueRejectsTotalPdTorqueOverflow) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("host_total_torque_limit", ranges);
    const auto actual = CalculateExpectedMotorPositions(0, 0);
    EmitDoubleCoupleFeedback(fixture, actual);
    fixture.master->PVTControl(0, 100, 0, 2, 0, PVTControlMode::HostTorque);
    EXPECT_THROW(fixture.slave->PVTControl(0, 100, 0, 0, 0, PVTControlMode::HostTorque),
                 JointException);
    ExpectPositionHoldMessages(fixture.adapter->GetCommandRecords(), actual, ranges);
}

TEST(DoubleCoupleJointPVTControlTest, PairedModesMustMatchForHostControl) {
    const auto modes = {PVTControlMode::MotorPVT, PVTControlMode::HostTorque,
                        PVTControlMode::HostKpMotorKd};
    for (auto first : modes) {
        for (auto second : modes) {
            auto fixture = CreatePairedDoubleCoupleFixture(
                "paired_modes_" + std::to_string(int(first)) + std::to_string(int(second)),
                CreateMotorRanges());
            EmitDoubleCoupleFeedback(fixture, CalculateExpectedMotorPositions(0, 0));
            fixture.master->PVTControl(10, 0.5, 0.01, 0, 0, first);
            fixture.slave->PVTControl(10, 0.5, -0.01, 0, 0, second);
            const auto records = fixture.adapter->GetCommandRecords();
            ASSERT_EQ(records.size(), 2u);
            for (const auto& record : records) {
                if (first == second && first == PVTControlMode::HostTorque) {
                    EXPECT_EQ(record.kind, FakeCommandKind::TorControl);
                } else {
                    ASSERT_EQ(record.kind, FakeCommandKind::PVTControl);
                    if (first == second && first == PVTControlMode::HostKpMotorKd)
                        EXPECT_FLOAT_EQ(DecodePvtKp(record), 0);
                    else
                        EXPECT_GT(DecodePvtKp(record), 0);
                }
            }
        }
    }
}

TEST(DoubleCoupleJointPVTControlTest, HostTorqueWithoutFeedbackFallsBackToMotorPvt) {
    auto fixture = CreatePairedDoubleCoupleFixture("host_no_feedback", CreateMotorRanges());
    fixture.master->PVTControl(10, 1, 0.01, 0, 0, PVTControlMode::HostTorque);
    fixture.slave->PVTControl(10, 1, -0.01, 0, 0, PVTControlMode::HostTorque);
    const auto records = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(records.size(), 2u);
    for (const auto& record : records) {
        ASSERT_EQ(record.kind, FakeCommandKind::PVTControl);
        EXPECT_GT(DecodePvtKp(record), 0);
    }
}

TEST(DoubleCoupleJointPVTControlTest, HostTorquePreservesLimitAndSingularityProtection) {
    const auto ranges = CreateMotorRanges();
    {
        auto fixture =
            CreatePairedDoubleCoupleFixture("host_limit", ranges, false, false, std::nullopt,
                                            std::nullopt, Range<float>{-0.1f, 0.1f});
        const auto actual = CalculateExpectedMotorPositions(0.09, 0);
        EmitDoubleCoupleFeedback(fixture, actual);
        fixture.master->PVTControl(10, 1, 0.2, 0, 0, PVTControlMode::HostTorque);
        fixture.slave->PVTControl(10, 1, 0, 0, 0, PVTControlMode::HostTorque);
        ExpectPositionHoldMessages(fixture.adapter->GetCommandRecords(), actual, ranges);
    }
    {
        auto fixture = CreatePairedDoubleCoupleFixture("host_singularity", ranges);
        const auto actual = CalculateExpectedMotorPositions(-0.5, -0.15);
        EmitDoubleCoupleFeedback(fixture, actual);
        fixture.master->PVTControl(10, 1, -0.5, 0, 0, PVTControlMode::HostTorque);
        fixture.slave->PVTControl(10, 1, -0.15, -0.2, 0, PVTControlMode::HostTorque);
        ExpectPositionHoldMessages(fixture.adapter->GetCommandRecords(), actual, ranges);
    }
}

TEST(DoubleCoupleJointPVTControlTest, InvalidModeCancelsPendingBatch) {
    for (int invalid : {-1, 3, 4, 5, 6}) {
        const auto ranges = CreateMotorRanges();
        auto fixture = CreatePairedDoubleCoupleFixture("invalid_pvt_mode", ranges);
        const auto actual = CalculateExpectedMotorPositions(0, 0);
        EmitDoubleCoupleFeedback(fixture, actual);
        fixture.master->PVTControl(10, 1, 0.05, 0, 0, PVTControlMode::HostTorque);
        EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());
        EXPECT_THROW(
            fixture.slave->PVTControl(10, 1, 0, 0, 0, static_cast<PVTControlMode>(invalid)),
            std::invalid_argument);
        ExpectPositionHoldMessages(fixture.adapter->GetCommandRecords(), actual, ranges);
        fixture.adapter->ClearCommandRecords();
        fixture.slave->PVTControl(10, 1, 0, 0, 0, PVTControlMode::HostTorque);
        EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());
        fixture.master->PVTControl(10, 1, 0, 0, 0, PVTControlMode::HostTorque);
        ExpectTorqueCommands(fixture.adapter->GetCommandRecords(), 0, 0);
    }
}

TEST(DoubleCoupleJointProtectionTest, HoldsInsideBothMotorLimitsWithTwoDegreeSlack) {
    const auto ranges = CreateMotorRanges();
    const Range<float> limit{-0.2f, 0.2f};
    auto fixture =
        CreatePairedDoubleCoupleFixture("hold_motor_slack", ranges, false, false, limit, limit);
    EmitDoubleCoupleFeedback(fixture, {0.3f, -0.3f});
    EXPECT_THROW(fixture.master->PVTControl(std::numeric_limits<float>::quiet_NaN(), 0, 0, 0, 0),
                 std::invalid_argument);
    ExpectPositionHoldMessages(fixture.adapter->GetCommandRecords(),
                               {limit.max - kLimitSlack, limit.min + kLimitSlack}, ranges);
}

TEST(DoubleCoupleJointProtectionTest, RawFeedbackOutsidePiClampsWithoutChangingTurns) {
    const auto ranges = CreateMotorRanges();
    const Range<float> limit{-3.1f, 3.1f};
    auto fixture =
        CreatePairedDoubleCoupleFixture("hold_raw_angle", ranges, false, false, limit, limit);
    EmitDoubleCoupleFeedback(fixture, {3.2f, -3.2f});
    EXPECT_THROW(fixture.master->PosControl(std::numeric_limits<float>::quiet_NaN()),
                 std::invalid_argument);
    ExpectPositionHoldMessages(fixture.adapter->GetCommandRecords(),
                               {limit.max - kLimitSlack, limit.min + kLimitSlack}, ranges);
}

TEST(DoubleCoupleJointProtectionTest, RelativeMotorAnglesDoNotJumpAcrossSolverPiBoundary) {
    for (double sign : {1.0, -1.0}) {
        couple_joint::CalibrationInput c;
        c.left_joint_center = {-20, 0, 215};
        c.right_joint_center = {-20, 0, 152};
        c.left_planar_endpoint = {-20, sign * 15.29, 219.7};
        c.right_planar_endpoint = {-20, -sign * 15.29, 156.71};
        c.roll_center = {0, 0, -15};
        c.pitch_center = {0, 0, 0};
        c.left_spatial_endpoint = {-19.45, sign * 13.5, 4.67};
        c.right_spatial_endpoint = {-19.45, -sign * 13.5, 4.67};
        c.rotate_direction = {1, 0, 0};
        c.use_rotate_direction = true;
        c.rotation_order = couple_joint::RotationOrder::PitchThenRoll;
        const auto ranges = CreateMotorRanges();
        auto fixture =
            CreatePairedDoubleCoupleFixture("relative_angle", ranges, false, false, std::nullopt,
                                            std::nullopt, std::nullopt, std::nullopt, c);
        const ExpectedMotorPositions actual{static_cast<float>(sign * 1.34107685),
                                            static_cast<float>(-sign * 0.713931084)};
        EmitDoubleCoupleFeedback(fixture, actual);
        const auto roll = fixture.master->GetStatus(false, 0);
        const auto pitch = fixture.slave->GetStatus(false, 0);
        ASSERT_TRUE(roll.has_value());
        ASSERT_TRUE(pitch.has_value());
        // 不可执行目标触发批次起点保持，起点必须保留原电机相对角而非 ±2π 跳圈。
        fixture.master->PosControl(3.0f, 1.0f, 1.0f);
        EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());
        fixture.master->PosControl(3.0f, 1.0f, 1.0f);
        const auto& hold = fixture.adapter->GetCommandRecords();
        ASSERT_EQ(hold.size(), 2u);
        for (int id : {1, 2}) {
            const auto* message = FindSentMessage(hold, id);
            ASSERT_NE(message, nullptr);
            ASSERT_EQ(message->kind, FakeCommandKind::PosControl);
            EXPECT_NEAR(DecodePosPosition(*message), id == 1 ? actual.motor1 : actual.motor2,
                        0.002f);
            EXPECT_LE(std::abs(DecodePosPosition(*message)), 3.141593f);
        }
    }
}

#if ENCOS_ENABLE_SPDLOG

TEST(JointProtectionLogTest, ManagerLoggerReceivesWarningsFromEveryJointTypeAndControlMode) {
    std::ostringstream output;
    const std::string logger_name = "joint_protection_test";
    auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(output);
    auto backend = std::make_shared<spdlog::logger>(logger_name, sink);
    backend->set_pattern("%l %v");
    spdlog::register_logger(backend);
    auto logger = CreateLogger(logger_name);
    TestableJointManager manager;
    manager.SetLogger(logger);
    auto adapter = CreateFakeAdapterHarness("warning_all_joints");
    adapter->SetReplyMode(FakeReplyMode::Automatic);
    manager.RegisterAdapter("adapter1", adapter->Adapter());
    const auto ranges = CreateMotorRanges();
    RJoint::Config rconfig{MotorParams{"adapter1", 0, 10, MotorInitParams{ranges}},
                           Range<float>{-1, 1}, 2, 3, false};
    auto rjoint = manager.AddJoint<RJoint>("test_rotary", rconfig);
    ContinuousJoint::Config continuous{};
    continuous.motorParams = MotorParams{"adapter1", 0, 11, MotorInitParams{ranges}};
    continuous.maxSpd = 2;
    continuous.maxTor = 3;
    auto cjoint = manager.AddJoint<ContinuousJoint>("test_continuous", continuous);
    adapter->SetReplyMode(FakeReplyMode::Automatic);
    auto slave = manager.AddJoint<DoubleCoupleJoint>(
        "test_pitch",
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Pitch, ranges));
    auto master = manager.AddJoint<DoubleCoupleJoint>(
        "test_roll", CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Roll, ranges));
    ASSERT_EQ(manager.GetLogger(), logger);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (const auto& joint : {rjoint, cjoint, master, slave}) {
        const std::vector<std::function<void()>> calls{
            [&] {
                joint->PVTControl(nan, 0, 0, 0, 0);
            },
            [&] {
                joint->PosControl(nan);
            },
            [&] {
                joint->SpdControl(nan);
            },
            [&] {
                joint->CurControl(nan);
            },
            [&] {
                joint->TorControl(nan);
            },
        };
        for (const auto& call : calls) {
            const auto before = output.str().size();
            EXPECT_ANY_THROW(call());
            const auto message = output.str().substr(before);
            EXPECT_NE(message.find("warning"), std::string::npos);
            EXPECT_NE(message.find(joint->GetProtectionName()), std::string::npos);
            EXPECT_NE(message.find("mode="), std::string::npos);
            EXPECT_NE(message.find("reason="), std::string::npos);
        }
    }
    // 幅值限幅是电平型保护：每个错误点只在进入保护时输出一条告警，重复调用不再复述。
    // 闭合链动态控制按合帧下发，需要另一轴各补一次调用才会联合映射并产生限幅告警
    const std::vector<std::pair<BaseJointPtr, BaseJointPtr>> clamped_calls{{rjoint, nullptr},
                                                                           {cjoint, nullptr}};
    for (const auto& [joint, companion] : clamped_calls) {
        const std::vector<std::pair<std::function<void()>, std::function<void()>>> calls{
            {[&] {
                 joint->SpdControl(1000);
             },
             [&] {
                 if (companion) {
                     companion->SpdControl(0.0f);
                 }
             }},
            {[&] {
                 joint->CurControl(1000);
             },
             [&] {
                 if (companion) {
                     companion->CurControl(0.0f);
                 }
             }},
            {[&] {
                 joint->TorControl(1000);
             },
             [&] {
                 if (companion) {
                     companion->TorControl(0.0f);
                 }
             }},
        };
        for (const auto& [call, flush] : calls) {
            const auto before = output.str().size();
            call();
            flush();
            EXPECT_NE(output.str().substr(before).find("state=enter"), std::string::npos);

            // 同一错误点已置位，重复同样的超限命令不再输出
            const auto repeated = output.str().size();
            call();
            flush();
            EXPECT_EQ(output.str().size(), repeated);
        }
    }

    // 耦合关节越界输入明确拒绝，并报告调用模式与原因。
    for (const auto& joint : {master, slave}) {
        const auto before = output.str().size();
        EXPECT_THROW(joint->SpdControl(1000), std::invalid_argument);
        EXPECT_NE(output.str().substr(before).find("invalid dynamic input"), std::string::npos);
    }
    spdlog::drop(logger_name);
}

TEST(JointProtectionLogTest, ReportsOnlyProtectionEntryAndExitForRepeatedCondition) {
    std::ostringstream output;
    const std::string logger_name = "joint_protection_edge_test";
    auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(output);
    auto backend = std::make_shared<spdlog::logger>(logger_name, sink);
    backend->set_pattern("%l %v");
    spdlog::register_logger(backend);
    auto logger = CreateLogger(logger_name);
    TestableJointManager manager;
    manager.SetLogger(logger);
    auto adapter = CreateFakeAdapterHarness("protection_edge");
    adapter->SetReplyMode(FakeReplyMode::Automatic);
    manager.RegisterAdapter("adapter1", adapter->Adapter());
    const auto ranges = CreateMotorRanges();
    RJoint::Config config{MotorParams{"adapter1", 0, 10, MotorInitParams{ranges}},
                          Range<float>{-1, 1}, 2, 3, false};
    auto joint = manager.AddJoint<RJoint>("edge_rotary", config);

    const auto count = [&output](const std::string& needle) {
        const std::string text = output.str();
        std::size_t total = 0;
        for (auto pos = text.find(needle); pos != std::string::npos;
             pos = text.find(needle, pos + needle.size())) {
            ++total;
        }
        return total;
    };

    // 连续 3 个周期越限：只输出一条进入保护告警
    for (int i = 0; i < 3; ++i) {
        joint->PosControl(5.0f);
    }
    EXPECT_EQ(count("state=enter"), 1u);
    EXPECT_EQ(count("state=exit"), 0u);

    // 回到限位内：输出一条退出保护日志
    joint->PosControl(0.5f);
    EXPECT_EQ(count("state=enter"), 1u);
    EXPECT_EQ(count("state=exit"), 1u);

    // 再次越限：重新输出一条进入保护告警
    joint->PosControl(5.0f);
    EXPECT_EQ(count("state=enter"), 2u);
    EXPECT_EQ(count("state=exit"), 1u);

    spdlog::drop(logger_name);
}

TEST(JointProtectionLogTest, ReportsLimitHoldOnlyOnEntryAndExit) {
    std::ostringstream output;
    const std::string logger_name = "joint_protection_hold_test";
    auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(output);
    auto backend = std::make_shared<spdlog::logger>(logger_name, sink);
    backend->set_pattern("%l %v");
    spdlog::register_logger(backend);
    auto logger = CreateLogger(logger_name);
    auto fixture = CreateRJointFixture("protection_hold_edge", CreateMotorRanges());
    fixture.joint->SetProtectionLogContext(logger, "hold_rotary");

    const auto count = [&output](const std::string& needle) {
        const std::string text = output.str();
        std::size_t total = 0;
        for (auto pos = text.find(needle); pos != std::string::npos;
             pos = text.find(needle, pos + needle.size())) {
            ++total;
        }
        return total;
    };
    const std::string hold_entry = "reason=outward position limit; position hold, state=enter";
    const std::string hold_exit = "reason=outward position limit; position hold, state=exit";

    // 反馈停在限位外侧：连续 3 个周期都在限位保持，只输出一条进入保护告警
    EmitFakeFeedback(*fixture.adapter, fixture.motor, 1.1f, 0.0f, 0.0f);
    for (int i = 0; i < 3; ++i) {
        fixture.joint->SpdControl(0.0f);
    }
    EXPECT_EQ(count(hold_entry), 1u);
    EXPECT_EQ(count(hold_exit), 0u);

    // 反馈回到限位内：恢复常规下发，输出一条退出保护日志
    EmitFakeFeedback(*fixture.adapter, fixture.motor, 0.0f, 0.0f, 0.0f);
    fixture.joint->SpdControl(0.0f);
    EXPECT_EQ(count(hold_entry), 1u);
    EXPECT_EQ(count(hold_exit), 1u);

    spdlog::drop(logger_name);
}

TEST(JointProtectionLogTest, ReportsClosedChainHoldOnlyOnEntryAndExit) {
    std::ostringstream output;
    const std::string logger_name = "joint_protection_chain_hold_test";
    auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(output);
    auto backend = std::make_shared<spdlog::logger>(logger_name, sink);
    backend->set_pattern("%l %v");
    spdlog::register_logger(backend);
    auto logger = CreateLogger(logger_name);
    auto fixture =
        CreatePairedDoubleCoupleFixture("protection_chain_hold", CreateMotorRanges(), false, false,
                                        std::nullopt, std::nullopt, Range<float>{-0.1f, 0.1f});
    fixture.master->SetProtectionLogContext(logger, "chain_hold");
    fixture.slave->SetProtectionLogContext(logger, "chain_hold");

    const auto count = [&output](const std::string& needle) {
        const std::string text = output.str();
        std::size_t total = 0;
        for (auto pos = text.find(needle); pos != std::string::npos;
             pos = text.find(needle, pos + needle.size())) {
            ++total;
        }
        return total;
    };
    const std::string hold_entry = "reason=position hold, state=enter";
    const std::string hold_exit = "reason=position hold, state=exit";

    // 目标继续向外而实际位姿已在限位保护区内：连续 3 帧都下发位置保持，
    // 只输出一条进入保护告警
    EmitDoubleCoupleFeedback(fixture, CalculateExpectedMotorPositions(0.09, 0.0));
    for (int i = 0; i < 3; ++i) {
        fixture.master->PVTControl(10.0f, 1.0f, 0.2f, 0.0f, 0.0f);
        fixture.slave->PVTControl(10.0f, 1.0f, 0.0f, 0.0f, 0.0f);
    }
    EXPECT_EQ(count(hold_entry), 1u);
    EXPECT_EQ(count(hold_exit), 0u);

    // 目标转回限位内并完成一次成功下发：输出一条退出保护日志
    fixture.master->PVTControl(10.0f, 1.0f, 0.0f, 0.0f, 0.0f);
    fixture.slave->PVTControl(10.0f, 1.0f, 0.0f, 0.0f, 0.0f);
    EXPECT_EQ(count(hold_entry), 1u);
    EXPECT_EQ(count(hold_exit), 1u);

    spdlog::drop(logger_name);
}
#endif

TEST(DoubleCoupleJointPVTControlTest, MotorDampingFloorPreservesExplicitZero) {
    for (auto mode : {PVTControlMode::MotorPVT, PVTControlMode::HostKpMotorKd}) {
        for (float kd : {0.0f, 0.001f}) {
            const auto ranges = CreateMotorRanges();
            auto fixture = CreatePairedDoubleCoupleFixture("kd_floor", ranges);
            EmitDoubleCoupleFeedback(fixture, CalculateExpectedMotorPositions(0, 0));
            fixture.master->PVTControl(0, kd, 0, 0, 0, mode);
            fixture.slave->PVTControl(0, kd, 0, 0, 0, mode);
            const auto& messages = fixture.adapter->GetCommandRecords();
            ASSERT_EQ(messages.size(), 2u);
            for (const auto& message : messages) {
                ASSERT_EQ(message.kind, FakeCommandKind::PVTControl);
                EXPECT_FLOAT_EQ(DecodePvtKp(message), 0.0f);
                EXPECT_NEAR(DecodePvtKd(message), kd == 0 ? 0.0f : 0.1f, 0.01f);
                EXPECT_NEAR(DecodePvtTorque(message, ranges), 0.0f, 0.02f);
            }
        }
    }
}

TEST(DoubleCoupleJointPVTControlTest, TorqueModeFallsBackToMotorPvtWithoutCachedStatus) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("pvt_torque_fallback_adapter", ranges);

    BaseJoint& master = *fixture.master;
    BaseJoint& slave = *fixture.slave;
    master.PVTControl(12.0f, 0.8f, 0.05f, 0.0f, 0.0f, PVTControlMode::HostKpMotorKd);
    slave.PVTControl(21.0f, 1.3f, -0.04f, 0.0f, 0.0f, PVTControlMode::HostKpMotorKd);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    EXPECT_EQ(messages[0].kind, FakeCommandKind::PVTControl);
    EXPECT_EQ(messages[1].kind, FakeCommandKind::PVTControl);
    const auto expected_kp = CalculateExpectedMotorGains(0.0, 0.0, 12.0f, 21.0f);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    EXPECT_NEAR(DecodePvtKp(*motor1_message), expected_kp.motor1, 0.15f);
    EXPECT_NEAR(DecodePvtKp(*motor2_message), expected_kp.motor2, 0.15f);
}

TEST(DoubleCoupleJointPVTControlTest, MixedTorqueModesUseMotorPvtInEitherCallOrder) {
    const auto ranges = CreateMotorRanges();
    const auto feedback_positions = CalculateExpectedMotorPositions(0.0, 0.0);

    auto normal_then_torque =
        CreatePairedDoubleCoupleFixture("pvt_mixed_mode_normal_first", ranges);
    EmitDoubleCoupleFeedback(normal_then_torque, feedback_positions);
    normal_then_torque.master->PVTControl(10.0f, 0.5f, 0.05f, 0.0f, 0.0f, PVTControlMode::MotorPVT);
    normal_then_torque.slave->PVTControl(20.0f, 0.75f, -0.04f, 0.0f, 0.0f,
                                         PVTControlMode::HostKpMotorKd);
    ASSERT_EQ(normal_then_torque.adapter->GetCommandRecords().size(), 2u);
    EXPECT_EQ(normal_then_torque.adapter->GetCommandRecords()[0].kind, FakeCommandKind::PVTControl);
    EXPECT_EQ(normal_then_torque.adapter->GetCommandRecords()[1].kind, FakeCommandKind::PVTControl);

    auto torque_then_normal =
        CreatePairedDoubleCoupleFixture("pvt_mixed_mode_torque_first", ranges);
    EmitDoubleCoupleFeedback(torque_then_normal, feedback_positions);
    torque_then_normal.master->PVTControl(10.0f, 0.5f, 0.05f, 0.0f, 0.0f,
                                          PVTControlMode::HostKpMotorKd);
    torque_then_normal.slave->PVTControl(20.0f, 0.75f, -0.04f, 0.0f, 0.0f,
                                         PVTControlMode::MotorPVT);
    ASSERT_EQ(torque_then_normal.adapter->GetCommandRecords().size(), 2u);
    EXPECT_EQ(torque_then_normal.adapter->GetCommandRecords()[0].kind, FakeCommandKind::PVTControl);
    EXPECT_EQ(torque_then_normal.adapter->GetCommandRecords()[1].kind, FakeCommandKind::PVTControl);
}

TEST(DoubleCoupleJointPositionControlTest, BatchesTwoAxisPositionControlsIntoFinalMotorCommand) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("pos_batched_axes_adapter", ranges);

    fixture.master->PosControl(0.04f, 1.0f, 0.5f);
    EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());

    fixture.slave->PosControl(-0.03f, 1.0f, 0.5f, 1);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    const auto expected = CalculateExpectedMotorPositions(0.04, -0.03);
    EXPECT_NEAR(DecodePosPosition(*motor1_message), expected.motor1, 0.002f);
    EXPECT_NEAR(DecodePosPosition(*motor2_message), expected.motor2, 0.002f);
}

TEST(DoubleCoupleJointPVTControlTest, PVTAcceptsNearSingularTargetWhileActualPoseIsSafe) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("pvt_near_singular_guard", ranges);
    constexpr double kCurrentRoll = -0.5;
    constexpr double kNearSingularPitch = -0.15;
    ExpectNearSingularPose(kCurrentRoll, kNearSingularPitch);

    const auto current_positions = CalculateExpectedMotorPositions(kCurrentRoll, 0.0);
    EmitDoubleCoupleFeedback(fixture, current_positions);
    fixture.adapter->ClearCommandRecords();
    fixture.master->PVTControl(10.0f, 1.0f, static_cast<float>(kCurrentRoll), 0.0f, 0.0f);
    fixture.slave->PVTControl(10.0f, 1.0f, static_cast<float>(kNearSingularPitch), 0.2f, 0.1f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto expected = CalculateExpectedMotorPositions(kCurrentRoll, kNearSingularPitch);
    EXPECT_NEAR(DecodePvtPosition(*FindSentMessage(messages, 1), ranges), expected.motor1, 0.002f);
    EXPECT_NEAR(DecodePvtPosition(*FindSentMessage(messages, 2), ranges), expected.motor2, 0.002f);
}

TEST(DoubleCoupleJointPVTControlTest, ActualVirtualJointLimitStopsOutwardNormalPvt) {
    const auto ranges = CreateMotorRanges();
    auto fixture =
        CreatePairedDoubleCoupleFixture("pvt_actual_virtual_limit", ranges, false, false,
                                        std::nullopt, std::nullopt, Range<float>{-0.1f, 0.1f});
    const auto actual_positions = CalculateExpectedMotorPositions(0.09, 0.0);
    EmitDoubleCoupleFeedback(fixture, actual_positions);
    fixture.adapter->ClearCommandRecords();

    fixture.master->PVTControl(10.0f, 1.0f, 0.2f, 0.0f, 0.0f);
    fixture.slave->PVTControl(10.0f, 1.0f, 0.0f, 0.0f, 0.0f);

    ExpectPositionHoldMessages(fixture.adapter->GetCommandRecords(), actual_positions, ranges);
}

TEST(DoubleCoupleJointPVTControlTest, ReverseVirtualJointUsesJointCoordinateLimitZone) {
    const auto ranges = CreateMotorRanges();
    auto fixture =
        CreatePairedDoubleCoupleFixture("pvt_reverse_virtual_limit", ranges, true, false,
                                        std::nullopt, std::nullopt, Range<float>{-0.1f, 0.05f});
    const auto actual_positions = CalculateExpectedMotorPositions(-0.045, 0.0);
    EmitDoubleCoupleFeedback(fixture, actual_positions);
    fixture.adapter->ClearCommandRecords();

    fixture.master->PVTControl(10.0f, 1.0f, 0.08f, 0.0f, 0.0f);
    fixture.slave->PVTControl(10.0f, 1.0f, 0.0f, 0.0f, 0.0f);

    ExpectPositionHoldMessages(fixture.adapter->GetCommandRecords(), actual_positions, ranges);
}

TEST(DoubleCoupleJointPVTControlTest, VirtualJointTargetIsNotClampedWhileActualPoseIsSafe) {
    const auto ranges = CreateMotorRanges();
    auto fixture =
        CreatePairedDoubleCoupleFixture("pvt_unclamped_virtual_target", ranges, false, false,
                                        std::nullopt, std::nullopt, Range<float>{-0.1f, 0.1f});
    EmitDoubleCoupleFeedback(fixture, CalculateExpectedMotorPositions(0.0, 0.0));
    fixture.adapter->ClearCommandRecords();

    fixture.master->PVTControl(10.0f, 1.0f, 0.2f, 0.0f, 0.0f);
    fixture.slave->PVTControl(10.0f, 1.0f, 0.0f, 0.0f, 0.0f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto expected = CalculateExpectedMotorPositions(0.2, 0.0);
    EXPECT_NEAR(DecodePvtPosition(*FindSentMessage(messages, 1), ranges), expected.motor1, 0.002f);
    EXPECT_NEAR(DecodePvtPosition(*FindSentMessage(messages, 2), ranges), expected.motor2, 0.002f);
}

TEST(DoubleCoupleJointPVTControlTest, HostTorqueUsesUnclampedTargetWhileActualPoseIsSafe) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("pvt_unclamped_virtual_target_torque_mode",
                                                   ranges, false, false, std::nullopt, std::nullopt,
                                                   Range<float>{-0.1f, 0.1f});
    EmitDoubleCoupleFeedback(fixture, CalculateExpectedMotorPositions(0.0, 0.0));
    fixture.adapter->ClearCommandRecords();

    fixture.master->PVTControl(10.0f, 0.0f, 0.2f, 0.0f, 0.0f, PVTControlMode::HostKpMotorKd);
    fixture.slave->PVTControl(10.0f, 0.0f, 0.0f, 0.0f, 0.0f, PVTControlMode::HostKpMotorKd);

    const auto expected = CalculateExpectedCombinedMotorTorques(0.0, 0.0, 2.0, 0.0, ranges);
    ExpectPvtTorqueCommands(fixture.adapter->GetCommandRecords(), expected.motor1, expected.motor2,
                            ranges);
}

TEST(DoubleCoupleJointPVTControlTest, ActualVirtualJointLimitHoldsOutwardHostTorquePvt) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("pvt_actual_virtual_limit_torque_mode", ranges,
                                                   false, false, std::nullopt, std::nullopt,
                                                   Range<float>{-0.1f, 0.1f});
    const auto actual_positions = CalculateExpectedMotorPositions(0.09, 0.0);
    EmitDoubleCoupleFeedback(fixture, actual_positions);
    fixture.adapter->ClearCommandRecords();

    fixture.master->PVTControl(10.0f, 1.0f, 0.2f, 0.0f, 0.0f, PVTControlMode::HostKpMotorKd);
    fixture.slave->PVTControl(10.0f, 1.0f, 0.0f, 0.0f, 0.0f, PVTControlMode::HostKpMotorKd);

    ExpectPositionHoldMessages(fixture.adapter->GetCommandRecords(), actual_positions, ranges);
}

TEST(DoubleCoupleJointPVTControlTest, ProtectedHostTorqueTargetDoesNotSeedNextBatch) {
    const auto ranges = CreateMotorRanges();
    auto fixture =
        CreatePairedDoubleCoupleFixture("pvt_protected_host_target_cache", ranges, false, false,
                                        std::nullopt, std::nullopt, Range<float>{-0.1f, 0.1f});
    constexpr double kActualRoll = 0.09;
    const auto actual_positions = CalculateExpectedMotorPositions(kActualRoll, 0.0);
    EmitDoubleCoupleFeedback(fixture, actual_positions);
    fixture.adapter->ClearCommandRecords();

    fixture.master->PVTControl(10.0f, 0.0f, 0.2f, 0.0f, 0.0f, PVTControlMode::HostKpMotorKd);
    fixture.slave->PVTControl(10.0f, 0.0f, 0.0f, 0.0f, 0.0f, PVTControlMode::HostKpMotorKd);
    ExpectPositionHoldMessages(fixture.adapter->GetCommandRecords(), actual_positions, ranges);

    fixture.adapter->ClearCommandRecords();
    fixture.slave->PVTControl(10.0f, 0.0f, 0.0f, 0.0f, 0.0f, PVTControlMode::HostKpMotorKd);
    fixture.slave->PVTControl(10.0f, 0.0f, 0.0f, 0.0f, 0.0f, PVTControlMode::HostKpMotorKd);

    const auto expected = CalculateExpectedCombinedMotorTorques(kActualRoll, 0.0, 0.0, 0.0, ranges);
    ExpectPvtTorqueCommands(fixture.adapter->GetCommandRecords(), expected.motor1, expected.motor2,
                            ranges);
}

TEST(DoubleCoupleJointPVTControlTest, VirtualJointCanMoveInwardWithinLimitZoneInBothPvtModes) {
    const auto ranges = CreateMotorRanges();
    for (const bool torque_mode : {false, true}) {
        auto fixture = CreatePairedDoubleCoupleFixture(
            torque_mode ? "pvt_virtual_partial_exit_torque_mode" : "pvt_virtual_partial_exit",
            ranges, false, false, std::nullopt, std::nullopt, Range<float>{-0.1f, 0.1f});
        EmitDoubleCoupleFeedback(fixture, CalculateExpectedMotorPositions(0.09, 0.0));
        fixture.adapter->ClearCommandRecords();

        fixture.master->PVTControl(
            10.0f, 0.0f, 0.08f, 0.0f, 0.0f,
            (torque_mode ? PVTControlMode::HostKpMotorKd : PVTControlMode::MotorPVT));
        fixture.slave->PVTControl(
            10.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            (torque_mode ? PVTControlMode::HostKpMotorKd : PVTControlMode::MotorPVT));

        const auto& messages = fixture.adapter->GetCommandRecords();
        ASSERT_EQ(messages.size(), 2u);
        EXPECT_EQ(messages.front().kind, FakeCommandKind::PVTControl);
        EXPECT_EQ(messages.back().kind, FakeCommandKind::PVTControl);
    }
}

TEST(DoubleCoupleJointPVTControlTest, ActualNearSingularPoseStopsOnlyEnteringPvtDirection) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("pvt_actual_near_singular", ranges);
    constexpr double kCurrentRoll = -0.5;
    constexpr double kCurrentPitch = -0.15;
    const auto actual_positions = CalculateExpectedMotorPositions(kCurrentRoll, kCurrentPitch);
    EmitDoubleCoupleFeedback(fixture, actual_positions);
    fixture.adapter->ClearCommandRecords();

    fixture.master->PVTControl(10.0f, 1.0f, static_cast<float>(kCurrentRoll), 0.0f, 0.0f);
    fixture.slave->PVTControl(10.0f, 1.0f, static_cast<float>(kCurrentPitch), -0.2f, 0.0f);

    ExpectPositionHoldMessages(fixture.adapter->GetCommandRecords(), actual_positions, ranges);
}

TEST(DoubleCoupleJointPVTControlTest, ActualNearSingularPoseHoldsEnteringHostTorquePvt) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("pvt_actual_near_singular_torque_mode", ranges);
    constexpr double kCurrentRoll = -0.5;
    constexpr double kCurrentPitch = -0.15;
    const auto actual_positions = CalculateExpectedMotorPositions(kCurrentRoll, kCurrentPitch);
    EmitDoubleCoupleFeedback(fixture, actual_positions);
    fixture.adapter->ClearCommandRecords();

    fixture.master->PVTControl(10.0f, 1.0f, static_cast<float>(kCurrentRoll), 0.0f, 0.0f,
                               PVTControlMode::HostKpMotorKd);
    fixture.slave->PVTControl(10.0f, 1.0f, static_cast<float>(kCurrentPitch), -0.2f, 0.0f,
                              PVTControlMode::HostKpMotorKd);

    ExpectPositionHoldMessages(fixture.adapter->GetCommandRecords(), actual_positions, ranges);
}

TEST(DoubleCoupleJointPVTControlTest, ActualNearSingularPoseAllowsExitingBothPvtModes) {
    const auto ranges = CreateMotorRanges();
    constexpr double kCurrentRoll = -0.5;
    constexpr double kCurrentPitch = -0.15;
    for (const bool torque_mode : {false, true}) {
        auto fixture = CreatePairedDoubleCoupleFixture(
            torque_mode ? "pvt_near_singular_exit_torque_mode" : "pvt_near_singular_exit", ranges);
        EmitDoubleCoupleFeedback(fixture,
                                 CalculateExpectedMotorPositions(kCurrentRoll, kCurrentPitch));
        fixture.adapter->ClearCommandRecords();

        fixture.master->PVTControl(
            4.0f, 1.0f, 0.0f, 0.0f, 0.0f,
            (torque_mode ? PVTControlMode::HostKpMotorKd : PVTControlMode::MotorPVT));
        fixture.slave->PVTControl(
            10.0f, 1.0f, -0.14f, 0.2f, 0.0f,
            (torque_mode ? PVTControlMode::HostKpMotorKd : PVTControlMode::MotorPVT));

        if (torque_mode) {
            const auto expected = CalculateExpectedCombinedMotorTorques(kCurrentRoll, kCurrentPitch,
                                                                        2.0, 0.1, ranges);
            ExpectPvtTorqueCommands(fixture.adapter->GetCommandRecords(), expected.motor1,
                                    expected.motor2, ranges);
        } else {
            ASSERT_EQ(fixture.adapter->GetCommandRecords().size(), 2u);
            EXPECT_EQ(fixture.adapter->GetCommandRecords().front().kind,
                      FakeCommandKind::PVTControl);
            EXPECT_EQ(fixture.adapter->GetCommandRecords().back().kind,
                      FakeCommandKind::PVTControl);
        }
    }
}

TEST(DoubleCoupleJointPVTControlTest, PVTRejectsTargetOutsideProtocolPositionRange) {
    const auto ranges = CreateMotorRanges(0.001f);
    auto fixture = CreatePairedDoubleCoupleFixture("pvt_clip_adapter", ranges);
    const auto user_target = CalculateExpectedMotorPositions(0.05, 0.0);
    const auto hold_positions = CalculateExpectedMotorPositions(0.0, 0.0);
    ASSERT_GT(std::max(std::abs(user_target.motor1), std::abs(user_target.motor2)),
              ranges.position.max);

    EmitDoubleCoupleFeedback(fixture, hold_positions);

    // 协议范围不足以容纳两侧各 2° 内缩，不能安全位控，回退制动。
    EXPECT_NO_THROW(fixture.master->PVTControl(10.0f, 1.0f, 0.05f, 0.4f, 0.5f));
    EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());
    EXPECT_NO_THROW(fixture.master->PVTControl(10.0f, 1.0f, 0.05f, 0.4f, 0.5f));
    ExpectStopMessagesWithZeroCurrent(fixture.adapter->GetCommandRecords(), 2u);
}

TEST(DoubleCoupleJointPVTControlTest, PVTDelegatesIndependentMotorSaturationToDriver) {
    const auto ranges = CreateMotorRanges(12.5f, 0.2f, 0.2f, 30.0f);
    auto fixture = CreatePairedDoubleCoupleFixture("pvt_speed_torque_limit_adapter", ranges);

    fixture.master->PVTControl(10.0f, 1.0f, 0.0f, 2.0f, 3.0f);
    fixture.slave->PVTControl(10.0f, 1.0f, 0.0f, 0.5f, 0.5f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);

    couple_joint::MechanismSolver solver(CreateValidCalibrationInput());
    solver.SolveInverse(0.0, 0.0);
    const auto speeds = solver.SolveInverseVelocity(2.0, 0.5);
    const auto torques = solver.SolveInverseTorque(3.0, 0.5);
    const ExpectedMotorValues expected_speeds{
        std::clamp(static_cast<float>(speeds.theta1), ranges.speed.min, ranges.speed.max),
        std::clamp(static_cast<float>(speeds.theta2), ranges.speed.min, ranges.speed.max)};
    const ExpectedMotorValues expected_torques{
        std::clamp(static_cast<float>(torques.theta1), ranges.torque.min, ranges.torque.max),
        std::clamp(static_cast<float>(torques.theta2), ranges.torque.min, ranges.torque.max)};
    const float motor1_speed = DecodePvtSpeed(*motor1_message, ranges);
    const float motor2_speed = DecodePvtSpeed(*motor2_message, ranges);
    const float motor1_torque = DecodePvtTorque(*motor1_message, ranges);
    const float motor2_torque = DecodePvtTorque(*motor2_message, ranges);
    EXPECT_NEAR(motor1_speed, expected_speeds.motor1, 0.002f);
    EXPECT_NEAR(motor2_speed, expected_speeds.motor2, 0.002f);
    EXPECT_NEAR(motor1_torque, expected_torques.motor1, 0.002f);
    EXPECT_NEAR(motor2_torque, expected_torques.motor2, 0.002f);
    EXPECT_LE(std::abs(motor1_speed), ranges.speed.max + 0.001f);
    EXPECT_LE(std::abs(motor2_speed), ranges.speed.max + 0.001f);
    EXPECT_LE(std::abs(motor1_torque), ranges.torque.max + 0.001f);
    EXPECT_LE(std::abs(motor2_torque), ranges.torque.max + 0.001f);
}

TEST(DoubleCoupleJointControlCacheTest, PosControlUpdatesCachedTargetForLaterPvt) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("pos_cache_adapter", ranges);

    fixture.master->PosControl(0.04f, 1.0f, 0.5f);
    fixture.adapter->ClearCommandRecords();
    fixture.slave->PVTControl(10.0f, 1.0f, -0.03f, 0.2f, 0.1f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    const auto expected = CalculateExpectedMotorPositions(0.04, -0.03);
    EXPECT_NEAR(DecodePvtPosition(*motor1_message, ranges), expected.motor1, 0.002f);
    EXPECT_NEAR(DecodePvtPosition(*motor2_message, ranges), expected.motor2, 0.002f);
}

TEST(DoubleCoupleJointControlCacheTest, OutOfLimitPvtTargetDoesNotPoisonOtherControlModes) {
    const auto ranges = CreateMotorRanges();
    const auto zero_positions = CalculateExpectedMotorPositions(0.0, 0.0);
    const auto pvt_target_positions = CalculateExpectedMotorPositions(0.2, 0.0);
    const Range<float> motor1_limit = pvt_target_positions.motor1 >= 0.0f
                                          ? Range<float>{-12.5f, pvt_target_positions.motor1 * 0.5f}
                                          : Range<float>{pvt_target_positions.motor1 * 0.5f, 12.5f};

    const auto create_seeded_fixture = [&](const std::string& name) {
        auto fixture =
            CreatePairedDoubleCoupleFixture(name, ranges, false, false, motor1_limit, std::nullopt);
        EmitDoubleCoupleFeedback(fixture, zero_positions);
        fixture.adapter->ClearCommandRecords();
        fixture.master->PVTControl(10.0f, 0.0f, 0.2f, 0.0f, 0.0f);
        fixture.slave->PVTControl(10.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        fixture.adapter->ClearCommandRecords();
        return fixture;
    };

    auto position_fixture = create_seeded_fixture("pvt_cache_then_position");
    EXPECT_NO_THROW(position_fixture.master->PosControl(0.0f, 1.0f, 0.5f));
    EXPECT_NO_THROW(position_fixture.slave->PosControl(0.0f, 1.0f, 0.5f));
    ASSERT_EQ(position_fixture.adapter->GetCommandRecords().size(), 2u);
    EXPECT_EQ(position_fixture.adapter->GetCommandRecords().front().kind,
              FakeCommandKind::PosControl);

    auto speed_fixture = create_seeded_fixture("pvt_cache_then_speed");
    EXPECT_NO_THROW(speed_fixture.master->SpdControl(0.0f, 0.5f));
    EXPECT_NO_THROW(speed_fixture.slave->SpdControl(0.0f, 0.5f));
    ExpectSpeedCommands(speed_fixture.adapter->GetCommandRecords(), 0.0f, 0.0f);
}

TEST(DoubleCoupleJointControlCacheTest, StatusCallbackCanRunDuringAutomaticPvtFeedback) {
    auto fixture = CreatePairedDoubleCoupleFixture("callback_automatic_pvt", CreateMotorRanges());
    fixture.adapter->SetReplyMode(FakeReplyMode::Automatic);
    int callback_count = 0;
    fixture.master->SetOnStatus([&callback_count](const JointStatus&) {
        ++callback_count;
    });
    fixture.master->PVTControl(1.0f, 1.0f, 0.04f, 0.0f, 0.0f);
    fixture.master->PVTControl(1.0f, 1.0f, 0.04f, 0.0f, 0.0f);
    EXPECT_GT(callback_count, 0);
}

TEST(DoubleCoupleJointPositionControlTest, PosControlMapsAxisSpeedToMotorSpeeds) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("pos_speed_mapping_adapter", ranges);

    fixture.master->PosControl(0.04f, 1.0f, 0.5f);
    fixture.master->PosControl(0.04f, 1.0f, 0.5f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    const auto expected =
        CalculateExpectedMotorSpeeds(DoubleCoupleJointAxis::Roll, 0.04, 0.0, 1.0f, ranges);
    EXPECT_NEAR(DecodePosSpeed(*motor1_message), std::abs(expected.motor1), 0.006f);
    EXPECT_NEAR(DecodePosSpeed(*motor2_message), std::abs(expected.motor2), 0.006f);
}

TEST(DoubleCoupleJointPositionControlTest, PosControlRejectsNearSingularTargetWithPositionHold) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("pos_near_singular_guard", ranges);
    constexpr double kCurrentRoll = -0.5;
    constexpr double kNearSingularPitch = -0.15;
    ExpectNearSingularPose(kCurrentRoll, kNearSingularPitch);

    fixture.master->PosControl(static_cast<float>(kCurrentRoll), 1.0f, 0.5f);
    fixture.master->PosControl(static_cast<float>(kCurrentRoll), 1.0f, 0.5f);
    fixture.adapter->ClearCommandRecords();
    fixture.slave->PosControl(static_cast<float>(kNearSingularPitch), 1.0f, 0.5f);
    EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());
    fixture.slave->PosControl(static_cast<float>(kNearSingularPitch), 1.0f, 0.5f);

    ExpectPositionHoldMessages(fixture.adapter->GetCommandRecords(),
                               CalculateExpectedMotorPositions(kCurrentRoll, 0.0), ranges);
}

TEST(DoubleCoupleJointPositionControlTest,
     RepeatedNearSingularPositionLimitKeepsLatchedHoldAfterFeedbackJitter) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("pos_near_singular_latched_hold", ranges);
    constexpr double kCurrentRoll = -0.5;
    constexpr double kNearSingularPitch = -0.15;
    constexpr double kFeedbackPitch = -0.04;
    ExpectNearSingularPose(kCurrentRoll, kNearSingularPitch);

    const auto latched_hold_positions = CalculateExpectedMotorPositions(kCurrentRoll, 0.0);
    EmitDoubleCoupleFeedback(fixture, latched_hold_positions);
    const auto initial_status = fixture.master->GetStatus(false, 0);
    ASSERT_TRUE(initial_status.has_value());
    EXPECT_NEAR(initial_status->pos, static_cast<float>(kCurrentRoll), 0.002f);

    fixture.adapter->ClearCommandRecords();
    fixture.slave->PosControl(static_cast<float>(kNearSingularPitch), 1.0f, 0.5f);
    EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());
    fixture.slave->PosControl(static_cast<float>(kNearSingularPitch), 1.0f, 0.5f);
    ExpectPositionHoldMessages(fixture.adapter->GetCommandRecords(), latched_hold_positions,
                               ranges);

    const auto jittered_feedback = CalculateExpectedMotorPositions(kCurrentRoll, kFeedbackPitch);
    EmitDoubleCoupleFeedback(fixture, jittered_feedback);
    const auto feedback_status = fixture.slave->GetStatus(false, 0);
    ASSERT_TRUE(feedback_status.has_value());
    EXPECT_NEAR(feedback_status->pos, static_cast<float>(kFeedbackPitch), 0.002f);

    fixture.adapter->ClearCommandRecords();
    fixture.slave->PosControl(static_cast<float>(kNearSingularPitch), 1.0f, 0.5f);
    EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());
    fixture.slave->PosControl(static_cast<float>(kNearSingularPitch), 1.0f, 0.5f);
    ExpectPositionHoldMessages(fixture.adapter->GetCommandRecords(), latched_hold_positions,
                               ranges);
}

TEST(DoubleCoupleJointPositionControlTest, H130NearSingularPitchHoldIsNotOverwrittenByRollCommand) {
    const auto ranges = CreateMotorRanges();
    constexpr float kDegToRad = static_cast<float>(3.14159265358979323846 / 180.0);
    const ExpectedMotorPositions feedback_positions{-38.934f * kDegToRad, 68.0f * kDegToRad};

    const auto run_repeated_hold = [&](const std::string& adapter_name, bool pitch_first) {
        auto fixture = CreateH130AnklePairedDoubleCoupleFixture(adapter_name, ranges);
        EmitDoubleCoupleFeedback(fixture, feedback_positions);
        const auto feedback_status = fixture.slave->GetStatus(false, 0);
        ASSERT_TRUE(feedback_status.has_value());
        EXPECT_NEAR(feedback_status->pos, -28.71f * kDegToRad, 0.01f);

        const auto expect_feedback_hold = [&fixture, &feedback_positions]() {
            const auto& messages = fixture.adapter->GetCommandRecords();
            ASSERT_EQ(messages.size(), 2u);
            const auto* motor1_message = FindSentMessage(messages, 5);
            const auto* motor2_message = FindSentMessage(messages, 6);
            ASSERT_NE(motor1_message, nullptr);
            ASSERT_NE(motor2_message, nullptr);
            EXPECT_EQ(motor1_message->kind, FakeCommandKind::PosControl);
            EXPECT_EQ(motor2_message->kind, FakeCommandKind::PosControl);
            EXPECT_NEAR(DecodePosPosition(*motor1_message), feedback_positions.motor1, 0.001f);
            EXPECT_NEAR(DecodePosPosition(*motor2_message), feedback_positions.motor2, 0.001f);
        };

        for (int i = 0; i < 50; i++) {
            fixture.adapter->ClearCommandRecords();
            if (pitch_first) {
                fixture.slave->PosControl(-31.8f * kDegToRad, 1.0f, 0.5f);
                fixture.master->PosControl(0.0f, 1.0f, 0.5f);
            } else {
                fixture.master->PosControl(0.0f, 1.0f, 0.5f);
                fixture.slave->PosControl(-31.8f * kDegToRad, 1.0f, 0.5f);
            }
            expect_feedback_hold();
        }
    };

    run_repeated_hold("h130_pos_near_singular_hold_roll_first", false);
    run_repeated_hold("h130_pos_near_singular_hold_pitch_first", true);
}

TEST(DoubleCoupleJointPositionControlTest,
     H130NearSingularHoldBrakesWhenFeedbackExceedsJointLimit) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreateH130AnklePairedDoubleCoupleFixture("h130_pos_near_singular_drift", ranges);
    constexpr float kDegToRad = static_cast<float>(3.14159265358979323846 / 180.0);
    const ExpectedMotorPositions initial_feedback{25.326f * kDegToRad, 8.514f * kDegToRad};
    EmitDoubleCoupleFeedback(fixture, initial_feedback);
    const auto feedback_status = fixture.master->GetStatus(false, 0);
    ASSERT_TRUE(feedback_status.has_value());
    EXPECT_GT(feedback_status->pos, 15.0f * kDegToRad);

    for (int i = 0; i < 20; i++) {
        fixture.adapter->ClearCommandRecords();
        fixture.slave->PosControl(-31.8f * kDegToRad, 1.0f, 0.5f);
        fixture.master->PosControl(0.0f, 1.0f, 0.5f);

        ExpectStopMessagesWithZeroCurrent(fixture.adapter->GetCommandRecords(), 2u);
        EmitDoubleCoupleFeedback(fixture, initial_feedback);
    }
}

TEST(DoubleCoupleJointMotorLimitTest, MotorLimit1AllowsTargetBeyondRangeWhileActualIsSafe) {
    auto ranges = CreateMotorRanges(12.5f);
    const auto target_positions = CalculateExpectedMotorPositions(0.2, 0.0);
    const Range<float> motor1_limit = target_positions.motor1 >= 0.0f
                                          ? Range<float>{-12.5f, target_positions.motor1 * 0.5f}
                                          : Range<float>{target_positions.motor1 * 0.5f, 12.5f};
    auto fixture = CreatePairedDoubleCoupleFixture("motor_limit1_runtime", ranges, false, false,
                                                   motor1_limit, std::nullopt);
    const auto zero_positions = CalculateExpectedMotorPositions(0.0, 0.0);
    EXPECT_FALSE(target_positions.motor1 >= motor1_limit.min &&
                 target_positions.motor1 <= motor1_limit.max);
    EmitDoubleCoupleFeedback(fixture, zero_positions);

    fixture.master->PVTControl(10.0f, 1.0f, 0.2f, 0.4f, 0.5f);
    fixture.slave->PVTControl(10.0f, 1.0f, 0.0f, 0.0f, 0.0f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    EXPECT_NEAR(DecodePvtPosition(*FindSentMessage(messages, 1), ranges), target_positions.motor1,
                0.002f);
    EXPECT_NEAR(DecodePvtPosition(*FindSentMessage(messages, 2), ranges), target_positions.motor2,
                0.002f);
}

TEST(DoubleCoupleJointMotorLimitTest, MotorLimit2AllowsTargetBeyondRangeWhileActualIsSafe) {
    auto ranges = CreateMotorRanges(12.5f);
    const auto target_positions = CalculateExpectedMotorPositions(0.0, 0.05);
    const Range<float> motor2_limit = target_positions.motor2 >= 0.0f
                                          ? Range<float>{-12.5f, target_positions.motor2 * 0.5f}
                                          : Range<float>{target_positions.motor2 * 0.5f, 12.5f};
    auto fixture = CreatePairedDoubleCoupleFixture("motor_limit2_runtime", ranges, false, false,
                                                   std::nullopt, motor2_limit);
    const auto zero_positions = CalculateExpectedMotorPositions(0.0, 0.0);
    EXPECT_FALSE(target_positions.motor2 >= motor2_limit.min &&
                 target_positions.motor2 <= motor2_limit.max);
    EmitDoubleCoupleFeedback(fixture, zero_positions);

    fixture.master->PVTControl(10.0f, 1.0f, 0.0f, 0.0f, 0.0f);
    fixture.slave->PVTControl(10.0f, 1.0f, 0.05f, 0.4f, 0.5f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    EXPECT_NEAR(DecodePvtPosition(*FindSentMessage(messages, 1), ranges), target_positions.motor1,
                0.002f);
    EXPECT_NEAR(DecodePvtPosition(*FindSentMessage(messages, 2), ranges), target_positions.motor2,
                0.002f);
}

TEST(DoubleCoupleJointMotorLimitTest, MissingMotorLimitsFallBackToMotorPvtPositionRange) {
    auto ranges = CreateMotorRanges(0.001f);
    auto fixture = CreatePairedDoubleCoupleFixture("motor_limit_fallback_runtime", ranges);
    const auto zero_positions = CalculateExpectedMotorPositions(0.0, 0.0);
    const auto target_positions = CalculateExpectedMotorPositions(0.05, 0.0);
    ASSERT_GT(std::max(std::abs(target_positions.motor1), std::abs(target_positions.motor2)),
              ranges.position.max);
    EmitDoubleCoupleFeedback(fixture, zero_positions);

    EXPECT_NO_THROW(fixture.master->PVTControl(10.0f, 1.0f, 0.05f, 0.4f, 0.5f));
    EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());
    EXPECT_NO_THROW(fixture.master->PVTControl(10.0f, 1.0f, 0.05f, 0.4f, 0.5f));

    ExpectStopMessagesWithZeroCurrent(fixture.adapter->GetCommandRecords(), 2u);
}

TEST(DoubleCoupleJointMotorLimitTest, PVTTargetWithinMotorLimitSlackIsAllowedWhileActualIsSafe) {
    auto ranges = CreateMotorRanges();
    const auto zero_positions = CalculateExpectedMotorPositions(0.0, 0.0);
    const auto target_positions = CalculateExpectedMotorPositions(0.05, 0.0);
    const Range<float> motor1_limit =
        target_positions.motor1 >= 0.0f
            ? Range<float>{-12.5f, target_positions.motor1 + 0.5f * kLimitSlack}
            : Range<float>{target_positions.motor1 - 0.5f * kLimitSlack, 12.5f};
    auto fixture = CreatePairedDoubleCoupleFixture("motor_limit_pvt_slack", ranges, false, false,
                                                   motor1_limit, std::nullopt);
    EmitDoubleCoupleFeedback(fixture, zero_positions);

    fixture.master->PVTControl(10.0f, 1.0f, 0.05f, 0.4f, 0.5f);
    fixture.slave->PVTControl(10.0f, 1.0f, 0.0f, 0.0f, 0.0f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    EXPECT_NEAR(DecodePvtPosition(*FindSentMessage(messages, 1), ranges), target_positions.motor1,
                0.002f);
    EXPECT_NEAR(DecodePvtPosition(*FindSentMessage(messages, 2), ranges), target_positions.motor2,
                0.002f);
}

TEST(DoubleCoupleJointMotorLimitTest, ConfiguredMotorLimitsAllowTargetWithinRange) {
    auto ranges = CreateMotorRanges(0.001f);
    const auto target_positions = CalculateExpectedMotorPositions(0.05, 0.0);
    const Range<float> motor1_limit{-std::abs(target_positions.motor1) - kLimitSlack - 0.01f,
                                    std::abs(target_positions.motor1) + kLimitSlack + 0.01f};
    const Range<float> motor2_limit{-std::abs(target_positions.motor2) - kLimitSlack - 0.01f,
                                    std::abs(target_positions.motor2) + kLimitSlack + 0.01f};
    auto fixture = CreatePairedDoubleCoupleFixture("motor_limit_allows_target", ranges, false,
                                                   false, motor1_limit, motor2_limit);

    fixture.master->PVTControl(10.0f, 1.0f, 0.05f, 0.4f, 0.5f);
    fixture.master->PVTControl(10.0f, 1.0f, 0.05f, 0.4f, 0.5f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    EXPECT_NEAR(DecodePvtPosition(*motor1_message, ranges), target_positions.motor1, 0.002f);
    EXPECT_NEAR(DecodePvtPosition(*motor2_message, ranges), target_positions.motor2, 0.002f);
}

TEST(DoubleCoupleJointMotorLimitTest, ActualMotorLimitStopsOutwardAndAllowsInwardBothPvtModes) {
    const auto ranges = CreateMotorRanges();
    constexpr double kActualRoll = 0.04;
    constexpr double kOutwardRoll = 0.08;
    const auto actual_positions = CalculateExpectedMotorPositions(kActualRoll, 0.0);
    const auto outward_positions = CalculateExpectedMotorPositions(kOutwardRoll, 0.0);
    const Range<float> motor1_limit =
        outward_positions.motor1 > actual_positions.motor1
            ? Range<float>{-12.5f, actual_positions.motor1 + 0.5f * kLimitSlack}
            : Range<float>{actual_positions.motor1 - 0.5f * kLimitSlack, 12.5f};

    for (const bool torque_mode : {false, true}) {
        auto outward = CreatePairedDoubleCoupleFixture(
            torque_mode ? "pvt_actual_motor_limit_torque_mode" : "pvt_actual_motor_limit", ranges,
            false, false, motor1_limit, std::nullopt);
        EmitDoubleCoupleFeedback(outward, actual_positions);
        outward.adapter->ClearCommandRecords();
        outward.master->PVTControl(
            10.0f, 0.0f, kOutwardRoll, 0.0f, 0.0f,
            (torque_mode ? PVTControlMode::HostKpMotorKd : PVTControlMode::MotorPVT));
        outward.slave->PVTControl(
            10.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            (torque_mode ? PVTControlMode::HostKpMotorKd : PVTControlMode::MotorPVT));
        if (torque_mode) {
            ExpectPositionHoldMessages(
                outward.adapter->GetCommandRecords(),
                {std::clamp(actual_positions.motor1, motor1_limit.min + kLimitSlack,
                            motor1_limit.max - kLimitSlack),
                 actual_positions.motor2},
                ranges);
        } else {
            ExpectPositionHoldMessages(
                outward.adapter->GetCommandRecords(),
                {std::clamp(actual_positions.motor1, motor1_limit.min + kLimitSlack,
                            motor1_limit.max - kLimitSlack),
                 actual_positions.motor2},
                ranges);
        }

        auto inward = CreatePairedDoubleCoupleFixture(
            torque_mode ? "pvt_actual_motor_exit_torque_mode" : "pvt_actual_motor_exit", ranges,
            false, false, motor1_limit, std::nullopt);
        // 零 Kd 的耦合模式将目标速度设为反馈速度；明确注入向内速度，
        // 避免协议将零速度量化成微小向外速度而触发方向保护。
        const float inward_speed =
            outward_positions.motor1 > actual_positions.motor1 ? -0.1f : 0.1f;
        EmitDoubleCoupleFeedback(inward, actual_positions, inward_speed);
        inward.adapter->ClearCommandRecords();
        inward.master->PVTControl(
            10.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            (torque_mode ? PVTControlMode::HostKpMotorKd : PVTControlMode::MotorPVT));
        inward.slave->PVTControl(
            10.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            (torque_mode ? PVTControlMode::HostKpMotorKd : PVTControlMode::MotorPVT));
        if (torque_mode) {
            const auto expected =
                CalculateExpectedCombinedMotorTorques(kActualRoll, 0.0, -0.4, 0.0, ranges);
            ExpectPvtTorqueCommands(inward.adapter->GetCommandRecords(), expected.motor1,
                                    expected.motor2, ranges);
        } else {
            ASSERT_EQ(inward.adapter->GetCommandRecords().size(), 2u);
            EXPECT_EQ(inward.adapter->GetCommandRecords().front().kind,
                      FakeCommandKind::PVTControl);
            EXPECT_EQ(inward.adapter->GetCommandRecords().back().kind, FakeCommandKind::PVTControl);
        }
    }
}

TEST(DoubleCoupleJointMotorLimitTest, OverlappingNarrowLimitZonesProtectBothDirections) {
    const auto ranges = CreateMotorRanges();
    const auto zero_positions = CalculateExpectedMotorPositions(0.0, 0.0);
    const auto positive_target = CalculateExpectedMotorPositions(0.05, 0.0);
    const double target_roll = positive_target.motor1 > zero_positions.motor1 ? 0.05 : -0.05;
    const Range<float> narrow_motor1_limit{-0.01f, 0.01f};

    for (const bool torque_mode : {false, true}) {
        auto fixture = CreatePairedDoubleCoupleFixture(
            torque_mode ? "pvt_narrow_motor_limit_torque_mode" : "pvt_narrow_motor_limit", ranges,
            false, false, narrow_motor1_limit, std::nullopt);
        EmitDoubleCoupleFeedback(fixture, zero_positions);
        fixture.adapter->ClearCommandRecords();

        fixture.master->PVTControl(
            10.0f, 0.0f, static_cast<float>(target_roll), 0.0f, 0.0f,
            (torque_mode ? PVTControlMode::HostKpMotorKd : PVTControlMode::MotorPVT));
        fixture.slave->PVTControl(
            10.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            (torque_mode ? PVTControlMode::HostKpMotorKd : PVTControlMode::MotorPVT));

        ASSERT_EQ(fixture.adapter->GetCommandRecords().size(), 2u);
        for (const auto& message : fixture.adapter->GetCommandRecords()) {
            EXPECT_EQ(message.kind, FakeCommandKind::Stop);
        }
    }
}

TEST(DoubleCoupleJointPositionControlTest, PosControlRejectsJointTargetOutsideSlack) {
    const auto ranges = CreateMotorRanges();
    const Range<float> master_limit{-0.1f, 0.1f};
    auto fixture = CreatePairedDoubleCoupleFixture("pos_joint_limit_slack", ranges, false, false,
                                                   std::nullopt, std::nullopt, master_limit);

    EXPECT_THROW(fixture.master->PosControl(0.2f, 1.0f, 0.5f), std::invalid_argument);
    ExpectStopMessagesWithZeroCurrent(fixture.adapter->GetCommandRecords(), 2u);
}

TEST(DoubleCoupleJointDynamicLimitTest, MasterDynamicControlsStopAtJointLimit) {
    const auto ranges = CreateMotorRanges();
    auto fixture =
        CreatePairedDoubleCoupleFixture("master_dynamic_joint_limit", ranges, false, false,
                                        std::nullopt, std::nullopt, Range<float>{-0.1f, 0.1f});
    EmitDoubleCoupleFeedback(fixture, CalculateExpectedMotorPositions(0.09, 0.0));
    const auto status = fixture.master->GetStatus(false, 0);
    ASSERT_TRUE(status.has_value());
    EXPECT_NEAR(status->pos, 0.09f, 0.002f);
    fixture.adapter->ClearCommandRecords();

    fixture.master->SpdControl(1.0f, 0.5f);
    fixture.slave->SpdControl(0.0f, 0.5f);
    ExpectSpeedCommands(fixture.adapter->GetCommandRecords(), 0.0f, 0.0f);

    fixture.adapter->ClearCommandRecords();
    fixture.master->TorControl(1.0f);
    fixture.slave->TorControl(0.0f);
    ExpectTorqueCommands(fixture.adapter->GetCommandRecords(), 0.0f, 0.0f);

    fixture.adapter->ClearCommandRecords();
    fixture.master->CurControl(1.0f);
    fixture.slave->CurControl(0.0f);
    ExpectCurrentCommands(fixture.adapter->GetCommandRecords(), 0.0f, 0.0f);
}

TEST(DoubleCoupleJointDynamicLimitTest, SlaveReverseDynamicControlsStopAtJointLimit) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("slave_reverse_dynamic_joint_limit", ranges,
                                                   false, true, std::nullopt, std::nullopt,
                                                   std::nullopt, Range<float>{-0.1f, 0.1f});
    EmitDoubleCoupleFeedback(fixture, CalculateExpectedMotorPositions(0.0, -0.09));
    const auto status = fixture.slave->GetStatus(false, 0);
    ASSERT_TRUE(status.has_value());
    EXPECT_NEAR(status->pos, 0.09f, 0.002f);
    fixture.adapter->ClearCommandRecords();

    fixture.slave->SpdControl(1.0f, 0.5f);
    fixture.master->SpdControl(0.0f, 0.5f);
    ExpectSpeedCommands(fixture.adapter->GetCommandRecords(), 0.0f, 0.0f);

    fixture.adapter->ClearCommandRecords();
    fixture.slave->TorControl(1.0f);
    fixture.master->TorControl(0.0f);
    ExpectTorqueCommands(fixture.adapter->GetCommandRecords(), 0.0f, 0.0f);

    fixture.adapter->ClearCommandRecords();
    fixture.slave->CurControl(1.0f);
    fixture.master->CurControl(0.0f);
    ExpectCurrentCommands(fixture.adapter->GetCommandRecords(), 0.0f, 0.0f);
}

TEST(DoubleCoupleJointDynamicLimitTest, DynamicControlsStopAtMotorLimit) {
    const auto ranges = CreateMotorRanges();
    constexpr double kCurrentRoll = 0.04;
    const auto current_positions = CalculateExpectedMotorPositions(kCurrentRoll, 0.0);
    const auto current_speeds =
        CalculateExpectedMotorSpeeds(DoubleCoupleJointAxis::Roll, kCurrentRoll, 0.0, 1.0f, ranges);
    const Range<float> motor1_limit = current_speeds.motor1 >= 0.0f
                                          ? Range<float>{-12.5f, current_positions.motor1 + 0.01f}
                                          : Range<float>{current_positions.motor1 - 0.01f, 12.5f};
    auto fixture = CreatePairedDoubleCoupleFixture("dynamic_motor_limit", ranges, false, false,
                                                   motor1_limit, std::nullopt);
    EmitDoubleCoupleFeedback(fixture, current_positions);
    const auto status = fixture.master->GetStatus(false, 0);
    ASSERT_TRUE(status.has_value());
    EXPECT_NEAR(status->pos, kCurrentRoll, 0.002f);
    fixture.adapter->ClearCommandRecords();

    fixture.master->SpdControl(1.0f, 0.5f);
    fixture.slave->SpdControl(0.0f, 0.5f);
    ExpectSpeedCommands(fixture.adapter->GetCommandRecords(), 0.0f, 0.0f);

    fixture.adapter->ClearCommandRecords();
    fixture.master->TorControl(1.0f);
    fixture.slave->TorControl(0.0f);
    ExpectTorqueCommands(fixture.adapter->GetCommandRecords(), 0.0f, 0.0f);

    fixture.adapter->ClearCommandRecords();
    fixture.master->CurControl(1.0f);
    fixture.slave->CurControl(0.0f);
    ExpectCurrentCommands(fixture.adapter->GetCommandRecords(), 0.0f, 0.0f);
}

TEST(DoubleCoupleJointDynamicLimitTest, ExitingNearSingularDirectionIsAllowed) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("dynamic_near_singular_exit", ranges);
    constexpr double kCurrentRoll = -0.5;
    constexpr double kCurrentPitch = -0.15;
    ExpectNearSingularPose(kCurrentRoll, kCurrentPitch);
    EmitDoubleCoupleFeedback(fixture, CalculateExpectedMotorPositions(kCurrentRoll, kCurrentPitch));
    const auto status = fixture.slave->GetStatus(false, 0);
    ASSERT_TRUE(status.has_value());
    EXPECT_NEAR(status->pos, static_cast<float>(kCurrentPitch), 0.002f);
    fixture.adapter->ClearCommandRecords();

    fixture.slave->SpdControl(0.01f, 0.5f);
    fixture.master->SpdControl(0.0f, 0.5f);

    const auto expected = CalculateExpectedMotorSpeeds(DoubleCoupleJointAxis::Pitch, kCurrentRoll,
                                                       kCurrentPitch, 0.01f, ranges);
    ExpectSpeedCommands(fixture.adapter->GetCommandRecords(), expected.motor1, expected.motor2,
                        0.006f);
}

TEST(DoubleCoupleJointDynamicLimitTest, EnteringNearSingularDirectionHoldsCurrentPosition) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("dynamic_near_singular_enter", ranges);
    constexpr double kCurrentRoll = -0.5;
    constexpr double kCurrentPitch = -0.15;
    ExpectNearSingularPose(kCurrentRoll, kCurrentPitch);
    const auto current_positions = CalculateExpectedMotorPositions(kCurrentRoll, kCurrentPitch);
    EmitDoubleCoupleFeedback(fixture, current_positions);
    const auto status = fixture.slave->GetStatus(false, 0);
    ASSERT_TRUE(status.has_value());
    EXPECT_NEAR(status->pos, static_cast<float>(kCurrentPitch), 0.002f);
    fixture.adapter->ClearCommandRecords();

    fixture.slave->SpdControl(-1.0f, 0.5f);
    EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());
    fixture.master->SpdControl(0.0f, 0.5f);

    ExpectPositionHoldMessages(fixture.adapter->GetCommandRecords(), current_positions, ranges);
}

// ============================================================
// 动态类控制（Spd/Tor/Cur）合帧
// ============================================================

TEST(DoubleCoupleJointDynamicBatchTest, BatchesBothAxesIntoOneMergedSpeedFrame) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("dynamic_batch_speed", ranges);

    fixture.master->SpdControl(1.0f, 0.5f);
    // 半帧不下发
    EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());

    fixture.slave->SpdControl(0.5f, 0.5f);

    // 两轴速度一起参与联合逆速度映射，roll 分量不再被覆盖
    const auto expected = CalculateExpectedCombinedMotorSpeeds(0.0, 0.0, 1.0f, 0.5f, ranges);
    ExpectSpeedCommands(fixture.adapter->GetCommandRecords(), expected.motor1, expected.motor2,
                        0.006f);
}

TEST(DoubleCoupleJointDynamicBatchTest, BatchesBothAxesIntoOneMergedTorqueFrame) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("dynamic_batch_torque", ranges);

    fixture.master->TorControl(1.0f);
    EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());

    fixture.slave->TorControl(0.5f);

    const auto expected = CalculateExpectedCombinedMotorTorques(0.0, 0.0, 1.0f, 0.5f, ranges);
    ExpectTorqueCommands(fixture.adapter->GetCommandRecords(), expected.motor1, expected.motor2);
}

TEST(DoubleCoupleJointDynamicBatchTest, BatchesBothAxesIntoOneMergedCurrentFrame) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("dynamic_batch_current", ranges);
    constexpr float kRollCurrent = 1.0f;
    constexpr float kPitchCurrent = 1.0f;

    fixture.master->CurControl(kRollCurrent);
    EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());

    fixture.slave->CurControl(kPitchCurrent);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    EXPECT_EQ(motor1_message->kind, FakeCommandKind::CurControl);
    EXPECT_EQ(motor2_message->kind, FakeCommandKind::CurControl);

    // 双轴电流分配是两轴各自单轴分配的线性叠加（默认范围下不触发统一缩放）
    const auto roll_only =
        CalculateExpectedMotorCurrents(DoubleCoupleJointAxis::Roll, 0.0, 0.0, kRollCurrent, ranges);
    const auto pitch_only = CalculateExpectedMotorCurrents(DoubleCoupleJointAxis::Pitch, 0.0, 0.0,
                                                           kPitchCurrent, ranges);
    const float motor1_current = DecodeCurrentCommand(*motor1_message);
    const float motor2_current = DecodeCurrentCommand(*motor2_message);
    EXPECT_NEAR(motor1_current, roll_only.motor1 + pitch_only.motor1, 0.02f);
    EXPECT_NEAR(motor2_current, roll_only.motor2 + pitch_only.motor2, 0.02f);
}

TEST(DoubleCoupleJointDynamicBatchTest, SwitchingModeDiscardsPendingHalfFrame) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("dynamic_batch_mode_switch", ranges);

    fixture.master->SpdControl(1.0f, 0.5f);
    EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());

    // 模式不一致：丢弃挂起的速度半帧，本次成为扭矩批次的第一条
    fixture.slave->TorControl(1.0f);
    EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());

    // 同模式第二条才成帧
    fixture.master->TorControl(1.0f);
    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    EXPECT_EQ(messages.front().kind, FakeCommandKind::TorControl);
    EXPECT_EQ(messages.back().kind, FakeCommandKind::TorControl);
}

TEST(DoubleCoupleJointDynamicBatchTest, KeepsLastCommandOfAxisOutsideCurrentBatch) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("dynamic_batch_remember", ranges);

    fixture.master->SpdControl(1.0f, 0.5f);
    fixture.slave->SpdControl(0.5f, 0.5f);
    const auto both_axes = CalculateExpectedCombinedMotorSpeeds(0.0, 0.0, 1.0f, 0.5f, ranges);
    ExpectSpeedCommands(fixture.adapter->GetCommandRecords(), both_axes.motor1, both_axes.motor2,
                        0.006f);

    // 下一批次只命令 roll：pitch 沿用上次的 0.5，成帧结果与两轴都命令时一致
    fixture.adapter->ClearCommandRecords();
    fixture.master->SpdControl(1.0f, 0.5f);
    fixture.master->SpdControl(1.0f, 0.5f);
    ExpectSpeedCommands(fixture.adapter->GetCommandRecords(), both_axes.motor1, both_axes.motor2,
                        0.006f);
}

TEST(DoubleCoupleJointDynamicBatchTest, StopClearsRememberedAxisCommand) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("dynamic_batch_stop_clear", ranges);

    fixture.master->SpdControl(1.0f, 0.5f);
    fixture.slave->SpdControl(0.5f, 0.5f);
    fixture.master->Stop(MotorStopMode::FullBrake);
    fixture.adapter->ClearCommandRecords();

    // Stop 取消尚未发出的半帧并清空轴槽位，pitch 不再沿用 0.5
    fixture.master->SpdControl(1.0f, 0.5f);
    fixture.master->SpdControl(1.0f, 0.5f);
    const auto roll_only = CalculateExpectedCombinedMotorSpeeds(0.0, 0.0, 1.0f, 0.0f, ranges);
    ExpectSpeedCommands(fixture.adapter->GetCommandRecords(), roll_only.motor1, roll_only.motor2,
                        0.006f);
    // 未命令过的轴不参与电流上限收紧：整帧按本轴的 0.5 A 下发，而不是被默认值 0 A 限死
    for (const auto* message : {FindSentMessage(fixture.adapter->GetCommandRecords(), 1),
                                FindSentMessage(fixture.adapter->GetCommandRecords(), 2)}) {
        ASSERT_NE(message, nullptr);
        EXPECT_NEAR(std::get<FakeSpdControlPayload>(message->payload).current, 0.5f, 0.001f);
    }
}

TEST(DoubleCoupleJointDynamicBatchTest, PositionControlDiscardsPendingDynamicHalfFrame) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("dynamic_batch_position_interrupt", ranges);

    fixture.master->SpdControl(1.0f, 0.5f);
    EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());

    // 位置类介入：丢弃动态半帧，并作为位置批次第一条
    fixture.master->PosControl(0.05f, 1.0f, 0.5f);
    EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());

    fixture.slave->PosControl(0.0f, 1.0f, 0.5f);
    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    EXPECT_EQ(messages.front().kind, FakeCommandKind::PosControl);
    EXPECT_EQ(messages.back().kind, FakeCommandKind::PosControl);
}

TEST(DoubleCoupleJointDynamicBatchTest, DynamicControlDiscardsPendingPositionHalfFrame) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("dynamic_batch_position_pending", ranges);

    fixture.master->PosControl(0.05f, 1.0f, 0.5f);
    EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());

    // 动态类介入：丢弃位置半帧，并作为速度批次第一条
    fixture.master->SpdControl(1.0f, 0.5f);
    EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());

    fixture.slave->SpdControl(0.5f, 0.5f);
    // 被丢弃的位置半帧不修改已提交目标，映射仍在原位姿进行
    const auto expected = CalculateExpectedCombinedMotorSpeeds(0.0, 0.0, 1.0f, 0.5f, ranges);
    ExpectSpeedCommands(fixture.adapter->GetCommandRecords(), expected.motor1, expected.motor2,
                        0.006f);
}

TEST(DoubleCoupleJointDynamicBatchTest, MergesSpeedCurrentLimitFromBothAxisCalls) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("dynamic_batch_current_limit", ranges);

    fixture.master->SpdControl(1.0f, 0.8f);
    fixture.slave->SpdControl(0.5f, 0.3f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    // 两台电机各取两条调用中较小的电流上限
    EXPECT_NEAR(std::get<FakeSpdControlPayload>(motor1_message->payload).current, 0.3f, 0.001f);
    EXPECT_NEAR(std::get<FakeSpdControlPayload>(motor2_message->payload).current, 0.3f, 0.001f);
}

TEST(DoubleCoupleJointControlCacheTest, SpdControlWithoutFeedbackKeepsCachedTargetForLaterPvt) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("speed_cache_adapter", ranges);

    fixture.master->SpdControl(1.0f, 0.5f);
    fixture.adapter->ClearCommandRecords();
    fixture.slave->PVTControl(10.0f, 1.0f, -0.03f, 0.2f, 0.1f);
    fixture.slave->PVTControl(10.0f, 1.0f, -0.03f, 0.2f, 0.1f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    const auto expected = CalculateExpectedMotorPositions(0.0, -0.03);
    EXPECT_NEAR(DecodePvtPosition(*motor1_message, ranges), expected.motor1, 0.002f);
    EXPECT_NEAR(DecodePvtPosition(*motor2_message, ranges), expected.motor2, 0.002f);
}

TEST(DoubleCoupleJointControlCacheTest, TorControlWithoutFeedbackKeepsCachedTargetForLaterPvt) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("torque_cache_adapter", ranges);

    fixture.master->TorControl(1.0f);
    fixture.adapter->ClearCommandRecords();
    fixture.slave->PVTControl(10.0f, 1.0f, -0.03f, 0.2f, 0.1f);
    fixture.slave->PVTControl(10.0f, 1.0f, -0.03f, 0.2f, 0.1f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    const auto expected = CalculateExpectedMotorPositions(0.0, -0.03);
    EXPECT_NEAR(DecodePvtPosition(*motor1_message, ranges), expected.motor1, 0.002f);
    EXPECT_NEAR(DecodePvtPosition(*motor2_message, ranges), expected.motor2, 0.002f);
}

TEST(DoubleCoupleJointControlCacheTest, SpdControlRejectsExcessiveMotorSpeedRange) {
    const auto ranges = CreateMotorRanges(12.5f, 0.2f, 30.0f, 30.0f);
    auto fixture = CreatePairedDoubleCoupleFixture("speed_limit_adapter", ranges);

    fixture.master->SpdControl(2.0f, 0.5f);
    EXPECT_THROW(fixture.slave->SpdControl(0.0f, 0.5f), JointException);

    const auto records = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(records.size(), 2u);
    for (const auto& record : records)
        EXPECT_EQ(record.kind, FakeCommandKind::PosControl);
}

TEST(DoubleCoupleJointControlCacheTest, TorControlRejectsExcessiveMotorTorqueRange) {
    const auto ranges = CreateMotorRanges(12.5f, 18.0f, 0.2f, 30.0f);
    auto fixture = CreatePairedDoubleCoupleFixture("torque_limit_adapter", ranges);

    fixture.master->TorControl(3.0f);
    EXPECT_THROW(fixture.slave->TorControl(0.0f), JointException);

    const auto records = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(records.size(), 2u);
    for (const auto& record : records)
        EXPECT_EQ(record.kind, FakeCommandKind::PosControl);
}

TEST(DoubleCoupleJointControlCacheTest, CurControlWithoutFeedbackKeepsCachedTargetForLaterPvt) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("current_cache_adapter", ranges);

    fixture.master->CurControl(1.0f);
    fixture.adapter->ClearCommandRecords();
    fixture.slave->PVTControl(10.0f, 1.0f, -0.03f, 0.2f, 0.1f);
    fixture.slave->PVTControl(10.0f, 1.0f, -0.03f, 0.2f, 0.1f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    const auto expected = CalculateExpectedMotorPositions(0.0, -0.03);
    EXPECT_NEAR(DecodePvtPosition(*motor1_message, ranges), expected.motor1, 0.002f);
    EXPECT_NEAR(DecodePvtPosition(*motor2_message, ranges), expected.motor2, 0.002f);
}

TEST(DoubleCoupleJointControlCacheTest, CurControlSplitsCurrentByTorqueRatio) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("current_split_adapter", ranges);

    fixture.master->CurControl(1.25f);
    fixture.slave->CurControl(0.0f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    EXPECT_EQ(motor1_message->kind, FakeCommandKind::CurControl);
    EXPECT_EQ(motor2_message->kind, FakeCommandKind::CurControl);

    const auto expected =
        CalculateExpectedMotorCurrents(DoubleCoupleJointAxis::Roll, 0.0, 0.0, 1.25f, ranges);
    const float motor1_current = DecodeCurrentCommand(*motor1_message);
    const float motor2_current = DecodeCurrentCommand(*motor2_message);
    EXPECT_NEAR(motor1_current, expected.motor1, 0.02f);
    EXPECT_NEAR(motor2_current, expected.motor2, 0.02f);
    EXPECT_NEAR(std::abs(motor1_current) + std::abs(motor2_current), 1.25f, 0.03f);
}

TEST(DoubleCoupleJointControlCacheTest,
     SlaveCurControlWithoutFeedbackKeepsCachedTargetForLaterPvt) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("slave_current_adapter", ranges);

    fixture.slave->CurControl(1.0f);
    fixture.adapter->ClearCommandRecords();
    fixture.master->PVTControl(10.0f, 1.0f, 0.04f, 0.2f, 0.1f);
    fixture.master->PVTControl(10.0f, 1.0f, 0.04f, 0.2f, 0.1f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    const auto expected = CalculateExpectedMotorPositions(0.04, 0.0);
    EXPECT_NEAR(DecodePvtPosition(*motor1_message, ranges), expected.motor1, 0.002f);
    EXPECT_NEAR(DecodePvtPosition(*motor2_message, ranges), expected.motor2, 0.002f);
}

TEST(DoubleCoupleJointControlCacheTest, DynamicControlsRefreshCachedTargetFromMotorFeedback) {
    const auto ranges = CreateMotorRanges();
    const auto feedback_positions = CalculateExpectedMotorPositions(0.07, -0.02);
    const std::vector<std::pair<std::string, std::function<void(PairedDoubleCoupleFixture&)>>>
        cases{
            {"master_speed",
             [](PairedDoubleCoupleFixture& fixture) {
                 fixture.master->SpdControl(1.0f, 0.5f);
             }},
            {"master_torque",
             [](PairedDoubleCoupleFixture& fixture) {
                 fixture.master->TorControl(1.0f);
             }},
            {"master_current",
             [](PairedDoubleCoupleFixture& fixture) {
                 fixture.master->CurControl(1.0f);
             }},
            {"slave_speed",
             [](PairedDoubleCoupleFixture& fixture) {
                 fixture.slave->SpdControl(1.0f, 0.5f);
             }},
            {"slave_torque",
             [](PairedDoubleCoupleFixture& fixture) {
                 fixture.slave->TorControl(1.0f);
             }},
            {"slave_current",
             [](PairedDoubleCoupleFixture& fixture) {
                 fixture.slave->CurControl(1.0f);
             }},
        };

    for (const auto& [name, action] : cases) {
        auto fixture = CreatePairedDoubleCoupleFixture("feedback_refresh_" + name, ranges);
        EmitDoubleCoupleFeedback(fixture, feedback_positions);

        action(fixture);

        fixture.adapter->ClearCommandRecords();
        fixture.slave->PVTControl(10.0f, 1.0f, -0.03f, 0.2f, 0.1f);
        fixture.slave->PVTControl(10.0f, 1.0f, -0.03f, 0.2f, 0.1f);

        const auto& slave_messages = fixture.adapter->GetCommandRecords();
        ASSERT_EQ(slave_messages.size(), 2u) << name;
        const auto* slave_motor1_message = FindSentMessage(slave_messages, 1);
        const auto* slave_motor2_message = FindSentMessage(slave_messages, 2);
        ASSERT_NE(slave_motor1_message, nullptr) << name;
        ASSERT_NE(slave_motor2_message, nullptr) << name;
        const auto expected_slave_target = CalculateExpectedMotorPositions(0.07, -0.03);
        EXPECT_NEAR(DecodePvtPosition(*slave_motor1_message, ranges), expected_slave_target.motor1,
                    0.002f)
            << name;
        EXPECT_NEAR(DecodePvtPosition(*slave_motor2_message, ranges), expected_slave_target.motor2,
                    0.002f)
            << name;

        fixture.adapter->ClearCommandRecords();
        fixture.master->PVTControl(10.0f, 1.0f, 0.06f, 0.2f, 0.1f);
        fixture.master->PVTControl(10.0f, 1.0f, 0.06f, 0.2f, 0.1f);

        const auto& master_messages = fixture.adapter->GetCommandRecords();
        ASSERT_EQ(master_messages.size(), 2u) << name;
        const auto* master_motor1_message = FindSentMessage(master_messages, 1);
        const auto* master_motor2_message = FindSentMessage(master_messages, 2);
        ASSERT_NE(master_motor1_message, nullptr) << name;
        ASSERT_NE(master_motor2_message, nullptr) << name;
        const auto expected_master_target = CalculateExpectedMotorPositions(0.06, -0.03);
        EXPECT_NEAR(DecodePvtPosition(*master_motor1_message, ranges),
                    expected_master_target.motor1, 0.002f)
            << name;
        EXPECT_NEAR(DecodePvtPosition(*master_motor2_message, ranges),
                    expected_master_target.motor2, 0.002f)
            << name;
    }
}

TEST(DoubleCoupleJointControlCacheTest, CurControlRejectsExcessiveMotorCurrentRange) {
    const auto ranges = CreateMotorRanges(12.5f, 18.0f, 30.0f, 0.2f);
    auto fixture = CreatePairedDoubleCoupleFixture("current_limit_adapter", ranges);

    fixture.master->CurControl(1.25f);
    EXPECT_THROW(fixture.slave->CurControl(0.0f), JointException);

    const auto records = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(records.size(), 2u);
    for (const auto& record : records)
        EXPECT_EQ(record.kind, FakeCommandKind::PosControl);
}

TEST(DoubleCoupleJointSafetyTest, RejectsNonFiniteControlInputsWithPositionHoldWhenFeedbackExists) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const auto ranges = CreateMotorRanges();
    const auto hold_positions = CalculateExpectedMotorPositions(0.04, -0.02);
    const std::vector<std::pair<std::string, std::function<void(PairedDoubleCoupleFixture&)>>>
        cases{
            {"master_pvt_gain",
             [nan](PairedDoubleCoupleFixture& fixture) {
                 fixture.master->PVTControl(nan, 1.0f, 0.0f, 0.0f, 0.0f);
             }},
            {"slave_pvt_pos",
             [nan](PairedDoubleCoupleFixture& fixture) {
                 fixture.slave->PVTControl(1.0f, 1.0f, nan, 0.0f, 0.0f);
             }},
            {"master_pos_current",
             [inf](PairedDoubleCoupleFixture& fixture) {
                 fixture.master->PosControl(0.0f, 1.0f, inf);
             }},
            {"slave_pos_speed",
             [nan](PairedDoubleCoupleFixture& fixture) {
                 fixture.slave->PosControl(0.0f, nan, 0.5f);
             }},
            {"master_speed_current",
             [nan](PairedDoubleCoupleFixture& fixture) {
                 fixture.master->SpdControl(0.5f, nan);
             }},
            {"slave_speed",
             [nan](PairedDoubleCoupleFixture& fixture) {
                 fixture.slave->SpdControl(nan, 0.5f);
             }},
            {"master_torque",
             [nan](PairedDoubleCoupleFixture& fixture) {
                 fixture.master->TorControl(nan);
             }},
            {"slave_current",
             [inf](PairedDoubleCoupleFixture& fixture) {
                 fixture.slave->CurControl(inf);
             }},
        };

    for (const auto& [name, action] : cases) {
        auto fixture = CreatePairedDoubleCoupleFixture("double_couple_nonfinite_" + name, ranges);
        EmitDoubleCoupleFeedback(fixture, hold_positions);

        try {
            action(fixture);
        } catch (const std::invalid_argument&) {
            ExpectPositionHoldMessages(fixture.adapter->GetCommandRecords(), hold_positions,
                                       ranges);
            continue;
        }
        FAIL() << "Expected std::invalid_argument for non-finite input: " << name;
    }
}

TEST(DoubleCoupleJointSafetyTest, RejectsNonFiniteControlInputsWithStopWhenFeedbackIsMissing) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const auto ranges = CreateMotorRanges();
    const std::vector<std::pair<std::string, std::function<void(PairedDoubleCoupleFixture&)>>>
        cases{
            {"master_speed_current",
             [nan](PairedDoubleCoupleFixture& fixture) {
                 fixture.master->SpdControl(0.5f, nan);
             }},
            {"slave_current",
             [inf](PairedDoubleCoupleFixture& fixture) {
                 fixture.slave->CurControl(inf);
             }},
        };

    for (const auto& [name, action] : cases) {
        auto fixture =
            CreatePairedDoubleCoupleFixture("double_couple_nonfinite_no_feedback_" + name, ranges);

        try {
            action(fixture);
        } catch (const std::invalid_argument&) {
            ExpectStopMessagesWithZeroCurrent(fixture.adapter->GetCommandRecords(), 2u);
            continue;
        }
        FAIL() << "Expected std::invalid_argument for non-finite input: " << name;
    }
}

TEST(DoubleCoupleJointSafetyTest, InvalidInputDiscardsPendingPvtTargetAndGains) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("invalid_discards_pending", ranges);
    fixture.master->PVTControl(100.0f, 2.0f, 0.08f, 0.0f, 0.0f);
    EXPECT_THROW(
        fixture.slave->PVTControl(1.0f, 1.0f, std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f),
        std::invalid_argument);
    fixture.adapter->ClearCommandRecords();
    fixture.slave->PVTControl(0.0f, 0.0f, -0.03f, 0.0f, 0.0f);
    EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());
    fixture.slave->PVTControl(0.0f, 0.0f, -0.03f, 0.0f, 0.0f);
    const auto records = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(records.size(), 2u);
    const auto expected = CalculateExpectedMotorPositions(0.0, -0.03);
    EXPECT_NEAR(DecodePvtPosition(*FindSentMessage(records, 1), ranges), expected.motor1, 0.002f);
    EXPECT_NEAR(DecodePvtPosition(*FindSentMessage(records, 2), ranges), expected.motor2, 0.002f);
    EXPECT_NEAR(DecodePvtKp(*FindSentMessage(records, 1)), 0.0f, 0.2f);
    EXPECT_NEAR(DecodePvtKp(*FindSentMessage(records, 2)), 0.0f, 0.2f);
}

TEST(DoubleCoupleJointSafetyTest, RejectsUnencodableSpeedCurrentBeforeSendingEitherMotor) {
    auto fixture =
        CreatePairedDoubleCoupleFixture("unencodable_speed_current", CreateMotorRanges());
    EXPECT_THROW(fixture.master->SpdControl(0.1f, 7000.0f), std::invalid_argument);
    ExpectStopMessagesWithZeroCurrent(fixture.adapter->GetCommandRecords(), 2u);
}

TEST(DoubleCoupleJointSafetyTest, PositionChecksOtherAxisLimitInMixedPvtBatch) {
    const auto ranges = CreateMotorRanges();
    auto fixture =
        CreatePairedDoubleCoupleFixture("mixed_position_joint_limit", ranges, false, false,
                                        std::nullopt, std::nullopt, Range<float>{-0.1f, 0.1f});
    fixture.master->PVTControl(1, 1, 0.2f, 0, 0);
    EXPECT_TRUE(fixture.adapter->GetCommandRecords().empty());
    fixture.slave->PosControl(0.0f);
    ExpectPositionHoldMessages(fixture.adapter->GetCommandRecords(),
                               CalculateExpectedMotorPositions(0.0, 0.0), ranges);
}

TEST(DoubleCoupleJointSafetyTest, PositionRejectsMappedMotorSpeedBeforeEitherMotorIsSent) {
    auto fixture = CreatePairedDoubleCoupleFixture("position_motor_speed_range",
                                                   CreateMotorRanges(12.5f, 0.01f));
    fixture.master->PosControl(0.05f, 1.0f, 0.5f);
    EXPECT_THROW(fixture.slave->PosControl(0.0f, 1.0f, 0.5f), JointException);
    ExpectStopMessagesWithZeroCurrent(fixture.adapter->GetCommandRecords(), 2u);
}

TEST(DoubleCoupleJointSafetyTest, SpeedAndPositionRejectCurrentAboveJointTorqueLimit) {
    for (const bool position : {false, true}) {
        auto ranges = CreateMotorRanges();
        ranges.kt = 2.0f;
        auto fixture =
            CreatePairedDoubleCoupleFixture("current_cap_" + std::to_string(position), ranges);
        if (position) {
            EXPECT_THROW(fixture.master->PosControl(0.05f, 1.0f, 2.0f), std::invalid_argument);
        } else {
            EXPECT_THROW(fixture.master->SpdControl(0.5f, 2.0f), std::invalid_argument);
        }
        ExpectStopMessagesWithZeroCurrent(fixture.adapter->GetCommandRecords(), 2u);
    }
}

TEST(DoubleCoupleJointSafetyTest, StopSanitizesNonFiniteCurrent) {
    auto fixture = CreatePairedDoubleCoupleFixture("double_couple_stop_nonfinite_current",
                                                   CreateMotorRanges());

    fixture.master->Stop(MotorStopMode::DynamicBrake, std::numeric_limits<float>::infinity(), 1);
    ExpectStopMessagesWithZeroCurrent(fixture.adapter->GetCommandRecords(), 2u);

    fixture.adapter->ClearCommandRecords();
    fixture.slave->Stop(MotorStopMode::RegenerativeBrake, std::numeric_limits<float>::quiet_NaN(),
                        1);
    ExpectStopMessagesWithZeroCurrent(fixture.adapter->GetCommandRecords(), 2u);
}

TEST(DoubleCoupleJointCoordinateTest, MasterReversePvtUsesSolverNegativeRoll) {
    const auto ranges = CreateMotorRanges();
    auto fixture =
        CreatePairedDoubleCoupleFixture("double_master_reverse_pvt", ranges, true, false);

    fixture.master->PVTControl(10.0f, 1.0f, 0.05f, 0.3f, 0.2f);
    fixture.master->PVTControl(10.0f, 1.0f, 0.05f, 0.3f, 0.2f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    const auto expected = CalculateExpectedMotorPositions(-0.05, 0.0);
    EXPECT_NEAR(DecodePvtPosition(*motor1_message, ranges), expected.motor1, 0.002f);
    EXPECT_NEAR(DecodePvtPosition(*motor2_message, ranges), expected.motor2, 0.002f);
}

TEST(DoubleCoupleJointCoordinateTest, SlaveReversePvtUsesSolverNegativePitch) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("double_slave_reverse_pvt", ranges, false, true);

    fixture.slave->PVTControl(10.0f, 1.0f, 0.04f, 0.3f, 0.2f);
    fixture.slave->PVTControl(10.0f, 1.0f, 0.04f, 0.3f, 0.2f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    const auto expected = CalculateExpectedMotorPositions(0.0, -0.04);
    EXPECT_NEAR(DecodePvtPosition(*motor1_message, ranges), expected.motor1, 0.002f);
    EXPECT_NEAR(DecodePvtPosition(*motor2_message, ranges), expected.motor2, 0.002f);
}

TEST(DoubleCoupleJointCoordinateTest, ReverseStatusReturnsJointCoordinates) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("double_reverse_status", ranges, true, true);
    const auto positions = CalculateExpectedMotorPositions(-0.02, 0.03);
    EmitDoubleCoupleFeedback(fixture, positions);

    const auto master_status = fixture.master->GetStatus(false, 0);
    const auto slave_status = fixture.slave->GetStatus(false, 0);

    ASSERT_TRUE(master_status.has_value());
    ASSERT_TRUE(slave_status.has_value());
    EXPECT_NEAR(master_status->pos, 0.02f, 0.002f);
    EXPECT_NEAR(slave_status->pos, -0.03f, 0.002f);
}

TEST(DoubleCoupleJointCoordinateTest, MasterReverseDynamicControlsUseSolverNegativeRoll) {
    const auto ranges = CreateMotorRanges();
    auto fixture =
        CreatePairedDoubleCoupleFixture("master_reverse_dynamic_controls", ranges, true, false);

    fixture.master->PosControl(0.04f, 1.0f, 0.5f);
    fixture.master->PosControl(0.04f, 1.0f, 0.5f);
    {
        const auto& messages = fixture.adapter->GetCommandRecords();
        ASSERT_EQ(messages.size(), 2u);
        const auto* motor1_message = FindSentMessage(messages, 1);
        const auto* motor2_message = FindSentMessage(messages, 2);
        ASSERT_NE(motor1_message, nullptr);
        ASSERT_NE(motor2_message, nullptr);
        const auto expected_positions = CalculateExpectedMotorPositions(-0.04, 0.0);
        const auto expected_speeds =
            CalculateExpectedMotorSpeeds(DoubleCoupleJointAxis::Roll, -0.04, 0.0, -1.0f, ranges);
        EXPECT_NEAR(DecodePosPosition(*motor1_message), expected_positions.motor1, 0.002f);
        EXPECT_NEAR(DecodePosPosition(*motor2_message), expected_positions.motor2, 0.002f);
        EXPECT_NEAR(DecodePosSpeed(*motor1_message), std::abs(expected_speeds.motor1), 0.006f);
        EXPECT_NEAR(DecodePosSpeed(*motor2_message), std::abs(expected_speeds.motor2), 0.006f);
    }

    fixture.adapter->ClearCommandRecords();
    fixture.master->SpdControl(1.0f, 0.5f);
    fixture.slave->SpdControl(0.0f, 0.5f);
    auto expected_speeds =
        CalculateExpectedMotorSpeeds(DoubleCoupleJointAxis::Roll, -0.04, 0.0, -1.0f, ranges);
    ExpectSpeedCommands(fixture.adapter->GetCommandRecords(), expected_speeds.motor1,
                        expected_speeds.motor2);

    fixture.adapter->ClearCommandRecords();
    fixture.master->TorControl(1.0f);
    fixture.slave->TorControl(0.0f);
    auto expected_torques =
        CalculateExpectedMotorTorques(DoubleCoupleJointAxis::Roll, -0.04, 0.0, -1.0f, ranges);
    ExpectTorqueCommands(fixture.adapter->GetCommandRecords(), expected_torques.motor1,
                         expected_torques.motor2);

    fixture.adapter->ClearCommandRecords();
    fixture.master->CurControl(1.0f);
    fixture.slave->CurControl(0.0f);
    auto expected_currents =
        CalculateExpectedMotorCurrents(DoubleCoupleJointAxis::Roll, -0.04, 0.0, -1.0f, ranges);
    ExpectCurrentCommands(fixture.adapter->GetCommandRecords(), expected_currents.motor1,
                          expected_currents.motor2);
}

TEST(DoubleCoupleJointCoordinateTest, SlaveReverseDynamicControlsUseSolverNegativePitch) {
    const auto ranges = CreateMotorRanges();
    auto fixture =
        CreatePairedDoubleCoupleFixture("slave_reverse_dynamic_controls", ranges, false, true);

    fixture.slave->PosControl(0.04f, 1.0f, 0.5f);
    fixture.slave->PosControl(0.04f, 1.0f, 0.5f);
    {
        const auto& messages = fixture.adapter->GetCommandRecords();
        ASSERT_EQ(messages.size(), 2u);
        const auto* motor1_message = FindSentMessage(messages, 1);
        const auto* motor2_message = FindSentMessage(messages, 2);
        ASSERT_NE(motor1_message, nullptr);
        ASSERT_NE(motor2_message, nullptr);
        const auto expected_positions = CalculateExpectedMotorPositions(0.0, -0.04);
        const auto expected_speeds =
            CalculateExpectedMotorSpeeds(DoubleCoupleJointAxis::Pitch, 0.0, -0.04, -1.0f, ranges);
        EXPECT_NEAR(DecodePosPosition(*motor1_message), expected_positions.motor1, 0.002f);
        EXPECT_NEAR(DecodePosPosition(*motor2_message), expected_positions.motor2, 0.002f);
        EXPECT_NEAR(DecodePosSpeed(*motor1_message), std::abs(expected_speeds.motor1), 0.006f);
        EXPECT_NEAR(DecodePosSpeed(*motor2_message), std::abs(expected_speeds.motor2), 0.006f);
    }

    fixture.adapter->ClearCommandRecords();
    fixture.slave->SpdControl(1.0f, 0.5f);
    fixture.master->SpdControl(0.0f, 0.5f);
    auto expected_speeds =
        CalculateExpectedMotorSpeeds(DoubleCoupleJointAxis::Pitch, 0.0, -0.04, -1.0f, ranges);
    ExpectSpeedCommands(fixture.adapter->GetCommandRecords(), expected_speeds.motor1,
                        expected_speeds.motor2);

    fixture.adapter->ClearCommandRecords();
    fixture.slave->TorControl(1.0f);
    fixture.master->TorControl(0.0f);
    auto expected_torques =
        CalculateExpectedMotorTorques(DoubleCoupleJointAxis::Pitch, 0.0, -0.04, -1.0f, ranges);
    ExpectTorqueCommands(fixture.adapter->GetCommandRecords(), expected_torques.motor1,
                         expected_torques.motor2);

    fixture.adapter->ClearCommandRecords();
    fixture.slave->CurControl(1.0f);
    fixture.master->CurControl(0.0f);
    auto expected_currents =
        CalculateExpectedMotorCurrents(DoubleCoupleJointAxis::Pitch, 0.0, -0.04, -1.0f, ranges);
    ExpectCurrentCommands(fixture.adapter->GetCommandRecords(), expected_currents.motor1,
                          expected_currents.motor2);
}

TEST(DoubleCoupleJointControlCacheTest, GetStatusUpdatesCachedTargetFromFeedback) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("status_cache_adapter", ranges);
    const auto zero_positions = CalculateExpectedMotorPositions(0.0, 0.0);

    fixture.master->PVTControl(10.0f, 1.0f, 0.3f, 0.3f, 0.2f);
    fixture.slave->PVTControl(10.0f, 1.0f, 0.0f, 0.0f, 0.0f);
    EmitDoubleCoupleFeedback(fixture, zero_positions);
    const auto status = fixture.master->GetStatus(false, 0);
    ASSERT_TRUE(status.has_value());
    EXPECT_NEAR(status->pos, 0.0f, 0.002f);

    fixture.adapter->ClearCommandRecords();
    fixture.slave->PVTControl(10.0f, 1.0f, -0.04f, 0.2f, 0.1f);
    fixture.slave->PVTControl(10.0f, 1.0f, -0.04f, 0.2f, 0.1f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    const auto expected = CalculateExpectedMotorPositions(0.0, -0.04);
    EXPECT_NEAR(DecodePvtPosition(*motor1_message, ranges), expected.motor1, 0.002f);
    EXPECT_NEAR(DecodePvtPosition(*motor2_message, ranges), expected.motor2, 0.002f);
}

TEST(DoubleCoupleJointControlCacheTest, SetOnStatusUpdatesCachedTargetFromFeedback) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("callback_cache_adapter", ranges);
    const auto zero_positions = CalculateExpectedMotorPositions(0.0, 0.0);
    bool callback_called = false;
    JointStatus callback_status{};

    fixture.master->PVTControl(10.0f, 1.0f, 0.05f, 0.3f, 0.2f);
    fixture.slave->PVTControl(10.0f, 1.0f, 0.0f, 0.0f, 0.0f);
    fixture.master->SetOnStatus([&](const JointStatus& status) {
        callback_called = true;
        callback_status = status;
    });
    EmitDoubleCoupleFeedback(fixture, zero_positions);

    ASSERT_TRUE(callback_called);
    EXPECT_NEAR(callback_status.pos, 0.0f, 0.002f);

    fixture.adapter->ClearCommandRecords();
    fixture.slave->PVTControl(10.0f, 1.0f, -0.04f, 0.2f, 0.1f);
    fixture.slave->PVTControl(10.0f, 1.0f, -0.04f, 0.2f, 0.1f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    const auto expected = CalculateExpectedMotorPositions(0.0, -0.04);
    EXPECT_NEAR(DecodePvtPosition(*motor1_message, ranges), expected.motor1, 0.002f);
    EXPECT_NEAR(DecodePvtPosition(*motor2_message, ranges), expected.motor2, 0.002f);
}

TEST(DoubleCoupleJointControlCacheTest, SetOnStatusReturnsBothAxesWithAggregatedDiagnostics) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("dual_callback_adapter", ranges);
    const auto actual_positions = CalculateExpectedMotorPositions(0.02, -0.01);
    bool roll_called = false;
    bool pitch_called = false;
    JointStatus roll_status{};
    JointStatus pitch_status{};

    fixture.master->SetOnStatus([&](const JointStatus& status) {
        roll_called = true;
        roll_status = status;
    });
    fixture.slave->SetOnStatus([&](const JointStatus& status) {
        pitch_called = true;
        pitch_status = status;
    });
    EmitDoubleCoupleFeedback(fixture, actual_positions);

    ASSERT_TRUE(roll_called);
    ASSERT_TRUE(pitch_called);
    EXPECT_NEAR(roll_status.pos, 0.02f, 0.002f);
    EXPECT_NEAR(pitch_status.pos, -0.01f, 0.002f);
    EXPECT_NEAR(roll_status.motorTemperature, 25.0f, 0.001f);
    EXPECT_NEAR(pitch_status.motorTemperature, 25.0f, 0.001f);
    EXPECT_NEAR(roll_status.mosTemperature, 25.0f, 0.001f);
    EXPECT_NEAR(pitch_status.mosTemperature, 25.0f, 0.001f);
    EXPECT_EQ(roll_status.error, MotorError::NoError);
    EXPECT_EQ(pitch_status.error, MotorError::NoError);
}

TEST(DoubleCoupleJointControlCacheTest, IncompleteFeedbackDoesNotUpdateCachedTarget) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("incomplete_feedback_adapter", ranges);
    const auto zero_positions = CalculateExpectedMotorPositions(0.0, 0.0);
    bool callback_called = false;

    fixture.master->PVTControl(10.0f, 1.0f, 0.05f, 0.3f, 0.2f);
    fixture.master->SetOnStatus([&](const JointStatus&) {
        callback_called = true;
    });
    EmitMotor1Feedback(fixture, zero_positions.motor1);

    EXPECT_FALSE(callback_called);

    fixture.adapter->ClearCommandRecords();
    fixture.slave->PVTControl(10.0f, 1.0f, -0.04f, 0.2f, 0.1f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    const auto expected = CalculateExpectedMotorPositions(0.05, -0.04);
    EXPECT_NEAR(DecodePvtPosition(*motor1_message, ranges), expected.motor1, 0.002f);
    EXPECT_NEAR(DecodePvtPosition(*motor2_message, ranges), expected.motor2, 0.002f);
}

TEST(DoubleCoupleJointControlCacheTest, StatusFailureDoesNotUpdateCachedTarget) {
    const auto ranges = CreateMotorRanges();
    auto fixture = CreatePairedDoubleCoupleFixture("status_failure_cache_adapter", ranges);
    const auto feedback_positions = CalculateExpectedMotorPositions(0.02, -0.03);

    fixture.master->PVTControl(10.0f, 1.0f, 0.05f, 0.3f, 0.2f);
    EmitDoubleCoupleFeedback(fixture, feedback_positions);
    fixture.slave.reset();

    const auto status = fixture.master->GetStatus(false, 0);
    EXPECT_FALSE(status.has_value());

    fixture.adapter->ClearCommandRecords();
    fixture.master->PVTControl(10.0f, 1.0f, 0.06f, 0.2f, 0.1f);

    const auto& messages = fixture.adapter->GetCommandRecords();
    ASSERT_EQ(messages.size(), 2u);
    const auto* motor1_message = FindSentMessage(messages, 1);
    const auto* motor2_message = FindSentMessage(messages, 2);
    ASSERT_NE(motor1_message, nullptr);
    ASSERT_NE(motor2_message, nullptr);
    const auto expected = CalculateExpectedMotorPositions(0.06, 0.0);
    EXPECT_NEAR(DecodePvtPosition(*motor1_message, ranges), expected.motor1, 0.002f);
    EXPECT_NEAR(DecodePvtPosition(*motor2_message, ranges), expected.motor2, 0.002f);
}

// ============================================================
// Compile-time checks via SFINAE
// ============================================================

TEST(CompileTimeTest, CreateMotorExists) {
    MotorParams params{"missing_adapter", 0, 1, MotorInitAuto{}};

    EXPECT_THROW((void) JointManager<>::CreateMotor(params, AdapterMap{}), std::invalid_argument);
}

TEST(CompileTimeTest, MasterJointSupportsWeakCallbackOwnership) {
    EXPECT_TRUE((std::is_base_of_v<std::enable_shared_from_this<MasterJoint>, MasterJoint>) );
}

// ============================================================
// JointManager<> AddJoint: duplicate name check
// ============================================================

TEST(JointManagerBaseTest, AddJointDuplicateNameIsPureLogic) {
    TestableJointManager manager;
    auto pending_config = CreateDoubleCoupleJointConfig(DoubleCoupleJointAxis::Roll);
    ASSERT_NE(manager.AddJoint<DoubleCoupleJoint>("duplicate_joint", pending_config), nullptr);

    RJoint::Config duplicate_config{};
    duplicate_config.motorParams = MotorParams{"missing_adapter", 0, 1, MotorInitAuto{}};
    duplicate_config.limit = Range<float>{-1.0f, 1.0f};
    duplicate_config.maxSpd = 2.0f;
    duplicate_config.maxTor = 3.0f;
    duplicate_config.reverse = false;

    try {
        (void) manager.AddJoint<RJoint>("duplicate_joint", duplicate_config);
    } catch (const std::invalid_argument& ex) {
        EXPECT_NE(std::string(ex.what()).find("already exists"), std::string::npos);
        return;
    }
    FAIL() << "Duplicate name should be rejected before adapter or motor lookup";
}

TEST(JointManagerBaseTest, GetStatusPropagatesUnhandledJointException) {
    JointManager<> manager;
    auto config = CreateDoubleCoupleJointConfig(DoubleCoupleJointAxis::Roll);
    manager.AddJoint<DoubleCoupleJoint>("pending_joint", config);

    EXPECT_THROW((void) manager.GetStatus(), JointException);
}

TEST(JointManagerBaseTest, PosControlPropagatesUnhandledJointException) {
    JointManager<> manager;
    auto config = CreateDoubleCoupleJointConfig(DoubleCoupleJointAxis::Roll);
    manager.AddJoint<DoubleCoupleJoint>("pending_joint", config);
    const std::map<std::string, float> positions{{"pending_joint", 0.1f}};

    EXPECT_THROW(manager.PosControl(positions), JointException);
}

TEST_F(MakePairKeyTest, DistinguishesIndexFromNumericName) {
    a.adapterId = std::uint32_t{0};
    b.adapterId = std::uint32_t{0};
    const auto indexed = DoubleCoupleJoint::MakePairKey(a, b);
    a.adapterId = "0";
    b.adapterId = "0";
    EXPECT_NE(indexed, DoubleCoupleJoint::MakePairKey(a, b));
}

TEST_F(MakePairKeyTest, SeparatorsInNamesCannotShiftPairBoundary) {
    a.adapterId = "x";
    b.adapterId = "y/0/1|z";
    const auto first = DoubleCoupleJoint::MakePairKey(a, b);
    a.adapterId = "x/0/1|y";
    b.adapterId = "z";
    EXPECT_NE(first, DoubleCoupleJoint::MakePairKey(a, b));
}

TEST(JointManagerLogTest, NumericNameAndIndexTrackAndLogBothMotors) {
    TemporaryDirectory directory;
    TestableJointManager manager;
    auto left = CreateFakeAdapterHarness("typed_left");
    auto zero = CreateFakeAdapterHarness("typed_zero");
    manager.TrackTestAdapter("left", left->Adapter());
    manager.TrackTestAdapter("0", zero->Adapter());
    RJoint::Config config{};
    config.motorParams = MotorParams{std::uint32_t{0}, 0, 1, MotorInitParams{CreateMotorRanges()}};
    config.limit = Range<float>{-1.0f, 1.0f};
    config.maxSpd = 2.0f;
    config.maxTor = 3.0f;
    manager.AddJoint<RJoint>("indexed", config);
    config.motorParams.adapterId = "0";
    manager.AddJoint<RJoint>("named", config);
    EXPECT_EQ(manager.TrackedMotorCount(), 2u);
    manager.EnableLog(directory.Path());
    EXPECT_TRUE(left->Adapter()->GetBus(0)->GetMotor(1)->IsLogged());
    EXPECT_TRUE(zero->Adapter()->GetBus(0)->GetMotor(1)->IsLogged());
    manager.DisableLog();
    std::size_t files = 0;
    for (const auto& entry : std::filesystem::directory_iterator(directory.Path())) {
        if (entry.is_regular_file())
            ++files;
    }
    EXPECT_EQ(files, 9u);
    EXPECT_TRUE(std::filesystem::exists(directory.Path() / "indexed_command.csv.zstd"));
    EXPECT_TRUE(std::filesystem::exists(directory.Path() / "indexed_status.csv.zstd"));
    EXPECT_TRUE(std::filesystem::exists(directory.Path() / "named_command.csv.zstd"));
    EXPECT_TRUE(std::filesystem::exists(directory.Path() / "named_status.csv.zstd"));
    EXPECT_TRUE(std::filesystem::exists(directory.Path() / "joint_protection.csv.zstd"));
}

TEST(DoubleCoupleJointDynamicBatchTest, DefaultCurrentPreservesExplicitLimit) {
    for (bool position : {false, true}) {
        for (bool same_axis : {false, true}) {
            for (const auto limits : {std::pair{0.3f, 0.0f}, std::pair{0.0f, 0.3f},
                                      std::pair{0.0f, 0.0f}, std::pair{0.8f, 0.3f}}) {
                SCOPED_TRACE(::testing::Message() << position << "," << same_axis << ","
                                                  << limits.first << "," << limits.second);
                auto fixture = CreatePairedDoubleCoupleFixture("default_current_regression",
                                                               CreateMotorRanges());
                BaseJoint* second = same_axis ? static_cast<BaseJoint*>(fixture.master.get())
                                              : static_cast<BaseJoint*>(fixture.slave.get());
                if (position) {
                    fixture.master->PosControl(0.01f, 1.0f, limits.first);
                    second->PosControl(0.02f, 1.0f, limits.second);
                } else {
                    fixture.master->SpdControl(1.0f, limits.first);
                    second->SpdControl(0.5f, limits.second);
                }
                // 同轴速度调用采用最新值；位置调用则合并本批次约束。
                const float expected = same_axis && !position ? limits.second
                                       : limits.first == 0.0f ? limits.second
                                       : limits.second == 0.0f
                                           ? limits.first
                                           : std::min(limits.first, limits.second);
                const auto& records = fixture.adapter->GetCommandRecords();
                ASSERT_EQ(records.size(), 2u);
                for (const auto& record : records) {
                    const float actual =
                        position ? std::get<FakePosControlPayload>(record.payload).current
                                 : std::get<FakeSpdControlPayload>(record.payload).current;
                    EXPECT_NEAR(actual, expected, 0.001f);
                }
            }
        }
    }
}

TEST(JointManagerPairingTest, NumericNameAndIndexPairIndependently) {
    TestableJointManager manager;
    auto left = CreateFakeAdapterHarness("pair_typed_left");
    auto zero = CreateFakeAdapterHarness("pair_typed_zero");
    manager.TrackTestAdapter("left", left->Adapter());
    manager.TrackTestAdapter("0", zero->Adapter());
    auto indexed =
        CreateDoubleCoupleJointConfigWithRanges(DoubleCoupleJointAxis::Roll, CreateMotorRanges());
    indexed.motorParams1.adapterId = std::uint32_t{0};
    indexed.motorParams2.adapterId = std::uint32_t{0};
    auto named = indexed;
    named.motorParams1.adapterId = "0";
    named.motorParams2.adapterId = "0";
    ASSERT_NO_THROW(manager.AddJoint<DoubleCoupleJoint>("index_roll", indexed));
    ASSERT_NO_THROW(manager.AddJoint<DoubleCoupleJoint>("name_roll", named));
    left->SetReplyMode(FakeReplyMode::Automatic);
    zero->SetReplyMode(FakeReplyMode::Automatic);
    indexed.axis = named.axis = DoubleCoupleJointAxis::Pitch;
    ASSERT_NO_THROW(manager.AddJoint<DoubleCoupleJoint>("index_pitch", indexed));
    ASSERT_NO_THROW(manager.AddJoint<DoubleCoupleJoint>("name_pitch", named));
    EXPECT_EQ(manager.TrackedMotorCount(), 4u);
}

TEST(DoubleCoupleJointDynamicBatchTest, OmittedCurrentPreservesCachedOtherAxisLimit) {
    auto fixture = CreatePairedDoubleCoupleFixture("cached_default_limit", CreateMotorRanges());
    fixture.slave->SpdControl(0.5f, 0.3f);
    fixture.master->SpdControl(1.0f);
    for (int batch = 0; batch < 2; ++batch) {
        const auto& records = fixture.adapter->GetCommandRecords();
        ASSERT_EQ(records.size(), 2u);
        for (const auto& record : records) {
            EXPECT_NEAR(std::get<FakeSpdControlPayload>(record.payload).current, 0.3f, 0.001f);
        }
        fixture.adapter->ClearCommandRecords();
        fixture.master->SpdControl(0.8f);
        fixture.master->SpdControl(0.6f);
    }
}
