// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

#include <atomic>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#if ENCOS_ENABLE_SPDLOG
#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>
#endif

#include "JointManager.h"
#include "fake_adapter_test_support.h"

namespace encos {
namespace {

enum class BrakeJointKind { Rotary, Continuous, Master, Slave };

MotorProperties BrakeProperties(bool supported) {
    auto properties = GetMotorModelRanges(MotorModel::EC_A4310_P2);
    properties.brake = supported;
    return properties;
}

DoubleCoupleJoint::Config BrakeCoupledConfig() {
    DoubleCoupleJoint::Config config{};
    config.motorParams1 = MotorParams{"brake", 0, 1, MotorInitParams{BrakeProperties(true)}};
    config.motorParams2 = MotorParams{"brake", 0, 2, MotorInitParams{BrakeProperties(true)}};
    config.axis = DoubleCoupleJointAxis::Roll;
    config.limit = {-1.0F, 1.0F};
    config.maxSpd = 2.0F;
    config.maxTor = 3.0F;
    config.calibration.left_joint_center = {-35.462080740, 29.491263548, 58.546474855};
    config.calibration.right_joint_center = {-35.462080740, -29.491263548, 58.546474855};
    config.calibration.left_planar_endpoint = {-35.5, 11.61629736, 56.42889465};
    config.calibration.right_planar_endpoint = {-35.5, -11.61629736, 56.42889465};
    config.calibration.roll_center = {0.0, 0.0, 0.0};
    config.calibration.pitch_center = {0.0, 0.0, 0.0};
    config.calibration.left_spatial_endpoint = {-36.6, 26.0, -5.0};
    config.calibration.right_spatial_endpoint = {-36.6, -26.0, -5.0};
    config.calibration.rotation_order = couple_joint::RotationOrder::PitchThenRoll;
    return config;
}

class BrakeJointTest : public testing::TestWithParam<BrakeJointKind> {
protected:
    void Create(bool motor1_supported = true, bool motor2_supported = true) {
        static std::atomic<int> next_id{0};
        adapter_ = CreateFakeAdapterHarness("brake_joint_" + std::to_string(next_id.fetch_add(1)));
        adapter_->SetReplyMode(FakeReplyMode::Automatic);
        auto* bus = adapter_->Adapter()->GetBus(0);
        motor1_ = bus->GetMotor(1, BrakeProperties(motor1_supported));
        if (GetParam() == BrakeJointKind::Rotary) {
            RJoint::Config config{};
            config.limit = {-1.0F, 1.0F};
            config.maxSpd = 2.0F;
            config.maxTor = 3.0F;
            joint_ = std::make_shared<RJoint>(config, motor1_);
        } else if (GetParam() == BrakeJointKind::Continuous) {
            ContinuousJoint::Config config{};
            config.maxSpd = 2.0F;
            config.maxTor = 3.0F;
            joint_ = std::make_shared<ContinuousJoint>(config, motor1_);
        } else {
            motor2_ = bus->GetMotor(2, BrakeProperties(motor2_supported));
            auto config = BrakeCoupledConfig();
            auto slave =
                std::static_pointer_cast<SlaveJoint>(DoubleCoupleJoint::CreateJoint(config));
            config.axis = DoubleCoupleJointAxis::Pitch;
            auto master = DoubleCoupleJoint::CreateJoint(config, motor1_, motor2_, slave);
            joint_ = GetParam() == BrakeJointKind::Master ? master : slave;
            paired_joint_ = GetParam() == BrakeJointKind::Master ? slave : master;
        }
        joint_->SetProtectionLogContext(JointSdkLogger::Get(), "brake_test_joint");
        adapter_->ClearCommandRecords();
    }

    void TearDown() override {
        if (adapter_) {
            adapter_->Control().ClearDecodedCommandObserver();
            joint_.reset();
            paired_joint_.reset();
            EXPECT_TRUE(DeleteAdapter(adapter_->Adapter()));
        }
    }

    bool IsCoupled() const {
        return GetParam() == BrakeJointKind::Master || GetParam() == BrakeJointKind::Slave;
    }

    void ExpectBrakeCommands(bool enabled) {
        const auto records = adapter_->SnapshotCommandRecords();
        ASSERT_EQ(records.size(), IsCoupled() ? 2U : 1U);
        for (std::size_t i = 0; i < records.size(); ++i) {
            EXPECT_EQ(records[i].kind, FakeCommandKind::Brake);
            EXPECT_EQ(records[i].motor_idx, static_cast<int>(i + 1));
            EXPECT_EQ(std::get<FakeBrakePayload>(records[i].payload).enabled, enabled);
        }
    }

    std::shared_ptr<FakeAdapterHarness> adapter_;
    BaseJointPtr joint_;
    BaseJointPtr paired_joint_;
    Motor* motor1_{nullptr};
    Motor* motor2_{nullptr};
};

TEST_P(BrakeJointTest, CapabilityQuerySendsNothing) {
    Create();
    EXPECT_TRUE(joint_->HasBrake());
    EXPECT_TRUE(adapter_->SnapshotCommandRecords().empty());
}

TEST_P(BrakeJointTest, BaseDefaultsDoNotOperateMotors) {
    Create(false, false);
    EXPECT_FALSE(joint_->BaseJoint::HasBrake());
    EXPECT_FALSE(joint_->BaseJoint::Brake(true));
    EXPECT_FALSE(joint_->BaseJoint::Brake(false));
    EXPECT_FALSE(joint_->BaseJoint::GetBrakeStatus());
    EXPECT_TRUE(adapter_->SnapshotCommandRecords().empty());
}

TEST_P(BrakeJointTest, ControlsEntireJointAndReturnsDriverResult) {
    Create();
    for (const bool enabled : {true, false}) {
        adapter_->ClearCommandRecords();
        EXPECT_TRUE(joint_->Brake(enabled));
        ExpectBrakeCommands(enabled);
        EXPECT_EQ(adapter_->Control().GetMotorSnapshot(0, 1).brake_enabled, enabled);
        if (IsCoupled()) {
            EXPECT_EQ(adapter_->Control().GetMotorSnapshot(0, 2).brake_enabled, enabled);
        }
    }
}

TEST_P(BrakeJointTest, UnsupportedMotorRejectsAllBrakeOperationsWithoutCommands) {
    Create(false, false);
    EXPECT_FALSE(joint_->HasBrake());
    EXPECT_FALSE(joint_->Brake(true));
    EXPECT_FALSE(joint_->Brake(false));
    EXPECT_FALSE(joint_->GetBrakeStatus());
    EXPECT_TRUE(adapter_->SnapshotCommandRecords().empty());
}

TEST_P(BrakeJointTest, DriverFailureReturnsFalseWithoutAdditionalCommands) {
    Create();
    adapter_->SetReplyMode(FakeReplyMode::Manual);
    for (const bool enabled : {true, false}) {
        adapter_->ClearCommandRecords();
        EXPECT_FALSE(joint_->Brake(enabled));
        ExpectBrakeCommands(enabled);
    }
}

TEST_P(BrakeJointTest, QueryUsesProtocolStateAndRepairsOnlyReleasedMotor) {
    Create();
    for (const bool first_engaged : {false, true}) {
        for (const bool second_engaged : {false, true}) {
            ASSERT_TRUE(motor1_->Brake(first_engaged));
            if (motor2_) {
                ASSERT_TRUE(motor2_->Brake(second_engaged));
            }
            adapter_->ClearCommandRecords();
            EXPECT_EQ(joint_->GetBrakeStatus(), first_engaged || (IsCoupled() && second_engaged));
            const bool repair = IsCoupled() && first_engaged != second_engaged;
            const auto records = adapter_->SnapshotCommandRecords();
            const std::size_t queries = IsCoupled() ? 2U : 1U;
            ASSERT_EQ(records.size(), queries + (repair ? 1U : 0U));
            for (std::size_t i = 0; i < queries; ++i) {
                EXPECT_EQ(records[i].kind, FakeCommandKind::GetParameter);
                EXPECT_EQ(records[i].motor_idx, static_cast<int>(i + 1));
                EXPECT_EQ(std::get<FakeGetParameterPayload>(records[i].payload).parameter,
                          MotorParameter::BrakeStatus);
            }
            if (repair) {
                EXPECT_EQ(records.back().kind, FakeCommandKind::Brake);
                EXPECT_EQ(records.back().motor_idx, first_engaged ? 2 : 1);
                EXPECT_TRUE(std::get<FakeBrakePayload>(records.back().payload).enabled);
                EXPECT_TRUE(adapter_->Control().GetMotorSnapshot(0, 1).brake_enabled);
                EXPECT_TRUE(adapter_->Control().GetMotorSnapshot(0, 2).brake_enabled);
            }
        }
    }
}

TEST_P(BrakeJointTest, DriverQueryExceptionPropagates) {
    Create();
    adapter_->SetReplyMode(FakeReplyMode::Manual);
    EXPECT_THROW(joint_->GetBrakeStatus(), std::runtime_error);
}

TEST_P(BrakeJointTest, MixedCapabilityRejectsCoupledControl) {
    if (!IsCoupled()) {
        GTEST_SKIP() << "Only coupled joints have mixed capabilities";
    }
    Create(GetParam() == BrakeJointKind::Master, GetParam() != BrakeJointKind::Master);
    EXPECT_FALSE(joint_->HasBrake());
    EXPECT_FALSE(paired_joint_->HasBrake());
    EXPECT_FALSE(joint_->Brake(true));
    EXPECT_FALSE(joint_->Brake(false));
    EXPECT_FALSE(joint_->GetBrakeStatus());
    EXPECT_TRUE(adapter_->SnapshotCommandRecords().empty());
}

TEST_P(BrakeJointTest, EitherMotorFailureStillCallsBothMotorsAndDoesNotRollBack) {
    if (!IsCoupled()) {
        GTEST_SKIP() << "Only coupled joints have two results";
    }
    Create();
    for (const bool first_fails : {false, true}) {
        for (const bool enabled : {true, false}) {
            std::vector<FakeCommandRecord> records;
            adapter_->SetReplyMode(first_fails ? FakeReplyMode::Manual : FakeReplyMode::Automatic);
            adapter_->Control().SetDecodedCommandObserver([&](const FakeCommandRecord& record) {
                records.push_back(record);
                if (record.motor_idx == 1) {
                    adapter_->SetReplyMode(first_fails ? FakeReplyMode::Automatic
                                                       : FakeReplyMode::Manual);
                }
            });
            EXPECT_FALSE(joint_->Brake(enabled));
            adapter_->Control().ClearDecodedCommandObserver();
            ASSERT_EQ(records.size(), 2U);
            EXPECT_EQ(records[0].motor_idx, 1);
            EXPECT_EQ(records[1].motor_idx, 2);
            for (const auto& record : records) {
                EXPECT_EQ(record.kind, FakeCommandKind::Brake);
                EXPECT_EQ(std::get<FakeBrakePayload>(record.payload).enabled, enabled);
            }
        }
    }
}

TEST_P(BrakeJointTest, FailedRepairStillReportsEngagedAndLogsMotor) {
    if (!IsCoupled()) {
        GTEST_SKIP() << "Only coupled joints reconcile states";
    }
    Create();
    ASSERT_TRUE(motor1_->Brake(true));
    ASSERT_TRUE(motor2_->Brake(false));
    std::vector<FakeCommandRecord> records;
    adapter_->Control().SetDecodedCommandObserver([&](const FakeCommandRecord& record) {
        records.push_back(record);
        if (record.kind == FakeCommandKind::GetParameter && record.motor_idx == 2) {
            adapter_->SetReplyMode(FakeReplyMode::Manual);
        }
    });
#if ENCOS_ENABLE_SPDLOG
    std::ostringstream output;
    const auto logger_name = "brake_repair_" + std::to_string(static_cast<int>(GetParam()));
    auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(output);
    auto backend = std::make_shared<spdlog::logger>(logger_name, sink);
    spdlog::register_logger(backend);
    auto logger = CreateLogger(logger_name);
    joint_->SetProtectionLogContext(logger, "brake_pair");
    paired_joint_->SetProtectionLogContext(logger, "brake_pair");
#endif
    EXPECT_TRUE(joint_->GetBrakeStatus());
    adapter_->Control().ClearDecodedCommandObserver();
    ASSERT_EQ(records.size(), 3U);
    EXPECT_EQ(records.back().kind, FakeCommandKind::Brake);
    EXPECT_EQ(records.back().motor_idx, 2);
    EXPECT_FALSE(adapter_->Control().GetMotorSnapshot(0, 2).brake_enabled);
#if ENCOS_ENABLE_SPDLOG
    EXPECT_NE(output.str().find("brake_pair"), std::string::npos);
    EXPECT_NE(output.str().find("电机2补抱闸失败"), std::string::npos);
    spdlog::drop(logger_name);
#endif
}

#if ENCOS_ENABLE_SPDLOG
TEST_P(BrakeJointTest, UnsupportedWarningIdentifiesJoint) {
    Create(false, false);
    std::ostringstream output;
    const auto logger_name = "brake_unsupported_" + std::to_string(static_cast<int>(GetParam()));
    auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(output);
    auto backend = std::make_shared<spdlog::logger>(logger_name, sink);
    spdlog::register_logger(backend);
    auto logger = CreateLogger(logger_name);
    joint_->SetProtectionLogContext(logger, "unsupported_joint");
    if (paired_joint_) {
        paired_joint_->SetProtectionLogContext(logger, "unsupported_joint");
    }
    EXPECT_FALSE(joint_->Brake(true));
    EXPECT_NE(output.str().find("unsupported_joint"), std::string::npos);
    EXPECT_NE(output.str().find("The current joint does not use brake-equipped motors"),
              std::string::npos);
    spdlog::drop(logger_name);
}
#endif

INSTANTIATE_TEST_SUITE_P(AllJointTypes, BrakeJointTest,
                         testing::Values(BrakeJointKind::Rotary, BrakeJointKind::Continuous,
                                         BrakeJointKind::Master, BrakeJointKind::Slave));

TEST(BrakeJointPairTest, UnpairedSlavePreservesPairingError) {
    auto slave = DoubleCoupleJoint::CreateJoint(BrakeCoupledConfig());
    EXPECT_THROW(slave->HasBrake(), JointException);
    EXPECT_THROW(slave->Brake(true), JointException);
    EXPECT_THROW(slave->Brake(false), JointException);
    EXPECT_THROW(slave->GetBrakeStatus(), JointException);
}

}  // namespace
}  // namespace encos
