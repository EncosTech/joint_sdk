// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

#pragma once

#ifndef ENCOS_SLAVE_JOINT_H_
#define ENCOS_SLAVE_JOINT_H_

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <utility>

#include "DoubleCoupleJoint/MasterJoint.h"

namespace encos {

/**
 * @brief 双电机关节从关节。
 *
 * SlaveJoint 是 DoubleCoupleJoint 的从动侧代理关节，不持有实际电机资源。
 * 除 Stop() 外的所有控制操作均转发至配对的 MasterJoint 的 *ForSlave 方法，
 * 由 MasterJoint 统一完成双电机运动学解算后再下发。
 */
class SlaveJoint : public BaseJoint {
public:
    /**
     * @brief 将当前位置设为零点。
     * @return true 表示成功，false 表示失败。
     * @note 委托给配对的 MasterJoint::SetZero() 执行。
     */
    bool SetZero() override {
        EnsureBound();
        return master_->SetZero();
    }

    /**
     * @brief 设置关节限位。
     * @param limit 限位范围（关节坐标系）。
     * @note 与 MasterJoint 同理，若 reverse 为 true 则内部翻转存储。
     */
    void SetLimit(Range<float> limit) override {
        SetValidatedLimit(limit);
    }

    /**
     * @brief PVT 位置-速度-力矩控制。
     * @param kp 位置比例增益。
     * @param kd 速度微分增益。
     * @param pos 目标位置（关节坐标系）。
     * @param spd 目标速度（关节坐标系）。
     * @param torque 目标力矩。
     * @note 委托给 MasterJoint::PVTControlForSlave()，由 MasterJoint 解算为双电机指令后下发。
     */
    void PVTControl(float kp, float kd, float pos, float spd, float torque) override {
        PVTControl(kp, kd, pos, spd, torque, PVTControlMode::MotorPVT);
    }

    /**
     * @brief 转发带上位机力矩模式选择的从轴 PVT 控制。
     * @param torque_mode PVT 解算模式；同一批次两轴模式一致时启用上位机解算。
     */
    void PVTControl(float kp, float kd, float pos, float spd, float torque,
                    PVTControlMode torque_mode) override {
        EnsureBound();
        master_->PVTControlForSlave(kp, kd, pos, spd, torque, torque_mode);
    }

    /**
     * @brief 位置控制。
     * @param position 目标位置（关节坐标系）。
     * @param speed 目标速度。
     * @param current 目标电流。
     * @param feedback 反馈源选择。
     * @note 委托给 MasterJoint::PosControlForSlave()，由 MasterJoint 解算为双电机指令后下发。
     */
    void PosControl(float position, float speed = 0, float current = 0, int feedback = 1) override {
        EnsureBound();
        master_->PosControlForSlave(position, speed, current, feedback);
    }

    /**
     * @brief 速度控制。
     * @param speed 目标速度（关节坐标系）。
     * @param current 目标电流。
     * @param feedback 反馈源选择。
     * @note 委托给 MasterJoint::SpdControlForSlave()，由 MasterJoint 解算为双电机指令后下发。
     */
    void SpdControl(float speed, float current = 0, int feedback = 1) override {
        EnsureBound();
        master_->SpdControlForSlave(speed, current, feedback);
    }

    /**
     * @brief 电流控制。
     * @param current 目标电流。
     * @param feedback 反馈源选择。
     * @note 委托给 MasterJoint::CurControlForSlave()，由 MasterJoint 解算为双电机指令后下发。
     */
    void CurControl(float current = 0, int feedback = 1) override {
        EnsureBound();
        master_->CurControlForSlave(current, feedback);
    }

    /**
     * @brief 力矩控制。
     * @param torque 目标力矩。
     * @param feedback 反馈源选择。
     * @note 委托给 MasterJoint::TorControlForSlave()，由 MasterJoint 解算为双电机指令后下发。
     */
    void TorControl(float torque = 0, int feedback = 1) override {
        EnsureBound();
        master_->TorControlForSlave(torque, feedback);
    }

    /**
     * @brief 停止电机。
     * @param mode 停止模式。
     * @param current 停止电流。
     * @param feedback 反馈源选择。
     * @note 委托给 MasterJoint::Stop() 执行，MasterJoint 会同时停止 Master 和 Slave 两侧电机。
     */
    void Stop(MotorStopMode mode, float current = 0, int feedback = 1) override {
        EnsureBound();
        master_->Stop(mode, current, feedback);
    }

    /**
     * @brief 运行到限位（未实现）。
     * @param dir 方向。
     * @param spd 运行速度。
     * @param cur 运行电流。
     * @param timeout 超时时间。
     * @return 始终返回 NaN。
     * @note 双电机关节的限位运行尚未实现，当前调用将直接返回 NaN。
     */
    float GotoLimit([[maybe_unused]] int dir, [[maybe_unused]] float spd = 2.f,
                    [[maybe_unused]] float cur = 2.f,
                    [[maybe_unused]] std::chrono::milliseconds timeout = std::chrono::milliseconds{
                        5000}) override {
        return std::nanf("");
    }

    /**
     * @brief 回到零点（未实现）。
     * @param offset 零点偏移。
     * @param spd 运行速度。
     * @param cur 运行电流。
     * @param timeout 超时时间。
     * @return 始终返回 false。
     * @note 双电机关节的回零操作尚未实现，当前调用将直接返回 false。
     */
    bool GotoZero([[maybe_unused]] float offset, [[maybe_unused]] float spd = 2.f,
                  [[maybe_unused]] float cur = 2.f,
                  [[maybe_unused]] std::chrono::milliseconds timeout = std::chrono::milliseconds{
                      5000}) override {
        return false;
    }

    /**
     * @brief 将当前位置记录为偏移量（未实现）。
     * @param dir 方向。
     * @return 始终返回 NaN。
     * @note 双电机关节的偏移量记录尚未实现，当前调用将直接返回 NaN。
     */
    float PosAsOffset([[maybe_unused]] int dir) override {
        return std::nanf("");
    }

    /**
     * @brief 校准（未实现）。
     * @param dir 方向。
     * @param spd 校准速度。
     * @param cur 校准电流。
     * @param timeout 超时时间。
     * @return 始终返回 false。
     * @note 双电机关节的校准尚未实现，当前调用将直接返回 false。
     */
    bool Calibrate([[maybe_unused]] int dir, [[maybe_unused]] float spd = 2.f,
                   [[maybe_unused]] float cur = 2.f,
                   [[maybe_unused]] std::chrono::milliseconds timeout = std::chrono::milliseconds{
                       5000}) override {
        return false;
    }

    /**
     * @brief 获取关节状态。
     * @param autoSendCommand 是否自动发送命令。
     * @param lifeCycleDeduction 生命周期扣减。
     * @return 关节状态，包含双电机数据经运动学合成后的结果。
     * @note 委托给 MasterJoint::GetStatusForSlave()，由 MasterJoint 读取双电机状态后
     *       合成从关节侧的关节状态返回。
     * @warning 关节运动过程中不得将 autoSendCommand 设为 true；内部零目标 PVT 反馈命令
     *          可能改变双电机关节运动状态。
     */
    std::optional<JointStatus> GetStatus(bool autoSendCommand = false,
                                         [[maybe_unused]] int lifeCycleDeduction = 1) override {
        EnsureBound();
        return master_->GetStatusForSlave(autoSendCommand, 0);
    }

    /**
     * @brief 注册状态回调。
     * @param onStatus 状态到达时调用的回调函数。
     * @note 委托给 MasterJoint::SetOnStatusForSlave()，回调触发时携带从关节侧的合成状态。
     */
    void SetOnStatus(std::function<void(const JointStatus&)> onStatus) override {
        if (!onStatus) {
            if (master_) {
                master_->SetOnStatusForSlave(nullptr);
            }
            return;
        }
        EnsureBound();
        master_->SetOnStatusForSlave(std::move(onStatus));
    }

private:
    friend struct DoubleCoupleJoint;
    friend class MasterJoint;

    /**
     * @brief 构造函数。
     * @param config 双电机关节配置，包含 axis、calibration 等参数。
     * @note limit、maxSpd、maxTor、reverse 均从 config 透传至 BaseJoint。
     */
    explicit SlaveJoint(const DoubleCoupleJoint::Config& config)
        : BaseJoint{config.limit, config.maxSpd, config.maxTor, config.reverse},
          axis_(config.axis),
          calibration_(config.calibration),
          motor_limit1_(config.motorLimit1),
          motor_limit2_(config.motorLimit2) {}

    /**
     * @brief 绑定配对的 MasterJoint。
     * @param master 待绑定的 MasterJoint 共享指针。
     * @note 由 DoubleCoupleJoint::CreateJoint() 在创建 MasterJoint 时调用。
     *       绑定后会将本关节的 axis_ 注册到 MasterJoint 的从轴侧。
     */
    void BindMaster(std::shared_ptr<MasterJoint> master) {
        master_ = std::move(master);
        master_->BindSlaveAxis(axis_);
    }

    /**
     * @brief 确保已绑定 MasterJoint，否则抛出异常。
     * @throw JointException 当 master_ 为空时抛出 kDoubleCoupleJointNotPaired 错误。
     * @note 所有转发到 MasterJoint 的 public 方法在调用前均需调用此检查。
     */
    void EnsureBound() const {
        if (!master_) {
            throw JointException(JointError::kDoubleCoupleJointNotPaired,
                                 "SlaveJoint 未绑定 MasterJoint，请先完成配对");
        }
    }

    /** 配对的 MasterJoint 共享指针。 */
    std::shared_ptr<MasterJoint> master_;
    /** 从关节所属轴线（Roll 或 Pitch）。 */
    DoubleCoupleJointAxis axis_{DoubleCoupleJointAxis::Roll};
    /** 校准参数，用于运动学解算。 */
    couple_joint::CalibrationInput calibration_;
    /** 配对前暂存的电机 1 相对位置限位。 */
    std::optional<Range<float>> motor_limit1_;
    /** 配对前暂存的电机 2 相对位置限位。 */
    std::optional<Range<float>> motor_limit2_;
};

/**
 * @brief 获取 master 或 slave 轴的运行时参数。
 * @param axis 需要查询的输出轴。
 * @return 该轴对应的限位、速度、扭矩和 reverse 配置。
 * @throw JointException 当查询从轴但从关节未绑定时抛出。
 */
inline MasterJoint::AxisRuntimeParams MasterJoint::AxisParams(DoubleCoupleJointAxis axis) const {
    if (axis == axis_) {
        return AxisRuntimeParams{limit_, maxSpd_, maxTor_, reverse_};
    }
    const auto slave = slave_.lock();
    if (!slave) {
        throw JointException(JointError::kDoubleCoupleJointNotPaired,
                             "DoubleCoupleJoint slave joint is not bound");
    }
    return AxisRuntimeParams{slave->limit_, slave->maxSpd_, slave->maxTor_, slave->reverse_};
}

/**
 * @brief 校验 master 配置与待配对 slave 的轴和标定是否匹配。
 * @param config master 侧双电机关节配置。
 * @param slave 待配对的 slave 关节。
 * @throw JointException 当 slave 缺失、轴重复或标定不一致时抛出。
 */
inline void DoubleCoupleJoint::ValidatePair(const Config& config,
                                            const std::shared_ptr<SlaveJoint>& slave) {
    if (!slave) {
        throw JointException(JointError::kDoubleCoupleJointNotPaired,
                             "DoubleCoupleJoint master creation requires a pending slave");
    }
    if (config.axis == slave->axis_) {
        throw JointException(
            JointError::kDoubleCoupleJointDuplicateAxis,
            "DoubleCoupleJoint pair must contain one roll axis and one pitch axis");
    }
    if (!CalibrationEqual(config.calibration, slave->calibration_)) {
        throw JointException(JointError::kDoubleCoupleJointCalibrationMismatch,
                             "DoubleCoupleJoint pair calibration mismatch");
    }
}

/**
 * @brief 校验双关节电机相对位置限位是否有限且有序。
 * @param limit 待校验的范围。
 * @throw std::invalid_argument 当范围无效时抛出。
 */
inline void DoubleCoupleJoint::ValidateMotorLimit(Range<float> limit) {
    if (!std::isfinite(limit.min) || !std::isfinite(limit.max) || limit.min > limit.max) {
        throw std::invalid_argument("DoubleCoupleJoint motor limit must be finite and ordered");
    }
}

/**
 * @brief 按标定容差比较两个范围。
 * @param lhs 左侧范围。
 * @param rhs 右侧范围。
 * @return true 表示两个范围的 min/max 都在容差内相等。
 */
inline bool DoubleCoupleJoint::RangeEqual(const Range<float>& lhs, const Range<float>& rhs) {
    return std::abs(lhs.min - rhs.min) <= kCalibrationTolerance &&
           std::abs(lhs.max - rhs.max) <= kCalibrationTolerance;
}

/**
 * @brief 合并 master/slave 两侧的可选电机限位。
 * @param master_limit master 侧配置的电机限位，可为空。
 * @param slave_limit slave 侧配置的电机限位，可为空。
 * @return 合并后的限位；两侧都未配置时为空。
 * @throw JointException 当两侧都配置但不一致时抛出。
 * @throw std::invalid_argument 当任一限位无效时抛出。
 */
inline std::optional<Range<float>> DoubleCoupleJoint::MergeMotorLimit(
    const std::optional<Range<float>>& master_limit,
    const std::optional<Range<float>>& slave_limit) {
    if (master_limit) {
        ValidateMotorLimit(*master_limit);
    }
    if (slave_limit) {
        ValidateMotorLimit(*slave_limit);
    }
    if (master_limit && slave_limit && !RangeEqual(*master_limit, *slave_limit)) {
        throw JointException(JointError::kDoubleCoupleJointMotorLimitMismatch,
                             "DoubleCoupleJoint pair motor limit mismatch");
    }
    if (master_limit) {
        return master_limit;
    }
    return slave_limit;
}

/**
 * @brief 创建双电机关节的 slave 或 master 实例。
 * @param config 双电机关节配置。
 * @param motor1 master 路径使用的第 1 个电机；slave 路径为空。
 * @param motor2 master 路径使用的第 2 个电机；slave 路径为空。
 * @param slave master 路径中待绑定的 slave；slave 路径为空。
 * @return 新创建的 BaseJoint 派生实例。
 * @throw std::invalid_argument 当 master 路径未提供两台电机时抛出。
 * @throw JointException 当配对校验失败时抛出。
 */
inline BaseJointPtr DoubleCoupleJoint::CreateJoint(const Config& config, Motor* motor1,
                                                   Motor* motor2,
                                                   std::shared_ptr<SlaveJoint> slave) {
    if (!motor1 && !motor2) {
        return std::shared_ptr<SlaveJoint>(new SlaveJoint(config));
    }
    if (!motor1 || !motor2) {
        throw std::invalid_argument("DoubleCoupleJoint master creation requires two motors");
    }
    ValidatePair(config, slave);
    auto motor_limit1 = MergeMotorLimit(config.motorLimit1, slave->motor_limit1_);
    auto motor_limit2 = MergeMotorLimit(config.motorLimit2, slave->motor_limit2_);

    auto master = std::shared_ptr<MasterJoint>(
        new MasterJoint(config, motor1, motor2, slave, motor_limit1, motor_limit2));
    slave->BindMaster(master);
    return master;
}

}  // namespace encos

#endif  // ENCOS_SLAVE_JOINT_H_
