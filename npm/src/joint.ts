// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

import type { NativeApi } from './native.js'
import { PVTControlMode } from './constants.js'
import type { JointStatus, Range, StatusOptions } from './types.js'
import type { JointManagerWrapper } from './joint-manager.js'

/**
 * @file joint.ts
 * @brief 单个关节的高层 TypeScript 包装（JointWrapper）。
 */

export class JointWrapper {
  private disposed_ = false

  /** @internal 由 JointManagerWrapper 创建，不建议外部直接实例化。 */
  constructor(
    private readonly native: NativeApi,
    public readonly handle: number,
    public readonly name: string,
    private readonly manager?: JointManagerWrapper,
  ) {}

  private assertAlive(): void {
    if (this.disposed_) {
      throw new Error(`JointWrapper "${this.name}" has been disposed`)
    }
  }

  dispose(): void {
    if (this.disposed_) {
      return
    }
    this.native.jointSetStatusCallback(this.handle, null)
    this.disposed_ = true
    this.manager?._unregisterJoint(this.handle)
  }

  async pvtControl(
    kp: number,
    kd: number,
    pos: number,
    spd: number,
    torque: number,
    torqueMode: PVTControlMode = PVTControlMode.MotorPVT,
  ): Promise<void> {
    this.assertAlive()
    await this.native.jointPVTControl(this.handle, kp, kd, pos, spd, torque, torqueMode)
  }

  async posControl(position: number, speed = 0, current = 0, feedback = 0): Promise<void> {
    this.assertAlive()
    await this.native.jointPosControl(this.handle, position, speed, current, feedback)
  }

  async spdControl(speed: number, current = 0, feedback = 0): Promise<void> {
    this.assertAlive()
    await this.native.jointSpdControl(this.handle, speed, current, feedback)
  }

  async curControl(current: number, feedback = 0): Promise<void> {
    this.assertAlive()
    await this.native.jointCurControl(this.handle, current, feedback)
  }

  async torControl(torque: number, feedback = 0): Promise<void> {
    this.assertAlive()
    await this.native.jointTorControl(this.handle, torque, feedback)
  }

  async stop(mode: number, current = 0, feedback = 0): Promise<void> {
    this.assertAlive()
    await this.native.jointStop(this.handle, mode, current, feedback)
  }


  /** 整个关节是否具备机械抱闸；双电机必须两台都支持。 */
  hasBrake(): boolean {
    this.assertAlive()
    return this.native.jointHasBrake(this.handle)
  }

  /** 控制机械抱闸：true 抱紧，false 释放；耦合独立两次调用合帧，首条 true 仅表示缓存，第二条按最后值下发并返回驱动结果；Stop 或第五次运动控制调用取消半帧。 */
  brake(enabled: boolean): Promise<boolean> {
    this.assertAlive()
    return this.native.jointBrake(this.handle, enabled)
  }

  /** 任一电机抱紧即返回 true；双电机状态不一致时向未抱紧的一台补发抱闸。 */
  getBrakeStatus(): Promise<boolean> {
    this.assertAlive()
    return this.native.jointGetBrakeStatus(this.handle)
  }

  posAsOffset(dir: number): number {
    this.assertAlive()
    return this.native.jointPosAsOffset(this.handle, dir)
  }

  getStatus(options?: StatusOptions): JointStatus | null {
    this.assertAlive()
    const json = this.native.jointGetStatusJson(
      this.handle,
      options?.autoSend ?? false,
      options?.lifeCycleDeduction ?? 1,
    )
    if (json === 'null') {
      return null
    }
    return JSON.parse(json) as JointStatus
  }

  setLimit(range: Range): void {
    this.assertAlive()
    this.native.jointSetLimit(this.handle, range.min, range.max)
  }

  setOnStatus(callback: ((status: JointStatus) => void) | null): void {
    this.assertAlive()
    if (callback === null) {
      this.native.jointSetStatusCallback(this.handle, null)
      return
    }
    this.native.jointSetStatusCallback(this.handle, (json: string) => {
      callback(JSON.parse(json) as JointStatus)
    })
  }

  onStatus(callback: (status: JointStatus) => void): () => void {
    this.setOnStatus(callback)
    return () => {
      if (this.disposed_) {
        return
      }
      this.setOnStatus(null)
    }
  }
}
