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

  setZero(): Promise<boolean> {
    this.assertAlive()
    return this.native.jointSetZero(this.handle)
  }

  gotoLimit(dir: number, speed: number, current: number, timeoutMs: number): Promise<number> {
    this.assertAlive()
    return this.native.jointGotoLimit(this.handle, dir, speed, current, timeoutMs)
  }

  gotoZero(offset: number, speed: number, current: number, timeoutMs: number): Promise<boolean> {
    this.assertAlive()
    return this.native.jointGotoZero(this.handle, offset, speed, current, timeoutMs)
  }

  posAsOffset(dir: number): number {
    this.assertAlive()
    return this.native.jointPosAsOffset(this.handle, dir)
  }

  calibrate(dir: number, speed: number, current: number, timeoutMs: number): Promise<boolean> {
    this.assertAlive()
    return this.native.jointCalibrate(this.handle, dir, speed, current, timeoutMs)
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
