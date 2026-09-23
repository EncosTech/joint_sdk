// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

import type { NativeApi } from './native.js'
import type {
  FakeCommandRecord,
  FakeFeedbackInput,
  FakeMotorSnapshot,
  FakeCommandKind,
} from './types.js'
import { FakeCommandKind as FakeCommandKindEnum } from './constants.js'

/**
 * @file fake-tools.ts
 * @brief FakeAdapter 命令记录、反馈注入与电机快照的高层封装。
 */

export class FakeJointTools {
  /** @internal */
  constructor(
    /** @internal */
    private readonly native: NativeApi,
    public readonly managerHandle: number,
  ) {}

  setReplyMode(mode: number): void {
    this.native.fakeSetReplyMode(this.managerHandle, mode)
  }

  enablePositionError(enabled = true): void {
    this.native.fakeEnablePositionError(this.managerHandle, enabled)
  }

  disablePositionError(): void {
    this.enablePositionError(false)
  }

  clearCommandRecords(): void {
    this.native.fakeClearCommandRecords(this.managerHandle)
  }

  getCommandCount(): number {
    return this.native.fakeGetCommandCount(this.managerHandle)
  }

  getCommandRecord(index: number): FakeCommandRecord {
    return {
      kind: this.native.fakeGetCommandKind(this.managerHandle, index) as FakeCommandKind,
      busIndex: this.native.fakeGetCommandBusIndex(this.managerHandle, index),
      motorIndex: this.native.fakeGetCommandMotorIndex(this.managerHandle, index),
      rawFrameFlags: this.native.fakeGetCommandIntField(
        this.managerHandle,
        index,
        FakeIntField.RawFrameFlags,
      ),
    }
  }

  getCommandRecords(): FakeCommandRecord[] {
    const count = this.getCommandCount()
    const records: FakeCommandRecord[] = []
    for (let i = 0; i < count; i++) {
      records.push(this.getCommandRecord(i))
    }
    return records
  }

  getLastCommand(): FakeCommandRecord | null {
    const count = this.getCommandCount()
    if (count === 0) {
      return null
    }
    return this.getCommandRecord(count - 1)
  }

  getPVTCommand(index: number): PVTCommandRecord {
    this.assertKind(index, FakeCommandKindEnum.PVTControl)
    return {
      kind: FakeCommandKindEnum.PVTControl,
      busIndex: this.native.fakeGetCommandBusIndex(this.managerHandle, index),
      motorIndex: this.native.fakeGetCommandMotorIndex(this.managerHandle, index),
      rawFrameFlags: this.native.fakeGetCommandIntField(
        this.managerHandle,
        index,
        FakeIntField.RawFrameFlags,
      ),
      kp: this.getNumber(index, FakeNumberField.PVTKp),
      kd: this.getNumber(index, FakeNumberField.PVTKd),
      position: this.getNumber(index, FakeNumberField.PVTPosition),
      speed: this.getNumber(index, FakeNumberField.PVTSpeed),
      torque: this.getNumber(index, FakeNumberField.PVTTorque),
    }
  }

  getPosCommand(index: number): PosCommandRecord {
    this.assertKind(index, FakeCommandKindEnum.PosControl)
    return {
      kind: FakeCommandKindEnum.PosControl,
      busIndex: this.native.fakeGetCommandBusIndex(this.managerHandle, index),
      motorIndex: this.native.fakeGetCommandMotorIndex(this.managerHandle, index),
      rawFrameFlags: this.native.fakeGetCommandIntField(
        this.managerHandle,
        index,
        FakeIntField.RawFrameFlags,
      ),
      position: this.getNumber(index, FakeNumberField.PosPosition),
      speed: this.getNumber(index, FakeNumberField.PosSpeed),
      current: this.getNumber(index, FakeNumberField.PosCurrent),
      feedbackType: this.getInt(index, FakeIntField.FeedbackType),
    }
  }

  getSpdCommand(index: number): SpdCommandRecord {
    this.assertKind(index, FakeCommandKindEnum.SpdControl)
    return {
      kind: FakeCommandKindEnum.SpdControl,
      busIndex: this.native.fakeGetCommandBusIndex(this.managerHandle, index),
      motorIndex: this.native.fakeGetCommandMotorIndex(this.managerHandle, index),
      rawFrameFlags: this.native.fakeGetCommandIntField(
        this.managerHandle,
        index,
        FakeIntField.RawFrameFlags,
      ),
      speed: this.getNumber(index, FakeNumberField.SpdSpeed),
      current: this.getNumber(index, FakeNumberField.SpdCurrent),
      feedbackType: this.getInt(index, FakeIntField.FeedbackType),
    }
  }

  getCurCommand(index: number): CurCommandRecord {
    this.assertKind(index, FakeCommandKindEnum.CurControl)
    return {
      kind: FakeCommandKindEnum.CurControl,
      busIndex: this.native.fakeGetCommandBusIndex(this.managerHandle, index),
      motorIndex: this.native.fakeGetCommandMotorIndex(this.managerHandle, index),
      rawFrameFlags: this.native.fakeGetCommandIntField(
        this.managerHandle,
        index,
        FakeIntField.RawFrameFlags,
      ),
      current: this.getNumber(index, FakeNumberField.CurCurrent),
      feedbackType: this.getInt(index, FakeIntField.FeedbackType),
    }
  }

  getTorCommand(index: number): TorCommandRecord {
    this.assertKind(index, FakeCommandKindEnum.TorControl)
    return {
      kind: FakeCommandKindEnum.TorControl,
      busIndex: this.native.fakeGetCommandBusIndex(this.managerHandle, index),
      motorIndex: this.native.fakeGetCommandMotorIndex(this.managerHandle, index),
      rawFrameFlags: this.native.fakeGetCommandIntField(
        this.managerHandle,
        index,
        FakeIntField.RawFrameFlags,
      ),
      torque: this.getNumber(index, FakeNumberField.TorTorque),
      feedbackType: this.getInt(index, FakeIntField.FeedbackType),
    }
  }

  getStopCommand(index: number): StopCommandRecord {
    this.assertKind(index, FakeCommandKindEnum.Stop)
    return {
      kind: FakeCommandKindEnum.Stop,
      busIndex: this.native.fakeGetCommandBusIndex(this.managerHandle, index),
      motorIndex: this.native.fakeGetCommandMotorIndex(this.managerHandle, index),
      rawFrameFlags: this.native.fakeGetCommandIntField(
        this.managerHandle,
        index,
        FakeIntField.RawFrameFlags,
      ),
      mode: this.getInt(index, FakeIntField.StopMode),
      current: this.getNumber(index, FakeNumberField.StopCurrent),
      feedbackType: this.getInt(index, FakeIntField.FeedbackType),
    }
  }

  injectFeedback(busId: number, motorId: number, input: FakeFeedbackInput): void {
    const values = [
      input.position,
      input.speed,
      input.current,
      input.motorTemperature,
      input.mosTemperature,
    ]
    const meta = [input.error ?? 0]
    this.native.fakeInjectFeedback(this.managerHandle, busId, motorId, values, meta)
  }

  getMotorSnapshot(busId: number, motorId: number): FakeMotorSnapshot {
    const { values, meta } = this.native.fakeGetMotorSnapshot(
      this.managerHandle,
      busId,
      motorId,
    )
    return {
      model: meta[0],
      position: values[13],
      speed: values[14],
      current: values[15],
      torque: values[16],
      motorTemperature: values[17],
      mosTemperature: values[18],
      acceleration: values[19],
      kt: values[20],
      canTimeoutMs: meta[1],
      replyFrameFlags: meta[2],
      communicationMode: meta[3],
      error: meta[4],
      brakeEnabled: meta[5] !== 0,
    }
  }

  private getNumber(index: number, fieldId: FakeNumberField): number {
    return this.native.fakeGetCommandNumberField(this.managerHandle, index, fieldId)
  }

  private getInt(index: number, fieldId: FakeIntField): number {
    return this.native.fakeGetCommandIntField(this.managerHandle, index, fieldId)
  }

  private assertKind(index: number, expected: FakeCommandKindEnum): void {
    const actual = this.native.fakeGetCommandKind(this.managerHandle, index)
    if (actual !== expected) {
      throw new Error(
        `Expected FakeCommandKind ${expected} at index ${index}, got ${actual}`,
      )
    }
  }
}

export enum FakeNumberField {
  PVTKp = 0,
  PVTKd = 1,
  PVTPosition = 2,
  PVTSpeed = 3,
  PVTTorque = 4,

  PosPosition = 10,
  PosSpeed = 11,
  PosCurrent = 12,

  SpdSpeed = 20,
  SpdCurrent = 21,

  CurCurrent = 30,

  TorTorque = 40,

  StopCurrent = 50,
}

export enum FakeIntField {
  FeedbackType = 0,
  StopMode = 1,
  RawFrameFlags = 2,
}

export interface PVTCommandRecord {
  kind: FakeCommandKind
  busIndex: number
  motorIndex: number
  rawFrameFlags: number
  kp: number
  kd: number
  position: number
  speed: number
  torque: number
}

export interface PosCommandRecord {
  kind: FakeCommandKind
  busIndex: number
  motorIndex: number
  rawFrameFlags: number
  position: number
  speed: number
  current: number
  feedbackType: number
}

export interface SpdCommandRecord {
  kind: FakeCommandKind
  busIndex: number
  motorIndex: number
  rawFrameFlags: number
  speed: number
  current: number
  feedbackType: number
}

export interface CurCommandRecord {
  kind: FakeCommandKind
  busIndex: number
  motorIndex: number
  rawFrameFlags: number
  current: number
  feedbackType: number
}

export interface TorCommandRecord {
  kind: FakeCommandKind
  busIndex: number
  motorIndex: number
  rawFrameFlags: number
  torque: number
  feedbackType: number
}

export interface StopCommandRecord {
  kind: FakeCommandKind
  busIndex: number
  motorIndex: number
  rawFrameFlags: number
  mode: number
  current: number
  feedbackType: number
}
