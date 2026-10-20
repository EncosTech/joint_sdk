// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

import type { FakeCommandKind, PVTControlMode } from './constants.js'

/** @internal */
export interface CFunction {
  (...args: unknown[]): unknown
}

/** @internal */
export interface WasmModule {
  _malloc(size: number): number
  _free(ptr: number): void
  UTF8ToString(ptr: number): string
  getValue(ptr: number, type: string): number
  setValue(ptr: number, value: number, type: string): void
  addFunction(fn: (...args: unknown[]) => unknown, sig: string): number
  removeFunction(id: number): void
  cwrap(
    name: string,
    returnType: string | null,
    argTypes: string[],
    opts?: { async?: boolean },
  ): CFunction
  HEAPU8: Uint8Array
  HEAP32: Int32Array
  HEAPF64: Float64Array
}

/** @internal */
export interface WasmModuleFactory {
  (opts?: Record<string, unknown>): Promise<WasmModule>
}

/** @internal */
export interface RuntimeHost {
  loadWasmModule(modulePath?: string): Promise<WasmModule>
  resolveUrl(input: string): URL
  fetch(input: string): Promise<Response>
}

export interface RuntimeOptions {
  modulePath?: string
}

export interface CreateJointManagerOptions {
  adapterType?: string
  adapterName?: string
  loggerName?: string
  logLevel?: number
}

export interface JointDefinition {
  type: string
  name: string
  config: Record<string, unknown>
}

/** 适配器创建配置，对应 C++ 侧 ParseAdapterDefinition 解析的 JSON 结构。 */
export interface AdapterDefinition {
  type: string
  name: string
  loggerName?: string
  logLevel?: number
  /**
   * 适配器扩展配置，字段语义由插件自行定义（如 `{ buses: [...] }`）。
   * 适配器已存在时由驱动器把其中的 `buses` 增量应用到既有实例。
   */
  extend?: Record<string, unknown>
}

export interface JointStatus {
  pos: number
  spd: number
  tor: number
  mosTemperature: number
  motorTemperature: number
  error: number
}

export interface PVTControlCommand {
  kp: number
  kd: number
  pos: number
  spd: number
  torque: number
  /** Coupled-joint solve mode; defaults to MotorPVT. Both axes must select the same host mode. */
  torqueMode?: PVTControlMode
}

export interface PosControlCommand {
  position: number
  speed?: number
  current?: number
  feedback?: number
}

export interface SpdControlCommand {
  speed: number
  current?: number
  feedback?: number
}

export interface CurrentControlCommand {
  current: number
  feedback?: number
}

export interface TorqueControlCommand {
  torque: number
  feedback?: number
}

export interface StopCommand {
  mode: number
  current?: number
  feedback?: number
}

export interface Range {
  min: number
  max: number
}

export interface StatusOptions {
  /**
   * 缓存缺失时是否下发反馈命令。关节运动过程中不得启用，否则零目标 PVT 反馈命令
   * 可能改变关节运动状态。
   */
  autoSend?: boolean
  lifeCycleDeduction?: number
}

export interface FakeCommandRecord {
  kind: FakeCommandKind
  busIndex: number
  motorIndex: number
  rawFrameFlags: number
}

export interface FakeFeedbackInput {
  position: number
  speed: number
  current: number
  motorTemperature: number
  mosTemperature: number
  error?: number
}

export interface FakeMotorSnapshot {
  model: number
  position: number
  speed: number
  current: number
  torque: number
  motorTemperature: number
  mosTemperature: number
  acceleration: number
  kt: number
  canTimeoutMs: number
  replyFrameFlags: number
  communicationMode: number
  error: number
  brakeEnabled: boolean
}

export type {
  FakeCommandKind,
  JointError,
  MotorModel,
  MotorStopMode,
  FakeReplyMode,
} from './constants.js'
