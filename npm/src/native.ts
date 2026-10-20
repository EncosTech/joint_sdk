// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

import type { CFunction, WasmModule } from './types.js'
import { LogLevel, PVTControlMode } from './constants.js'

/**
 * @file native.ts
 * @brief 底层 C API 封装与字符串/内存辅助函数。
 */

export class EncosWasmError extends Error {
  constructor(
    message: string,
    public readonly code: number,
    public readonly jointError?: number,
  ) {
    super(message)
    this.name = 'EncosWasmError'
  }
}

/** @internal */
export interface NativeApi {
  getLastErrorCode(): number
  getLastErrorMessage(): string
  getLastJointError(): number
  clearLastError(): void
  getVersion(): string

  createManager(
    adapterType: string,
    adapterName: string,
    loggerName: string,
    logLevel: number,
  ): number
  createEmptyManager(): number
  managerCreateAdapter(
    manager: number,
    adapterType: string,
    adapterName: string,
    loggerName: string,
    logLevel: number,
    extendJson?: string,
  ): void
  managerCreateAdapterJson(manager: number, adapterJson: string): void
  disposeManager(manager: number): void

  managerAddJointJson(manager: number, json: string): Promise<number>
  managerGetJoint(manager: number, name: string): number
  managerGetJointSchemaJson(manager: number): string
  managerGetAvailableJointTypesJson(manager: number): string
  managerGetStatusJson(manager: number, autoSend: boolean, lifeCycleDeduction: number): string
  managerEnableJointMotorCanfd(manager: number): void
  managerDisableJointMotorCanfd(manager: number): void
  managerIsJointMotorCanfd(manager: number): boolean

  jointPVTControl(
    joint: number,
    kp: number,
    kd: number,
    pos: number,
    spd: number,
    torque: number,
    torqueMode: PVTControlMode,
  ): Promise<void>
  jointPosControl(joint: number, position: number, speed: number, current: number, feedback: number): Promise<void>
  jointSpdControl(joint: number, speed: number, current: number, feedback: number): Promise<void>
  jointCurControl(joint: number, current: number, feedback: number): Promise<void>
  jointTorControl(joint: number, torque: number, feedback: number): Promise<void>
  jointStop(joint: number, mode: number, current: number, feedback: number): Promise<void>
  jointHasBrake(joint: number): boolean
  jointBrake(joint: number, enabled: boolean): Promise<boolean>
  jointGetBrakeStatus(joint: number): Promise<boolean>
  jointPosAsOffset(joint: number, dir: number): number
  jointGetStatusJson(joint: number, autoSend: boolean, lifeCycleDeduction: number): string
  jointSetLimit(joint: number, min: number, max: number): void
  jointSetStatusCallback(joint: number, callback: ((json: string) => void) | null): void

  fakeSetReplyMode(manager: number, mode: number): void
  fakeEnablePositionError(manager: number, enabled: boolean): void
  fakeClearCommandRecords(manager: number): void
  fakeGetCommandCount(manager: number): number
  fakeGetCommandKind(manager: number, index: number): number
  fakeGetCommandBusIndex(manager: number, index: number): number
  fakeGetCommandMotorIndex(manager: number, index: number): number
  fakeGetCommandNumberField(manager: number, index: number, fieldId: number): number
  fakeGetCommandIntField(manager: number, index: number, fieldId: number): number
  fakeInjectFeedback(
    manager: number,
    busId: number,
    motorId: number,
    values: number[],
    meta: number[],
  ): void
  fakeGetMotorSnapshot(
    manager: number,
    busId: number,
    motorId: number,
  ): { values: number[]; meta: number[] }

  freeString(ptr: number): void
}

function checkCode(
  module: WasmModule,
  code: number,
  context: string,
  getLastErrorMessage: () => string,
  getLastJointError: () => number,
): void {
  if (code !== 0) {
    const message = getLastErrorMessage()
    const jointError = getLastJointError()
    throw new EncosWasmError(
      `${context}: ${message}`,
      code,
      jointError >= 0 ? jointError : undefined,
    )
  }
}

function allocString(module: WasmModule, s: string): number {
  const bytes = new TextEncoder().encode(s)
  const ptr = module._malloc(bytes.length + 1)
  if (ptr === 0) {
    throw new EncosWasmError('Failed to allocate string', -1)
  }
  module.HEAPU8.set(bytes, ptr)
  module.HEAPU8[ptr + bytes.length] = 0
  return ptr
}

function readString(module: WasmModule, ptr: number): string {
  if (ptr === 0) {
    return ''
  }
  const s = module.UTF8ToString(ptr)
  return s
}

function allocUint32(module: WasmModule): number {
  const ptr = module._malloc(4)
  if (ptr === 0) {
    throw new EncosWasmError('Failed to allocate uint32', -1)
  }
  module.setValue(ptr, 0, 'i32')
  return ptr
}

function readUint32(module: WasmModule, ptr: number): number {
  return module.getValue(ptr, 'i32') >>> 0
}

function readInt32(module: WasmModule, ptr: number): number {
  return module.getValue(ptr, 'i32')
}

function readFloat(module: WasmModule, ptr: number): number {
  return module.getValue(ptr, 'float')
}

function allocFloat(module: WasmModule): number {
  const ptr = module._malloc(4)
  if (ptr === 0) {
    throw new EncosWasmError('Failed to allocate float', -1)
  }
  module.setValue(ptr, 0, 'float')
  return ptr
}

/** @internal */
export function createNativeApi(module: WasmModule): NativeApi {
  const wrap = (
    name: string,
    returnType: string | null,
    argTypes: string[],
    async = false,
  ): CFunction => module.cwrap(name, returnType, argTypes, { async })

  const getLastErrorCodeFn = wrap('joint_sdk_get_last_error_code', 'number', []) as () => number
  const getLastErrorMessageFn = wrap('joint_sdk_get_last_error_message', 'number', []) as () => number
  const getLastJointErrorFn = wrap('joint_sdk_get_last_joint_error', 'number', []) as () => number
  const clearLastErrorFn = wrap('joint_sdk_clear_last_error', null, []) as () => void
  const getVersionFn = wrap('joint_sdk_get_version', 'number', []) as () => number
  const createManagerFn = wrap('joint_sdk_create_manager', 'number', [
    'string',
    'string',
    'string',
    'number',
    'number',
  ]) as (
    adapterType: string,
    adapterName: string,
    loggerName: string,
    logLevel: number,
    outManager: number,
  ) => number
  const createEmptyManagerFn = wrap('joint_sdk_create_empty_manager', 'number', [
    'number',
  ]) as (outManager: number) => number
  const managerCreateAdapterFn = wrap('joint_sdk_manager_create_adapter', 'number', [
    'number',
    'string',
    'string',
    'string',
    'number',
    'string',
  ]) as (
    manager: number,
    adapterType: string,
    adapterName: string,
    loggerName: string,
    logLevel: number,
    extendJson: string,
  ) => number
  const managerCreateAdapterJsonFn = wrap(
    'joint_sdk_manager_create_adapter_json',
    'number',
    ['number', 'string'],
  ) as (manager: number, adapterJson: string) => number
  const disposeManagerFn = wrap('joint_sdk_dispose_manager', 'number', ['number']) as (
    manager: number,
  ) => number
  const managerAddJointJsonFn = wrap(
    'joint_sdk_manager_add_joint_json',
    'number',
    [
      'number',
      'string',
      'number',
    ],
    true,
  ) as (manager: number, json: string, outJoint: number) => Promise<number>
  const managerGetJointFn = wrap('joint_sdk_manager_get_joint', 'number', [
    'number',
    'string',
    'number',
  ]) as (manager: number, name: string, outJoint: number) => number
  const managerGetJointSchemaJsonFn = wrap(
    'joint_sdk_manager_get_joint_schema_json',
    'number',
    ['number'],
  ) as (manager: number) => number
  const managerGetAvailableJointTypesJsonFn = wrap(
    'joint_sdk_manager_get_available_joint_types_json',
    'number',
    ['number'],
  ) as (manager: number) => number
  const managerGetStatusJsonFn = wrap('joint_sdk_manager_get_status_json', 'number', [
    'number',
    'number',
    'number',
  ]) as (manager: number, autoSend: number, lifeCycleDeduction: number) => number
  const managerEnableJointMotorCanfdFn = wrap(
    'joint_sdk_manager_enable_joint_motor_canfd',
    'number',
    ['number'],
  ) as (manager: number) => number
  const managerDisableJointMotorCanfdFn = wrap(
    'joint_sdk_manager_disable_joint_motor_canfd',
    'number',
    ['number'],
  ) as (manager: number) => number
  const managerIsJointMotorCanfdFn = wrap('joint_sdk_manager_is_joint_motor_canfd', 'number', [
    'number',
    'number',
  ]) as (manager: number, outEnabled: number) => number

  const jointPVTControlFn = wrap('joint_sdk_joint_pvt_control', 'number', [
    'number',
    'number',
    'number',
    'number',
    'number',
    'number',
    'number',
  ], true) as (
    joint: number,
    kp: number,
    kd: number,
    pos: number,
    spd: number,
    torque: number,
    torqueMode: number,
  ) => Promise<number>
  const jointPosControlFn = wrap('joint_sdk_joint_pos_control', 'number', [
    'number',
    'number',
    'number',
    'number',
    'number',
  ], true) as (
    joint: number,
    position: number,
    speed: number,
    current: number,
    feedback: number,
  ) => Promise<number>
  const jointSpdControlFn = wrap('joint_sdk_joint_spd_control', 'number', [
    'number',
    'number',
    'number',
    'number',
  ], true) as (joint: number, speed: number, current: number, feedback: number) => Promise<number>
  const jointCurControlFn = wrap('joint_sdk_joint_cur_control', 'number', [
    'number',
    'number',
    'number',
  ], true) as (joint: number, current: number, feedback: number) => Promise<number>
  const jointTorControlFn = wrap('joint_sdk_joint_tor_control', 'number', [
    'number',
    'number',
    'number',
  ], true) as (joint: number, torque: number, feedback: number) => Promise<number>
  const jointStopFn = wrap('joint_sdk_joint_stop', 'number', [
    'number',
    'number',
    'number',
    'number',
  ], true) as (joint: number, mode: number, current: number, feedback: number) => Promise<number>
  const jointHasBrakeFn = wrap('joint_sdk_joint_has_brake', 'number', ['number', 'number']) as (
    joint: number,
    outOk: number,
  ) => number
  const jointBrakeFn = wrap('joint_sdk_joint_brake', 'number', ['number', 'number', 'number'], true) as (
    joint: number,
    enabled: number,
    outOk: number,
  ) => Promise<number>
  const jointGetBrakeStatusFn = wrap('joint_sdk_joint_get_brake_status', 'number', ['number', 'number'], true) as (
    joint: number,
    outOk: number,
  ) => Promise<number>
  const jointPosAsOffsetFn = wrap('joint_sdk_joint_pos_as_offset', 'number', [
    'number',
    'number',
    'number',
  ]) as (joint: number, dir: number, outOffset: number) => number
  const jointGetStatusJsonFn = wrap('joint_sdk_joint_get_status_json', 'number', [
    'number',
    'number',
    'number',
  ]) as (joint: number, autoSend: number, lifeCycleDeduction: number) => number
  const jointSetLimitFn = wrap('joint_sdk_joint_set_limit', 'number', [
    'number',
    'number',
    'number',
  ]) as (joint: number, min: number, max: number) => number
  const jointSetStatusCallbackFn = wrap('joint_sdk_joint_set_status_callback', 'number', [
    'number',
    'number',
  ]) as (joint: number, callbackPtr: number) => number
  const jointClearStatusCallbackFn = wrap('joint_sdk_joint_clear_status_callback', 'number', [
    'number',
  ]) as (joint: number) => number

  const fakeSetReplyModeFn = wrap('joint_sdk_fake_set_reply_mode', 'number', [
    'number',
    'number',
  ]) as (manager: number, mode: number) => number
  const fakeEnablePositionErrorFn = wrap('joint_sdk_fake_enable_position_error', 'number', [
    'number',
    'number',
  ]) as (manager: number, enabled: number) => number
  const fakeClearCommandRecordsFn = wrap('joint_sdk_fake_clear_command_records', 'number', [
    'number',
  ]) as (manager: number) => number
  const fakeGetCommandCountFn = wrap('joint_sdk_fake_get_command_count', 'number', [
    'number',
    'number',
  ]) as (manager: number, outCount: number) => number
  const fakeGetCommandKindFn = wrap('joint_sdk_fake_get_command_kind', 'number', [
    'number',
    'number',
    'number',
  ]) as (manager: number, index: number, outKind: number) => number
  const fakeGetCommandBusIndexFn = wrap('joint_sdk_fake_get_command_bus_index', 'number', [
    'number',
    'number',
    'number',
  ]) as (manager: number, index: number, outBus: number) => number
  const fakeGetCommandMotorIndexFn = wrap('joint_sdk_fake_get_command_motor_index', 'number', [
    'number',
    'number',
    'number',
  ]) as (manager: number, index: number, outMotor: number) => number
  const fakeGetCommandNumberFieldFn = wrap('joint_sdk_fake_get_command_number_field', 'number', [
    'number',
    'number',
    'number',
    'number',
  ]) as (manager: number, index: number, fieldId: number, outValue: number) => number
  const fakeGetCommandIntFieldFn = wrap('joint_sdk_fake_get_command_int_field', 'number', [
    'number',
    'number',
    'number',
    'number',
  ]) as (manager: number, index: number, fieldId: number, outValue: number) => number
  const fakeInjectFeedbackFn = wrap('joint_sdk_fake_inject_feedback', 'number', [
    'number',
    'number',
    'number',
    'number',
    'number',
    'number',
    'number',
  ]) as (
    manager: number,
    busId: number,
    motorId: number,
    valuesPtr: number,
    valuesCount: number,
    metaPtr: number,
    metaCount: number,
  ) => number
  const fakeGetMotorSnapshotFn = wrap('joint_sdk_fake_get_motor_snapshot', 'number', [
    'number',
    'number',
    'number',
    'number',
    'number',
    'number',
    'number',
  ]) as (
    manager: number,
    busId: number,
    motorId: number,
    valuesPtr: number,
    valuesCount: number,
    metaPtr: number,
    metaCount: number,
  ) => number
  const freeStringFn = wrap('joint_sdk_free_string', null, ['number']) as (ptr: number) => void

  const statusCallbackMap = new Map<number, number>()

  function getLastErrorMessage(): string {
    const ptr = getLastErrorMessageFn()
    try {
      return readString(module, ptr)
    } finally {
      freeStringFn(ptr)
    }
  }

  function getLastJointError(): number {
    return getLastJointErrorFn()
  }

  function throwOnError(code: number, context: string): void {
    checkCode(module, code, context, getLastErrorMessage, getLastJointError)
  }

  function callReturningString(fn: () => number, context: string): string {
    const ptr = fn()
    try {
      const s = readString(module, ptr)
      if (s === 'null') {
        throwOnError(getLastErrorCodeFn(), context)
        return s
      }
      return s
    } finally {
      freeStringFn(ptr)
    }
  }

  return {
    getLastErrorCode: getLastErrorCodeFn,
    getLastErrorMessage,
    getLastJointError,
    clearLastError: clearLastErrorFn,
    getVersion(): string {
      return callReturningString(getVersionFn, 'getVersion')
    },

    createManager(adapterType, adapterName, loggerName, logLevel): number {
      const outPtr = allocUint32(module)
      try {
        const code = createManagerFn(adapterType, adapterName, loggerName, logLevel, outPtr)
        throwOnError(code, 'createManager')
        return readUint32(module, outPtr)
      } finally {
        module._free(outPtr)
      }
    },

    createEmptyManager(): number {
      const outPtr = allocUint32(module)
      try {
        const code = createEmptyManagerFn(outPtr)
        throwOnError(code, 'createEmptyManager')
        return readUint32(module, outPtr)
      } finally {
        module._free(outPtr)
      }
    },

    managerCreateAdapter(
      manager,
      adapterType,
      adapterName,
      loggerName,
      logLevel,
      extendJson = '',
    ): void {
      throwOnError(
        managerCreateAdapterFn(
          manager,
          adapterType,
          adapterName,
          loggerName,
          logLevel,
          extendJson,
        ),
        'managerCreateAdapter',
      )
    },

    managerCreateAdapterJson(manager, adapterJson): void {
      throwOnError(
        managerCreateAdapterJsonFn(manager, adapterJson),
        'managerCreateAdapterJson',
      )
    },

    disposeManager(manager): void {
      const code = disposeManagerFn(manager)
      throwOnError(code, 'disposeManager')
    },

    async managerAddJointJson(manager, json): Promise<number> {
      const outPtr = allocUint32(module)
      try {
        const code = await managerAddJointJsonFn(manager, json, outPtr)
        throwOnError(code, 'managerAddJointJson')
        return readUint32(module, outPtr)
      } finally {
        module._free(outPtr)
      }
    },

    managerGetJoint(manager, name): number {
      const outPtr = allocUint32(module)
      try {
        const code = managerGetJointFn(manager, name, outPtr)
        throwOnError(code, 'managerGetJoint')
        return readUint32(module, outPtr)
      } finally {
        module._free(outPtr)
      }
    },

    managerGetJointSchemaJson(manager): string {
      return callReturningString(() => managerGetJointSchemaJsonFn(manager), 'managerGetJointSchemaJson')
    },

    managerGetAvailableJointTypesJson(manager): string {
      return callReturningString(
        () => managerGetAvailableJointTypesJsonFn(manager),
        'managerGetAvailableJointTypesJson',
      )
    },

    managerGetStatusJson(manager, autoSend, lifeCycleDeduction): string {
      return callReturningString(
        () => managerGetStatusJsonFn(manager, autoSend ? 1 : 0, lifeCycleDeduction),
        'managerGetStatusJson',
      )
    },

    managerEnableJointMotorCanfd(manager): void {
      throwOnError(managerEnableJointMotorCanfdFn(manager), 'managerEnableJointMotorCanfd')
    },

    managerDisableJointMotorCanfd(manager): void {
      throwOnError(managerDisableJointMotorCanfdFn(manager), 'managerDisableJointMotorCanfd')
    },

    managerIsJointMotorCanfd(manager): boolean {
      const outPtr = allocInt32(module)
      try {
        throwOnError(managerIsJointMotorCanfdFn(manager, outPtr), 'managerIsJointMotorCanfd')
        return readInt32(module, outPtr) !== 0
      } finally {
        module._free(outPtr)
      }
    },

    async jointPVTControl(joint, kp, kd, pos, spd, torque, torqueMode): Promise<void> {
      if (!Number.isInteger(torqueMode) || torqueMode < PVTControlMode.MotorPVT || torqueMode > PVTControlMode.HostKpMotorKd) {
        throw new RangeError('Invalid PVT control mode')
      }
      throwOnError(
        await jointPVTControlFn(joint, kp, kd, pos, spd, torque, torqueMode),
        'jointPVTControl',
      )
    },

    async jointPosControl(joint, position, speed, current, feedback): Promise<void> {
      throwOnError(
        await jointPosControlFn(joint, position, speed, current, feedback),
        'jointPosControl',
      )
    },

    async jointSpdControl(joint, speed, current, feedback): Promise<void> {
      throwOnError(await jointSpdControlFn(joint, speed, current, feedback), 'jointSpdControl')
    },

    async jointCurControl(joint, current, feedback): Promise<void> {
      throwOnError(await jointCurControlFn(joint, current, feedback), 'jointCurControl')
    },

    async jointTorControl(joint, torque, feedback): Promise<void> {
      throwOnError(await jointTorControlFn(joint, torque, feedback), 'jointTorControl')
    },

    async jointStop(joint, mode, current, feedback): Promise<void> {
      throwOnError(await jointStopFn(joint, mode, current, feedback), 'jointStop')
    },


    jointHasBrake(joint): boolean {
      const outPtr = allocInt32(module)
      try {
        throwOnError(jointHasBrakeFn(joint, outPtr), 'jointHasBrake')
        return readInt32(module, outPtr) !== 0
      } finally {
        module._free(outPtr)
      }
    },

    async jointBrake(joint, enabled): Promise<boolean> {
      const outPtr = allocInt32(module)
      try {
        throwOnError(await jointBrakeFn(joint, enabled ? 1 : 0, outPtr), 'jointBrake')
        return readInt32(module, outPtr) !== 0
      } finally {
        module._free(outPtr)
      }
    },

    async jointGetBrakeStatus(joint): Promise<boolean> {
      const outPtr = allocInt32(module)
      try {
        throwOnError(await jointGetBrakeStatusFn(joint, outPtr), 'jointGetBrakeStatus')
        return readInt32(module, outPtr) !== 0
      } finally {
        module._free(outPtr)
      }
    },

    jointPosAsOffset(joint, dir): number {
      const outPtr = allocFloat(module)
      try {
        throwOnError(jointPosAsOffsetFn(joint, dir, outPtr), 'jointPosAsOffset')
        return readFloat(module, outPtr)
      } finally {
        module._free(outPtr)
      }
    },

    jointGetStatusJson(joint, autoSend, lifeCycleDeduction): string {
      return callReturningString(
        () => jointGetStatusJsonFn(joint, autoSend ? 1 : 0, lifeCycleDeduction),
        'jointGetStatusJson',
      )
    },

    jointSetLimit(joint, min, max): void {
      throwOnError(jointSetLimitFn(joint, min, max), 'jointSetLimit')
    },

    jointSetStatusCallback(joint, callback): void {
      if (callback === null) {
        throwOnError(jointClearStatusCallbackFn(joint), 'jointClearStatusCallback')
        const existing = statusCallbackMap.get(joint)
        if (existing !== undefined) {
          module.removeFunction(existing)
          statusCallbackMap.delete(joint)
        }
        return
      }
      const cb = module.addFunction((...args: unknown[]): unknown => {
        const json = readString(module, args[0] as number)
        callback(json)
        return undefined
      }, 'vi')
      const code = jointSetStatusCallbackFn(joint, cb)
      if (code !== 0) {
        module.removeFunction(cb)
        throwOnError(code, 'jointSetStatusCallback')
      }
      const existing = statusCallbackMap.get(joint)
      if (existing !== undefined) {
        module.removeFunction(existing)
      }
      statusCallbackMap.set(joint, cb)
    },

    fakeSetReplyMode(manager, mode): void {
      throwOnError(fakeSetReplyModeFn(manager, mode), 'fakeSetReplyMode')
    },

    fakeEnablePositionError(manager, enabled): void {
      throwOnError(
        fakeEnablePositionErrorFn(manager, enabled ? 1 : 0),
        'fakeEnablePositionError',
      )
    },

    fakeClearCommandRecords(manager): void {
      throwOnError(fakeClearCommandRecordsFn(manager), 'fakeClearCommandRecords')
    },

    fakeGetCommandCount(manager): number {
      const outPtr = allocInt32(module)
      try {
        throwOnError(fakeGetCommandCountFn(manager, outPtr), 'fakeGetCommandCount')
        return readInt32(module, outPtr)
      } finally {
        module._free(outPtr)
      }
    },

    fakeGetCommandKind(manager, index): number {
      const outPtr = allocInt32(module)
      try {
        throwOnError(fakeGetCommandKindFn(manager, index, outPtr), 'fakeGetCommandKind')
        return readInt32(module, outPtr)
      } finally {
        module._free(outPtr)
      }
    },

    fakeGetCommandBusIndex(manager, index): number {
      const outPtr = allocInt32(module)
      try {
        throwOnError(fakeGetCommandBusIndexFn(manager, index, outPtr), 'fakeGetCommandBusIndex')
        return readInt32(module, outPtr)
      } finally {
        module._free(outPtr)
      }
    },

    fakeGetCommandMotorIndex(manager, index): number {
      const outPtr = allocInt32(module)
      try {
        throwOnError(fakeGetCommandMotorIndexFn(manager, index, outPtr), 'fakeGetCommandMotorIndex')
        return readInt32(module, outPtr)
      } finally {
        module._free(outPtr)
      }
    },

    fakeGetCommandNumberField(manager, index, fieldId): number {
      const outPtr = module._malloc(8)
      if (outPtr === 0) {
        throw new EncosWasmError('Failed to allocate double', -1)
      }
      try {
        throwOnError(
          fakeGetCommandNumberFieldFn(manager, index, fieldId, outPtr),
          'fakeGetCommandNumberField',
        )
        return module.getValue(outPtr, 'double')
      } finally {
        module._free(outPtr)
      }
    },

    fakeGetCommandIntField(manager, index, fieldId): number {
      const outPtr = allocInt32(module)
      try {
        throwOnError(fakeGetCommandIntFieldFn(manager, index, fieldId, outPtr), 'fakeGetCommandIntField')
        return readInt32(module, outPtr)
      } finally {
        module._free(outPtr)
      }
    },

    fakeInjectFeedback(manager, busId, motorId, values, meta): void {
      if (values.length < 5 || meta.length < 1) {
        throw new EncosWasmError('feedback buffers are too small', -1)
      }
      const valuesPtr = module._malloc(values.length * 8)
      const metaPtr = module._malloc(meta.length * 4)
      if (valuesPtr === 0 || metaPtr === 0) {
        if (valuesPtr) module._free(valuesPtr)
        if (metaPtr) module._free(metaPtr)
        throw new EncosWasmError('Failed to allocate feedback buffers', -1)
      }
      try {
        module.HEAPF64.set(values, valuesPtr / 8)
        module.HEAP32.set(meta, metaPtr / 4)
        throwOnError(
          fakeInjectFeedbackFn(manager, busId, motorId, valuesPtr, values.length, metaPtr, meta.length),
          'fakeInjectFeedback',
        )
      } finally {
        module._free(valuesPtr)
        module._free(metaPtr)
      }
    },

    fakeGetMotorSnapshot(manager, busId, motorId): { values: number[]; meta: number[] } {
      const valuesCount = 21
      const metaCount = 6
      const valuesPtr = module._malloc(valuesCount * 8)
      const metaPtr = module._malloc(metaCount * 4)
      if (valuesPtr === 0 || metaPtr === 0) {
        if (valuesPtr) module._free(valuesPtr)
        if (metaPtr) module._free(metaPtr)
        throw new EncosWasmError('Failed to allocate snapshot buffers', -1)
      }
      try {
        throwOnError(
          fakeGetMotorSnapshotFn(manager, busId, motorId, valuesPtr, valuesCount, metaPtr, metaCount),
          'fakeGetMotorSnapshot',
        )
        const values = Array.from(module.HEAPF64.subarray(valuesPtr / 8, valuesPtr / 8 + valuesCount))
        const meta = Array.from(module.HEAP32.subarray(metaPtr / 4, metaPtr / 4 + metaCount))
        return { values, meta }
      } finally {
        module._free(valuesPtr)
        module._free(metaPtr)
      }
    },

    freeString(ptr): void {
      freeStringFn(ptr)
    },
  }
}

function allocInt32(module: WasmModule): number {
  const ptr = module._malloc(4)
  if (ptr === 0) {
    throw new EncosWasmError('Failed to allocate int32', -1)
  }
  module.setValue(ptr, 0, 'i32')
  return ptr
}
