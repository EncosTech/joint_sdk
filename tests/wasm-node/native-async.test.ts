// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

import { describe, expect, it } from 'vitest'

import { JointWrapper } from '../../npm/src/joint.js'
import { createNativeApi, type NativeApi } from '../../npm/src/native.js'
import type { CFunction, WasmModule } from '../../npm/src/types.js'

function createRecordingModule() {
  const cwrapOptions = new Map<string, { async?: boolean } | undefined>()
  const calls = new Map<string, unknown[][]>()
  const module = {
    _malloc: () => 8,
    _free: () => undefined,
    UTF8ToString: () => '',
    getValue: () => 0,
    setValue: () => undefined,
    addFunction: () => 1,
    removeFunction: () => undefined,
    cwrap(name: string, _returnType: string | null, _argTypes: string[], opts?: { async?: boolean }): CFunction {
      cwrapOptions.set(name, opts)
      return ((...args: unknown[]) => {
        calls.set(name, [...(calls.get(name) ?? []), args])
        return opts?.async ? Promise.resolve(0) : 0
      }) as CFunction
    },
    HEAPU8: new Uint8Array(32),
    HEAP32: new Int32Array(8),
    HEAPF64: new Float64Array(4),
  } satisfies WasmModule

  return { module, cwrapOptions, calls }
}

describe('native async wrappers', () => {
  it('wraps joint control calls with Asyncify-aware cwrap functions', () => {
    const { module, cwrapOptions } = createRecordingModule()

    createNativeApi(module)

    expect(cwrapOptions.get('joint_sdk_manager_add_joint_json')?.async).toBe(true)
    for (const name of [
      'joint_sdk_joint_pvt_control',
      'joint_sdk_joint_pos_control',
      'joint_sdk_joint_spd_control',
      'joint_sdk_joint_cur_control',
      'joint_sdk_joint_tor_control',
      'joint_sdk_joint_stop',
    ]) {
      expect(cwrapOptions.get(name)?.async, name).toBe(true)
    }
  })

  it('waits for native async PVT control before resolving JointWrapper.pvtControl', async () => {
    let resolveNative: (() => void) | undefined
    const native = {
      jointPVTControl: () =>
        new Promise<void>((resolve) => {
          resolveNative = resolve
        }),
      jointSetStatusCallback: () => undefined,
    } as unknown as NativeApi
    const joint = new JointWrapper(native, 1, 'joint_1')

    let settled = false
    const pending = joint.pvtControl(0, 0, 0, 0, 0).then(() => {
      settled = true
    })

    await Promise.resolve()
    expect(settled).toBe(false)

    resolveNative?.()
    await pending
    expect(settled).toBe(true)
  })

  it('preserves all numeric PVT modes at the WASM boundary', async () => {
    const { module, calls } = createRecordingModule()
    const native = createNativeApi(module)

    for (const mode of [0, 1, 2]) {
      await native.jointPVTControl(7, 1, 2, 3, 4, 5, mode)
    }

    expect(calls.get('joint_sdk_joint_pvt_control')).toEqual([
      [7, 1, 2, 3, 4, 5, 0], [7, 1, 2, 3, 4, 5, 1], [7, 1, 2, 3, 4, 5, 2],
    ])
  })

  it.each([-1, 3, 4, 5, 6, 1.5, NaN, true])('rejects invalid PVT mode %s before calling WASM', async (mode) => {
    const { module, calls } = createRecordingModule()
    const native = createNativeApi(module)
    await expect(native.jointPVTControl(7, 1, 2, 3, 4, 5, mode as any)).rejects.toThrow(/mode/iu)
    expect(calls.get('joint_sdk_joint_pvt_control')).toBeUndefined()
  })
})

describe('brake wrappers', () => {
  it('keeps capability synchronous and wraps communication with Asyncify', () => {
    const { module, cwrapOptions } = createRecordingModule()
    createNativeApi(module)
    expect(cwrapOptions.get('joint_sdk_joint_has_brake')?.async).not.toBe(true)
    for (const name of ['brake', 'get_brake_status']) {
      expect(cwrapOptions.get(`joint_sdk_joint_${name}`)?.async).toBe(true)
    }
  })

  it.each([0, 1])('preserves native boolean output %s through JointWrapper', async (value) => {
    const { module, calls } = createRecordingModule()
    module.getValue = () => value
    const joint = new JointWrapper(createNativeApi(module), 17, 'brake_joint')
    expect(joint.hasBrake()).toBe(value !== 0)
    expect(await joint.brake(true)).toBe(value !== 0)
    expect(await joint.brake(false)).toBe(value !== 0)
    expect(await joint.getBrakeStatus()).toBe(value !== 0)
    expect(calls.get('joint_sdk_joint_brake')).toEqual([[17, 1, 8], [17, 0, 8]])
    for (const name of ['has_brake', 'get_brake_status']) {
      expect(calls.get(`joint_sdk_joint_${name}`)).toEqual([[17, 8]])
    }
  })

  it.each(['brake', 'getBrakeStatus'] as const)(
    'waits for %s and preserves rejection', async (method) => {
      let resolveNative: ((value: boolean) => void) | undefined
      const nativeMethod = {
        brake: 'jointBrake',
        getBrakeStatus: 'jointGetBrakeStatus',
      }[method]
      const native = {
        [nativeMethod]: () => new Promise<boolean>((resolve) => { resolveNative = resolve }),
      } as unknown as NativeApi
      const joint = new JointWrapper(native, 1, 'brake_joint')
      const invoke = () => method === 'brake' ? joint.brake(true) : joint.getBrakeStatus()
      let settled = false
      const pending = invoke().then((value) => { settled = true; return value })
      await Promise.resolve()
      expect(settled).toBe(false)
      resolveNative!(true)
      expect(await pending).toBe(true)
      Object.assign(native, { [nativeMethod]: () => Promise.reject(new Error('driver failure')) })
      await expect(invoke()).rejects.toThrow('driver failure')
    },
  )

  it('rejects all brake calls on disposed wrappers', () => {
    const { module, calls } = createRecordingModule()
    const joint = new JointWrapper(createNativeApi(module), 1, 'disposed_brake_joint')
    joint.dispose()
    expect(() => joint.brake(true)).toThrow(/disposed/u)
    expect(() => joint.brake(false)).toThrow(/disposed/u)
    for (const method of ['hasBrake', 'getBrakeStatus'] as const) {
      expect(() => joint[method]()).toThrow(/disposed/u)
    }
    expect(calls.has('joint_sdk_joint_has_brake')).toBe(false)
  })
})
