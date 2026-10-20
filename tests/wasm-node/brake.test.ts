// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

import { describe, expect, it } from 'vitest'
import { createJointRuntime } from '@encos/joint-sdk'
import { createNativeApi, EncosWasmError } from '../../npm/src/native.js'
import { createDefaultHost } from '../../npm/src/host.js'

const calibration = {
  leftJointCenter: { x: -35.462080740, y: 29.491263548, z: 58.546474855 },
  rightJointCenter: { x: -35.462080740, y: -29.491263548, z: 58.546474855 },
  leftPlanarEndpoint: { x: -35.5, y: 11.61629736, z: 56.42889465 },
  rightPlanarEndpoint: { x: -35.5, y: -11.61629736, z: 56.42889465 },
  rollCenter: { x: 0, y: 0, z: 0 },
  pitchCenter: { x: 0, y: 0, z: 0 },
  leftSpatialEndpoint: { x: -36.6, y: 26, z: -5 },
  rightSpatialEndpoint: { x: -36.6, y: -26, z: -5 },
  rotationOrder: 'PitchThenRoll',
}

describe('brake WASM integration', () => {
  it.each(['RJoint', 'ContinuousJoint', 'DoubleCoupleJoint'])('%s rejects non-brake models without commands', async (type) => {
    const runtime = await createJointRuntime()
    const manager = runtime.createJointManager({ adapterType: 'Fake', adapterName: `brake_${type}` })
    const motor = (motorId: number) => ({
      adapterId: `brake_${type}`, busId: 0, motorId, initParam: { model: 'EC_A4310_P2' },
    })
    const common = { maxSpd: 2, maxTor: 3, reverse: false }
    try {
      const joints = []
      if (type === 'DoubleCoupleJoint') {
        for (const axis of ['roll', 'pitch']) {
          joints.push(await manager.addJoint({
            type, name: axis,
            config: { ...common, motorParams1: motor(1), motorParams2: motor(2), axis, calibration, limit: { min: -1, max: 1 } },
          }))
        }
      } else {
        joints.push(await manager.addJoint({
          type, name: 'brake_joint',
          config: { ...common, motorParams: motor(1), ...(type === 'RJoint' ? { limit: { min: -1, max: 1 } } : {}) },
        }))
      }
      manager.fake!.clearCommandRecords()
      for (const joint of joints) {
        expect(joint.hasBrake()).toBe(false)
        expect(await joint.brake(true)).toBe(false)
        expect(await joint.brake(false)).toBe(false)
        expect(await joint.getBrakeStatus()).toBe(false)
      }
      expect(manager.fake!.getCommandCount()).toBe(0)
    } finally {
      manager.dispose()
      runtime.dispose()
    }
  })

  it('validates handles and output pointers at the WASM boundary', async () => {
    const host = await createDefaultHost()
    const module = await host.loadWasmModule()
    const runtime = await createJointRuntime({ module })
    try {
      const native = createNativeApi(module)
      expect(() => native.jointHasBrake(0)).toThrow(EncosWasmError)
      await expect(native.jointBrake(0, true)).rejects.toThrow(/Invalid joint handle/u)
      await expect(native.jointBrake(0, false)).rejects.toThrow(/Invalid joint handle/u)
      await expect(native.jointGetBrakeStatus(0)).rejects.toThrow(/Invalid joint handle/u)
      const brakeFn = module.cwrap('joint_sdk_joint_brake', 'number', ['number', 'number', 'number'])
      expect(brakeFn(0, 1, 0)).toBe(1)
      expect(brakeFn(0, 0, 0)).toBe(1)
      for (const name of ['has_brake', 'get_brake_status']) {
        const fn = module.cwrap(`joint_sdk_joint_${name}`, 'number', ['number', 'number'])
        expect(fn(0, 0)).toBe(1)
      }
    } finally {
      runtime.dispose()
    }
  })
})
