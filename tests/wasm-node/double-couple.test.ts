// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

import { describe, it, expect, beforeAll, afterAll, beforeEach, afterEach } from 'vitest'
import {
  createJointRuntime,
  JointError,
  EncosWasmError,
  type JointRuntime,
  type JointManagerWrapper,
} from '@encos/joint-sdk'

const ADAPTER_NAME = 'fake_dc'

const calibration = {
  leftJointCenter: { x: -0.1, y: 0, z: 0 },
  rightJointCenter: { x: 0.1, y: 0, z: 0 },
  leftPlanarEndpoint: { x: -0.1, y: 0.1, z: 0 },
  rightPlanarEndpoint: { x: 0.1, y: 0.1, z: 0 },
  rollCenter: { x: 0, y: 0.05, z: 0 },
  pitchCenter: { x: 0, y: 0.05, z: 0 },
  leftSpatialEndpoint: { x: -0.05, y: 0.1, z: 0.05 },
  rightSpatialEndpoint: { x: 0.05, y: 0.1, z: 0.05 },
  rotationOrder: 'PitchThenRoll',
}

const dcDefinition = (name: string, axis: 'roll' | 'pitch') => ({
  type: 'DoubleCoupleJoint',
  name,
  config: {
    motorParams1: {
      adapterId: ADAPTER_NAME,
      busId: 1,
      motorId: 1,
      initParam: { model: 'EC_A2806_P2' },
    },
    motorParams2: {
      adapterId: ADAPTER_NAME,
      busId: 1,
      motorId: 2,
      initParam: { model: 'EC_A2806_P2' },
    },
    axis,
    calibration,
    limit: { min: -1, max: 1 },
    maxSpd: 2,
    maxTor: 3,
    reverse: false,
  },
})

describe('DoubleCoupleJoint WASM smoke', () => {
  let runtime: JointRuntime
  let manager: JointManagerWrapper

  beforeAll(async () => {
    runtime = await createJointRuntime()
  })

  beforeEach(() => {
    manager = runtime.createJointManager({
      adapterType: 'Fake',
      adapterName: ADAPTER_NAME,
    })
  })

  afterEach(() => {
    manager.dispose()
  })

  afterAll(() => {
    runtime.dispose()
  })

  it('pairs roll and pitch joints', async () => {
    const roll = await manager.addJoint(dcDefinition('dc_roll', 'roll'))
    const pitch = await manager.addJoint(dcDefinition('dc_pitch', 'pitch'))
    expect(roll).toBeDefined()
    expect(pitch).toBeDefined()

    manager.fake!.injectFeedback(1, 1, { position: 0, speed: 0, current: 0, motorTemperature: 25, mosTemperature: 25, error: 0 })
    manager.fake!.injectFeedback(1, 2, { position: 0, speed: 0, current: 0, motorTemperature: 25, mosTemperature: 25, error: 0 })
    const status = manager.getStatus({ autoSend: false, lifeCycleDeduction: 1 })
    expect(status).toHaveProperty('dc_roll')
    expect(status).toHaveProperty('dc_pitch')
  })

  it('rejects duplicate axis', async () => {
    await manager.addJoint(dcDefinition('dc_roll_2', 'roll'))
    let thrown = false
    try {
      await manager.addJoint(dcDefinition('dc_pitch_2', 'roll'))
    } catch (error) {
      thrown = true
      expect(error).toBeInstanceOf(EncosWasmError)
      expect((error as EncosWasmError).jointError).toBe(JointError.DoubleCoupleJointDuplicateAxis)
    }
    expect(thrown).toBe(true)
  })

  it('lists DoubleCoupleJoint in schema', async () => {
    const schema = manager.getJointSchema()
    expect(schema).toHaveProperty('DoubleCoupleJoint')
    const dcSchema = schema.DoubleCoupleJoint as Record<string, unknown>
    expect(dcSchema).toHaveProperty('properties')
  })
})
