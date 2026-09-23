// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

import { describe, it, expect, beforeAll, afterAll } from 'vitest'
import {
  createJointRuntime,
  FakeCommandKind,
  MotorStopMode,
  MotorModel,
  type JointRuntime,
  type JointManagerWrapper,
  type JointWrapper,
} from '@encos/joint-sdk'

const rJointDefinition = {
  type: 'RJoint',
  name: 'test_r_joint',
  config: {
    motorParams: {
      adapterId: 'fake_adapter',
      busId: 1,
      motorId: 1,
      initParam: { model: 'EC_A2806_P2' },
    },
    limit: { min: -3.14159, max: 3.14159 },
    maxSpd: 10,
    maxTor: 5,
    reverse: false,
  },
}

const sleep = (ms: number) => new Promise((resolve) => setTimeout(resolve, ms))

describe('JointRuntime', () => {
  let runtime: JointRuntime

  beforeAll(async () => {
    runtime = await createJointRuntime()
  })

  it('reports a version', async () => {
    expect(runtime.version()).toMatch(/^\d+\.\d+\.\d+/u)
  })

  it('creates a manager', async () => {
    const manager = runtime.createJointManager({
      adapterType: 'Fake',
      adapterName: 'fake_adapter',
    })
    expect(manager.handle).toBeGreaterThan(0)
    manager.dispose()
  })

  it('creates multiple adapters in one manager', async () => {
    const manager = runtime.createJointManager({
      loggerName: 'multi_adapter_manager',
    })
    manager.createAdapter('Fake', 'fake_adapter_a')
    manager.createAdapter('Fake', 'fake_adapter_b')

    const jointA = await manager.addJoint({
      ...rJointDefinition,
      name: 'joint_a',
      config: {
        ...rJointDefinition.config,
        motorParams: {
          ...rJointDefinition.config.motorParams,
          adapterId: 'fake_adapter_a',
          busId: 1,
          motorId: 1,
        },
      },
    })
    const jointB = await manager.addJoint({
      ...rJointDefinition,
      name: 'joint_b',
      config: {
        ...rJointDefinition.config,
        motorParams: {
          ...rJointDefinition.config.motorParams,
          adapterId: 'fake_adapter_b',
          busId: 1,
          motorId: 2,
        },
      },
    })

    expect(jointA.name).toBe('joint_a')
    expect(jointB.name).toBe('joint_b')
    manager.dispose()
  })
})

describe('JointManagerWrapper with Fake adapter', () => {
  let runtime: JointRuntime
  let manager: JointManagerWrapper
  let joint: JointWrapper

  beforeAll(async () => {
    runtime = await createJointRuntime()
    manager = runtime.createJointManager({
      adapterType: 'Fake',
      adapterName: 'fake_adapter',
    })
    joint = await manager.addJoint(rJointDefinition)
  })

  afterAll(() => {
    runtime.dispose()
  })

  it('lists available joint types', async () => {
    const types = manager.getAvailableJointTypes()
    expect(types).toContain('RJoint')
    expect(types).toContain('DoubleCoupleJoint')
  })

  it('returns joint schema', async () => {
    const schema = manager.getJointSchema()
    expect(schema).toHaveProperty('RJoint')
    expect(schema).toHaveProperty('DoubleCoupleJoint')
  })

  it('returns null for missing joint', async () => {
    expect(manager.getJoint('missing')).toBeNull()
  })

  it('exposes fake tools for Fake managers', async () => {
    expect(manager.fake).toBeDefined()
    expect(manager.fake).toBeInstanceOf(Object)
  })

  it('injects feedback and reads joint status', async () => {
    manager.fake!.injectFeedback(1, 1, {
      position: 0.5,
      speed: 0.1,
      current: 0.2,
      motorTemperature: 30,
      mosTemperature: 40,
      error: 0,
    })

    const status = joint.getStatus({ autoSend: false, lifeCycleDeduction: 1 })
    expect(status).not.toBeNull()
    expect(status!.pos).toBeCloseTo(0.5)
    expect(status!.spd).toBeCloseTo(0.1, 1)
    expect(status!.motorTemperature).toBeCloseTo(30)
    expect(status!.mosTemperature).toBeCloseTo(40)
  })

  it('sends position control and records a Fake command', async () => {
    manager.fake!.clearCommandRecords()
    await joint.posControl(1.0, 1.0, 0.5, 1)
    expect(manager.fake!.getCommandCount()).toBeGreaterThan(0)
    const record = manager.fake!.getLastCommand()
    expect(record).not.toBeNull()
    expect(record!.kind).toBe(FakeCommandKind.PosControl)
    expect(record!.busIndex).toBe(1)
    expect(record!.motorIndex).toBe(1)
  })

  it('reads decoded PosControl fields', async () => {
    manager.fake!.clearCommandRecords()
    await joint.posControl(1.0, 1.0, 0.5, 1)
    const cmd = manager.fake!.getPosCommand(0)
    expect(cmd.position).toBeCloseTo(1.0)
    expect(cmd.speed).toBeCloseTo(1.0, 1)
    expect(cmd.current).toBeCloseTo(0.5)
    expect(cmd.feedbackType).toBe(1)
  })

  it('sends stop and records stop command', async () => {
    manager.fake!.clearCommandRecords()
    await joint.stop(MotorStopMode.FullBrake, 0.5, 1)
    const cmd = manager.fake!.getStopCommand(0)
    expect(cmd.mode).toBe(MotorStopMode.FullBrake)
    expect(cmd.current).toBeCloseTo(0.5)
  })

  it('sets status callback and receives updates', () =>
    new Promise<void>((resolve) => {
      manager.fake!.clearCommandRecords()
      const unsubscribe = joint.onStatus((status) => {
        expect(status.pos).toBeCloseTo(0.75)
        unsubscribe()
        resolve()
      })
      manager.fake!.injectFeedback(1, 1, {
        position: 0.75,
        speed: 0,
        current: 0,
        motorTemperature: 25,
        mosTemperature: 25,
      })
    }))

  it('reads motor snapshot', async () => {
    const snapshot = manager.fake!.getMotorSnapshot(1, 1)
    expect(snapshot.model).toBe(MotorModel.EC_A2806_P2)
    expect(snapshot.brakeEnabled).toBe(false)
  })

  it('toggles fake position error', async () => {
    manager.fake!.disablePositionError()
    await joint.spdControl(2.0, 1.0)
    const initial = manager.fake!.getMotorSnapshot(1, 1).position
    const start = performance.now()
    await sleep(100)
    await joint.spdControl(1.0, 0.5)
    const elapsed = (performance.now() - start) / 1000

    const snapshot = manager.fake!.getMotorSnapshot(1, 1)
    expect(snapshot.position - initial).toBeCloseTo(2.0 * elapsed, 1)
    manager.fake!.enablePositionError()
  })

  it('toggles CAN FD', async () => {
    manager.enableCanFd()
    expect(manager.isCanFdEnabled()).toBe(true)
    manager.disableCanFd()
    expect(manager.isCanFdEnabled()).toBe(false)
  })

  it('returns null status after a previous missing-joint lookup', async () => {
    expect(manager.getJoint('does_not_exist')).toBeNull()
    const fresh = await manager.addJoint({
      ...rJointDefinition,
      name: 'stale_error_status',
      config: {
        ...rJointDefinition.config,
        motorParams: {
          ...rJointDefinition.config.motorParams,
          motorId: 3,
        },
      },
    })
    expect(fresh.getStatus({ autoSend: false, lifeCycleDeduction: 0 })).toBeNull()
  })

  it('returns the existing wrapper for a registered joint', async () => {
    const joint = await manager.addJoint({
      ...rJointDefinition,
      name: 'stable_handle',
      config: {
        ...rJointDefinition.config,
        motorParams: {
          ...rJointDefinition.config.motorParams,
          motorId: 2,
        },
      },
    })
    expect(manager.getJoint('stable_handle')).toBe(joint)
  })
})
