// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

import { describe, it, expect, beforeAll, afterAll, beforeEach, afterEach } from 'vitest'
import {
  createJointRuntime,
  EncosWasmError,
  FakeCommandKind,
  JointError,
  LogLevel,
  MotorModel,
  MotorStopMode,
  PVTControlMode,
  type JointRuntime,
  type JointManagerWrapper,
  type JointWrapper,
} from '@encos/joint-sdk'

const ADAPTER_NAME = 'fake_framework'
const BUS_ID = 1

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

const h130AnkleCalibration = {
  leftJointCenter: { x: -27.5, y: 0, z: 260 },
  rightJointCenter: { x: -27.5, y: 0, z: 200 },
  leftPlanarEndpoint: { x: -27.5, y: 17.497758355829838, z: 266.00653415215356 },
  rightPlanarEndpoint: { x: -27.5, y: -17.495529250415565, z: 206.0130238855216 },
  rollCenter: { x: 0, y: 0, z: -30 },
  pitchCenter: { x: 0, y: 0, z: 0 },
  leftSpatialEndpoint: { x: -24.76, y: 12, z: 3.48 },
  rightSpatialEndpoint: { x: -24.76, y: -12, z: 3.48 },
  rotateDirection: { x: -1, y: 0, z: 0 },
  useRotateDirection: true,
  rotationOrder: 'PitchThenRoll',
}

function motorParams(adapterId: string, motorId: number, initParam: Record<string, unknown> = { model: 'EC_A2806_P2' }) {
  return {
    adapterId,
    busId: BUS_ID,
    motorId,
    initParam,
  }
}

function rJointDef(
  name: string,
  opts: {
    reverse?: boolean
    limit?: { min: number; max: number }
    maxSpd?: number
    maxTor?: number
    initParam?: Record<string, unknown>
  } = {},
) {
  return {
    type: 'RJoint',
    name,
    config: {
      motorParams: motorParams(ADAPTER_NAME, 1, opts.initParam ?? { model: 'EC_A2806_P2' }),
      limit: opts.limit ?? { min: -1, max: 1 },
      maxSpd: opts.maxSpd ?? 2,
      maxTor: opts.maxTor ?? 3,
      reverse: opts.reverse ?? false,
    },
  }
}

function continuousJointDef(
  name: string,
  opts: {
    reverse?: boolean
    maxSpd?: number
    maxTor?: number
    initParam?: Record<string, unknown>
    motorId?: number
    adapterId?: string
  } = {},
) {
  return {
    type: 'ContinuousJoint',
    name,
    config: {
      motorParams: motorParams(
        opts.adapterId ?? ADAPTER_NAME,
        opts.motorId ?? 1,
        opts.initParam ?? { model: 'EC_A2806_P2' },
      ),
      maxSpd: opts.maxSpd ?? 2,
      maxTor: opts.maxTor ?? 3,
      reverse: opts.reverse ?? false,
    },
  }
}

function dcJointDef(
  name: string,
  axis: 'roll' | 'pitch',
  opts: {
    reverse?: boolean
    limit?: { min: number; max: number }
    motorLimit1?: { min: number; max: number }
    motorLimit2?: { min: number; max: number }
    initParam?: Record<string, unknown>
    motor1Id?: number
    motor2Id?: number
    adapterId?: string
  } = {},
) {
  const adapterId = opts.adapterId ?? ADAPTER_NAME
  const config: Record<string, unknown> = {
    motorParams1: motorParams(adapterId, opts.motor1Id ?? 1, opts.initParam ?? { model: 'EC_A2806_P2' }),
    motorParams2: motorParams(adapterId, opts.motor2Id ?? 2, opts.initParam ?? { model: 'EC_A2806_P2' }),
    axis,
    calibration,
    limit: opts.limit ?? { min: -1, max: 1 },
    maxSpd: 2,
    maxTor: 3,
    reverse: opts.reverse ?? false,
  }
  if (opts.motorLimit1) {
    config.motorLimit1 = opts.motorLimit1
  }
  if (opts.motorLimit2) {
    config.motorLimit2 = opts.motorLimit2
  }
  return { type: 'DoubleCoupleJoint', name, config }
}

function h130AnkleDcJointDef(name: string, axis: 'roll' | 'pitch') {
  return {
    type: 'DoubleCoupleJoint',
    name,
    config: {
      motorParams1: motorParams(ADAPTER_NAME, 5, {}),
      motorParams2: motorParams(ADAPTER_NAME, 6, {}),
      axis,
      calibration: h130AnkleCalibration,
      motorLimit1: { min: -1.2217304763960306, max: 1.2217304763960306 },
      motorLimit2: { min: -1.2217304763960306, max: 1.2217304763960306 },
      limit: axis === 'pitch'
        ? { min: -0.8726646, max: 0.5235988 }
        : { min: -0.2617994, max: 0.2617994 },
      maxSpd: axis === 'pitch' ? 27.25 : 57.37,
      maxTor: axis === 'pitch' ? 80 : 38,
      reverse: false,
    },
  }
}

async function expectEncosError(
  fn: () => void | Promise<unknown>,
  expectedCode: number,
  expectedJointError?: number,
): Promise<EncosWasmError> {
  try {
    await fn()
  } catch (error) {
    expect(error).toBeInstanceOf(EncosWasmError)
    const err = error as EncosWasmError
    expect(err.code).toBe(expectedCode)
    if (expectedJointError !== undefined) {
      expect(err.jointError).toBe(expectedJointError)
    }
    return err
  }
  throw new Error('Expected EncosWasmError but no exception was thrown')
}

async function createManager(runtime: JointRuntime, adapterName = ADAPTER_NAME) {
  return runtime.createJointManager({
    adapterType: 'Fake',
    adapterName,
    logLevel: LogLevel.Off,
  })
}

describe('JointRuntime', () => {
  let runtime: JointRuntime

  beforeAll(async () => {
    runtime = await createJointRuntime()
  })

  afterAll(() => {
    runtime.dispose()
  })

  it('reports a version in semantic form', async () => {
    expect(runtime.version()).toMatch(/^\d+\.\d+\.\d+/u)
  })

  it('creates and disposes managers independently', async () => {
    const m1 = runtime.createJointManager({ adapterType: 'Fake', adapterName: `${ADAPTER_NAME}_1` })
    const m2 = runtime.createJointManager({ adapterType: 'Fake', adapterName: `${ADAPTER_NAME}_2` })
    expect(m1.handle).not.toBe(m2.handle)
    m1.dispose()
    m2.dispose()
  })
})

describe('JointManager type system', () => {
  let runtime: JointRuntime
  let manager: JointManagerWrapper

  beforeAll(async () => {
    runtime = await createJointRuntime()
    manager = await createManager(runtime)
  })

  afterAll(() => {
    manager.dispose()
    runtime.dispose()
  })

  it('exposes all three joint types', async () => {
    const types = manager.getAvailableJointTypes()
    expect(types).toContain('RJoint')
    expect(types).toContain('ContinuousJoint')
    expect(types).toContain('DoubleCoupleJoint')
  })

  it('returns parseable schemas for all three joint types', async () => {
    const schema = manager.getJointSchema()
    expect(schema).toHaveProperty('RJoint')
    expect(schema).toHaveProperty('ContinuousJoint')
    expect(schema).toHaveProperty('DoubleCoupleJoint')
    const r = schema.RJoint as Record<string, unknown>
    const continuous = schema.ContinuousJoint as Record<string, unknown>
    const dc = schema.DoubleCoupleJoint as Record<string, unknown>
    expect(r.properties).toBeInstanceOf(Object)
    expect(continuous.properties).toBeInstanceOf(Object)
    expect(continuous.properties).not.toHaveProperty('limit')
    expect(dc.properties).toBeInstanceOf(Object)
    expect(dc.properties).toHaveProperty('axis')
    expect(dc.properties).toHaveProperty('calibration')
  })

  it('returns null for a missing joint', async () => {
    expect(manager.getJoint('missing')).toBeNull()
  })

  it('rejects duplicate joint names before parsing config', async () => {
    await manager.addJoint(rJointDef('dup_name'))
    const err = await expectEncosError(async () => await manager.addJoint(rJointDef('dup_name')), 1)
    expect(err.message).toContain('already exists')
  })

  it('rejects unknown joint types without requiring config', async () => {
    const err = await expectEncosError(
      async () => await manager.addJoint({ type: 'UnknownJoint', name: 'unknown', config: {} }),
      1,
    )
    expect(err.message).toContain('Unknown joint type')
  })

  it('adds ContinuousJoint from JSON without a position limit', async () => {
    const joint = await manager.addJoint(continuousJointDef('continuous_json', { motorId: 3 }))
    expect(manager.getJoint('continuous_json')?.handle).toBe(joint.handle)
  })

  it('toggles CAN FD state and reflects it in command flags', async () => {
    const joint = await manager.addJoint(rJointDef('canfd_joint'))

    manager.fake!.clearCommandRecords()
    joint.posControl(0.1, 1, 1, 1)
    const beforeFlags = manager.fake!.getLastCommand()!.rawFrameFlags

    manager.enableCanFd()
    expect(manager.isCanFdEnabled()).toBe(true)

    manager.fake!.clearCommandRecords()
    joint.posControl(0.2, 1, 1, 1)
    const afterFlags = manager.fake!.getLastCommand()!.rawFrameFlags

    manager.disableCanFd()
    expect(manager.isCanFdEnabled()).toBe(false)

    // The exact bit layout is private, but enabling CAN FD must change the flags.
    expect(afterFlags).not.toBe(beforeFlags)
  })
})

describe('RJoint construction and validation', () => {
  let runtime: JointRuntime
  let manager: JointManagerWrapper

  beforeAll(async () => {
    runtime = await createJointRuntime()
    manager = await createManager(runtime)
  })

  afterAll(() => {
    manager.dispose()
    runtime.dispose()
  })

  it('rejects non-positive torque constant', async () => {
    const def = rJointDef('bad_kt', {
      initParam: {
        ranges: {
          kp: { min: 0, max: 500 },
          kd: { min: 0, max: 5 },
          position: { min: -12.5, max: 12.5 },
          speed: { min: -18, max: 18 },
          torque: { min: -30, max: 30 },
          current: { min: -30, max: 30 },
          kt: 0,
        },
      },
    })
    await expectEncosError(async () => await manager.addJoint(def), 1)
  })

  it('rejects reversed position limit', async () => {
    await expectEncosError(async () => await manager.addJoint(rJointDef('bad_limit', { limit: { min: 1, max: -1 } })), 1)
  })

  it('rejects non-finite position limit', async () => {
    await expectEncosError(
      async () => await manager.addJoint(rJointDef('nan_limit', { limit: { min: -1, max: Number.NaN } })),
      1,
    )
  })

  it('rejects zero maximum speed', async () => {
    await expectEncosError(async () => await manager.addJoint(rJointDef('zero_spd', { maxSpd: 0 })), 1)
  })

  it('rejects negative maximum torque', async () => {
    await expectEncosError(async () => await manager.addJoint(rJointDef('neg_tor', { maxTor: -3 })), 1)
  })
})

describe('RJoint runtime controls', () => {
  let runtime: JointRuntime
  let manager: JointManagerWrapper
  let joint: JointWrapper

  beforeAll(async () => {
    runtime = await createJointRuntime()
    manager = await createManager(runtime)
    joint = await manager.addJoint(rJointDef('r_controls'))
  })

  afterAll(() => {
    manager.dispose()
    runtime.dispose()
  })

  it('records PVT command with decoded fields', async () => {
    manager.fake!.injectFeedback(BUS_ID, 1, {
      position: 0,
      speed: 0,
      current: 0,
      motorTemperature: 25,
      mosTemperature: 25,
      error: 0,
    })
    manager.fake!.clearCommandRecords()
    await joint.pvtControl(10, 1, 0.25, 0.5, 0.75)
    const cmd = manager.fake!.getPVTCommand(0)
    expect(cmd.kind).toBe(FakeCommandKind.PVTControl)
    expect(cmd.kp).toBeCloseTo(10, 0)
    expect(cmd.position).toBeCloseTo(0.25)
    expect(cmd.speed).toBeCloseTo(0.5, 1)
    expect(cmd.torque).toBeCloseTo(0.75, 1)
  })

  it('records position control command with decoded fields', async () => {
    manager.fake!.clearCommandRecords()
    await joint.posControl(0.25, 1, 2, 1)
    const cmd = manager.fake!.getPosCommand(0)
    expect(cmd.position).toBeCloseTo(0.25)
    expect(cmd.feedbackType).toBe(1)
  })

  it('records speed control command', async () => {
    manager.fake!.clearCommandRecords()
    await joint.spdControl(1.5, 2, 1)
    const cmd = manager.fake!.getSpdCommand(0)
    expect(cmd.speed).toBeCloseTo(1.5)
  })

  it('records current control command', async () => {
    manager.fake!.clearCommandRecords()
    await joint.curControl(1.2, 1)
    const cmd = manager.fake!.getCurCommand(0)
    expect(cmd.current).toBeCloseTo(1.2)
  })

  it('records torque control command', async () => {
    manager.fake!.clearCommandRecords()
    await joint.torControl(0.8, 1)
    const cmd = manager.fake!.getTorCommand(0)
    expect(cmd.torque).toBeCloseTo(0.8)
  })

  it('records stop command and sanitizes non-finite current', async () => {
    manager.fake!.clearCommandRecords()
    await joint.stop(MotorStopMode.FullBrake, Number.POSITIVE_INFINITY, 1)
    const cmd = manager.fake!.getStopCommand(0)
    expect(cmd.mode).toBe(MotorStopMode.FullBrake)
    expect(cmd.current).toBeCloseTo(0)
  })

  it('rejects non-finite control inputs and emits a safety stop', async () => {
    manager.fake!.clearCommandRecords()
    const err = await expectEncosError(() => joint.posControl(Number.NaN, 1, 1, 1), 1)
    expect(err.message).toContain('finite')
    const last = manager.fake!.getLastCommand()
    expect(last).not.toBeNull()
    expect(last!.kind).toBe(FakeCommandKind.Stop)
    expect(manager.fake!.getStopCommand(manager.fake!.getCommandCount() - 1).current).toBeCloseTo(0)
  })

  it('rejects invalid setLimit range', async () => {
    await expectEncosError(() => joint.setLimit({ min: 1, max: -1 }), 1)
    await expectEncosError(() => joint.setLimit({ min: -1, max: Number.NaN }), 1)
  })

  it('reads status from injected feedback', async () => {
    manager.fake!.injectFeedback(BUS_ID, 1, {
      position: 0.3,
      speed: 0.4,
      current: 0.5,
      motorTemperature: 35,
      mosTemperature: 45,
      error: 0,
    })
    const status = joint.getStatus({ autoSend: false, lifeCycleDeduction: 1 })
    expect(status).not.toBeNull()
    expect(status!.pos).toBeCloseTo(0.3)
    expect(status!.spd).toBeCloseTo(0.4)
    expect(status!.motorTemperature).toBeCloseTo(35)
    expect(status!.mosTemperature).toBeCloseTo(45)
  })

  it('delivers status callbacks', () =>
    new Promise<void>((resolve) => {
      manager.fake!.clearCommandRecords()
      const unsubscribe = joint.onStatus((status) => {
        expect(status.pos).toBeCloseTo(0.6)
        unsubscribe()
        resolve()
      })
      manager.fake!.injectFeedback(BUS_ID, 1, {
        position: 0.6,
        speed: 0,
        current: 0,
        motorTemperature: 25,
        mosTemperature: 25,
      })
    }))
})

describe('ContinuousJoint runtime controls', () => {
  let runtime: JointRuntime
  let manager: JointManagerWrapper
  let joint: JointWrapper

  beforeAll(async () => {
    runtime = await createJointRuntime()
    manager = await createManager(runtime)
    joint = await manager.addJoint(continuousJointDef('continuous_controls', {
      initParam: {
        ranges: {
          kp: { min: 0, max: 500 },
          kd: { min: 0, max: 5 },
          position: { min: -12.5, max: 12.5 },
          speed: { min: -18, max: 18 },
          torque: { min: -30, max: 30 },
          current: { min: -30, max: 30 },
          kt: 1,
        },
      },
    }))
  })

  afterAll(() => {
    manager.dispose()
    runtime.dispose()
  })

  it('rejects position, PVT, limit, and calibration operations with a structured error', async () => {
    manager.fake!.clearCommandRecords()
    const unsupported = JointError.ContinuousJointUnsupportedOperation

    await expectEncosError(() => joint.pvtControl(1, 1, 0, 0, 0), 5, unsupported)
    await expectEncosError(() => joint.posControl(0), 5, unsupported)
    await expectEncosError(() => joint.setLimit({ min: -1, max: 1 }), 5, unsupported)
    await expectEncosError(() => joint.gotoLimit(1, 1, 1, 100), 5, unsupported)
    await expectEncosError(() => joint.gotoZero(0, 1, 1, 100), 5, unsupported)
    await expectEncosError(() => {
      joint.posAsOffset(1)
    }, 5, unsupported)
    await expectEncosError(() => joint.calibrate(1, 1, 1, 100), 5, unsupported)
    expect(manager.fake!.getCommandCount()).toBe(0)
  })

  it('keeps dynamic magnitude limits without applying position-limit protection', async () => {
    manager.fake!.injectFeedback(BUS_ID, 1, {
      position: 12.4,
      speed: 0,
      current: 0,
      motorTemperature: 25,
      mosTemperature: 25,
    })
    manager.fake!.clearCommandRecords()

    await joint.spdControl(10, 100, 1)
    await joint.curControl(100, 1)
    await joint.torControl(100, 1)

    expect(manager.fake!.getSpdCommand(0).speed).toBeCloseTo(2)
    expect(manager.fake!.getCurCommand(1).current).toBeCloseTo(3)
    expect(manager.fake!.getTorCommand(2).torque).toBeCloseTo(3)
  })

  it('unwraps feedback across the motor position range boundary', () => {
    manager.fake!.injectFeedback(BUS_ID, 1, {
      position: 12.4,
      speed: 0.2,
      current: 0.3,
      motorTemperature: 25,
      mosTemperature: 25,
    })
    expect(joint.getStatus({ autoSend: false, lifeCycleDeduction: 0 })!.pos).toBeCloseTo(12.4, 1)

    manager.fake!.injectFeedback(BUS_ID, 1, {
      position: -12.4,
      speed: 0.2,
      current: 0.3,
      motorTemperature: 25,
      mosTemperature: 25,
    })
    expect(joint.getStatus({ autoSend: false, lifeCycleDeduction: 0 })!.pos).toBeCloseTo(12.6, 1)
  })

  it('applies CAN FD manager changes to its motor', async () => {
    manager.disableCanFd()
    manager.fake!.clearCommandRecords()
    await joint.spdControl(1, 1, 1)
    const beforeFlags = manager.fake!.getLastCommand()!.rawFrameFlags

    manager.enableCanFd()
    manager.fake!.clearCommandRecords()
    await joint.spdControl(1, 1, 1)
    const afterFlags = manager.fake!.getLastCommand()!.rawFrameFlags

    expect(afterFlags).not.toBe(beforeFlags)
  })

  it('uses an internal PVT command for autoSend while public PVT remains unsupported', async () => {
    const adapterId = 'continuous_auto_send_adapter'
    const freshManager = await createManager(runtime, adapterId)
    try {
      const freshJoint = await freshManager.addJoint(
        continuousJointDef('continuous_auto_send', { adapterId }),
      )
      freshManager.fake!.clearCommandRecords()

      expect(freshJoint.getStatus({ autoSend: true, lifeCycleDeduction: 0 })).toBeNull()
      expect(freshManager.fake!.getCommandCount()).toBe(1)
      expect(freshManager.fake!.getLastCommand()!.kind).toBe(FakeCommandKind.PVTControl)
    } finally {
      freshManager.dispose()
    }
  })
})

describe('RJoint reverse coordinate frame', () => {
  let runtime: JointRuntime
  let manager: JointManagerWrapper
  let joint: JointWrapper

  beforeAll(async () => {
    runtime = await createJointRuntime()
    manager = await createManager(runtime)
    joint = await manager.addJoint(
      rJointDef('r_reverse', {
        reverse: true,
        limit: { min: -1, max: 1 },
        initParam: {
          ranges: {
            kp: { min: 0, max: 500 },
            kd: { min: 0, max: 5 },
            position: { min: -12.5, max: 12.5 },
            speed: { min: -18, max: 18 },
            torque: { min: -30, max: 30 },
            current: { min: -30, max: 30 },
            kt: 1,
          },
        },
      }),
    )
  })

  afterAll(() => {
    manager.dispose()
    runtime.dispose()
  })

  it('converts status from motor frame to joint frame', async () => {
    manager.fake!.injectFeedback(BUS_ID, 1, {
      position: -0.2,
      speed: -0.3,
      current: -0.4,
      motorTemperature: 25,
      mosTemperature: 25,
      error: 0,
    })
    const status = joint.getStatus({ autoSend: false, lifeCycleDeduction: 1 })
    expect(status!.pos).toBeCloseTo(0.2)
    expect(status!.spd).toBeCloseTo(0.3)
    expect(status!.tor).toBeCloseTo(0.4)
  })

  it('sends inverted position command', async () => {
    manager.fake!.clearCommandRecords()
    await joint.posControl(0.3, 1, 1, 1)
    const cmd = manager.fake!.getPosCommand(0)
    expect(cmd.position).toBeCloseTo(-0.3)
  })

  it('clamps position in joint coordinates before reversing', async () => {
    manager.fake!.clearCommandRecords()
    await joint.posControl(2.0, 1, 1, 1)
    const cmd = manager.fake!.getPosCommand(0)
    // limit.max - kLimitSlack, then reversed.
    expect(cmd.position).toBeCloseTo(-0.965, 0.01)
  })

  it('converts callback status to joint frame', () =>
    new Promise<void>((resolve) => {
      const unsubscribe = joint.onStatus((status) => {
        expect(status.pos).toBeCloseTo(0.15)
        expect(status.spd).toBeCloseTo(0.25)
        unsubscribe()
        resolve()
      })
      manager.fake!.injectFeedback(BUS_ID, 1, {
        position: -0.15,
        speed: -0.25,
        current: 0,
        motorTemperature: 25,
        mosTemperature: 25,
      })
    }))
})

describe('DoubleCoupleJoint pairing validation', () => {
  let runtime: JointRuntime
  let manager: JointManagerWrapper

  beforeAll(async () => {
    runtime = await createJointRuntime()
  })

  beforeEach(async () => {
    manager = runtime.createJointManager({
      adapterType: 'Fake',
      adapterName: ADAPTER_NAME,
      logLevel: LogLevel.Off,
    })
  })

  afterEach(() => {
    manager.dispose()
  })

  afterAll(() => {
    runtime.dispose()
  })

  it('pairs roll and pitch joints on the same motor pair', async () => {
    const roll = await manager.addJoint(dcJointDef('pair_roll', 'roll'))
    const pitch = await manager.addJoint(dcJointDef('pair_pitch', 'pitch'))
    expect(roll).toBeDefined()
    expect(pitch).toBeDefined()
  })

  it('rejects duplicate axis', async () => {
    await manager.addJoint(dcJointDef('dup_axis_a', 'roll'))
    const err = await expectEncosError(async () => await manager.addJoint(dcJointDef('dup_axis_b', 'roll')), 5)
    expect(err.jointError).toBe(JointError.DoubleCoupleJointDuplicateAxis)
  })

  it('rejects calibration mismatch', async () => {
    const cal2 = JSON.parse(JSON.stringify(calibration))
    cal2.leftJointCenter.x += 0.01
    await manager.addJoint(dcJointDef('cal_mismatch_roll', 'roll'))
    const err = await expectEncosError(
      async () =>
        await manager.addJoint({
          type: 'DoubleCoupleJoint',
          name: 'cal_mismatch_pitch',
          config: { ...dcJointDef('cal_mismatch_pitch', 'pitch').config, calibration: cal2 },
        }),
      5,
    )
    expect(err.jointError).toBe(JointError.DoubleCoupleJointCalibrationMismatch)
  })

  it('rejects conflicting motor limits', async () => {
    await manager.addJoint(dcJointDef('lim_mismatch_roll', 'roll', { motorLimit1: { min: -0.4, max: 0.4 } }))
    const err = await expectEncosError(
      async () =>
        await manager.addJoint(
          dcJointDef('lim_mismatch_pitch', 'pitch', { motorLimit1: { min: -0.5, max: 0.5 } }),
        ),
      5,
    )
    expect(err.jointError).toBe(JointError.DoubleCoupleJointMotorLimitMismatch)
  })

  it('rejects invalid motor limit range', async () => {
    await manager.addJoint(dcJointDef('lim_invalid_roll', 'roll'))
    await expectEncosError(
      async () => await manager.addJoint(dcJointDef('lim_invalid_pitch', 'pitch', { motorLimit1: { min: 0.3, max: -0.3 } })),
      1,
    )
  })

  it('accepts matching motor limits', async () => {
    await manager.addJoint(dcJointDef('match_lim_roll', 'roll', { motorLimit1: { min: -0.4, max: 0.4 } }))
    await expect(
      manager.addJoint(dcJointDef('match_lim_pitch', 'pitch', { motorLimit1: { min: -0.4, max: 0.4 } })),
    ).resolves.toBeDefined()
  })

  it('pending slave unsupported helpers return NaN or false', async () => {
    const slave = await manager.addJoint(dcJointDef('pending_pitch', 'pitch'))
    expect(Number.isNaN(await slave.gotoLimit(1, 1, 1, 1000))).toBe(true)
    expect(await slave.gotoZero(0, 1, 1, 1000)).toBe(false)
    expect(Number.isNaN(slave.posAsOffset(1))).toBe(true)
    expect(await slave.calibrate(1, 1, 1, 1000)).toBe(false)
  })
})

describe('DoubleCoupleJoint controls', () => {
  let runtime: JointRuntime
  let manager: JointManagerWrapper
  let roll: JointWrapper
  let pitch: JointWrapper

  beforeAll(async () => {
    runtime = await createJointRuntime()
    manager = await createManager(runtime)
    roll = await manager.addJoint(dcJointDef('dc_roll', 'roll'))
    pitch = await manager.addJoint(dcJointDef('dc_pitch', 'pitch'))
    const ok = await roll.setZero()
    expect(ok).toBe(true)
  })

  afterAll(() => {
    manager.dispose()
    runtime.dispose()
  })

  it('PVT control sends one command per motor', async () => {
    manager.fake!.clearCommandRecords()
    await roll.pvtControl(10, 1, 0.05, 0.4, 0.5)
    await roll.pvtControl(10, 1, 0.05, 0.4, 0.5)
    expect(manager.fake!.getCommandCount()).toBe(2)
    const recs = manager.fake!.getCommandRecords()
    expect(recs.every((r) => r.kind === FakeCommandKind.PVTControl)).toBe(true)
    expect(recs.map((r) => r.motorIndex).sort()).toEqual([1, 2])
  })

  it('position control sends one command per motor', async () => {
    manager.fake!.clearCommandRecords()
    await roll.posControl(0.04, 1, 1, 1)
    await roll.posControl(0.04, 1, 1, 1)
    expect(manager.fake!.getCommandCount()).toBe(2)
    const recs = manager.fake!.getCommandRecords()
    expect(recs.every((r) => r.kind === FakeCommandKind.PosControl)).toBe(true)
  })

  it('speed control sends one command per motor', async () => {
    manager.fake!.clearCommandRecords()
    await roll.spdControl(1.0, 0.5, 1)
    // 动态类控制按合帧下发，需要另一轴各一次调用
    await pitch.spdControl(0, 0.5, 1)
    expect(manager.fake!.getCommandCount()).toBe(2)
    expect(manager.fake!.getCommandRecords().every((r) => r.kind === FakeCommandKind.SpdControl)).toBe(true)
  })

  it('torque control sends one command per motor', async () => {
    manager.fake!.clearCommandRecords()
    await roll.torControl(1.0, 1)
    await pitch.torControl(0, 1)
    expect(manager.fake!.getCommandCount()).toBe(2)
    expect(manager.fake!.getCommandRecords().every((r) => r.kind === FakeCommandKind.TorControl)).toBe(true)
  })

  it('current control sends one command per motor', async () => {
    manager.fake!.clearCommandRecords()
    await roll.curControl(1.25, 1)
    await pitch.curControl(0, 1)
    expect(manager.fake!.getCommandCount()).toBe(2)
    const recs = manager.fake!.getCommandRecords()
    expect(recs.every((r) => r.kind === FakeCommandKind.CurControl)).toBe(true)
    const total = recs.reduce((sum, r) => sum + Math.abs(manager.fake!.getCurCommand(recs.indexOf(r)).current), 0)
    expect(total).toBeCloseTo(1.25, 0.03)
  })

  it('stop sends one stop per motor', async () => {
    manager.fake!.clearCommandRecords()
    await roll.stop(MotorStopMode.FullBrake, 0.5, 1)
    expect(manager.fake!.getCommandCount()).toBe(2)
    const recs = manager.fake!.getCommandRecords()
    expect(recs.every((r) => r.kind === FakeCommandKind.Stop)).toBe(true)
  })

  it('slave pitch PVT uses cached roll target after master position control', async () => {
    manager.fake!.clearCommandRecords()
    await roll.posControl(0.04, 1, 1, 1)
    manager.fake!.clearCommandRecords()
    await pitch.pvtControl(10, 1, -0.03, 0.2, 0.1)
    expect(manager.fake!.getCommandCount()).toBe(2)
    expect(manager.fake!.getCommandRecords().every((r) => r.kind === FakeCommandKind.PVTControl)).toBe(true)
  })

  it.each([
    [PVTControlMode.HostTorque, FakeCommandKind.TorControl],
    [PVTControlMode.HostKpMotorKd, FakeCommandKind.PVTControl],
  ])('forwards torqueMode %s to command kind %s', async (mode, kind) => {
    const adapterId = `${ADAPTER_NAME}_torque_mode`
    const isolatedManager = await createManager(runtime, adapterId)
    try {
      const isolatedRoll = await isolatedManager.addJoint(
        dcJointDef('torque_mode_roll', 'roll', { adapterId }),
      )
      const isolatedPitch = await isolatedManager.addJoint(
        dcJointDef('torque_mode_pitch', 'pitch', { adapterId }),
      )
      expect(await isolatedRoll.setZero()).toBe(true)
      isolatedManager.fake!.injectFeedback(BUS_ID, 1, {
        position: 0,
        speed: 0,
        current: 0,
        motorTemperature: 25,
        mosTemperature: 25,
      })
      isolatedManager.fake!.injectFeedback(BUS_ID, 2, {
        position: 0,
        speed: 0,
        current: 0,
        motorTemperature: 25,
        mosTemperature: 25,
      })

      isolatedManager.fake!.clearCommandRecords()

      await isolatedRoll.pvtControl(10, 0.5, 0.05, 0.2, 0.3, mode)
      await isolatedPitch.pvtControl(20, 0.75, -0.04, -0.1, -0.2, mode)

      expect(isolatedManager.fake!.getCommandCount()).toBe(2)
      expect(isolatedManager.fake!.getCommandRecords().map((record) => record.kind)).toEqual([
        kind,
        kind,
      ])
    } finally {
      isolatedManager.dispose()
    }
  })
})

describe('DoubleCoupleJoint limits and scaling', () => {
  let runtime: JointRuntime

  beforeAll(async () => {
    runtime = await createJointRuntime()
  })

  afterAll(() => {
    runtime.dispose()
  })

  it('brakes when protocol position range cannot contain two-degree hold slack', async () => {
    const adapterId = `${ADAPTER_NAME}_limit_pvt`
    const manager = await createManager(runtime, adapterId)
    const initParam = {
      ranges: {
        kp: { min: 0, max: 500 },
        kd: { min: 0, max: 5 },
        position: { min: -0.001, max: 0.001 },
        speed: { min: -18, max: 18 },
        torque: { min: -30, max: 30 },
        current: { min: -30, max: 30 },
        kt: 1,
      },
    }
    const roll = await manager.addJoint(
      dcJointDef('limit_pvt_roll', 'roll', { initParam, adapterId }),
    )
    const pitch = await manager.addJoint(
      dcJointDef('limit_pvt_pitch', 'pitch', { initParam, adapterId }),
    )
    expect(await roll.setZero()).toBe(true)
    manager.fake!.injectFeedback(BUS_ID, 1, { position: 0, speed: 0, current: 0, motorTemperature: 25, mosTemperature: 25 })
    manager.fake!.injectFeedback(BUS_ID, 2, { position: 0, speed: 0, current: 0, motorTemperature: 25, mosTemperature: 25 })

    manager.fake!.clearCommandRecords()
    await roll.pvtControl(10, 1, 0.05, 0.4, 0.5)
    expect(manager.fake!.getCommandCount()).toBe(2)
    expect(manager.fake!.getCommandRecords().every((r) => r.kind === FakeCommandKind.Stop)).toBe(true)
    manager.dispose()
  })

  it('uniformly limits PVT speed and torque to motor ranges', async () => {
    const adapterId = `${ADAPTER_NAME}_limit_pvt_st`
    const manager = await createManager(runtime, adapterId)
    const initParam = {
      ranges: {
        kp: { min: 0, max: 500 },
        kd: { min: 0, max: 5 },
        position: { min: -12.5, max: 12.5 },
        speed: { min: -0.2, max: 0.2 },
        torque: { min: -0.2, max: 0.2 },
        current: { min: -30, max: 30 },
        kt: 1,
      },
    }
    const roll = await manager.addJoint(
      dcJointDef('limit_pvt_st_roll', 'roll', { initParam, adapterId }),
    )
    await manager.addJoint(
      dcJointDef('limit_pvt_st_pitch', 'pitch', { initParam, adapterId }),
    )
    expect(await roll.setZero()).toBe(true)

    manager.fake!.clearCommandRecords()
    await roll.pvtControl(10, 1, 0, 2, 3)
    await roll.pvtControl(10, 1, 0, 2, 3)
    expect(manager.fake!.getCommandCount()).toBe(2)
    const recs = manager.fake!.getCommandRecords()
    expect(recs.every((r) => r.kind === FakeCommandKind.PVTControl)).toBe(true)
    for (const rec of recs) {
      const idx = recs.indexOf(rec)
      expect(Math.abs(manager.fake!.getPVTCommand(idx).speed)).toBeLessThanOrEqual(0.2 + 1e-3)
      expect(Math.abs(manager.fake!.getPVTCommand(idx).torque)).toBeLessThanOrEqual(0.2 + 1e-3)
    }
    manager.dispose()
  })

  it('limits speed control to motor speed range', async () => {
    const adapterId = `${ADAPTER_NAME}_limit_spd`
    const manager = await createManager(runtime, adapterId)
    const initParam = {
      ranges: {
        kp: { min: 0, max: 500 },
        kd: { min: 0, max: 5 },
        position: { min: -12.5, max: 12.5 },
        speed: { min: -0.2, max: 0.2 },
        torque: { min: -30, max: 30 },
        current: { min: -30, max: 30 },
        kt: 1,
      },
    }
    const roll = await manager.addJoint(
      dcJointDef('limit_spd_roll', 'roll', { initParam, adapterId }),
    )
    const pitch = await manager.addJoint(
      dcJointDef('limit_spd_pitch', 'pitch', { initParam, adapterId }),
    )
    expect(await roll.setZero()).toBe(true)

    manager.fake!.clearCommandRecords()
    await roll.spdControl(2.0, 0.5, 1)
    await pitch.spdControl(0, 0.5, 1)
    const recs = manager.fake!.getCommandRecords()
    expect(recs).toHaveLength(2)
    expect(recs.every((r) => r.kind === FakeCommandKind.SpdControl)).toBe(true)
    for (const rec of recs) {
      const idx = recs.indexOf(rec)
      expect(Math.abs(manager.fake!.getSpdCommand(idx).speed)).toBeLessThanOrEqual(0.2 + 1e-3)
    }
    manager.dispose()
  })

  it('limits torque control to motor torque range', async () => {
    const adapterId = `${ADAPTER_NAME}_limit_tor`
    const manager = await createManager(runtime, adapterId)
    const initParam = {
      ranges: {
        kp: { min: 0, max: 500 },
        kd: { min: 0, max: 5 },
        position: { min: -12.5, max: 12.5 },
        speed: { min: -18, max: 18 },
        torque: { min: -0.2, max: 0.2 },
        current: { min: -30, max: 30 },
        kt: 1,
      },
    }
    const roll = await manager.addJoint(
      dcJointDef('limit_tor_roll', 'roll', { initParam, adapterId }),
    )
    const pitch = await manager.addJoint(
      dcJointDef('limit_tor_pitch', 'pitch', { initParam, adapterId }),
    )
    expect(await roll.setZero()).toBe(true)

    manager.fake!.clearCommandRecords()
    await roll.torControl(3.0, 1)
    await pitch.torControl(0, 1)
    const recs = manager.fake!.getCommandRecords()
    expect(recs).toHaveLength(2)
    expect(recs.every((r) => r.kind === FakeCommandKind.TorControl)).toBe(true)
    for (const rec of recs) {
      const idx = recs.indexOf(rec)
      expect(Math.abs(manager.fake!.getTorCommand(idx).torque)).toBeLessThanOrEqual(0.2 + 1e-3)
    }
    manager.dispose()
  })

  it('brakes when motorLimit1 cannot contain two-degree hold slack', async () => {
    const adapterId = `${ADAPTER_NAME}_motorlim1`
    const manager = await createManager(runtime, adapterId)
    const roll = await manager.addJoint(
      dcJointDef('motorlim1_roll', 'roll', {
        motorLimit1: { min: -0.001, max: 0.001 },
        adapterId,
      }),
    )
    await manager.addJoint(dcJointDef('motorlim1_pitch', 'pitch', { adapterId }))
    expect(await roll.setZero()).toBe(true)
    manager.fake!.injectFeedback(BUS_ID, 1, { position: 0, speed: 0, current: 0, motorTemperature: 25, mosTemperature: 25 })
    manager.fake!.injectFeedback(BUS_ID, 2, { position: 0, speed: 0, current: 0, motorTemperature: 25, mosTemperature: 25 })

    manager.fake!.clearCommandRecords()
    await roll.posControl(0.05, 1, 1, 1)
    expect(manager.fake!.getCommandCount()).toBe(2)
    expect(manager.fake!.getCommandRecords().every((r) => r.kind === FakeCommandKind.Stop)).toBe(true)
    manager.dispose()
  })

  it('stops dynamic controls at the joint limit', async () => {
    const adapterId = `${ADAPTER_NAME}_dynlim`
    const manager = await createManager(runtime, adapterId)
    const roll = await manager.addJoint(
      dcJointDef('dynlim_roll', 'roll', { limit: { min: -0.1, max: 0.1 }, adapterId }),
    )
    const pitch = await manager.addJoint(dcJointDef('dynlim_pitch', 'pitch', { adapterId }))
    expect(await roll.setZero()).toBe(true)

    manager.fake!.clearCommandRecords()
    await roll.posControl(0.2, 1, 1, 1)
    await roll.posControl(0.2, 1, 1, 1)
    const recs = manager.fake!.getCommandRecords()
    expect(recs).toHaveLength(2)
    const motor1Pos = manager.fake!.getPosCommand(recs.findIndex((r) => r.motorIndex === 1)).position
    const motor2Pos = manager.fake!.getPosCommand(recs.findIndex((r) => r.motorIndex === 2)).position

    manager.fake!.injectFeedback(BUS_ID, 1, { position: motor1Pos, speed: 0, current: 0, motorTemperature: 25, mosTemperature: 25 })
    manager.fake!.injectFeedback(BUS_ID, 2, { position: motor2Pos, speed: 0, current: 0, motorTemperature: 25, mosTemperature: 25 })

    manager.fake!.clearCommandRecords()
    await roll.spdControl(1.0, 0.5, 1)
    await pitch.spdControl(0, 0.5, 1)
    const spdRecs = manager.fake!.getCommandRecords()
    expect(spdRecs).toHaveLength(2)
    expect(spdRecs.every((r) => r.kind === FakeCommandKind.SpdControl)).toBe(true)
    for (let idx = 0; idx < spdRecs.length; idx++) {
      expect(Math.abs(manager.fake!.getSpdCommand(idx).speed)).toBeLessThanOrEqual(1e-4)
    }

    manager.fake!.clearCommandRecords()
    await roll.torControl(1.0, 1)
    await pitch.torControl(0, 1)
    const torRecs = manager.fake!.getCommandRecords()
    expect(torRecs).toHaveLength(2)
    for (let idx = 0; idx < torRecs.length; idx++) {
      expect(Math.abs(manager.fake!.getTorCommand(idx).torque)).toBeLessThanOrEqual(1e-4)
    }

    manager.fake!.clearCommandRecords()
    await roll.curControl(1.0, 1)
    await pitch.curControl(0, 1)
    const curRecs = manager.fake!.getCommandRecords()
    expect(curRecs).toHaveLength(2)
    for (let idx = 0; idx < curRecs.length; idx++) {
      expect(Math.abs(manager.fake!.getCurCommand(idx).current)).toBeLessThanOrEqual(1e-4)
    }
    manager.dispose()
  })
})

describe('DoubleCoupleJoint H130 ankle guard', () => {
  let runtime: JointRuntime

  beforeAll(async () => {
    runtime = await createJointRuntime()
  })

  afterAll(() => {
    runtime.dispose()
  })

  it('holds default zero motor positions when pitch target enters near singular zone', async () => {
    const manager = await createManager(runtime)
    const pitch = await manager.addJoint(h130AnkleDcJointDef('left_ankle_pitch_joint', 'pitch'))
    await manager.addJoint(h130AnkleDcJointDef('left_ankle_roll_joint', 'roll'))

    manager.fake!.clearCommandRecords()
    await pitch.posControl((-31.8 * Math.PI) / 180, 1, 0.5, 1)

    const recs = manager.fake!.getCommandRecords()
    expect(recs).toHaveLength(2)
    expect(recs.every((r) => r.kind === FakeCommandKind.PosControl)).toBe(true)
    const motor5Index = recs.findIndex((r) => r.motorIndex === 5)
    const motor6Index = recs.findIndex((r) => r.motorIndex === 6)
    expect(motor5Index).toBeGreaterThanOrEqual(0)
    expect(motor6Index).toBeGreaterThanOrEqual(0)
    expect(manager.fake!.getPosCommand(motor5Index).position).toBeCloseTo(0, 6)
    expect(manager.fake!.getPosCommand(motor6Index).position).toBeCloseTo(0, 6)
    manager.dispose()
  })

  it('keeps fake motor snapshots at zero when repeatedly commanding roll and near-singular pitch', async () => {
    const manager = await createManager(runtime)
    const pitch = await manager.addJoint(h130AnkleDcJointDef('left_ankle_pitch_joint', 'pitch'))
    const roll = await manager.addJoint(h130AnkleDcJointDef('left_ankle_roll_joint', 'roll'))

    for (let i = 0; i < 30; i++) {
      manager.fake!.clearCommandRecords()
      await pitch.posControl((-31.8 * Math.PI) / 180, 1, 0.5, 1)
      await roll.posControl(0, 1, 0.5, 1)

      const motor5 = manager.fake!.getMotorSnapshot(BUS_ID, 5)
      const motor6 = manager.fake!.getMotorSnapshot(BUS_ID, 6)
      expect(motor5.position).toBeCloseTo(0, 6)
      expect(motor6.position).toBeCloseTo(0, 6)
    }

    manager.dispose()
  })
})

describe('DoubleCoupleJoint reverse slave', () => {
  let runtime: JointRuntime
  let manager: JointManagerWrapper
  let roll: JointWrapper
  let pitch: JointWrapper

  beforeAll(async () => {
    runtime = await createJointRuntime()
    manager = await createManager(runtime)
    roll = await manager.addJoint(dcJointDef('rev_roll', 'roll'))
    pitch = await manager.addJoint(dcJointDef('rev_pitch', 'pitch', { reverse: true, limit: { min: -0.1, max: 0.1 } }))
    expect(await roll.setZero()).toBe(true)
  })

  afterAll(() => {
    manager.dispose()
    runtime.dispose()
  })

  it('reports inverted status for the reversed slave', async () => {
    manager.fake!.clearCommandRecords()
    await pitch.posControl(-0.05, 1, 1, 1)
    await pitch.posControl(-0.05, 1, 1, 1)
    const recs = manager.fake!.getCommandRecords()
    const motor1Pos = manager.fake!.getPosCommand(recs.findIndex((r) => r.motorIndex === 1)).position
    const motor2Pos = manager.fake!.getPosCommand(recs.findIndex((r) => r.motorIndex === 2)).position

    manager.fake!.injectFeedback(BUS_ID, 1, { position: motor1Pos, speed: 0, current: 0, motorTemperature: 25, mosTemperature: 25 })
    manager.fake!.injectFeedback(BUS_ID, 2, { position: motor2Pos, speed: 0, current: 0, motorTemperature: 25, mosTemperature: 25 })

    const status = pitch.getStatus({ autoSend: false, lifeCycleDeduction: 1 })
    expect(status!.pos).toBeCloseTo(0.05, 0.02)
  })

  it('stops dynamic controls at the reversed slave joint limit', async () => {
    manager.fake!.clearCommandRecords()
    await pitch.posControl(0.2, 1, 1, 1)
    await pitch.posControl(0.2, 1, 1, 1)
    const recs = manager.fake!.getCommandRecords()
    const motor1Pos = manager.fake!.getPosCommand(recs.findIndex((r) => r.motorIndex === 1)).position
    const motor2Pos = manager.fake!.getPosCommand(recs.findIndex((r) => r.motorIndex === 2)).position

    manager.fake!.injectFeedback(BUS_ID, 1, { position: motor1Pos, speed: 0, current: 0, motorTemperature: 25, mosTemperature: 25 })
    manager.fake!.injectFeedback(BUS_ID, 2, { position: motor2Pos, speed: 0, current: 0, motorTemperature: 25, mosTemperature: 25 })

    manager.fake!.clearCommandRecords()
    await pitch.spdControl(1.0, 0.5, 1)
    await roll.spdControl(0, 0.5, 1)
    const revSpdRecs = manager.fake!.getCommandRecords()
    expect(revSpdRecs).toHaveLength(2)
    for (let idx = 0; idx < revSpdRecs.length; idx++) {
      expect(Math.abs(manager.fake!.getSpdCommand(idx).speed)).toBeLessThanOrEqual(1e-4)
    }
  })
})

describe('Fake tools and records', () => {
  let runtime: JointRuntime
  let manager: JointManagerWrapper
  let joint: JointWrapper

  beforeAll(async () => {
    runtime = await createJointRuntime()
    manager = await createManager(runtime)
    joint = await manager.addJoint(rJointDef('fake_tools'))
  })

  afterAll(() => {
    manager.dispose()
    runtime.dispose()
  })

  it('returns no commands when records are empty', async () => {
    manager.fake!.clearCommandRecords()
    expect(manager.fake!.getCommandCount()).toBe(0)
    expect(manager.fake!.getLastCommand()).toBeNull()
  })

  it('keeps command records across multiple controls', async () => {
    manager.fake!.clearCommandRecords()
    await joint.posControl(0.1, 1, 1, 1)
    await joint.spdControl(0.2, 1, 1)
    expect(manager.fake!.getCommandCount()).toBe(2)
  })

  it('provides a motor snapshot for an auto-created motor', async () => {
    const snapshot = manager.fake!.getMotorSnapshot(BUS_ID, 1)
    expect(snapshot.model).toBe(MotorModel.EC_A2806_P2)
    expect(snapshot.brakeEnabled).toBe(false)
    expect(Number.isFinite(snapshot.position)).toBe(true)
  })
})

describe('Adapter creation from JSON and per-motor motorParams', () => {
  let runtime: JointRuntime
  let manager: JointManagerWrapper

  beforeAll(async () => {
    runtime = await createJointRuntime()
    manager = runtime.createJointManager({})
  })

  afterAll(() => {
    manager.dispose()
    runtime.dispose()
  })

  it('creates an adapter from an object definition', async () => {
    const name = await manager.createAdapter({ type: 'Fake', name: `${ADAPTER_NAME}_obj` })
    expect(name).toBe(`${ADAPTER_NAME}_obj`)
  })

  it('creates an adapter from a JSON string definition', async () => {
    const name = await manager.createAdapter(
      JSON.stringify({ type: 'Fake', name: `${ADAPTER_NAME}_str` }),
    )
    expect(name).toBe(`${ADAPTER_NAME}_str`)
  })

  it('rejects an adapter definition without string type and name', async () => {
    await expect(manager.createAdapter({ type: 'Fake' } as never)).rejects.toThrow(
      /string type and name/u,
    )
  })

  it('dispatches adapterId lookup by JSON type', async () => {
    const positional = runtime.createJointManager({})
    await positional.createAdapter({ type: 'Fake', name: `${ADAPTER_NAME}_pos0` })
    await positional.createAdapter({ type: 'Fake', name: `${ADAPTER_NAME}_pos1` })

    const jointDefinition = (adapterId: string | number, name: string) => ({
      type: 'RJoint',
      name,
      config: {
        motorParams: {
          adapterId,
          busId: 1,
          motorId: 1,
          initParam: { model: 'EC_A2806_P2' },
        },
        limit: { min: -1, max: 1 },
        maxSpd: 2,
        maxTor: 3,
        reverse: false,
      },
    })

    // 整数引用 → 建立顺序索引
    expect((await positional.addJoint(jointDefinition(1, 'by_index'))).handle).toBeGreaterThan(0)
    // 字符串引用 → 适配器名称
    expect(
      (await positional.addJoint(jointDefinition(`${ADAPTER_NAME}_pos1`, 'by_name'))).handle,
    ).toBeGreaterThan(0)
    // 字符串 "1" 不是任何适配器名称，也不回退到索引
    await expect(positional.addJoint(jointDefinition('1', 'string_one'))).rejects.toThrow(
      /adapter/u,
    )
    // 越界索引同样报找不到适配器
    await expect(positional.addJoint(jointDefinition(9, 'index_nine'))).rejects.toThrow(
      /adapter/u,
    )

    positional.dispose()
  })

  it('exposes slaveId and canfd as optional motorParams schema fields', () => {
    const schema = manager.getJointSchema() as Record<string, Record<string, unknown>>
    const definitions = schema.RJoint!.definitions as Record<string, Record<string, unknown>>
    const motorParams = definitions.MotorParams!

    expect(motorParams.properties).toEqual(
      expect.objectContaining({
        adapterId: expect.anything(),
        slaveId: expect.anything(),
        busId: expect.anything(),
        motorId: expect.anything(),
        canfd: expect.anything(),
        initParam: expect.anything(),
      }),
    )
    expect(motorParams.required as string[]).not.toContain('slaveId')
    expect(motorParams.required as string[]).not.toContain('canfd')
    // adapterId 同时接受名称字符串与建立顺序索引
    const properties = motorParams.properties as Record<string, { type?: unknown }>
    expect(properties.adapterId!.type).toEqual(['string', 'integer'])
  })

  it('applies slaveId and canfd from motorParams to the created motor', async () => {
    const joint = await manager.addJoint({
      type: 'RJoint',
      name: 'slave_id_joint',
      config: {
        motorParams: {
          adapterId: `${ADAPTER_NAME}_obj`,
          slaveId: 3,
          busId: 1,
          motorId: 1,
          canfd: true,
          initParam: { model: 'EC_A2806_P2' },
        },
        limit: { min: -1, max: 1 },
        maxSpd: 2,
        maxTor: 3,
        reverse: false,
      },
    })

    manager.fake!.clearCommandRecords()
    await joint.posControl(0.1, 1, 1, 1)
    const command = manager.fake!.getLastCommand()!
    // 合成总线索引 slaveId << 16 | busId
    expect(command.busIndex).toBe((3 << 16) | 1)
    // CAN FD 双标志位（0x02 | 0x04）
    expect(command.rawFrameFlags & 0x06).toBe(0x06)
  })

  it('ignores canfd in motorParams when the global switch was explicitly disabled', async () => {
    const joint = await manager.addJoint({
      type: 'RJoint',
      name: 'global_disabled_joint',
      config: {
        motorParams: {
          adapterId: `${ADAPTER_NAME}_str`,
          busId: 1,
          motorId: 2,
          canfd: true,
          initParam: { model: 'EC_A2806_P2' },
        },
        limit: { min: -1, max: 1 },
        maxSpd: 2,
        maxTor: 3,
        reverse: false,
      },
    })

    manager.disableCanFd()
    manager.fake!.clearCommandRecords()
    await joint.posControl(0.1, 1, 1, 1)
    expect(manager.fake!.getLastCommand()!.rawFrameFlags & 0x06).toBe(0)
  })
})
