// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

import { test, expect } from '@playwright/test'
import { FakeCommandKind } from '../../npm/dist/index.js'

const rJointDefinition = {
  type: 'RJoint',
  name: 'browser_r_joint',
  config: {
    motorParams: {
      adapterId: 'fake_browser',
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

test('loads WASM module and reports version', async ({ page }) => {
  await page.goto('/tests/wasm-browser/')
  const version = await page.evaluate(async () => {
    const runtime = await window.JointSDK.createJointRuntime()
    return runtime.version()
  })
  expect(version).toMatch(/^\d+\.\d+\.\d+/u)
})

test('loads WASM module when consumed as a Vite package dependency', async ({ page }) => {
  await page.goto('/tests/wasm-browser/package-import.html')
  const version = await page.evaluate(async () => {
    const runtime = await window.JointSDK.createJointRuntime()
    return runtime.version()
  })
  expect(version).toMatch(/^\d+\.\d+\.\d+/u)
})

test('creates Fake manager and controls a joint', async ({ page }) => {
  await page.goto('/tests/wasm-browser/')
  const result = await page.evaluate(async (definition) => {
    const { createJointRuntime, FakeCommandKind, MotorStopMode } = await import('/npm/dist/index.js')
    const runtime = await createJointRuntime()
    const manager = runtime.createJointManager({
      adapterType: 'Fake',
      adapterName: 'fake_browser',
    })
    const joint = await manager.addJoint(definition)

    manager.fake!.injectFeedback(1, 1, {
      position: 0.5,
      speed: 0.1,
      current: 0.2,
      motorTemperature: 30,
      mosTemperature: 40,
    })
    const status = joint.getStatus({ autoSend: false, lifeCycleDeduction: 1 })
    await joint.stop(MotorStopMode.FullBrake, 0, 0)
    const lastCommand = manager.fake!.getLastCommand()
    manager.dispose()
    runtime.dispose()
    return {
      pos: status?.pos ?? null,
      kind: lastCommand?.kind ?? null,
    }
  }, rJointDefinition)
  expect(result.pos).toBeCloseTo(0.5)
  expect(result.kind).toBe(FakeCommandKind.Stop)
})

test('non-brake joint returns false without sending brake commands', async ({ page }) => {
  await page.goto('/tests/wasm-browser/')
  const result = await page.evaluate(async (definition) => {
    const runtime = await window.JointSDK.createJointRuntime()
    const manager = runtime.createJointManager({ adapterType: 'Fake', adapterName: 'fake_browser' })
    try {
      const joint = await manager.addJoint(definition)
      manager.fake!.clearCommandRecords()
      return {
        supported: joint.hasBrake(),
        enabled: await joint.brake(true),
        disabled: await joint.brake(false),
        engaged: await joint.getBrakeStatus(),
        commandCount: manager.fake!.getCommandCount(),
      }
    } finally {
      manager.dispose()
      runtime.dispose()
    }
  }, rJointDefinition)
  expect(result).toEqual({ supported: false, enabled: false, disabled: false, engaged: false, commandCount: 0 })
})
