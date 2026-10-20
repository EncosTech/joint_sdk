// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

import { describe, expect, it, vi } from 'vitest'
import { JointRuntimeImpl } from '../../npm/src/runtime.js'

function createRuntime(nativeOverrides: Record<string, unknown> = {}) {
  const native = {
    getVersion: () => '1.0.0',
    createEmptyManager: vi.fn(() => 1),
    disposeManager: vi.fn(),
    managerCreateAdapter: vi.fn(),
    ...nativeOverrides,
  } as any
  const host = {
    resolveUrl: (value: string) => new URL(value, 'https://relay/'),
    fetch: vi.fn(async () => ({ ok: true, status: 200, statusText: 'OK', json: async () => ({ session: 's1' }) })),
  } as any
  return { runtime: new JointRuntimeImpl({} as any, native, host), native, host }
}

describe('RelayWs runtime lifecycle', () => {
  it('keeps a shared session alive until the last manager is disposed', async () => {
    const { runtime, host } = createRuntime()
    const first = runtime.createJointManager({})
    const second = runtime.createJointManager({})
    const names = await Promise.all([
      first.createAdapter('RelayWs', 'https://relay/start'),
      second.createAdapter('RelayWs', 'https://relay/start'),
    ])
    expect(names[0]).toBe(names[1])
    expect(host.fetch).toHaveBeenCalledTimes(1)
    first.dispose()
    await Promise.resolve()
    expect(host.fetch).toHaveBeenCalledTimes(1)
    second.dispose()
    await Promise.resolve()
    expect(host.fetch).toHaveBeenCalledTimes(2)
    expect(host.fetch).toHaveBeenLastCalledWith(expect.stringContaining('/release?session=s1'))
  })

  it('rejects an HTTP /start URL in the synchronous manager shortcut', () => {
    const { runtime, native } = createRuntime()
    expect(() => runtime.createJointManager({ adapterType: 'RelayWs', adapterName: 'https://relay/start' }))
      .toThrow(/createAdapter/u)
    expect(native.createEmptyManager).not.toHaveBeenCalled()
  })

  it('does not release another manager session when adapter creation fails', async () => {
    const { runtime, native, host } = createRuntime()
    const first = runtime.createJointManager({})
    await first.createAdapter('RelayWs', 'https://relay/start')
    native.managerCreateAdapter.mockImplementationOnce(() => { throw new Error('native failure') })
    const second = runtime.createJointManager({})
    await expect(second.createAdapter('RelayWs', 'https://relay/start')).rejects.toThrow('native failure')
    await Promise.resolve()
    expect(host.fetch).toHaveBeenCalledTimes(1)
    runtime.dispose()
    await Promise.resolve()
    expect(host.fetch).toHaveBeenCalledTimes(2)
  })

  it('releases a session acquired after its manager was disposed', async () => {
    const { runtime, native, host } = createRuntime()
    const manager = runtime.createJointManager({})
    const creating = manager.createAdapter('RelayWs', 'https://relay/start')
    manager.dispose()
    await expect(creating).rejects.toThrow('disposed')
    await Promise.resolve()
    expect(native.managerCreateAdapter).not.toHaveBeenCalled()
    expect(host.fetch).toHaveBeenLastCalledWith(expect.stringContaining('/release?session=s1'))
  })

  it('balances repeated acquisitions by the same manager', async () => {
    const { runtime, host } = createRuntime()
    const manager = runtime.createJointManager({})
    await manager.createAdapter('RelayWs', 'https://relay/start')
    await manager.createAdapter('RelayWs', 'https://relay/start')
    manager.dispose()
    manager.dispose()
    await Promise.resolve()
    expect(host.fetch).toHaveBeenCalledTimes(2)
  })

  it.each(['https://relay/start', '/start'])('releases %s when native adapter creation fails', async (startUrl) => {
    const { runtime, native, host } = createRuntime({
      managerCreateAdapter: vi.fn(() => { throw new Error('native failure') }),
    })
    const manager = runtime.createJointManager({})
    await expect(manager.createAdapter('RelayWs', startUrl)).rejects.toThrow('native failure')
    await Promise.resolve()
    expect(host.fetch).toHaveBeenCalledWith(expect.stringContaining('/release?session=s1'))
  })
})
