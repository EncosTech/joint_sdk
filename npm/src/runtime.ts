// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

import type { NativeApi } from './native.js'
import type { CreateJointManagerOptions, RuntimeHost, WasmModule } from './types.js'
import { LogLevel } from './constants.js'
import { JointManagerWrapper } from './joint-manager.js'

/**
 * @file runtime.ts
 * @brief JointRuntime 接口与实现。
 */

export interface JointRuntime {
  version(): string
  createJointManager(options: CreateJointManagerOptions): JointManagerWrapper
  dispose(): void
}

type RelayWsSession = {
  interfaceName: string
  releaseUrl: string
}

type SharedRelayWsSession = {
  pending: Promise<RelayWsSession>
  users: number
}

/** @internal 具体实现，不对外暴露。 */
export class JointRuntimeImpl implements JointRuntime {
  private readonly managers_ = new Set<JointManagerWrapper>()
  private readonly relayWsSessions_ = new Map<string, SharedRelayWsSession>()

  constructor(
    private readonly module_: WasmModule,
    private readonly native_: NativeApi,
    private readonly host_: RuntimeHost,
  ) {}

  version(): string {
    return this.native_.getVersion()
  }

  createJointManager(options: CreateJointManagerOptions): JointManagerWrapper {
    const hasInitialAdapter = options.adapterType !== undefined || options.adapterName !== undefined
    if (hasInitialAdapter && (options.adapterType === undefined || options.adapterName === undefined)) {
      throw new Error('adapterType and adapterName must be provided together')
    }
    if (hasInitialAdapter && options.adapterType === 'RelayWs' && /^https?:\/\//iu.test(options.adapterName!)) {
      throw new Error('RelayWs HTTP /start URLs require createAdapter() on an empty manager')
    }
    const handle = hasInitialAdapter
      ? this.native_.createManager(
          options.adapterType!,
          options.adapterName!,
          options.loggerName ?? options.adapterName!,
          options.logLevel ?? LogLevel.Info,
        )
      : this.native_.createEmptyManager()
    const manager = new JointManagerWrapper(this.native_, handle, this)
    if (hasInitialAdapter) {
      manager._registerAdapterType(options.adapterType!)
    }
    this.managers_.add(manager)
    return manager
  }

  dispose(): void {
    for (const manager of Array.from(this.managers_)) {
      manager.dispose()
    }
    this.managers_.clear()
  }

  /** @internal runtime 与 manager 之间的生命周期协调，不对外公开。 */
  unregisterManager(manager: JointManagerWrapper): void {
    this.managers_.delete(manager)
  }

  async resolveRelayWsInterfaceName(interfaceName: string): Promise<string> {
    if (interfaceName.startsWith('ws://') || interfaceName.startsWith('wss://')) {
      return interfaceName
    }

    const startUrl = this.host_.resolveUrl(interfaceName)
    const cacheKey = startUrl.toString()
    const cached = this.relayWsSessions_.get(cacheKey)
    if (cached) {
      cached.users += 1
      return (await cached.pending).interfaceName
    }

    const pending = this.resolveRelayWsStartUrl(startUrl)
    const entry = { pending, users: 1 }
    this.relayWsSessions_.set(cacheKey, entry)
    try {
      return (await pending).interfaceName
    } catch (error) {
      if (this.relayWsSessions_.get(cacheKey) === entry) {
        this.relayWsSessions_.delete(cacheKey)
      }
      throw error
    }
  }

  releaseRelayWsInterfaceName(interfaceName: string): void {
    if (interfaceName.startsWith('ws://') || interfaceName.startsWith('wss://')) {
      return
    }

    const startUrl = this.host_.resolveUrl(interfaceName)
    const cacheKey = startUrl.toString()
    const entry = this.relayWsSessions_.get(cacheKey)
    if (!entry || --entry.users > 0) {
      return
    }

    this.relayWsSessions_.delete(cacheKey)
    void entry.pending
      .then(({ releaseUrl }) => this.host_.fetch(releaseUrl))
      .catch(() => undefined)
  }

  private async resolveRelayWsStartUrl(startUrl: URL): Promise<RelayWsSession> {
    const response = await this.host_.fetch(startUrl.toString())
    if (!response.ok) {
      throw new Error(`RelayWs /start failed: ${response.status} ${response.statusText}`)
    }

    const body = (await response.json()) as { session?: unknown }
    if (typeof body.session !== 'string' || body.session.length === 0) {
      throw new Error('RelayWs /start response missing session')
    }

    const wsUrl = new URL('/ws', startUrl)
    wsUrl.protocol = startUrl.protocol === 'https:' ? 'wss:' : 'ws:'
    wsUrl.searchParams.set('session', body.session)

    const releaseUrl = new URL('/release', startUrl)
    releaseUrl.searchParams.set('session', body.session)
    for (const [key, value] of startUrl.searchParams) {
      releaseUrl.searchParams.append(key, value)
    }

    return {
      interfaceName: wsUrl.toString(),
      releaseUrl: releaseUrl.toString(),
    }
  }
}
