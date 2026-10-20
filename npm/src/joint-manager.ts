// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

import type { NativeApi } from './native.js'
import type { AdapterDefinition, JointDefinition, JointStatus, StatusOptions } from './types.js'
import type { JointRuntimeImpl } from './runtime.js'
import { JointWrapper } from './joint.js'
import { FakeJointTools } from './fake-tools.js'
import { EncosWasmError } from './native.js'
import { LogLevel } from './constants.js'

/**
 * @file joint-manager.ts
 * @brief 关节管理器的高层 TypeScript 包装（JointManagerWrapper）。
 */

function isRecord(value: unknown): value is Record<string, unknown> {
  return !!value && typeof value === 'object' && !Array.isArray(value)
}

async function rewriteAdapterIdsForWasm(
  value: unknown,
  resolveAdapterId: (adapterId: string) => Promise<string>,
): Promise<unknown> {
  if (Array.isArray(value)) {
    return Promise.all(value.map((item) => rewriteAdapterIdsForWasm(item, resolveAdapterId)))
  }

  if (!isRecord(value)) {
    return value
  }

  const rewritten = Object.fromEntries(
    await Promise.all(
      Object.entries(value).map(async ([key, child]) => [
        key,
        await rewriteAdapterIdsForWasm(child, resolveAdapterId),
      ] as const),
    ),
  )

  if (typeof value.adapterId === 'string') {
    rewritten.adapterId = await resolveAdapterId(value.adapterId)
  }

  return rewritten
}

export class JointManagerWrapper {
  private readonly joints_ = new Map<number, JointWrapper>()
  private readonly adapterTypes_ = new Set<string>()
  private readonly adapterNames_ = new Map<string, string>()
  private readonly wasmAdapterNames_ = new Set<string>()
  private readonly relayWsAdapterNames_ = new Set<string>()
  private disposed_ = false

  /** @internal 由 JointRuntimeImpl 构造，不建议外部直接实例化。 */
  constructor(
    private readonly native: NativeApi,
    public readonly handle: number,
    private readonly runtime?: JointRuntimeImpl,
  ) {}

  dispose(): void {
    if (this.disposed_) {
      return
    }
    this.disposed_ = true
    for (const wrapper of Array.from(this.joints_.values())) {
      wrapper.dispose()
    }
    this.joints_.clear()
    this.native.disposeManager(this.handle)
    this.releaseRelayWsAdapters()
    this.runtime?.unregisterManager(this)
  }

  /** @internal 由 JointWrapper 在创建时注册。 */
  _registerJoint(wrapper: JointWrapper): void {
    this.joints_.set(wrapper.handle, wrapper)
  }

  /** @internal 由 JointWrapper.dispose 注销。 */
  _unregisterJoint(handle: number): void {
    this.joints_.delete(handle)
  }

  /** @internal 由 JointRuntimeImpl 在兼容快捷创建路径注册。 */
  _registerAdapterType(adapterType: string): void {
    this.adapterTypes_.add(adapterType)
  }

  async createAdapter(
    adapterType: string,
    adapterName: string,
    options?: { loggerName?: string; logLevel?: number; extend?: Record<string, unknown> },
  ): Promise<string>
  async createAdapter(definition: AdapterDefinition | string): Promise<string>
  async createAdapter(
    adapterTypeOrDefinition: string | AdapterDefinition,
    adapterName?: string,
    options?: { loggerName?: string; logLevel?: number; extend?: Record<string, unknown> },
  ): Promise<string> {
    // 对象/JSON 形态的适配器配置先归一成 (type, name)，复用下方的创建流程
    if (typeof adapterTypeOrDefinition !== 'string' || adapterName === undefined) {
      const definition =
        typeof adapterTypeOrDefinition === 'string'
          ? (JSON.parse(adapterTypeOrDefinition) as AdapterDefinition)
          : adapterTypeOrDefinition
      if (!definition || typeof definition.type !== 'string' || typeof definition.name !== 'string') {
        throw new Error('Adapter definition must provide string type and name')
      }
      return this.createAdapter(definition.type, definition.name, {
        loggerName: definition.loggerName,
        logLevel: definition.logLevel,
        extend: definition.extend,
      })
    }
    const adapterType = adapterTypeOrDefinition
    if (this.disposed_) throw new Error('Joint manager is disposed')
    const interfaceName = adapterType === 'RelayWs' && this.runtime
      ? await this.runtime.resolveRelayWsInterfaceName(adapterName)
      : adapterName
    try {
      if (this.disposed_) throw new Error('Joint manager is disposed')
      this.native.managerCreateAdapter(
        this.handle,
        adapterType,
        interfaceName,
        options?.loggerName ?? interfaceName,
        options?.logLevel ?? LogLevel.Info,
        options?.extend === undefined ? '' : JSON.stringify(options.extend),
      )
    } catch (error) {
      if (adapterType === 'RelayWs' && this.runtime) {
        this.runtime.releaseRelayWsInterfaceName(adapterName)
      }
      throw error
    }
    this.adapterTypes_.add(adapterType)
    this.adapterNames_.set(adapterName, interfaceName)
    this.adapterNames_.set(interfaceName, interfaceName)
    this.setNormalizedAdapterName(adapterName, interfaceName)
    this.wasmAdapterNames_.add(interfaceName)
    if (adapterType === 'RelayWs') {
      if (this.relayWsAdapterNames_.has(adapterName)) {
        this.runtime?.releaseRelayWsInterfaceName(adapterName)
      }
      this.relayWsAdapterNames_.add(adapterName)
    }
    return interfaceName
  }

  get fake(): FakeJointTools | undefined {
    if (!this.adapterTypes_.has('Fake')) {
      return undefined
    }
    return new FakeJointTools(this.native, this.handle)
  }

  async addJoint(definition: JointDefinition): Promise<JointWrapper> {
    const wasmDefinition = await rewriteAdapterIdsForWasm(
      definition,
      (adapterId) => this.resolveAdapterIdForWasm(adapterId),
    )
    const handle = await this.native.managerAddJointJson(this.handle, JSON.stringify(wasmDefinition))
    const wrapper = new JointWrapper(this.native, handle, definition.name, this)
    this._registerJoint(wrapper)
    return wrapper
  }

  getJoint(name: string): JointWrapper | null {
    try {
      const handle = this.native.managerGetJoint(this.handle, name)
      const existing = this.joints_.get(handle)
      if (existing) {
        return existing
      }
      const wrapper = new JointWrapper(this.native, handle, name, this)
      this._registerJoint(wrapper)
      return wrapper
    } catch (error) {
      if (error instanceof EncosWasmError && error.code === 2) {
        return null
      }
      throw error
    }
  }

  getJointSchema(): Record<string, unknown> {
    const json = this.native.managerGetJointSchemaJson(this.handle)
    return JSON.parse(json) as Record<string, unknown>
  }

  getAvailableJointTypes(): string[] {
    const json = this.native.managerGetAvailableJointTypesJson(this.handle)
    return JSON.parse(json) as string[]
  }

  getStatus(options?: StatusOptions): Record<string, JointStatus> {
    const json = this.native.managerGetStatusJson(
      this.handle,
      options?.autoSend ?? false,
      options?.lifeCycleDeduction ?? 1,
    )
    return JSON.parse(json) as Record<string, JointStatus>
  }

  enableCanFd(): void {
    this.native.managerEnableJointMotorCanfd(this.handle)
  }

  disableCanFd(): void {
    this.native.managerDisableJointMotorCanfd(this.handle)
  }

  isCanFdEnabled(): boolean {
    return this.native.managerIsJointMotorCanfd(this.handle)
  }

  private setNormalizedAdapterName(adapterName: string, interfaceName: string): void {
    try {
      this.adapterNames_.set(new URL(adapterName).toString(), interfaceName)
    } catch {
      // Not a URL; exact-name lookup above is sufficient.
    }
  }

  private async resolveAdapterIdForWasm(adapterId: string): Promise<string> {
    const existing = this.adapterNames_.get(adapterId)
    if (existing) {
      return existing
    }

    if (!this.runtime || !/^https?:\/\//iu.test(adapterId)) {
      return adapterId
    }

    const interfaceName = await this.runtime.resolveRelayWsInterfaceName(adapterId)
    if (this.disposed_) {
      this.runtime.releaseRelayWsInterfaceName(adapterId)
      throw new Error('Joint manager is disposed')
    }
    if (this.relayWsAdapterNames_.has(adapterId)) {
      this.runtime.releaseRelayWsInterfaceName(adapterId)
      return interfaceName
    }
    if (!this.wasmAdapterNames_.has(interfaceName)) {
      try {
        this.native.managerCreateAdapter(
          this.handle,
          'RelayWs',
          interfaceName,
          interfaceName,
          LogLevel.Info,
        )
      } catch (error) {
        this.runtime.releaseRelayWsInterfaceName(adapterId)
        throw error
      }
      this.adapterTypes_.add('RelayWs')
      this.wasmAdapterNames_.add(interfaceName)
    }

    this.adapterNames_.set(adapterId, interfaceName)
    this.adapterNames_.set(interfaceName, interfaceName)
    this.setNormalizedAdapterName(adapterId, interfaceName)
    this.relayWsAdapterNames_.add(adapterId)
    return interfaceName
  }

  private releaseRelayWsAdapters(): void {
    if (!this.runtime) {
      return
    }
    for (const adapterName of this.relayWsAdapterNames_) {
      this.runtime.releaseRelayWsInterfaceName(adapterName)
    }
    this.relayWsAdapterNames_.clear()
  }
}
