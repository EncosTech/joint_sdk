// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

import { createNativeApi } from './native.js'
import { JointRuntimeImpl } from './runtime.js'
import { createDefaultHost } from './host.js'
import type { RuntimeOptions, WasmModule } from './types.js'
import type { JointRuntime } from './runtime.js'

/**
 * @file create-runtime.ts
 * @brief 加载 WASM 模块并构造 JointRuntime。
 */

export function createJointRuntime(options?: RuntimeOptions): Promise<JointRuntime>
/** @internal 测试注入用，允许传入预加载的 WasmModule。 */
export function createJointRuntime(options?: RuntimeOptions & { module?: WasmModule }): Promise<JointRuntime>
export async function createJointRuntime(
  options?: RuntimeOptions & { module?: WasmModule },
): Promise<JointRuntime> {
  const host = await createDefaultHost()
  const module = options?.module ?? (await host.loadWasmModule(options?.modulePath))
  const native = createNativeApi(module)
  return new JointRuntimeImpl(module, native, host)
}
