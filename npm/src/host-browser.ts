// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

import type { WasmModule, WasmModuleFactory } from './types.js'
import type { RuntimeHost } from './host.js'

const DEFAULT_MODULE_PATH = './JointSDKWasm.js'
const VITE_NODE_MODULES_MODULE_PATH = '/node_modules/@encos/joint-sdk/dist/JointSDKWasm.js'

function resolveModuleUrl(modulePath?: string): string {
  const input = modulePath ?? DEFAULT_MODULE_PATH
  if (/^[a-zA-Z][a-zA-Z\d+.-]*:/u.test(input)) {
    return input
  }
  return new URL(input, import.meta.url).href
}

function wasmUrlFor(moduleUrl: string): string {
  return moduleUrl.replace(/\.js$/u, '.wasm')
}

function defaultModuleUrls(modulePath?: string): string[] {
  if (modulePath !== undefined) {
    return [resolveModuleUrl(modulePath)]
  }
  const urls = [resolveModuleUrl()]
  if (typeof window !== 'undefined') {
    urls.push(new URL(VITE_NODE_MODULES_MODULE_PATH, window.location.href).href)
  }
  return urls
}

async function importWasmFactory(moduleUrl: string): Promise<WasmModuleFactory> {
  const factoryModule = await import(/* @vite-ignore */ moduleUrl)
  const createModule: WasmModuleFactory = factoryModule.default ?? factoryModule
  if (typeof createModule !== 'function') {
    throw new Error('WASM module factory is not callable')
  }
  return createModule
}

async function loadFromModuleUrl(moduleUrl: string): Promise<WasmModule> {
  const createModule = await importWasmFactory(moduleUrl)
  const wasmUrl = wasmUrlFor(moduleUrl)
  return await createModule({
    locateFile(path: string): string {
      if (path.endsWith('.wasm')) {
        return wasmUrl
      }
      return path
    },
  })
}

/** @internal */
export function createBrowserHost(): RuntimeHost {
  return {
    async loadWasmModule(modulePath?: string): Promise<WasmModule> {
      let lastError: unknown
      for (const moduleUrl of defaultModuleUrls(modulePath)) {
        try {
          return await loadFromModuleUrl(moduleUrl)
        } catch (error) {
          lastError = error
        }
      }
      throw lastError
    },

    resolveUrl(input: string): URL {
      return new URL(input, typeof window !== 'undefined' ? window.location.href : undefined)
    },

    async fetch(input: string): Promise<Response> {
      return await globalThis.fetch(input)
    },
  }
}
