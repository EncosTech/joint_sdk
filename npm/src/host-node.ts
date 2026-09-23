// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

import { existsSync, readFileSync } from 'node:fs'
import { dirname, resolve } from 'node:path'
import { fileURLToPath, pathToFileURL } from 'node:url'

import type { WasmModule, WasmModuleFactory } from './types.js'
import type { RuntimeHost } from './host.js'

const here = dirname(fileURLToPath(import.meta.url))

function defaultModulePath(): string {
  const bundledPath = resolve(here, './JointSDKWasm.js')
  if (existsSync(bundledPath)) {
    return bundledPath
  }
  return resolve(here, '../../build-wasm/JointSDKWasm.js')
}

function toImportUrl(modulePath: string): string {
  if (/^[a-zA-Z][a-zA-Z\d+.-]*:/u.test(modulePath)) {
    return modulePath
  }
  return pathToFileURL(modulePath).href
}

function toFilePath(modulePath: string): string | null {
  if (modulePath.startsWith('file://')) {
    return fileURLToPath(modulePath)
  }
  if (/^[a-zA-Z][a-zA-Z\d+.-]*:/u.test(modulePath)) {
    return null
  }
  return modulePath
}

function wasmPathFor(modulePath: string): string | null {
  const filePath = toFilePath(modulePath)
  if (!filePath) {
    return null
  }
  return filePath.replace(/\.js$/u, '.wasm')
}

async function importWasmFactory(modulePath: string): Promise<WasmModuleFactory> {
  const loaded = await import(toImportUrl(modulePath))
  const factory = loaded.default ?? loaded
  if (typeof factory !== 'function') {
    throw new Error('WASM module factory is not callable')
  }
  return factory as WasmModuleFactory
}

/** @internal */
export function createNodeHost(): RuntimeHost {
  return {
    async loadWasmModule(modulePath?: string): Promise<WasmModule> {
      const resolvedModulePath = modulePath ?? defaultModulePath()
      const createModule = await importWasmFactory(resolvedModulePath)
      const wasmPath = wasmPathFor(resolvedModulePath)
      const wasmBinary =
        wasmPath && existsSync(wasmPath) ? readFileSync(wasmPath) : undefined
      return await createModule({
        locateFile(path: string): string {
          if (path.endsWith('.wasm') && wasmPath) {
            return pathToFileURL(wasmPath).href
          }
          return path
        },
        wasmBinary,
      })
    },

    resolveUrl(input: string): URL {
      return new URL(input)
    },

    async fetch(input: string): Promise<Response> {
      if (typeof globalThis.fetch !== 'function') {
        throw new Error('fetch is not available in this Node runtime')
      }
      return await globalThis.fetch(input)
    },
  }
}
