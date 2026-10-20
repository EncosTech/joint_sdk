#!/usr/bin/env node
// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

import { copyFileSync, existsSync, mkdirSync } from 'node:fs'
import { resolve, dirname } from 'node:path'
import { fileURLToPath } from 'node:url'

const here = dirname(fileURLToPath(import.meta.url))
const distDir = resolve(here, '../dist')
const buildWasmDir = resolve(here, '../../build-wasm')

const artifacts = ['JointSDKWasm.js', 'JointSDKWasm.wasm']

if (!existsSync(distDir)) {
  mkdirSync(distDir, { recursive: true })
}

for (const name of artifacts) {
  const src = resolve(buildWasmDir, name)
  const dst = resolve(distDir, name)
  if (!existsSync(src)) {
    throw new Error(`WASM artifact not found: ${src}`)
  }
  copyFileSync(src, dst)
  console.log(`Copied ${name} to dist`)
}

// 将根许可证同步到 npm 包，避免打包时使用过期副本。
copyFileSync(resolve(here, "../../LICENSE"), resolve(here, "../LICENSE"))
