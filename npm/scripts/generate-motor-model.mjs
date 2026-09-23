#!/usr/bin/env node
// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

import { createReadStream, existsSync } from 'node:fs'
import { readFile, writeFile } from 'node:fs/promises'
import { dirname, resolve } from 'node:path'
import { fileURLToPath } from 'node:url'
import { createInterface } from 'node:readline'

/**
 * @file generate-motor-model.mjs
 * @brief 从 motor_driver 的 motor_models.csv 自动生成 TypeScript MotorModel 枚举。
 *
 * C++ 侧的电机型号枚举由 motor_models.csv 经 generate_motor_params.py 生成。
 * 本脚本读取同一份 CSV，按相同顺序生成 TS enum，保证 TS 与 C++ 枚举值一致。
 */

const here = dirname(fileURLToPath(import.meta.url))
const outPath = resolve(here, '../src/motor-model.generated.ts')

function candidateCsvPaths() {
  const candidates = [
    // WASM FetchContent 构建产物目录
    resolve(here, '../../build-wasm/_deps/encos_driver-src/src/motor/motor_models.csv'),
    resolve(here, '../../build-wasm/_deps/motor_driver-src/src/motor/motor_models.csv'),
    // 与 joint_sdk 同级的 motor_driver 仓库
    resolve(here, '../../motor_driver/src/motor/motor_models.csv'),
    // 父目录下的 motor_driver 仓库
    resolve(here, '../motor_driver/src/motor/motor_models.csv'),
  ]
  if (process.env.MOTOR_MODELS_CSV) {
    candidates.unshift(resolve(process.env.MOTOR_MODELS_CSV))
  }
  return candidates
}

function findCsvPath() {
  for (const path of candidateCsvPaths()) {
    if (existsSync(path)) {
      return path
    }
  }
  return undefined
}

async function parseCsv(path) {
  const stream = createReadStream(path, { encoding: 'utf-8' })
  const lines = []
  for await (const line of createInterface(stream)) {
    const trimmed = line.trim()
    if (!trimmed || trimmed.startsWith('model,')) {
      continue
    }
    lines.push(trimmed)
  }

  const models = []
  for (const line of lines) {
    const model = line.split(',')[0]?.trim()
    if (model) {
      models.push(model)
    }
  }
  return models
}

function generateEnum(models) {
  const entries = models.map((model, index) => `  ${model} = ${index},`).join('\n')
  return `// Auto-generated from motor_models.csv. Do not edit manually.
// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

// C++ MotorModel enum is generated from the same CSV by motor_driver/scripts/generate_motor_params.py.

export enum MotorModel {
${entries}
}
`
}

async function main() {
  const csvPath = findCsvPath()
  if (!csvPath) {
    if (existsSync(outPath)) {
      console.warn(
        `Warning: motor_models.csv not found in any candidate location, keeping existing ${outPath}`,
      )
      return
    }
    throw new Error(
      `motor_models.csv not found. Searched:\n${candidateCsvPaths().join('\n')}\n` +
        'Set MOTOR_MODELS_CSV to the CSV path, or build WASM first to fetch motor_driver.',
    )
  }

  const models = await parseCsv(csvPath)
  if (models.length === 0) {
    throw new Error(`No motor models found in ${csvPath}`)
  }

  const content = generateEnum(models)
  const existing = await readFile(outPath, 'utf-8').catch(() => '')
  if (existing === content) {
    console.log(`MotorModel enum is up to date (${models.length} models)`)
    return
  }

  await writeFile(outPath, content, 'utf-8')
  console.log(`Generated ${outPath} with ${models.length} models from ${csvPath}`)
}

main().catch((error) => {
  console.error(error.message)
  process.exit(1)
})
