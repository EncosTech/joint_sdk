#!/usr/bin/env node
// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

import { readFileSync, writeFileSync } from 'node:fs'
import { resolve, dirname } from 'node:path'
import { fileURLToPath } from 'node:url'

const here = dirname(fileURLToPath(import.meta.url))
const packageJsonPath = resolve(here, '../package.json')
const cmakeListsPath = resolve(here, '../../CMakeLists.txt')

const cmakeContent = readFileSync(cmakeListsPath, 'utf8')
const match = cmakeContent.match(/project\s*\(\s*\w+\s+VERSION\s+([\d.]+)/)
if (!match) {
  throw new Error('Cannot extract project version from CMakeLists.txt')
}

const version = match[1]
const pkg = JSON.parse(readFileSync(packageJsonPath, 'utf8'))
pkg.version = version
writeFileSync(packageJsonPath, JSON.stringify(pkg, null, 2) + '\n')
console.log(`Synced package version to ${version}`)
