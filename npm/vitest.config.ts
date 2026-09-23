// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

import { defineConfig } from 'vitest/config'
import { resolve } from 'node:path'

export default defineConfig({
  resolve: {
    alias: {
      '@encos/joint-sdk': resolve(import.meta.dirname, 'dist/index.js'),
    },
  },
  test: {
    include: ['../tests/wasm-node/**/*.test.ts'],
    pool: 'forks',
    testTimeout: 10000,
  },
})
