#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Encos
# SPDX-License-Identifier: MIT

set -euo pipefail

cd "$(dirname "$0")/.."

source "$HOME/emsdk/emsdk_env.sh" >/tmp/encos-joint-sdk-test-emsdk-env.log

export EM_CACHE="${EM_CACHE:-/tmp/encos-emscripten-cache}"
mkdir -p "$EM_CACHE"

echo "Initializing submodules..."
git submodule update --init

echo "Configuring WASM test build..."
rm -rf build-wasm
emcmake cmake -S . -B build-wasm \
  -DJOINT_SDK_BUILD_WASM_BINDINGS=ON \
  -DJOINT_SDK_BUILD_WASM_TESTS=ON \
  -DJOINT_SDK_BUILD_TESTS=OFF \
  -DEigen3_DIR=/usr/share/eigen3/cmake

echo "Building WASM..."
cmake --build build-wasm

echo "TypeScript checks..."
pnpm --dir npm typecheck

echo "Building TypeScript package..."
pnpm --dir npm build

echo "Preparing browser package fixture..."
pnpm --dir npm run setup:browser-fixture

echo "Running Node (Vitest) tests..."
pnpm --dir npm run test:unit

echo "Starting dev server for browser tests..."
(
  cd npm
  pnpm exec vite .. --port 5173 --strictPort
) &
server_pid=$!
trap 'kill $server_pid &>/dev/null || true' EXIT
sleep 2

echo "Running browser (Playwright) tests..."
pnpm --dir npm run test:browser

echo "WASM tests complete."
