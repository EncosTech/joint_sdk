#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Encos
# SPDX-License-Identifier: MIT

set -euo pipefail

cd "$(dirname "$0")/.."

source "$HOME/emsdk/emsdk_env.sh" >/tmp/encos-joint-sdk-build-emsdk-env.log

export EM_CACHE="${EM_CACHE:-/tmp/encos-emscripten-cache}"
mkdir -p "$EM_CACHE"

echo "Initializing submodules..."
git submodule update --init

echo "Configuring WASM build..."
rm -rf build-wasm
emcmake cmake -S . -B build-wasm \
  -DJOINT_SDK_BUILD_WASM_BINDINGS=ON \
  -DJOINT_SDK_BUILD_WASM_TESTS=OFF \
  -DJOINT_SDK_BUILD_TESTS=OFF \
  -DEigen3_DIR=/usr/share/eigen3/cmake

echo "Building WASM..."
cmake --build build-wasm

echo "Building TypeScript package..."
pnpm --dir npm typecheck
pnpm --dir npm build

echo "WASM build complete: build-wasm/JointSDKWasm.js"
