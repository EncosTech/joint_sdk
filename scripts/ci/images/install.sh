#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Encos
# SPDX-License-Identifier: MIT
set -euo pipefail
mkdir -p /opt/jointsdk/provenance
case "$VARIANT" in
    portable-glibc2.28)
        cd /tmp/jointsdk-image
        echo 'c5ca6e3442fb48ae75159ca7568854d9ba737bc351460f27ee91b6f3f9fd1f3d  eigen.tar.gz' | sha256sum -c -
        tar -xzf eigen.tar.gz
        cmake -S eigen-3.3.4 -B eigen-build -DCMAKE_INSTALL_PREFIX=/opt/jointsdk/eigen-3.3.4 \
            -DBUILD_TESTING=ON -DCMAKE_POLICY_VERSION_MINIMUM=3.5
        cmake --install eigen-build
        test -f /opt/jointsdk/eigen-3.3.4/share/eigen3/cmake/Eigen3Config.cmake
        sha256sum eigen.tar.gz > /opt/jointsdk/provenance/eigen.sha256
        ;;
    almalinux-8|almalinux-9)
        repo=powertools
        [[ $VARIANT != almalinux-9 ]] || repo=crb
        dnf --enablerepo="$repo" install -y eigen3-devel
        rpm -q eigen3-devel > /opt/jointsdk/provenance/eigen-package.txt
        ;;
    22.04|24.04)
        # The digest-pinned parent already has the complete native/wasm toolchain.
        dpkg-query -W libeigen3-dev libgtest-dev libgmock-dev libfmt-dev libspdlog-dev libzstd-dev \
            > /opt/jointsdk/provenance/native-packages.txt
        test -f /root/emsdk/emsdk_env.sh
        command -v clang-format clang-tidy
        ;;
    *) echo "Unknown image variant: $VARIANT" >&2; exit 1 ;;
esac
if [[ $WITH_JS == true ]]; then
    mkdir -p /opt/jointsdk-js
    cp /tmp/jointsdk-image/{package.json,pnpm-lock.yaml,pnpm-workspace.yaml} /opt/jointsdk-js/
    cd /opt/jointsdk-js
    pnpm install --frozen-lockfile --store-dir /opt/jointsdk-pnpm-store
    pnpm exec playwright install chromium
    pnpm exec tsc --version
    pnpm exec playwright --version
    sha256sum pnpm-lock.yaml > pnpm-lock.sha256
    node -e 'Promise.all([import("typescript"), import("vite"), import("vitest/node"), import("@playwright/test")]).catch(e => { console.error(e); process.exit(1) })'
fi
cmake --version
c++ --version
python3 --version
command -v git tar sha256sum readelf
printf '%s\n' "$VARIANT/$TARGETARCH" > /opt/jointsdk/provenance/variant
