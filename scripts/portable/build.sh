#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Encos
# SPDX-License-Identifier: MIT
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
output=${1:?Usage: build.sh OUTPUT DRIVER_SOURCE}
driver=${2:?Usage: build.sh OUTPUT DRIVER_SOURCE}
mkdir -p "$output"
output=$(cd "$output" && pwd)
driver=$(cd "$driver" && pwd)
[[ $(getconf GNU_LIBC_VERSION) == 'glibc 2.28' ]] || {
    echo 'Portable builds require the glibc 2.28 builder image' >&2; exit 1;
}
# Use the Eigen installation provisioned in the portable CI image.
eigen_prefix=${JOINT_SDK_EIGEN_PREFIX:-/opt/jointsdk/eigen-3.3.4}
[[ -f $eigen_prefix/share/eigen3/cmake/Eigen3Config.cmake ]] || {
    echo 'Missing preinstalled Eigen: rebuild the joint_sdk portable CI image' >&2; exit 1;
}
# The dependency remains a separate package; never copy its payload into jointsdk.
bash "$driver/scripts/portable/build.sh" "$output/driver"
bash "$driver/scripts/portable/package.sh" "$output/driver"
prefix="$output/driver/stage/usr"
cmake -S "$repo" -B "$output/build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_INSTALL_LIBDIR=lib \
    -DCMAKE_PREFIX_PATH="$prefix;$eigen_prefix" -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DBUILD_SHARED_LIBS=ON -DJOINT_SDK_PORTABLE_LINUX=ON -DJOINT_SDK_BUILD_TESTS=OFF
cmake --build "$output/build" -j"${JOINT_SDK_BUILD_JOBS:-2}"
# A reused staging tree must not retain headers from an older install layout.
rm -rf -- "$output/stage"
DESTDIR="$output/stage" cmake --install "$output/build"
python3 "$repo/scripts/verify-install-headers.py" "$output/stage/usr"
python3 "$repo/scripts/portable/audit-elf.py" "$output/stage/usr" --audit-only > "$output/elf-audit.json"
cmake -S "$repo/tests/installed-consumer" -B "$output/consumer" \
    -DCMAKE_PREFIX_PATH="$output/stage/usr;$prefix;$eigen_prefix"
cmake --build "$output/consumer" -j2
LD_LIBRARY_PATH="$output/stage/usr/lib:$prefix/lib" ENCOS_PLUGIN_PATH="$prefix/lib/encosPlugins" \
    "$output/consumer/joint_sdk_installed_consumer"
mkdir -p "$output/packages"
(cd "$output/packages" && cpack --config "$output/build/CPackConfig.cmake")
rm -rf "$output/packages/_CPack_Packages"
cp "$output/driver/packages/"libencosdriver_* "$output/packages/"
cp "$output/elf-audit.json" "$output/packages/jointsdk-elf-audit.json"
python3 "$repo/scripts/verify-package-license.py" "$output/packages/"jointsdk_*.deb
(cd "$output/packages" && sha256sum ./*.deb ./*.rpm ./*.tar.gz ./*.json > SHA256SUMS)
