#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Encos
# SPDX-License-Identifier: MIT
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
packages=$(cd "${1:?Usage: test-package.sh CANDIDATE_DIRECTORY deb|rpm|tgz}" && pwd)
format=${2:?Package format required}
(cd "$packages" && sha256sum -c SHA256SUMS)
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
as_root=()
if [[ $EUID -ne 0 ]]; then as_root=(sudo); fi
shopt -s nullglob
suffix=$format
if [[ $format == tgz ]]; then suffix=tar.gz; fi
sdk=("$packages"/jointsdk_*.$suffix)
driver=("$packages"/libencosdriver_*.$suffix)
[[ ${#sdk[@]} == 1 && ${#driver[@]} == 1 ]]
case "$format" in
    deb)
        "${as_root[@]}" dpkg --path-include='/usr/share/doc/*' -i "${driver[0]}" "${sdk[0]}"
        prefix=/usr
        dpkg-query -L jointsdk > "$work/paths" ;;
    rpm)
        "${as_root[@]}" rpm -Uvh "${driver[0]}" "${sdk[0]}"
        prefix=/usr
        rpm -ql jointsdk > "$work/paths" ;;
    tgz)
        mkdir -p "$work/relocated"
        # CPack TGZ includes one top-level directory; driver's staged TGZ does not.
        tar -xzf "${sdk[0]}" --strip-components=1 -C "$work/relocated"
        prefix="$work/relocated/usr"
        find "$prefix" -type f -o -type l > "$work/sdk-paths"
        tar -xzf "${driver[0]}" -C "$work/relocated"
        find "$prefix" -type f -o -type l > "$work/paths" ;;
    *) echo "Unknown package format: $format" >&2; exit 1 ;;
esac
if [[ $format != tgz ]]; then cp "$work/paths" "$work/sdk-paths"; fi
python3 "$repo/scripts/verify-install-headers.py" "$prefix" --manifest "$work/sdk-paths"
while IFS= read -r path; do
    if [[ -f $path || -L $path ]]; then printf '%s\n' "$path"; fi
done < "$work/paths" > "$work/owned"
[[ -s $work/owned && -s $prefix/share/doc/jointsdk/LICENSE ]]
python3 - "$repo/LICENSE" "$prefix/share/doc/jointsdk/LICENSE" <<'PYTHON'
from pathlib import Path
import sys
if Path(sys.argv[1]).read_bytes() != Path(sys.argv[2]).read_bytes():
    raise SystemExit("Packaged LICENSE differs from repository LICENSE")
PYTHON
cmake -S "$repo/tests/installed-consumer" -B "$work/consumer" \
    -DCMAKE_PREFIX_PATH="$prefix" -DCMAKE_BUILD_TYPE=Release
cmake --build "$work/consumer" -j2
LD_LIBRARY_PATH="$prefix/lib" ENCOS_PLUGIN_PATH="$prefix/lib/encosPlugins" \
    "$work/consumer/joint_sdk_installed_consumer"
case "$format" in
    deb) "${as_root[@]}" dpkg --purge jointsdk libencosdriver ;;
    rpm) "${as_root[@]}" rpm -e jointsdk libencosdriver ;;
    tgz) rm -rf "$work/relocated" ;;
esac
while IFS= read -r path; do
    [[ ! -e $path && ! -L $path ]] || { echo "File remains after uninstall: $path" >&2; exit 1; }
done < "$work/owned"
printf 'PASS: %s on %s\n' "$format" "$(uname -m)"
