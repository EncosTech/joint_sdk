#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Encos
# SPDX-License-Identifier: MIT
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
variant=${1:?Usage: build.sh VARIANT amd64|arm64 [--push]}
arch=${2:?Architecture required}
base=$(python3 - "$repo/scripts/ci/images/bases.json" "$variant/$arch" <<'PY'
import json, sys
print(json.load(open(sys.argv[1]))[sys.argv[2]])
PY
)
registry=${REGISTRY:-git.in.encos.cn/container/ubuntu}
release=${RELEASE:-20261104-v1}
cache=${JOINT_SDK_IMAGE_CACHE:-${XDG_CACHE_HOME:-$HOME/.cache}/jointsdk-ci}
mkdir -p "$cache"
# Downloads happen only on the image-building host, never in a workflow job.
exec 9>"$cache/download.lock"
flock 9
archive="$cache/eigen-3.3.4.tar.gz"
sha=c5ca6e3442fb48ae75159ca7568854d9ba737bc351460f27ee91b6f3f9fd1f3d
if ! echo "$sha  $archive" | sha256sum -c - >/dev/null 2>&1; then
    curl -fL --retry 3 https://gitlab.com/libeigen/eigen/-/archive/3.3.4/eigen-3.3.4.tar.gz -o "$archive.part"
    echo "$sha  $archive.part" | sha256sum -c -
    mv "$archive.part" "$archive"
fi
flock -u 9
context=$(mktemp -d "$cache/context.XXXXXX")
trap 'rm -rf "$context"' EXIT
cp "$archive" "$context/eigen.tar.gz"
cp "$repo/scripts/ci/images/install.sh" "$context/"
cp "$repo/npm/"{package.json,pnpm-lock.yaml,pnpm-workspace.yaml} "$context/"
dockerfile="$repo/scripts/ci/images/Dockerfile"
extra_args=()
if [[ $variant == 22.04 || $variant == 24.04 ]]; then
    dockerfile="$repo/scripts/ci/images/Dockerfile.ubuntu"
    target=native
    [[ $variant/$arch != 24.04/amd64 ]] || target=wasm
    js_base=$(python3 - "$repo/scripts/ci/images/bases.json" <<'PYTHON'
import json, sys
print(json.load(open(sys.argv[1]))['portable-glibc2.28/amd64'])
PYTHON
)
    extra_args=(--target "$target" --build-arg "JS_BUILD_IMAGE=$js_base")
fi
python3 - "$context/inventory.json" "$base" "$variant" "$arch" "$release" <<'PYTHON'
import json, sys
from pathlib import Path
Path(sys.argv[1]).write_text(json.dumps(dict(zip(('base', 'variant', 'architecture', 'release'), sys.argv[2:])), indent=2) + '\n')
PYTHON
if [[ $variant/$arch == 24.04/amd64 ]]; then
    # Export only the prepared dependency payload, then append it as linked
    # layers. This avoids unpacking/reinstalling the inherited Ubuntu toolchain.
    docker buildx build --builder "${JS_BUILDER:-default}" --platform linux/amd64 \
        --file "$dockerfile" --progress plain --target js-files \
        --build-arg "BASE_IMAGE=$base" --build-arg "JS_BUILD_IMAGE=$js_base" \
        --output "type=local,dest=$context" "$context"
fi
output=--load
if [[ ${3:-} == --push ]]; then output=--push; elif [[ $# != 2 ]]; then exit 2; fi
docker buildx build --builder "${BUILDER:-persistent}" --platform "linux/$arch" \
    --file "$dockerfile" --progress plain "${extra_args[@]}" \
    --build-arg "BASE_IMAGE=$base" --build-arg "VARIANT=$variant" \
    --tag "$registry:jointsdk-ci-$variant-$release-$arch" "$output" "$context"
