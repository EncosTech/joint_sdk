#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Encos
# SPDX-License-Identifier: MIT
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
record=/opt/jointsdk-js/pnpm-lock.sha256
[[ -r $record ]] || { echo 'Rebuild the joint_sdk CI image: JS dependencies are absent' >&2; exit 1; }
read -r expected _ < "$record"
actual=$(sha256sum "$repo/npm/pnpm-lock.yaml")
[[ ${actual%% *} == "$expected" ]] || {
    echo 'Lockfile changed: rebuild and version the joint_sdk CI image before running CI' >&2; exit 1;
}
python3 - "$repo/npm" /opt/jointsdk-js <<'PYTHON'
import json
from pathlib import Path
import sys
checkout, image = map(Path, sys.argv[1:])
current = json.loads((checkout / 'package.json').read_text())
installed = json.loads((image / 'package.json').read_text())
fields = ('dependencies', 'devDependencies', 'optionalDependencies', 'peerDependencies',
          'peerDependenciesMeta', 'pnpm', 'packageManager', 'engines', 'os', 'cpu')
if any(current.get(field) != installed.get(field) for field in fields):
    raise SystemExit('Dependency declarations changed: update the lockfile and rebuild the CI image')
if (checkout / 'pnpm-workspace.yaml').read_bytes() != (image / 'pnpm-workspace.yaml').read_bytes():
    raise SystemExit('Workspace installation settings changed: rebuild the CI image')
PYTHON
[[ ! -e $repo/npm/node_modules && ! -L $repo/npm/node_modules ]] || {
    echo 'Expected a clean checkout without npm/node_modules' >&2; exit 1;
}
ln -s /opt/jointsdk-js/node_modules "$repo/npm/node_modules"
