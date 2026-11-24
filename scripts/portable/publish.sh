#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Encos
# SPDX-License-Identifier: MIT
set -euo pipefail
: "${USER_PASSWORD:?Missing package credentials}"
root=${1:?Usage: publish.sh CANDIDATE_ROOT}
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
exec python3 "${script_dir}/release-packages.py" publish "${root}"
