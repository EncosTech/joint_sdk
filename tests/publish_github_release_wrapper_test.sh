#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Encos
# SPDX-License-Identifier: MIT

set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
wrapper="${repo_root}/scripts/publish-github-release.sh"
test_root=$(mktemp -d)
trap 'rm -rf "${test_root}"' EXIT

valid_env="${test_root}/valid.env"
cat >"${valid_env}" <<'EOF'
GITEA_USERNAME=test-user
GITEA_TOKEN=secret-gitea-token
GH_TOKEN=secret-github-token
SOURCE_REPO=https://gitea.test/acme/source.git
MAIN_BRANCH=main
OPTIONAL_BRANCHES=develop,feature-x
GITHUB_RELEASE_REPOSITORY=acme/mirror
GITHUB_RELEASE_REPOSITORY_URL=https://github.com/acme/mirror.git
GITEA_URL=https://gitea.test
GITEA_PACKAGE_OWNER=packages
DEBIAN_DISTRIBUTIONS=jammy,noble
DEBIAN_ARCHITECTURES=amd64,arm64
EOF

main_sha1=0123456789abcdef0123456789abcdef01234567
dry_run_output=$(bash "${wrapper}" \
  --env-file "${valid_env}" \
  --main-sha1 "${main_sha1}" \
  --dry-run)
[[ "${dry_run_output}" == *"acme/mirror"* ]]
[[ "${dry_run_output}" == *"jammy,noble"* ]]
[[ "${dry_run_output}" == *"main"* ]]
[[ "${dry_run_output}" == *"develop,feature-x"* ]]
[[ "${dry_run_output}" == *"${main_sha1}"* ]]
[[ "${dry_run_output}" != *"secret-gitea-token"* ]]
[[ "${dry_run_output}" != *"secret-github-token"* ]]

invalid_env="${test_root}/invalid.env"
printf 'GITEA_USERNAME=test-user\n' >"${invalid_env}"
if bash "${wrapper}" --env-file "${invalid_env}" --dry-run >/dev/null 2>&1; then
  printf 'wrapper unexpectedly accepted incomplete configuration\n' >&2
  exit 1
fi

if bash "${wrapper}" \
  --env-file "${valid_env}" \
  --main-sha1 invalid \
  --dry-run >/dev/null 2>&1; then
  printf 'wrapper unexpectedly accepted an invalid main SHA1\n' >&2
  exit 1
fi

git -C "${repo_root}" check-ignore -q scripts/.env

cp "${wrapper}" "${test_root}/publish-github-release.sh"
cat >"${test_root}/release-to-github.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
printf '%s\n' "$*" >>"${WRAPPER_CALL_LOG}"
while (($#)); do
  if [[ "$1" == "--version-output-file" ]]; then
    printf '1.5.1\n' >"$2"
    shift 2
  else
    shift
  fi
done
EOF
chmod +x "${test_root}/release-to-github.sh"
printf '\nMAIN_BRANCH=stable\nGITHUB_MAIN_BRANCH=main\n' >>"${valid_env}"
export WRAPPER_CALL_LOG="${test_root}/calls"
bash "${test_root}/publish-github-release.sh" --env-file "${valid_env}" >/dev/null
mapfile -t calls <"${WRAPPER_CALL_LOG}"
[[ "${#calls[@]}" == 3 ]]
[[ "${calls[0]}" == *"--source-ref stable "* ]]
[[ "${calls[0]}" == *"--github-branch main "* ]]
[[ "${calls[1]}" == *"--source-ref develop "* ]]
[[ "${calls[1]}" == *"--github-branch develop "* ]]
[[ "${calls[2]}" == *"--github-branch feature-x "* ]]
printf '\nGITHUB_MAIN_BRANCH=bad..branch\n' >>"${valid_env}"
if bash "${wrapper}" --env-file "${valid_env}" --dry-run >/dev/null 2>&1; then
  printf 'wrapper unexpectedly accepted an invalid GitHub branch\n' >&2
  exit 1
fi

printf 'publish_github_release_wrapper_test: PASS\n'
