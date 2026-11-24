#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Encos
# SPDX-License-Identifier: MIT

set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
script_path="${repo_root}/scripts/release-to-github.sh"
wrapper_path="${repo_root}/scripts/publish-github-release.sh"
test_root=$(mktemp -d)
trap 'rm -rf "${test_root}"' EXIT

export GIT_AUTHOR_NAME="Release Test"
export GIT_AUTHOR_EMAIL="release-test@example.com"
export GIT_COMMITTER_NAME="Release Test"
export GIT_COMMITTER_EMAIL="release-test@example.com"
export GIT_ALLOW_PROTOCOL=file

leaf_repo="${test_root}/leaf"
git init -q -b main "${leaf_repo}"
printf 'recursive payload\n' >"${leaf_repo}/leaf.txt"
git -C "${leaf_repo}" add .
git -C "${leaf_repo}" commit -q -m "initial leaf"

submodule_repo="${test_root}/submodule"
git init -q -b main "${submodule_repo}"
mkdir -p "${submodule_repo}/nested"
printf 'submodule payload\n' >"${submodule_repo}/nested/payload.txt"
git -C "${submodule_repo}" -c protocol.file.allow=always submodule add -q \
  "${leaf_repo}" vendor/leaf
git -C "${submodule_repo}" add .
git -C "${submodule_repo}" commit -q -m "initial submodule"

source_repo="${test_root}/source"
git init -q -b main "${source_repo}"
printf 'cmake_minimum_required(VERSION 3.18)\nproject(joint_sdk VERSION 3.2.0)\n' \
  >"${source_repo}/CMakeLists.txt"
mkdir -p "${source_repo}/include" "${source_repo}/.gitea" "${source_repo}/openspec"
printf 'public header\n' >"${source_repo}/include/public.h"
printf 'private workflow\n' >"${source_repo}/.gitea/private.yml"
printf 'internal spec\n' >"${source_repo}/openspec/change.md"
git -C "${source_repo}" -c protocol.file.allow=always submodule add -q \
  "${submodule_repo}" external/example
git -C "${source_repo}" config -f .gitmodules \
  submodule.external/example.url http://gitea.test/acme/submodule.git
for internal_root in "${source_repo}" "${source_repo}/include"; do
  printf 'agent rules\n' >"${internal_root}/AGENTS.md"
  printf 'claude rules\n' >"${internal_root}/CLAUDE.md"
  for private_dir in .claude .codex .agents .cursor .gemini .opencode; do
    mkdir -p "${internal_root}/${private_dir}"
    printf 'private configuration\n' >"${internal_root}/${private_dir}/config.txt"
  done
done
git -C "${source_repo}" add .
git -C "${source_repo}" commit -q -m "source snapshot"
initial_source_sha1=$(git -C "${source_repo}" rev-parse HEAD)

git_config="${test_root}/gitconfig"
git config -f "${git_config}" url."${submodule_repo}".insteadOf \
  https://gitea.test/acme/submodule.git

target_seed="${test_root}/target-seed"
target_bare="${test_root}/target.git"
git init -q -b main "${target_seed}"
printf 'stale\n' >"${target_seed}/stale.txt"
mkdir -p "${target_seed}/.gitea" "${target_seed}/openspec"
printf 'stale workflow\n' >"${target_seed}/.gitea/stale.yml"
printf 'stale spec\n' >"${target_seed}/openspec/stale.md"
printf 'stale agent rules\n' >"${target_seed}/AGENTS.md"
printf 'stale claude rules\n' >"${target_seed}/CLAUDE.md"
for private_dir in .claude .codex .agents .cursor .gemini .opencode; do
  mkdir -p "${target_seed}/${private_dir}"
  printf 'stale configuration\n' >"${target_seed}/${private_dir}/stale.txt"
done
git -C "${target_seed}" add .
git -C "${target_seed}" commit -q -m "seed"
git clone -q --bare "${target_seed}" "${target_bare}"

fake_bin="${test_root}/bin"
mkdir -p "${fake_bin}"
curl_log="${test_root}/curl.log"
export CURL_LOG="${curl_log}"
export TARGET_BARE="${target_bare}"
export TEST_REGISTRY="${test_root}/registry"
export TEST_RELEASE_ASSETS="${test_root}/release-assets"
mkdir -p "${TEST_REGISTRY}" "${TEST_RELEASE_ASSETS}"
candidate_root="${test_root}/candidate"
for arch in amd64 arm64; do
  candidate="${candidate_root}/${arch}"
  mkdir -p "${candidate}"
  for suffix in deb rpm tar.gz; do
    printf 'original SDK package %s/%s\n' "${arch}" "${suffix}" \
      >"${candidate}/jointsdk_3.2.0_linux-glibc2.28_${arch}.${suffix}"
    printf 'pinned driver package %s/%s\n' "${arch}" "${suffix}" \
      >"${candidate}/libencosdriver_4.1.2_linux-glibc2.28_${arch}.${suffix}"
  done
  printf '{}\n' >"${candidate}/jointsdk-elf-audit.json"
  (cd "${candidate}" && sha256sum ./*.deb ./*.rpm ./*.tar.gz ./*.json) >"${candidate}/SHA256SUMS"
done

python3 -B - "${repo_root}" "${candidate_root}" "${test_root}" <<'PY'
import importlib.util
import io
import json
from pathlib import Path
import sys
from unittest.mock import patch
import zipfile

repo, candidates, work = map(Path, sys.argv[1:])
spec = importlib.util.spec_from_file_location("download_artifact", repo / "scripts/ci/download-artifact.py")
downloader = importlib.util.module_from_spec(spec)
spec.loader.exec_module(downloader)

base = downloader.native_run_api("https://gitea.test", "https://gitea.test/acme/source.git")
run_id = 2446
sha = "a" * 40
run_url = f"{base}/actions/runs/{run_id}"
run = dict(id=run_id, path="build-native.yml@refs/heads/stable", event="push", head_branch="stable",
           head_sha=sha, status="completed", conclusion="success")
jobs = {"jobs": [{"name": "publish-tested-packages", "conclusion": "success"}]}
artifacts = []
archives = {}
for artifact_id, arch in enumerate(("amd64", "arm64"), start=1):
    artifacts.append({"id": artifact_id, "name": f"jointsdk-portable-{arch}-{sha}", "expired": False,
                      "workflow_run": {"id": run_id, "head_sha": sha}})
    archive = io.BytesIO()
    with zipfile.ZipFile(archive, "w") as package:
        for path in (candidates / arch).iterdir():
            package.write(path, arcname=path.name)
    archives[f"{base}/actions/artifacts/{artifact_id}/zip"] = archive.getvalue()


def get(url, auth):
    if url == run_url:
        return json.dumps(run).encode()
    if url == f"{run_url}/jobs":
        return json.dumps(jobs).encode()
    if url == f"{run_url}/artifacts":
        return json.dumps({"artifacts": artifacts}).encode()
    return archives[url]


with patch.object(downloader, "get", side_effect=get) as request:
    wrong_destination = work / "wrong-run"
    try:
        downloader.download_native_run(base, run_id, "b" * 40, wrong_destination, "Basic test")
    except ValueError as error:
        assert "selected commit" in str(error)
    else:
        raise AssertionError("unexpectedly accepted a native workflow run for a different SHA")
    assert request.call_count == 1 and not wrong_destination.exists()
    downloader.download_native_run(base, run_id, sha, work / "historical-candidate", "Basic test")
PY

mkdir "${test_root}/historical-release"
python3 "${repo_root}/scripts/portable/release-packages.py" \
  stage "${test_root}/historical-candidate" 3.2.0 "${test_root}/historical-release"
(cd "${test_root}/historical-release" && sha256sum --strict -c SHA256SUMS)

cat >"${fake_bin}/curl" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

output=""
data_binary=""
upload_file=""
url=""
method="GET"
while (($#)); do
  case "$1" in
    --output|-o)
      output=$2
      shift 2
      ;;
    --request|-X)
      method=$2
      shift 2
      ;;
    --header|-H|--data|--user|--write-out|--proto-redir)
      shift 2
      ;;
    --data-binary)
      data_binary=$2
      shift 2
      ;;
    --upload-file)
      method="PUT"
      upload_file=$2
      shift 2
      ;;
    --fail|--fail-with-body|--silent|--show-error|--location)
      shift
      ;;
    http://*|https://*)
      url=$1
      shift
      ;;
    *)
      shift
      ;;
  esac
done

printf '%s %s\n' "${method}" "${url}" >>"${CURL_LOG}"
case "${url}" in
  https://*/api/packages/packages/generic/jointsdk-linux/3.2.0/*)
    name=${url##*/}
    if [[ "${method}" == PUT ]]; then
      if [[ -f "${TEST_REGISTRY}/${name}" ]]; then
        printf '409'
        exit 22
      fi
      cp "${upload_file}" "${TEST_REGISTRY}/${name}"
      printf '201'
      exit 0
    fi
    if [[ "${name}" == SHA256SUMS-arm64 && "${MISSING_ARM64:-}" == 1 ]]; then
      exit 22
    fi
    cp "${TEST_REGISTRY}/${name}" "${output}"
    if [[ "${name}" == *_arm64.rpm && "${CORRUPT_ARM64:-}" == 1 ]]; then
      printf 'corrupt\n' >>"${output}"
    fi
    ;;
  https://git.in.encos.cn/api/packages/packages/debian/pool/*/main/upload|\
    https://git.in.encos.cn/api/packages/packages/rpm/upload)
    test "${method}" = PUT
    printf '201'
    ;;
  */repos/acme/mirror/releases)
    printf '{"id":42,"upload_url":"https://uploads.github.test/repos/acme/mirror/releases/42/assets{?name,label}"}\n'
    ;;
  */repos/acme/mirror/releases/42)
    if [[ "${method}" == PATCH && "${FAIL_PATCH:-}" == 1 ]]; then
      git --git-dir="${TARGET_BARE}" update-ref refs/tags/v3.2.0 main
      exit 22
    fi
    printf '{"id":42,"draft":false}\n'
    ;;
  *uploads.github.test*/assets*)
    if [[ "${FAIL_UPLOAD:-}" == 1 ]]; then
      exit 22
    fi
    cp "${data_binary#@}" "${TEST_RELEASE_ASSETS}/${url##*name=}"
    printf '{"state":"uploaded"}\n'
    ;;
  *)
    printf 'unexpected URL: %s\n' "${url}" >&2
    exit 1
    ;;
esac
EOF
chmod +x "${fake_bin}/curl"

publish_candidates() {
  PATH="${fake_bin}:${PATH}" USER_PASSWORD="gitea-token" \
    bash "${repo_root}/scripts/portable/publish.sh" "${candidate_root}"
}

printf 'modified audit\n' >>"${candidate_root}/arm64/jointsdk-elf-audit.json"
if publish_candidates; then
  printf 'native publication unexpectedly accepted a corrupt candidate\n' >&2
  exit 1
fi
test ! -s "${curl_log}"
printf '{}\n' >"${candidate_root}/arm64/jointsdk-elf-audit.json"

publish_candidates
for arch in amd64 arm64; do
  for file in "${candidate_root}/${arch}"/jointsdk_*; do
    cmp "${file}" "${TEST_REGISTRY}/${file##*/}"
  done
  (cd "${TEST_REGISTRY}" && sha256sum --strict -c "SHA256SUMS-${arch}")
done
test "$(find "${TEST_REGISTRY}" -type f | wc -l)" -eq 8
test "$(sed -n '7,8p' "${curl_log}" | grep -c 'generic/jointsdk-linux/3.2.0/SHA256SUMS-')" -eq 2
test "$(grep -c '/debian/pool/' "${curl_log}")" -eq 4
test "$(grep -c '/rpm/upload' "${curl_log}")" -eq 2

: >"${curl_log}"
publish_candidates
test "$(grep -c '^GET .*generic/jointsdk-linux/3.2.0/' "${curl_log}")" -eq 8

run_release() {
  PATH="${fake_bin}:${PATH}" \
    GIT_CONFIG_GLOBAL="${git_config}" \
    GITEA_USERNAME="gitea-user" \
    GITEA_TOKEN="gitea-token" \
    GH_TOKEN="github-token" \
    bash "${script_path}" \
      --source-repo "${source_repo}" \
      --source-ref main \
      --github-repo acme/mirror \
      --github-repo-url "${target_bare}" \
      --github-branch main \
      --github-api-url https://api.github.test \
      --github-upload-url https://uploads.github.test \
      --gitea-url https://gitea.test \
      --gitea-package-owner packages \
      "$@"
}

run_release

mirror_checkout="${test_root}/mirror-checkout"
git clone -q --branch main "${target_bare}" "${mirror_checkout}"
test -f "${mirror_checkout}/include/public.h"
test -f "${mirror_checkout}/external/example/nested/payload.txt"
test -f "${mirror_checkout}/external/example/vendor/leaf/leaf.txt"
test ! -e "${mirror_checkout}/.gitmodules"
test ! -e "${mirror_checkout}/.gitea"
test ! -e "${mirror_checkout}/openspec"
test ! -e "${mirror_checkout}/stale.txt"
for internal_root in "${mirror_checkout}" "${mirror_checkout}/include"; do
  test ! -e "${internal_root}/AGENTS.md"
  test ! -e "${internal_root}/CLAUDE.md"
  for private_dir in .claude .codex .agents .cursor .gemini .opencode; do
    test ! -e "${internal_root}/${private_dir}"
  done
done
test ! -e "${mirror_checkout}/external/example/.git"
test ! -e "${mirror_checkout}/external/example/vendor/leaf/.git"
test "$(git -C "${mirror_checkout}" log -1 --format=%s)" = "release: 发布v3.2.0"
test "$(grep -c 'uploads.github.test.*/assets' "${curl_log}")" -eq 7
for name in "${TEST_REGISTRY}"/jointsdk_*; do
  cmp "${name}" "${TEST_RELEASE_ASSETS}/${name##*/}"
done
(cd "${TEST_RELEASE_ASSETS}" && sha256sum --strict -c SHA256SUMS)
grep -q 'PATCH https://api.github.test/repos/acme/mirror/releases/42' "${curl_log}"

commit_count_before=$(git --git-dir="${target_bare}" rev-list --count main)
: >"${curl_log}"
run_release
commit_count_after=$(git --git-dir="${target_bare}" rev-list --count main)
test "${commit_count_before}" = "${commit_count_after}"
test ! -s "${curl_log}"

git -C "${source_repo}" switch -q -c develop
printf 'develop snapshot\n' >"${source_repo}/develop.txt"
printf 'cmake_minimum_required(VERSION 3.18)\nproject(joint_sdk)\n' \
  >"${source_repo}/CMakeLists.txt"
git -C "${source_repo}" add .
git -C "${source_repo}" commit -q -m "develop snapshot"
develop_sha1=$(git -C "${source_repo}" rev-parse HEAD)
git -C "${source_repo}" switch -q main

multi_branch_env="${test_root}/multi-branch.env"
cat >"${multi_branch_env}" <<EOF
GITEA_USERNAME=gitea-user
GITEA_TOKEN=gitea-token
GH_TOKEN=github-token
SOURCE_REPO=${source_repo}
MAIN_BRANCH=main
OPTIONAL_BRANCHES=develop
GITHUB_RELEASE_REPOSITORY=acme/mirror
GITHUB_RELEASE_REPOSITORY_URL=${target_bare}
GITEA_URL=https://gitea.test
GITEA_PACKAGE_OWNER=packages
GITHUB_API_URL=https://api.github.test
GITHUB_UPLOAD_URL=https://uploads.github.test
EOF

: >"${curl_log}"
PATH="${fake_bin}:${PATH}" GIT_CONFIG_GLOBAL="${git_config}" \
  bash "${wrapper_path}" --env-file "${multi_branch_env}"
test ! -s "${curl_log}"
develop_checkout="${test_root}/develop-checkout"
git clone -q --branch develop "${target_bare}" "${develop_checkout}"
test -f "${develop_checkout}/develop.txt"
test "$(git -C "${develop_checkout}" log -1 --format=%s)" = "release: 发布v3.2.0"

printf 'changed snapshot\n' >"${source_repo}/include/public.h"
git -C "${source_repo}" add .
git -C "${source_repo}" commit -q -m "changed source snapshot"

: >"${curl_log}"
PATH="${fake_bin}:${PATH}" GIT_CONFIG_GLOBAL="${git_config}" \
  bash "${wrapper_path}" \
    --env-file "${multi_branch_env}" \
    --main-sha1 "${initial_source_sha1}"
test ! -s "${curl_log}"
main_after_pinned_wrapper=$(git --git-dir="${target_bare}" show main:include/public.h)
test "${main_after_pinned_wrapper}" = "public header"
test "$(git --git-dir="${target_bare}" show develop:develop.txt)" = "develop snapshot"

run_release \
  --source-sha1 "${initial_source_sha1}" \
  --github-branch pinned \
  --snapshot-only
pinned_checkout="${test_root}/pinned-checkout"
git clone -q --branch pinned "${target_bare}" "${pinned_checkout}"
grep -q '^public header$' "${pinned_checkout}/include/public.h"

set +e
run_release \
  --source-sha1 "${develop_sha1}" \
  --github-branch rejected \
  --snapshot-only >/dev/null 2>&1
unrelated_sha1_status=$?
set -e
test "${unrelated_sha1_status}" -ne 0
test -z "$(git --git-dir="${target_bare}" branch --list rejected)"

set +e
run_release \
  --source-sha1 ffffffffffffffffffffffffffffffffffffffff \
  --github-branch missing \
  --snapshot-only >/dev/null 2>&1
missing_sha1_status=$?
set -e
test "${missing_sha1_status}" -ne 0
test -z "$(git --git-dir="${target_bare}" branch --list missing)"

: >"${curl_log}"
set +e
MISSING_ARM64=1 run_release
missing_status=$?
set -e
test "${missing_status}" -ne 0
test "$(git --git-dir="${target_bare}" rev-list --count main)" = "${commit_count_after}"
test "$(grep -c 'github.test' "${curl_log}" || true)" -eq 0

: >"${curl_log}"
if CORRUPT_ARM64=1 run_release; then
  printf 'release unexpectedly accepted a corrupt RPM\n' >&2
  exit 1
fi
test "$(git --git-dir="${target_bare}" rev-list --count main)" = "${commit_count_after}"
test "$(grep -c 'github.test' "${curl_log}" || true)" -eq 0

: >"${curl_log}"
set +e
FAIL_UPLOAD=1 run_release
upload_status=$?
set -e
test "${upload_status}" -ne 0
test "$(git --git-dir="${target_bare}" rev-list --count main)" = "${commit_count_after}"
grep -q 'DELETE https://api.github.test/repos/acme/mirror/releases/42' "${curl_log}"

: >"${curl_log}"
set +e
FAIL_PATCH=1 run_release
patch_status=$?
set -e
test "${patch_status}" -ne 0
test "$(git --git-dir="${target_bare}" rev-list --count main)" = "${commit_count_after}"
test -z "$(git --git-dir="${target_bare}" tag --list v3.2.0)"
grep -q 'DELETE https://api.github.test/repos/acme/mirror/releases/42' "${curl_log}"

unsafe_urls=(
  'http://example.test/acme/unsafe.git'
  'https://token@gitea.test/acme/unsafe.git'
  'https://example.test/acme/unsafe.git'
)
for unsafe_url in "${unsafe_urls[@]}"; do
  git -C "${source_repo}" config -f .gitmodules \
    submodule.external/example.url "${unsafe_url}"
  git -C "${source_repo}" add .gitmodules
  git -C "${source_repo}" commit -q -m "unsafe submodule fixture"
  set +e
  run_release >/dev/null 2>&1
  unsafe_status=$?
  set -e
  test "${unsafe_status}" -ne 0
  test "$(git --git-dir="${target_bare}" rev-list --count main)" = "${commit_count_after}"
done

printf 'release_to_github_test: PASS\n'
