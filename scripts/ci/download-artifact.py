#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Encos
# SPDX-License-Identifier: MIT
"""通过 Gitea REST API 下载候选文件，不依赖外部下载 action。"""
import argparse
import base64
import io
import json
import os
from pathlib import Path
import re
import sys
import urllib.error
import urllib.parse
import urllib.request
import zipfile


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def get(url, authorization):
    origin = urllib.parse.urlsplit(url).netloc
    opener = urllib.request.build_opener(NoRedirect)
    headers = {"Authorization": authorization} if authorization else {}
    for _ in range(4):
        try:
            with opener.open(urllib.request.Request(url, headers=headers), timeout=180) as response:
                return response.read()
        except urllib.error.HTTPError as error:
            if error.code not in (301, 302, 303, 307, 308):
                raise ValueError(f"Artifact request failed: HTTP {error.code}") from None
            url = urllib.parse.urljoin(url, error.headers.get("Location", ""))
            if urllib.parse.urlsplit(url).scheme != "https":
                raise ValueError("Artifact redirect must use HTTPS")
            # Gitea 同源 /zip/raw 需要认证；跨源对象存储不接收认证头。
            if urllib.parse.urlsplit(url).netloc != origin:
                headers = {}
        except urllib.error.URLError:
            raise ValueError("Artifact request transport failed") from None
    raise ValueError("Too many artifact redirects")


def download_artifact(base, artifacts, name, destination, run_id, sha, auth):
    matches = [a for a in artifacts if a["name"] == name]
    if len(matches) != 1:
        raise ValueError("Expected exactly one candidate artifact for this run")
    artifact = matches[0]
    workflow_run = artifact["workflow_run"]
    if artifact.get("expired") or workflow_run["head_sha"] != sha or workflow_run["id"] != run_id:
        raise ValueError("Artifact expired or producer commit does not match")
    archive = get(f"{base}/actions/artifacts/{int(artifact['id'])}/zip", auth)
    destination.mkdir(parents=True, exist_ok=True)
    if any(destination.iterdir()):
        raise ValueError("Artifact destination must be empty")
    with zipfile.ZipFile(io.BytesIO(archive)) as package:
        names = set()
        for member in package.infolist():
            filename = member.filename
            if (not filename or filename in (".", "..") or "/" in filename or "\\" in filename
                    or filename in names or member.is_dir()
                    or (member.external_attr >> 16) & 0o170000 == 0o120000):
                raise ValueError("Candidate artifact must contain unique flat regular files")
            names.add(filename)
        for member in package.infolist():
            (destination / member.filename).write_bytes(package.read(member))
    print(f"Downloaded {name}: {len(names)} files from workflow run {run_id}")


def native_run_api(gitea_url, source_repo):
    gitea = urllib.parse.urlsplit(gitea_url)
    source = urllib.parse.urlsplit(source_repo)
    prefix = gitea.path.rstrip("/") + "/"
    if (gitea.scheme != "https" or not gitea.hostname or gitea.username is not None
            or source.scheme != "https" or source.netloc != gitea.netloc
            or not source.path.startswith(prefix)):
        raise ValueError("Native run requires an HTTPS source repository on the configured Gitea host")
    repository = source.path[len(prefix):].removesuffix(".git")
    if not re.fullmatch(r"[\w.-]+/[\w.-]+", repository) or any(
            part in {".", ".."} for part in repository.split("/")):
        raise ValueError("Native run requires a Gitea source repository URL with owner and repository")
    return f"{gitea_url.rstrip('/')}/api/v1/repos/{repository}"


def download_native_run(base, run_id, sha, destination, auth):
    run_url = f"{base}/actions/runs/{run_id}"
    run = json.loads(get(run_url, auth))
    expected = {
        "id": run_id,
        "path": "build-native.yml@refs/heads/stable",
        "event": "push",
        "head_branch": "stable",
        "head_sha": sha,
        "status": "completed",
        "conclusion": "success",
    }
    if any(run.get(key) != value for key, value in expected.items()):
        raise ValueError("Expected a successful stable build-native push run for the selected commit")
    jobs = json.loads(get(f"{run_url}/jobs", auth))["jobs"]
    if not any(job["name"] == "publish-tested-packages" and job.get("conclusion") == "success" for job in jobs):
        raise ValueError("Native run is missing the successful publish-tested-packages job")
    artifacts = json.loads(get(f"{run_url}/artifacts", auth))["artifacts"]
    for arch in ("amd64", "arm64"):
        download_artifact(base, artifacts, f"jointsdk-portable-{arch}-{sha}",
                          destination / arch, run_id, sha, auth)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", nargs="?")
    parser.add_argument("destination", type=Path)
    parser.add_argument("--native-run-id", type=int)
    parser.add_argument("--gitea-url")
    parser.add_argument("--source-repo")
    parser.add_argument("--source-sha")
    args = parser.parse_args()
    if args.native_run_id is not None:
        if (args.native_run_id < 1 or args.name or not args.gitea_url or not args.source_repo
                or not re.fullmatch(r"[0-9a-f]{40}", args.source_sha or "")):
            parser.error("a native run requires its ID, Gitea URL, source repo and exact SHA")
        base = native_run_api(args.gitea_url, args.source_repo)
        download_native_run(base, args.native_run_id, args.source_sha, args.destination,
                            "Basic " + os.environ["GITEA_RELEASE_AUTH"])
        return
    if not args.name:
        parser.error("the current run requires an artifact name")
    secret = os.environ["USER_PASSWORD"]
    auth = "Basic " + base64.b64encode(f"bismarckkk:{secret}".encode()).decode()
    repository = os.environ["GITHUB_REPOSITORY"]
    run_id = int(os.environ["GITHUB_RUN_ID"])
    sha = os.environ["GITHUB_SHA"]
    base = "https://git.in.encos.cn/api/v1/repos/" + repository
    artifacts = json.loads(get(f"{base}/actions/runs/{run_id}/artifacts", auth))["artifacts"]
    download_artifact(base, artifacts, args.name, args.destination, run_id, sha, auth)


if __name__ == "__main__":
    try:
        main()
    except (ValueError, KeyError, OSError, zipfile.BadZipFile) as error:
        print(f"Artifact download aborted: {error}", file=sys.stderr)
        sys.exit(1)
