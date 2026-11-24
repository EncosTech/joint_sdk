#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Encos
# SPDX-License-Identifier: MIT
"""保存 stable CI 的 SDK 原件，并验证 GitHub Release 的下载。"""

import argparse
import hashlib
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile


ARCHITECTURES = ("amd64", "arm64")
SUFFIXES = (".deb", ".rpm", ".tar.gz")
VERSION = r"[0-9]+\.[0-9]+\.[0-9]+"
GITEA_REGISTRY = "https://git.in.encos.cn/api/packages/packages"
CURL = ("curl", "--fail-with-body", "--silent", "--show-error")


def package_names(version, arch, prefix="jointsdk"):
    stem = f"{prefix}_{version}_linux-glibc2.28_{arch}"
    return [stem + suffix for suffix in SUFFIXES]


def read_checksums(manifest):
    if manifest.is_symlink():
        raise ValueError(f"Checksum manifest must not be a symlink: {manifest}")
    entries = {}
    for line in manifest.read_text(encoding="utf-8").splitlines():
        match = re.fullmatch(r"([0-9a-f]{64})  (?:\./)?([a-zA-Z0-9][a-zA-Z0-9_.-]*)", line)
        if not match:
            raise ValueError(f"Invalid checksum entry in {manifest}")
        checksum, name = match.groups()
        if name in entries:
            raise ValueError(f"Duplicate checksum entry for {name} in {manifest}")
        entries[name] = checksum
    return entries


def write_checksums(path, entries):
    path.write_text("".join(f"{entries[name]}  {name}\n" for name in sorted(entries)), encoding="utf-8")


def package_checksum(path):
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"Invalid candidate file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as package:
        while chunk := package.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def verify_checksum(path, expected):
    if package_checksum(path) != expected:
        raise ValueError(f"Checksum mismatch: {path.name}")


def find_version(names, prefix, arch):
    pattern = re.compile(rf"{prefix}_({VERSION})_linux-glibc2\.28_{arch}\.deb")
    versions = [match[1] for name in names if (match := pattern.fullmatch(name))]
    if len(versions) != 1:
        raise ValueError(f"Expected one {prefix} DEB for {arch}")
    return versions[0]


def release_candidates(root):
    versions = set()
    release = {}
    for arch in ARCHITECTURES:
        directory = root / arch
        entries = read_checksums(directory / "SHA256SUMS")
        version = find_version(entries, "jointsdk", arch)
        driver_version = find_version(entries, "libencosdriver", arch)
        release_names = package_names(version, arch)
        expected = {
            *release_names,
            *package_names(driver_version, arch, "libencosdriver"),
            "jointsdk-elf-audit.json",
        }
        if set(entries) != expected or {path.name for path in directory.iterdir()} != expected | {"SHA256SUMS"}:
            raise ValueError(f"Unexpected candidate set for {arch}")
        for name, checksum in entries.items():
            verify_checksum(directory / name, checksum)
        release[arch] = {name: entries[name] for name in release_names}
        versions.add(version)
    if len(versions) != 1:
        raise ValueError("Architecture candidates have different SDK versions")
    return versions.pop(), release


def download(url, path, authentication):
    subprocess.run([
        *CURL, *authentication, "--location", "--proto-redir", "=https",
        "--output", str(path), url,
    ], check=True)


def upload(path, url, authentication, existing=None):
    result = subprocess.run([
        *CURL, *authentication, "--upload-file", str(path), "--output", "/dev/null",
        "--write-out", "%{http_code}", url,
    ], stdout=subprocess.PIPE, text=True)
    if result.returncode == 0 and result.stdout in ("200", "201", "204"):
        return
    if existing is not None and result.stdout == "409":
        download(url, existing, authentication)
        verify_checksum(existing, package_checksum(path))
        return
    raise ValueError(f"Upload failed for {path.name}: HTTP {result.stdout or 'unavailable'}")


def publish(root):
    version, release = release_candidates(root)
    authentication = ["--user", f"bismarckkk:{os.environ['USER_PASSWORD']}"]
    registry = f"{GITEA_REGISTRY}/generic/jointsdk-linux/{version}"
    with tempfile.TemporaryDirectory() as temporary:
        work = Path(temporary)
        existing = work / "existing-file"
        for arch, entries in release.items():
            for name in entries:
                upload(root / arch / name, f"{registry}/{name}", authentication, existing)
        # 两份清单最后上传，作为该版本可供 GitHub 发布的标记。
        for arch, entries in release.items():
            manifest = work / f"SHA256SUMS-{arch}"
            write_checksums(manifest, entries)
            upload(manifest, f"{registry}/{manifest.name}", authentication, existing)

    for arch in ARCHITECTURES:
        deb, rpm, _ = package_names(version, arch)
        for distribution in ("jammy", "noble"):
            upload(root / arch / deb, f"{GITEA_REGISTRY}/debian/pool/{distribution}/main/upload", authentication)
        upload(root / arch / rpm, f"{GITEA_REGISTRY}/rpm/upload", authentication)


def prepare(registry, version, destination):
    if not re.fullmatch(VERSION, version):
        raise ValueError("Invalid SDK version")
    authentication = ["--header", f"Authorization: Basic {os.environ['GITEA_RELEASE_AUTH']}"]
    entries = {}
    with tempfile.TemporaryDirectory(dir=destination.parent) as temporary:
        for arch in ARCHITECTURES:
            manifest = Path(temporary) / f"SHA256SUMS-{arch}"
            download(f"{registry}/{manifest.name}", manifest, authentication)
            checksums = read_checksums(manifest)
            if set(checksums) != set(package_names(version, arch)):
                raise ValueError(f"Unexpected SDK package set in {manifest.name} for version {version}")
            entries.update(checksums)
    for name, checksum in entries.items():
        path = destination / name
        download(f"{registry}/{name}", path, authentication)
        verify_checksum(path, checksum)
    write_checksums(destination / "SHA256SUMS", entries)


def stage(root, version, destination):
    actual_version, release = release_candidates(root)
    if actual_version != version:
        raise ValueError(f"Native workflow built SDK {actual_version}, expected {version}")
    entries = {}
    for arch, checksums in release.items():
        for name in checksums:
            shutil.copyfile(root / arch / name, destination / name)
        entries.update(checksums)
    write_checksums(destination / "SHA256SUMS", entries)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    publish_parser = commands.add_parser("publish")
    publish_parser.add_argument("root", type=Path)
    prepare_parser = commands.add_parser("prepare")
    prepare_parser.add_argument("registry")
    prepare_parser.add_argument("version")
    prepare_parser.add_argument("destination", type=Path)
    stage_parser = commands.add_parser("stage")
    stage_parser.add_argument("root", type=Path)
    stage_parser.add_argument("version")
    stage_parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    if args.command == "publish":
        publish(args.root)
    elif args.command == "prepare":
        prepare(args.registry, args.version, args.destination)
    else:
        stage(args.root, args.version, args.destination)


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as error:
        print(f"Portable SDK package transfer failed: {error.cmd[-1]}", file=sys.stderr)
        sys.exit(1)
    except (KeyError, OSError, ValueError) as error:
        print(f"Portable SDK release aborted: {error}", file=sys.stderr)
        sys.exit(1)
