#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Encos
# SPDX-License-Identifier: MIT

"""Validate the license metadata shipped inside DEB and npm packages."""

import io
import json
import subprocess
import sys
import tarfile
from pathlib import Path


def verify(package: Path) -> None:
    if package.suffix == ".deb":
        payload = subprocess.check_output(["dpkg-deb", "--fsys-tarfile", str(package)])
        archive = tarfile.open(fileobj=io.BytesIO(payload))
    else:
        archive = tarfile.open(package, "r:gz")
    with archive:
        files = {entry.name.removeprefix("./"): entry for entry in archive if entry.isfile()}
        if package.suffix == ".deb":
            matches = [name for name in files if name.endswith("share/doc/jointsdk/LICENSE")]
            assert len(matches) == 1, "DEB must contain exactly one joint_sdk LICENSE"
            license_path = matches[0]
            copyright_path = license_path.removesuffix("LICENSE") + "copyright"
            copyright_text = archive.extractfile(files[copyright_path]).read().decode()
            assert "License: MIT" in copyright_text, "DEB copyright must declare MIT"
            assert "License: BSL-1.0" in copyright_text, "Preserve bundled Boost license"
        else:
            license_path = "package/LICENSE"
            metadata = json.load(archive.extractfile(files["package/package.json"]))
            assert metadata.get("license") == "MIT", "npm license must be MIT"
        license_text = archive.extractfile(files[license_path]).read().decode()
        canonical = Path(__file__).resolve().parents[1].joinpath("LICENSE").read_text()
        assert license_text == canonical, "Packaged LICENSE differs from repository LICENSE"
        assert "Permission is hereby granted, free of charge" in license_text
        assert "THE SOFTWARE IS PROVIDED" in license_text
    print(f"License verified: {package}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit("Usage: verify-package-license.py PACKAGE.deb|PACKAGE.tgz [...]")
    for argument in sys.argv[1:]:
        verify(Path(argument))
