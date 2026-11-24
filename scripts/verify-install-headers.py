#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Encos
# SPDX-License-Identifier: MIT
"""Check the installed SDK header namespace, optionally using package-owned paths."""

import argparse
from pathlib import Path


def verify(prefix, includedir="include", manifest=None):
    root = prefix / includedir
    paths = (map(Path, manifest.read_text().splitlines()) if manifest else root.rglob("*"))
    for path in paths:
        try:
            relative = path.relative_to(root)
        except ValueError:
            continue
        if relative.parts and relative.parts[0] != "jointsdk":
            raise ValueError(f"SDK installs outside its header namespace: {path}")
    for name in ("JointManager.h", "JsonSchemaGenerator.h", "boost/preprocessor.hpp",
                 "boost/preprocessor/seq/for_each.hpp", "couple_joint/mechanism_solver.h"):
        if not (root / "jointsdk" / name).is_file():
            raise ValueError(f"Missing SDK header: {root / 'jointsdk' / name}")
    print(f"Header namespace verified: {root / 'jointsdk'}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prefix", type=Path)
    parser.add_argument("--includedir", default="include")
    parser.add_argument("--manifest", type=Path)
    args = parser.parse_args()
    try:
        verify(args.prefix.absolute(), args.includedir, args.manifest)
    except (ValueError, OSError) as error:
        parser.exit(1, f"{error}\n")
