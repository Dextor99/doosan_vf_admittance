#!/usr/bin/env python3
"""Verify the frozen legacy source tree against its SHA-256 manifest."""

from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
SNAPSHOT = REPO / "legacy-baseline" / "doosan_ft_calibration_cpp"
MANIFEST = REPO / "legacy-baseline" / "manifest.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    expected = {entry["path"]: entry for entry in data["files"]}
    actual = {
        path.relative_to(SNAPSHOT).as_posix(): path
        for path in SNAPSHOT.rglob("*")
        if path.is_file()
    }
    problems: list[str] = []
    for name in sorted(expected.keys() - actual.keys()):
        problems.append(f"missing: {name}")
    for name in sorted(actual.keys() - expected.keys()):
        problems.append(f"unexpected: {name}")
    for name in sorted(expected.keys() & actual.keys()):
        entry = expected[name]
        path = actual[name]
        if path.stat().st_size != entry["size_bytes"] or sha256(path) != entry["sha256"]:
            problems.append(f"changed: {name}")
    if problems:
        print("\n".join(problems), file=sys.stderr)
        return 1
    print(f"Verified {len(expected)} legacy files against {MANIFEST.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
