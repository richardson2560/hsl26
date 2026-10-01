#!/usr/bin/env python3
"""Verify that every lock hash belongs to exactly one local wheel."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import re
import sys


HASH_PATTERN = re.compile(r"--hash=sha256:([0-9a-f]{64})")
PIN_PATTERN = re.compile(r"^[a-z0-9][a-z0-9_.-]*==[^\s]+", re.IGNORECASE)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--wheelhouse", type=Path, required=True)
    parser.add_argument("--lock", type=Path, required=True)
    args = parser.parse_args()
    if not args.lock.is_file():
        raise SystemExit(f"lock file does not exist: {args.lock}")
    wheel_hashes = {sha256(path): path.name for path in args.wheelhouse.glob("*.whl")}
    failures: list[str] = []
    count = 0
    for number, raw in enumerate(args.lock.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if not PIN_PATTERN.match(line):
            failures.append(f"line {number}: requirement is not exactly pinned")
            continue
        hashes = HASH_PATTERN.findall(line)
        if len(hashes) != 1:
            failures.append(f"line {number}: expected one SHA-256 hash")
            continue
        if hashes[0] not in wheel_hashes:
            failures.append(f"line {number}: hash is absent from wheelhouse")
        count += 1
    if count == 0:
        failures.append("lock has no requirements")
    if failures:
        raise SystemExit("requirements lock verification failed:\n- " + "\n- ".join(failures))
    print(f"OK: {count} pinned distributions are present in {args.wheelhouse}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
