#!/usr/bin/env python3
"""Capture auditable R0 facts from a local Docker image.

This is intentionally evidence capture, not an approval tool. It never starts
robot drivers and only runs package/version inspection commands in the image.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from typing import Any


PROBES = {
    "os_release": "cat /etc/os-release",
    "architecture": "uname -m",
    "dpkg": "dpkg-query -W -f='${Package}\\t${Version}\\t${Architecture}\\n' | LC_ALL=C sort",
    "apt_sources": "grep -R --no-filename --line-number -E '^(deb|Types:|URIs:|Suites:|Components:|Signed-By:)' /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null || true",
    "apt_python3_pip": "apt-cache policy python3-pip",
    "python": "python3 --version",
    "pip": "python3 -m pip --version",
    "ros": "source /opt/ros/humble/setup.bash && ros2 doctor --report",
    "colcon": "python3 -c \"import importlib.metadata; print('colcon-core=' + importlib.metadata.version('colcon-core'))\"",
}


def run(command: list[str], *, check: bool = True, timeout_s: int = 60) -> str:
    completed = subprocess.run(
        command, text=True, capture_output=True, check=False, timeout=timeout_s
    )
    if check and completed.returncode:
        raise RuntimeError(
            f"command failed ({completed.returncode}): {' '.join(command)}\n{completed.stderr}"
        )
    return completed.stdout


def pull_image(image: str, timeout_s: int) -> None:
    """Pull with inherited output so a long first download is observable."""
    print(f"pulling {image} (timeout {timeout_s}s)...", file=sys.stderr, flush=True)
    completed = subprocess.run(["docker", "pull", image], check=False, timeout=timeout_s)
    if completed.returncode:
        raise RuntimeError(f"docker pull failed with exit code {completed.returncode}")


def digest(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def text_or_empty(value: str | bytes | None) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8", "replace")
    return value or ""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-image", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--pull", action="store_true", help="pull before inspection")
    parser.add_argument(
        "--pull-timeout-s", type=int, default=1800,
        help="maximum duration for docker pull (default: 1800)",
    )
    parser.add_argument(
        "--probe-timeout-s", type=int, default=90,
        help="maximum duration per in-container probe (default: 90)",
    )
    args = parser.parse_args()
    if args.pull_timeout_s <= 0 or args.probe_timeout_s <= 0:
        raise SystemExit("timeouts must be positive integers")
    if args.pull:
        pull_image(args.base_image, args.pull_timeout_s)
    inspect_raw = run(["docker", "image", "inspect", args.base_image])
    inspected: list[dict[str, Any]] = json.loads(inspect_raw)
    if len(inspected) != 1:
        raise RuntimeError("docker image inspect did not return exactly one image")
    image = inspected[0]
    probes: dict[str, dict[str, str | int]] = {}
    for name, command in PROBES.items():
        print(f"probing {name}...", file=sys.stderr, flush=True)
        try:
            completed = subprocess.run(
                ["docker", "run", "--rm", "--network", "none", "--entrypoint", "/bin/bash", args.base_image, "-lc", command],
                text=True,
                capture_output=True,
                check=False,
                timeout=args.probe_timeout_s,
            )
            exit_code = completed.returncode
            stdout = completed.stdout
            stderr = completed.stderr
        except subprocess.TimeoutExpired as exc:
            exit_code = 124
            stdout = text_or_empty(exc.stdout)
            stderr = text_or_empty(exc.stderr) + (
                f"\nR0 probe exceeded configured timeout of {args.probe_timeout_s}s"
            )
        probes[name] = {
            "command": command,
            "exit_code": exit_code,
            "stdout": stdout,
            "stderr": stderr,
            "stdout_sha256": digest(stdout),
        }
    payload = {
        "schema": "hsl26.r0.environment-capture.v1",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "requested_image": args.base_image,
        "image_id": image.get("Id"),
        "repo_digests": image.get("RepoDigests", []),
        "architecture": image.get("Architecture"),
        "os": image.get("Os"),
        "docker_config_digest": digest(json.dumps(image.get("Config", {}), sort_keys=True)),
        "probe_timeout_s": args.probe_timeout_s,
        "probes": probes,
        "decision": "CAPTURED_NOT_APPROVED",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"wrote {args.output}; review repo_digests and probe exit codes before use")
    return 0


if __name__ == "__main__":
    sys.exit(main())
