# tools/evaluate_p45_fidelity.py
"""Produce evaluation-only profile metrics from labeled Phase-4 sequence JSON."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from hsl_core.perception.fidelity import evaluate_profiles


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_json", help="evaluation-only cases with referee labels")
    parser.add_argument("output_json")
    parser.add_argument("--minimum-labeled-samples", type=int, default=20)
    args = parser.parse_args()
    samples = json.loads(Path(args.input_json).read_text(encoding="utf-8"))
    if not isinstance(samples, list):
        raise ValueError("input JSON root must be a list of labeled cases")
    report = evaluate_profiles(samples, minimum_labeled_samples=args.minimum_labeled_samples)
    report["source_sha256"] = hashlib.sha256(Path(args.input_json).read_bytes()).hexdigest()
    Path(args.output_json).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output_json).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
