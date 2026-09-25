# tools/train_gpis_prior.py
"""Train and persist a deterministic Hermite-GPIS-W prior from JSON observations."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from hsl_core.perception.implicit_surface import HermiteGPIS, HermiteObservation


def load_observations(path: str | Path) -> list[HermiteObservation]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not isinstance(data.get("observations"), list):
        raise ValueError("training input must contain an observations list")
    result = []
    for item in data["observations"]:
        if not isinstance(item, dict):
            raise ValueError("each observation must be an object")
        result.append(
            HermiteObservation(
                tuple(item["point_m"]),
                item["kind"],
                tuple(item.get("direction", (1.0, 0.0, 0.0))),
                float(item["value"]),
                float(item["noise_variance"]),
            )
        )
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_json")
    parser.add_argument("output_directory")
    parser.add_argument("--support-radius-m", type=float, required=True)
    parser.add_argument("--amplitude", type=float, default=1.0)
    parser.add_argument("--regularization", type=float, default=1e-8)
    args = parser.parse_args()
    observations = load_observations(args.input_json)
    model = HermiteGPIS(
        observations,
        support_radius_m=args.support_radius_m,
        amplitude=args.amplitude,
        regularization=args.regularization,
    )
    source_hash = hashlib.sha256(Path(args.input_json).read_bytes()).hexdigest()
    manifest = model.save(args.output_directory, source_hashes={str(args.input_json): source_hash})
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
