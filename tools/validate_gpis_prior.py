# tools/validate_gpis_prior.py
"""Validate a P4.1 model artifact without enabling implicit pickle loading."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def validate(directory: str | Path) -> dict:
    path = Path(directory)
    manifest = json.loads((path / "manifest.json").read_text(encoding="utf-8"))
    model_path = path / "model.npz"
    digest = hashlib.sha256(model_path.read_bytes()).hexdigest()
    if digest != manifest.get("artifact_sha256"):
        raise ValueError("model artifact hash does not match manifest")
    with np.load(model_path, allow_pickle=False) as arrays:
        required = {
            "support_points_m", "operator_kind", "operator_direction",
            "observations", "noise_variance", "alpha", "cholesky",
            "support_radius_m", "amplitude", "regularization",
            "base_from_model", "normalization_center_m", "normalization_scale",
        }
        if set(arrays.files) != required:
            raise ValueError("model arrays do not match the P4.1 schema")
        for name in arrays.files:
            if not np.all(np.isfinite(arrays[name])):
                raise ValueError(f"array {name} contains non-finite values")
    if manifest.get("uncertainty_method") != "exact_cholesky":
        raise ValueError("unsupported uncertainty method")
    return {
        "status": "PASS",
        "artifact_sha256": digest,
        "array_shapes": manifest["array_shapes"],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("directory")
    args = parser.parse_args()
    print(json.dumps(validate(args.directory), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
