# tools/validate_gpis_prior.py
"""Validate a P4.1 model artifact without enabling implicit pickle loading."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from hsl_core.perception.implicit_surface import KERNEL_ID, SCHEMA_VERSION


def validate(directory: str | Path) -> dict:
    path = Path(directory)
    manifest = json.loads((path / "manifest.json").read_text(encoding="utf-8"))
    model_path = path / "model.npz"
    digest = hashlib.sha256(model_path.read_bytes()).hexdigest()
    if digest != manifest.get("artifact_sha256"):
        raise ValueError("model artifact hash does not match manifest")
    if manifest.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("unsupported GPIS artifact schema")
    if manifest.get("kernel_id") != KERNEL_ID:
        raise ValueError("GPIS kernel_id does not match the normative implementation")
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
        actual_shapes = {name: list(arrays[name].shape) for name in arrays.files}
        if manifest.get("array_shapes") != actual_shapes:
            raise ValueError("manifest array_shapes do not match model.npz")
        points = arrays["support_points_m"]
        count = len(points) if points.ndim == 2 else -1
        if points.ndim != 2 or points.shape[1] != 3 or count == 0:
            raise ValueError("support_points_m must have shape (N, 3), N > 0")
        if arrays["operator_kind"].shape != (count,) or not np.all(
            np.isin(arrays["operator_kind"], (0, 1))
        ):
            raise ValueError("operator_kind schema is invalid")
        if arrays["operator_direction"].shape != (count, 3):
            raise ValueError("operator_direction shape does not match observations")
        direction_norms = np.linalg.norm(arrays["operator_direction"], axis=1)
        if not np.allclose(direction_norms, 1.0, atol=1e-9):
            raise ValueError("operator directions must be unit length")
        if arrays["observations"].shape != (count,) or arrays["noise_variance"].shape != (count,):
            raise ValueError("observation arrays do not match support count")
        if np.any(arrays["noise_variance"] <= 0.0) or arrays["alpha"].shape != (count,):
            raise ValueError("noise variances and coefficients are invalid")
        factor = arrays["cholesky"]
        if (
            factor.shape != (count, count)
            or not np.allclose(factor, np.tril(factor), atol=1e-12)
            or np.any(np.diag(factor) <= 0.0)
        ):
            raise ValueError("Cholesky factor is not a valid lower-triangular factor")
        if (
            arrays["support_radius_m"].shape != (1,)
            or arrays["support_radius_m"][0] <= 0.0
            or arrays["amplitude"].shape != (1,)
            or arrays["amplitude"][0] <= 0.0
            or arrays["regularization"].shape != (1,)
            or arrays["regularization"][0] < 0.0
        ):
            raise ValueError("kernel scalar parameters are invalid")
        transform = arrays["base_from_model"]
        if (
            transform.shape != (4, 4)
            or not np.allclose(transform[3], (0.0, 0.0, 0.0, 1.0), atol=1e-9)
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-8)
        ):
            raise ValueError("base_from_model is not a rigid homogeneous transform")
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
