"""Run a reproducible synthetic registration smoke test for a cloud-trained GPIS."""

from __future__ import annotations

import argparse
import hashlib
import json
import time
from pathlib import Path

import numpy as np

from hsl_core.perception.implicit_surface import HermiteGPIS
from hsl_core.perception.registration import PlanarRegistrar, RegistrationConfig


def evaluate(
    input_npz: Path,
    model_directory: Path,
    *,
    array_name: str = "points_base_link",
    test_point_count: int = 80,
    truth_pose_xyyaw: tuple[float, float, float] = (0.25, -0.15, 0.45),
    initial_pose_xyyaw: tuple[float, float, float] = (0.245, -0.146, 0.44),
) -> dict:
    if test_point_count < 6:
        raise ValueError("test_point_count must be at least 6")
    model = HermiteGPIS.load(model_directory)
    with np.load(input_npz, allow_pickle=False) as arrays:
        if array_name not in arrays.files:
            raise ValueError(f"input NPZ has no {array_name!r} array")
        points = np.asarray(arrays[array_name], dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 6:
        raise ValueError("input point cloud must have shape (N, 3), N >= 6")
    if not np.all(np.isfinite(points)):
        raise ValueError("input cloud contains non-finite points")

    cloud_origin = np.median(points, axis=0)
    centered_points = points - cloud_origin
    selected = np.linspace(
        0, len(centered_points) - 1, min(test_point_count, len(centered_points)), dtype=int
    )
    test_points = centered_points[selected]
    truth = np.asarray(truth_pose_xyyaw, dtype=float)
    initial = np.asarray(initial_pose_xyyaw, dtype=float)
    if (
        truth.shape != (3,)
        or initial.shape != (3,)
        or not np.all(np.isfinite(truth))
        or not np.all(np.isfinite(initial))
    ):
        raise ValueError("truth and initial poses must be finite x, y, yaw vectors")
    cosine, sine = np.cos(truth[2]), np.sin(truth[2])
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    observed_points = np.column_stack(
        (test_points[:, :2] @ rotation.T + truth[:2], test_points[:, 2])
    )
    config = RegistrationConfig(
        max_iterations=30,
        max_translation_step_m=0.05,
        max_yaw_step_rad=0.1,
        damping=1e-3,
        min_support_fraction=0.4,
        min_points=6,
        min_yaw_information=1e-12,
        max_residual_rms=20.0,
    )
    point_variance_m2 = 1e-3
    start = time.perf_counter()
    result = PlanarRegistrar(model, config).register(
        observed_points,
        initial_pose_xyyaw=initial,
        point_variance_m2=point_variance_m2,
    )
    elapsed_ms = (time.perf_counter() - start) * 1000.0
    yaw_error = (result.pose_xyyaw[2] - truth[2] + np.pi) % (2.0 * np.pi) - np.pi
    digest = hashlib.sha256(input_npz.read_bytes()).hexdigest()
    manifest = json.loads((model_directory / "manifest.json").read_text(encoding="utf-8"))
    return {
        "status": "SYNTHETIC_SMOKE_ACCEPTED" if result.status == "ACCEPTED" else "SYNTHETIC_SMOKE_REJECTED",
        "evidence_type": "synthetic rigid transform of a subset from the same source cloud",
        "not_evidence_of": [
            "independent physical-view accuracy",
            "cross-robot hardware equivalence",
            "production origin or yaw calibration",
        ],
        "model_id": manifest["model_id"],
        "model_sha256": manifest["artifact_sha256"],
        "source_cloud_sha256": digest,
        "source_array": array_name,
        "test_point_count": len(test_points),
        "registration_config": {
            "max_iterations": config.max_iterations,
            "max_translation_step_m": config.max_translation_step_m,
            "max_yaw_step_rad": config.max_yaw_step_rad,
            "damping": config.damping,
            "min_support_fraction": config.min_support_fraction,
            "max_residual_rms": config.max_residual_rms,
            "point_variance_m2": point_variance_m2,
        },
        "truth_pose_xyyaw": truth.tolist(),
        "initial_pose_xyyaw": initial.tolist(),
        "estimated_pose_xyyaw": list(result.pose_xyyaw),
        "translation_error_m": float(np.linalg.norm(np.asarray(result.pose_xyyaw[:2]) - truth[:2])),
        "yaw_error_rad": float(yaw_error),
        "registration_status": result.status,
        "registration_reason": result.reason,
        "support_fraction": result.support_fraction,
        "residual_rms": result.residual_rms,
        "position_valid": result.position_valid,
        "yaw_valid": result.yaw_valid,
        "iterations": result.iterations,
        "registration_latency_ms": elapsed_ms,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_npz", type=Path)
    parser.add_argument("model_directory", type=Path)
    parser.add_argument("--array", default="points_base_link")
    parser.add_argument("--test-point-count", type=int, default=80)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = evaluate(
        args.input_npz,
        args.model_directory,
        array_name=args.array,
        test_point_count=args.test_point_count,
    )
    output = args.output or args.model_directory / "synthetic_registration_smoke_test.json"
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result["registration_status"] == "ACCEPTED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
