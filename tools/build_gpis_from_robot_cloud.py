"""Fit a compact Hermite-GPIS-W model directly from a robot point cloud."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from hsl_core.perception.implicit_surface import HermiteGPIS, HermiteObservation


def _farthest_point_sample_indices(points: np.ndarray, count: int) -> np.ndarray:
    if count <= 0 or len(points) < count:
        raise ValueError("not enough point-cloud samples for requested model resolution")
    center = np.median(points, axis=0)
    first = int(np.argmax(np.sum((points - center) ** 2, axis=1)))
    selected = np.empty(count, dtype=np.int64)
    selected[0] = first
    nearest_squared = np.sum((points - points[first]) ** 2, axis=1)
    for index in range(1, count):
        selected[index] = int(np.argmax(nearest_squared))
        distance_squared = np.sum((points - points[selected[index]]) ** 2, axis=1)
        nearest_squared = np.minimum(nearest_squared, distance_squared)
    return selected


def build_observations(
    points_base_link: np.ndarray,
    *,
    max_surface_points: int = 100,
    normal_radius_m: float = 0.04,
    normal_max_neighbors: int = 30,
    sigma_lidar_m: float = 0.012,
    sigma_gradient: float = 0.05,
    sensor_origin_base_link_m: tuple[float, float, float] = (0.0, 0.0, 0.3),
) -> tuple[list[HermiteObservation], np.ndarray, dict]:
    points = np.asarray(points_base_link, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 6:
        raise ValueError("points_base_link must have shape (N, 3), N >= 6")
    if not np.all(np.isfinite(points)):
        raise ValueError("point cloud contains non-finite points")
    sensor_origin = np.asarray(sensor_origin_base_link_m, dtype=float)
    if sensor_origin.shape != (3,) or not np.all(np.isfinite(sensor_origin)):
        raise ValueError("sensor_origin_base_link_m must be a finite 3-vector")
    if (
        max_surface_points < 3
        or max_surface_points > len(points)
        or not math.isfinite(normal_radius_m)
        or normal_radius_m <= 0.0
        or normal_max_neighbors < 3
        or not math.isfinite(sigma_lidar_m)
        or sigma_lidar_m <= 0.0
        or not math.isfinite(sigma_gradient)
        or sigma_gradient <= 0.0
    ):
        raise ValueError("invalid resolution, normal-estimation or noise parameters")

    try:
        import open3d as o3d
    except ImportError as exc:
        raise RuntimeError(
            "building cloud normals requires Open3D in the tooling environment"
        ) from exc

    cloud_origin = np.median(points, axis=0)
    centered_points = points - cloud_origin
    point_cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(centered_points))
    point_cloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=normal_radius_m,
            max_nn=normal_max_neighbors,
        )
    )
    sensor_origin_model = sensor_origin - cloud_origin
    point_cloud.orient_normals_towards_camera_location(sensor_origin_model)
    all_normals = np.asarray(point_cloud.normals, dtype=float)
    normal_lengths = np.linalg.norm(all_normals, axis=1)
    valid_normals = (
        np.all(np.isfinite(all_normals), axis=1)
        & np.isfinite(normal_lengths)
        & (normal_lengths > 1e-12)
    )
    if np.count_nonzero(valid_normals) < max_surface_points:
        raise ValueError(
            "Open3D produced too few valid normals for the requested model resolution"
        )
    valid_indices = np.flatnonzero(valid_normals)
    valid_points = centered_points[valid_indices]
    selected_local_indices = _farthest_point_sample_indices(
        valid_points,
        max_surface_points,
    )
    selected_indices = valid_indices[selected_local_indices]
    samples = centered_points[selected_indices]
    normals = all_normals[selected_indices] / normal_lengths[selected_indices, None]

    value_noise_variance = sigma_lidar_m**2
    derivative_noise_variance = sigma_gradient**2
    observations: list[HermiteObservation] = []
    placeholder_direction = (1.0, 0.0, 0.0)
    for point, normal in zip(samples, normals):
        observations.append(
            HermiteObservation(
                tuple(float(value) for value in point),
                "value",
                placeholder_direction,
                0.0,
                value_noise_variance,
            )
        )
        observations.append(
            HermiteObservation(
                tuple(float(value) for value in point),
                "derivative",
                tuple(float(value) for value in normal),
                1.0,
                derivative_noise_variance,
            )
        )

    base_from_model = np.eye(4)
    base_from_model[:3, 3] = cloud_origin
    report = {
        "status": "CLOUD_FIT_CANDIDATE",
        "input_point_count": int(len(points)),
        "valid_normal_count": int(np.count_nonzero(valid_normals)),
        "surface_sample_count": int(len(samples)),
        "hermite_observation_count": int(len(observations)),
        "normal_estimator": "Open3D PointCloud.estimate_normals",
        "normal_radius_m": float(normal_radius_m),
        "normal_max_neighbors": int(normal_max_neighbors),
        "normal_orientation": "towards_sensor_origin",
        "sensor_origin_base_link_m": sensor_origin.tolist(),
        "value_noise_variance_m2": float(value_noise_variance),
        "directional_derivative_noise_variance": float(derivative_noise_variance),
        "surface_constraints": {
            "value_at_return_m": 0.0,
            "normal_derivative": 1.0,
            "off_surface_anchors": False,
        },
        "model_origin_base_link_m": cloud_origin.tolist(),
        "model_frame_semantics": (
            "The coordinate-wise cloud median defines a convenient local model frame; "
            "it is not asserted to be the robot's mechanical origin."
        ),
    }
    return observations, base_from_model, report


def build_model(
    input_npz: Path,
    output_directory: Path,
    *,
    array_name: str = "points_base_link",
    max_surface_points: int = 100,
    normal_radius_m: float = 0.04,
    normal_max_neighbors: int = 30,
    sigma_lidar_m: float = 0.012,
    sigma_gradient: float = 0.05,
    support_radius_m: float = 0.06,
    amplitude: float = 1.0,
    regularization: float = 1e-8,
    sensor_origin_base_link_m: tuple[float, float, float] = (0.0, 0.0, 0.3),
) -> dict:
    with np.load(input_npz, allow_pickle=False) as arrays:
        if array_name not in arrays.files:
            raise ValueError(f"input NPZ has no {array_name!r} array")
        points = np.asarray(arrays[array_name], dtype=float)
    observations, base_from_model, report = build_observations(
        points,
        max_surface_points=max_surface_points,
        normal_radius_m=normal_radius_m,
        normal_max_neighbors=normal_max_neighbors,
        sigma_lidar_m=sigma_lidar_m,
        sigma_gradient=sigma_gradient,
        sensor_origin_base_link_m=sensor_origin_base_link_m,
    )
    model = HermiteGPIS(
        observations,
        support_radius_m=support_radius_m,
        amplitude=amplitude,
        regularization=regularization,
        model_frame="static_1m_robot_cloud_centered",
        base_from_model=base_from_model,
    )
    source_hash = hashlib.sha256(input_npz.read_bytes()).hexdigest()
    manifest = model.save(
        output_directory,
        source_hashes={str(input_npz): source_hash},
    )
    parameters = {
        "array_name": array_name,
        "max_surface_points": max_surface_points,
        "normal_radius_m": normal_radius_m,
        "normal_max_neighbors": normal_max_neighbors,
        "sigma_lidar_m": sigma_lidar_m,
        "sigma_gradient": sigma_gradient,
        "support_radius_m": support_radius_m,
        "amplitude": amplitude,
        "regularization": regularization,
        "sensor_origin_base_link_m": list(sensor_origin_base_link_m),
    }
    manifest["training_method"] = "sampled_points_with_oriented_open3d_normals"
    manifest["training_status"] = "CLOUD_FIT_CANDIDATE"
    manifest["training_parameters"] = parameters
    manifest["training_report"] = report
    manifest["training_config_hash"] = hashlib.sha256(
        json.dumps(parameters, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    (output_directory / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n",
        encoding="utf-8",
    )
    training_report = {
        **report,
        "model_id": manifest["model_id"],
        "model_sha256": manifest["artifact_sha256"],
        "condition_number": model.condition_number,
        "training_parameters": parameters,
    }
    (output_directory / "training_report.json").write_text(
        json.dumps(training_report, indent=2) + "\n",
        encoding="utf-8",
    )
    return training_report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_npz", type=Path)
    parser.add_argument("output_directory", type=Path)
    parser.add_argument("--array", default="points_base_link")
    parser.add_argument("--surface-points", type=int, default=100)
    parser.add_argument("--normal-radius-m", type=float, default=0.04)
    parser.add_argument("--normal-max-neighbors", type=int, default=30)
    parser.add_argument("--sigma-lidar-m", type=float, default=0.012)
    parser.add_argument("--sigma-gradient", type=float, default=0.05)
    parser.add_argument("--support-radius-m", type=float, default=0.06)
    parser.add_argument("--amplitude", type=float, default=1.0)
    parser.add_argument("--regularization", type=float, default=1e-8)
    parser.add_argument(
        "--sensor-origin-base",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.3),
        metavar=("X", "Y", "Z"),
    )
    args = parser.parse_args()
    result = build_model(
        args.input_npz,
        args.output_directory,
        array_name=args.array,
        max_surface_points=args.surface_points,
        normal_radius_m=args.normal_radius_m,
        normal_max_neighbors=args.normal_max_neighbors,
        sigma_lidar_m=args.sigma_lidar_m,
        sigma_gradient=args.sigma_gradient,
        support_radius_m=args.support_radius_m,
        amplitude=args.amplitude,
        regularization=args.regularization,
        sensor_origin_base_link_m=tuple(args.sensor_origin_base),
    )
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
