"""Build the same cloud-fit HGW model at several point-sample resolutions."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

if __package__:
    from .build_gpis_from_robot_cloud import build_model
else:
    from build_gpis_from_robot_cloud import build_model


DEFAULT_RESOLUTIONS = (100, 150, 200, 250)


def create_resolution_suite(
    input_npz: Path,
    output_directory: Path,
    *,
    array_name: str = "points_base_link",
    resolutions: tuple[int, ...] = DEFAULT_RESOLUTIONS,
    normal_radius_m: float = 0.04,
    normal_max_neighbors: int = 30,
    sigma_lidar_m: float = 0.012,
    sigma_gradient: float = 0.05,
    support_radius_m: float = 0.06,
    amplitude: float = 1.0,
    regularization: float = 1e-8,
    sensor_origin_base_link_m: tuple[float, float, float] = (0.0, 0.0, 0.3),
) -> dict:
    if not resolutions or any(count < 3 for count in resolutions):
        raise ValueError("each resolution must request at least 3 surface points")
    if len(set(resolutions)) != len(resolutions):
        raise ValueError("resolutions must be unique")
    output_directory.mkdir(parents=True, exist_ok=True)

    with np.load(input_npz, allow_pickle=False) as arrays:
        if array_name not in arrays.files:
            raise ValueError(f"input NPZ has no {array_name!r} array")
        input_point_count = int(len(arrays[array_name]))
    source_hash = hashlib.sha256(input_npz.read_bytes()).hexdigest()
    metadata_path = input_npz.with_suffix(".json")
    metadata_hash = (
        hashlib.sha256(metadata_path.read_bytes()).hexdigest()
        if metadata_path.is_file()
        else None
    )

    variants = []
    for surface_count in resolutions:
        variant_directory = output_directory / f"surface_points_{surface_count:03d}"
        if variant_directory.exists() and any(variant_directory.iterdir()):
            raise FileExistsError(f"refusing to overwrite existing variant: {variant_directory}")
        report = build_model(
            input_npz,
            variant_directory,
            array_name=array_name,
            max_surface_points=surface_count,
            normal_radius_m=normal_radius_m,
            normal_max_neighbors=normal_max_neighbors,
            sigma_lidar_m=sigma_lidar_m,
            sigma_gradient=sigma_gradient,
            support_radius_m=support_radius_m,
            amplitude=amplitude,
            regularization=regularization,
            sensor_origin_base_link_m=sensor_origin_base_link_m,
        )
        with np.load(variant_directory / "model.npz", allow_pickle=False) as model_arrays:
            support_points = model_arrays["support_points_m"]
            operator_kind = model_arrays["operator_kind"]
            actual_surface_count = len(np.unique(support_points, axis=0))
            actual_observation_count = len(operator_kind)
            value_count = int(np.count_nonzero(operator_kind == 0))
            derivative_count = int(np.count_nonzero(operator_kind == 1))
        if (
            actual_surface_count != surface_count
            or actual_observation_count != 2 * surface_count
            or value_count != surface_count
            or derivative_count != surface_count
            or report["surface_sample_count"] != surface_count
            or report["hermite_observation_count"] != 2 * surface_count
        ):
            raise RuntimeError(
                f"{variant_directory} contains {actual_surface_count} unique surface "
                f"samples and {actual_observation_count} Hermite observations; expected "
                f"{surface_count} surface samples and {2 * surface_count} observations"
            )
        variants.append(
            {
                "surface_sample_count": actual_surface_count,
                "hermite_observation_count": actual_observation_count,
                "value_observation_count": value_count,
                "normal_derivative_observation_count": derivative_count,
                "model_id": report["model_id"],
                "model_sha256": report["model_sha256"],
                "condition_number": report["condition_number"],
                "model_directory": variant_directory.relative_to(output_directory).as_posix(),
            }
        )

    summary = {
        "status": "CLOUD_FIT_RESOLUTION_CANDIDATES",
        "input_npz": str(input_npz),
        "input_npz_sha256": source_hash,
        "input_point_count": input_point_count,
        "metadata_json": str(metadata_path) if metadata_hash else None,
        "metadata_json_sha256": metadata_hash,
        "array": array_name,
        "normal_estimator": "Open3D PointCloud.estimate_normals",
        "normal_radius_m": normal_radius_m,
        "normal_max_neighbors": normal_max_neighbors,
        "support_radius_m": support_radius_m,
        "amplitude": amplitude,
        "off_surface_anchors": False,
        "variants": variants,
        "note": (
            "These are alternative model sample counts, not a measured identification "
            "accuracy or a resolution threshold."
        ),
    }
    (output_directory / "resolution_candidates.json").write_text(
        json.dumps(summary, indent=2) + "\n",
        encoding="utf-8",
    )
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_npz", type=Path)
    parser.add_argument("output_directory", type=Path)
    parser.add_argument("--array", default="points_base_link")
    parser.add_argument(
        "--resolutions",
        type=int,
        nargs="+",
        default=DEFAULT_RESOLUTIONS,
        help="number of unique sampled surface points; default: 100 150 200 250",
    )
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
    result = create_resolution_suite(
        args.input_npz,
        args.output_directory,
        array_name=args.array,
        resolutions=tuple(args.resolutions),
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
