# hsl_core/hsl_core/geometry.py
"""Finite planar geometry helpers used by the pure domain layer.

"""

from typing import Tuple

from .rules import segment_intersection


def transform_point(
    point: Tuple[float, float],
    pose_x_m: float,
    pose_y_m: float,
    pose_theta_rad: float,
) -> Tuple[float, float]:
    """Transform a point from a body frame into its parent planar frame."""

    import math

    if not all(math.isfinite(float(value)) for value in (
        *point, pose_x_m, pose_y_m, pose_theta_rad
    )):
        raise ValueError("point transform inputs must be finite")
    cosine = math.cos(pose_theta_rad)
    sine = math.sin(pose_theta_rad)
    return (
        pose_x_m + cosine * point[0] - sine * point[1],
        pose_y_m + sine * point[0] + cosine * point[1],
    )
