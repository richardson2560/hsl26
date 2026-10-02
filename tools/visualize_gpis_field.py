"""Visualize a GPIS zero-level wireframe and a residual-colored point cloud."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from hsl_core.perception.implicit_surface import HermiteGPIS
from scipy.spatial import cKDTree


_CUBE_CORNERS = (
    (0, 0, 0),
    (1, 0, 0),
    (1, 1, 0),
    (0, 1, 0),
    (0, 0, 1),
    (1, 0, 1),
    (1, 1, 1),
    (0, 1, 1),
)
_TETRAHEDRA = (
    (0, 5, 1, 6),
    (0, 1, 2, 6),
    (0, 2, 3, 6),
    (0, 3, 7, 6),
    (0, 7, 4, 6),
    (0, 4, 5, 6),
)
_TETRA_EDGES = ((0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3))


def _marching_tetrahedra(
    axes: tuple[np.ndarray, np.ndarray, np.ndarray],
    volume: np.ndarray,
    *,
    level: float = 0.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Extract a triangle mesh from a regular scalar grid without extra packages."""
    if volume.shape != tuple(len(axis) for axis in axes):
        raise ValueError("volume shape must match the three grid axes")
    if any(len(axis) < 2 for axis in axes) or not np.all(np.isfinite(volume)):
        raise ValueError("grid axes must have at least two finite values")
    if not np.isfinite(level):
        raise ValueError("isosurface level must be finite")

    grid_points = np.stack(
        np.meshgrid(*axes, indexing="ij"), axis=-1
    ).reshape(-1, 3)
    grid_values = volume.ravel() - level
    cube_indices = np.indices(
        (len(axes[0]) - 1, len(axes[1]) - 1, len(axes[2]) - 1),
        dtype=np.int64,
    ).reshape(3, -1).T
    cube_corner_indices = []
    grid_shape = volume.shape
    for offset in _CUBE_CORNERS:
        ijk = cube_indices + np.asarray(offset)
        cube_corner_indices.append(np.ravel_multi_index(ijk.T, grid_shape))
    cube_corner_indices = np.stack(cube_corner_indices, axis=1)

    all_vertices = []
    all_triangles = []
    vertex_offset = 0
    for tetrahedron in _TETRAHEDRA:
        corner_ids = cube_corner_indices[:, tetrahedron]
        corner_values = grid_values[corner_ids]
        case_codes = np.sum(
            (corner_values > 0.0).astype(np.uint8)
            * (1 << np.arange(4, dtype=np.uint8)),
            axis=1,
        )
        for case_code in range(1, 15):
            selected = np.flatnonzero(case_codes == case_code)
            if selected.size == 0:
                continue
            inside = [
                index for index in range(4) if case_code & (1 << index)
            ]
            outside = [index for index in range(4) if index not in inside]
            ids = corner_ids[selected]
            values = corner_values[selected]
            crossings = []
            for first, second in _TETRA_EDGES:
                if (first in inside) == (second in inside):
                    continue
                first_value = values[:, first]
                second_value = values[:, second]
                fraction = first_value / (first_value - second_value)
                first_points = grid_points[ids[:, first]]
                second_points = grid_points[ids[:, second]]
                crossings.append(first_points + fraction[:, None] * (second_points - first_points))
            if len(crossings) not in (3, 4):
                raise RuntimeError("marching tetrahedra produced an invalid crossing polygon")
            polygon = np.stack(crossings, axis=1)

            inside_centers = np.mean(grid_points[ids[:, inside]], axis=1)
            outside_centers = np.mean(grid_points[ids[:, outside]], axis=1)
            normal = outside_centers - inside_centers
            normal_norm = np.linalg.norm(normal, axis=1)
            normal = normal / np.maximum(normal_norm[:, None], 1e-15)
            reference_axis = np.eye(3)[np.argmin(np.abs(normal), axis=1)]
            axis_u = np.cross(normal, reference_axis)
            axis_u /= np.maximum(np.linalg.norm(axis_u, axis=1)[:, None], 1e-15)
            axis_v = np.cross(normal, axis_u)
            relative = polygon - np.mean(polygon, axis=1, keepdims=True)
            angles = np.arctan2(
                np.einsum("nki,ni->nk", relative, axis_v),
                np.einsum("nki,ni->nk", relative, axis_u),
            )
            order = np.argsort(angles, axis=1)
            polygon = np.take_along_axis(polygon, order[:, :, None], axis=1)

            first_triangle = polygon[:, [0, 1, 2]]
            all_vertices.append(first_triangle.reshape(-1, 3))
            all_triangles.append(
                np.arange(vertex_offset, vertex_offset + 3 * len(selected)).reshape(-1, 3)
            )
            vertex_offset += 3 * len(selected)
            if len(crossings) == 4:
                second_triangle = polygon[:, [0, 2, 3]]
                all_vertices.append(second_triangle.reshape(-1, 3))
                all_triangles.append(
                    np.arange(vertex_offset, vertex_offset + 3 * len(selected)).reshape(-1, 3)
                )
                vertex_offset += 3 * len(selected)

    if not all_vertices:
        return np.empty((0, 3)), np.empty((0, 3), dtype=np.int32)
    vertices = np.concatenate(all_vertices)
    triangles = np.concatenate(all_triangles)
    rounded = np.round(vertices, decimals=10)
    _, unique_indices, inverse = np.unique(
        rounded, axis=0, return_index=True, return_inverse=True
    )
    unique_vertices = vertices[unique_indices]
    unique_triangles = inverse[triangles].astype(np.int32)
    nondegenerate = (
        (unique_triangles[:, 0] != unique_triangles[:, 1])
        & (unique_triangles[:, 1] != unique_triangles[:, 2])
        & (unique_triangles[:, 0] != unique_triangles[:, 2])
    )
    return unique_vertices, unique_triangles[nondegenerate]


def _rotation_from_z(direction: np.ndarray) -> np.ndarray:
    z_axis = np.array([0.0, 0.0, 1.0])
    vector = np.cross(z_axis, direction)
    cosine = float(np.dot(z_axis, direction))
    sine_squared = float(np.dot(vector, vector))
    if sine_squared <= 1e-12:
        return np.eye(3) if cosine >= 0.0 else np.diag([1.0, -1.0, -1.0])
    skew = np.array(
        [
            [0.0, -vector[2], vector[1]],
            [vector[2], 0.0, -vector[0]],
            [-vector[1], vector[0], 0.0],
        ]
    )
    return np.eye(3) + skew + (skew @ skew) / (1.0 + cosine)


def _create_arrow(o3d, origin: np.ndarray, direction: np.ndarray, length: float):
    arrow = o3d.geometry.TriangleMesh.create_arrow(
        cylinder_radius=0.0012,
        cone_radius=0.003,
        cylinder_height=length * 0.7,
        cone_height=length * 0.3,
    )
    arrow.rotate(_rotation_from_z(direction), center=(0.0, 0.0, 0.0))
    arrow.translate(origin)
    arrow.paint_uniform_color((1.0, 0.45, 0.05))
    return arrow


def _field_volume(
    model: HermiteGPIS,
    axes: tuple[np.ndarray, np.ndarray, np.ndarray],
    *,
    chunk_size: int = 2048,
) -> np.ndarray:
    x_grid, y_grid, z_grid = np.meshgrid(*axes, indexing="ij")
    query_points = np.column_stack((x_grid.ravel(), y_grid.ravel(), z_grid.ravel()))
    field = np.empty(len(query_points), dtype=float)
    for start in range(0, len(query_points), chunk_size):
        end = min(start + chunk_size, len(query_points))
        field[start:end] = model.evaluate_mean_many(query_points[start:end])
    return field.reshape(x_grid.shape)


def _filter_supported_surface(
    vertices: np.ndarray,
    triangles: np.ndarray,
    support_points: np.ndarray,
    support_radius_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    if len(vertices) == 0 or len(triangles) == 0:
        return np.empty((0, 3)), np.empty((0, 3), dtype=np.int32)
    tree = cKDTree(support_points)
    supported_vertices = tree.query(vertices, k=1)[0] < support_radius_m
    face_centers = np.mean(vertices[triangles], axis=1)
    supported_faces = np.all(supported_vertices[triangles], axis=1)
    supported_faces &= tree.query(face_centers, k=1)[0] < support_radius_m
    kept_triangles = triangles[supported_faces]
    if len(kept_triangles) == 0:
        return np.empty((0, 3)), np.empty((0, 3), dtype=np.int32)
    used_vertices, inverse = np.unique(kept_triangles, return_inverse=True)
    return vertices[used_vertices], inverse.reshape(-1, 3).astype(np.int32)


def visualize(
    model_directory: Path,
    cloud_npz: Path,
    *,
    array_name: str = "points_base_link",
    grid_resolution_m: float = 0.015,
    max_cloud_points: int = 8000,
    heatmap_limit: float = 0.015,
    hide_primitives: bool = False,
    solid_mesh: bool = False,
) -> None:
    try:
        import open3d as o3d
    except ImportError as exc:
        raise RuntimeError("visualization requires Open3D") from exc
    try:
        from skimage.measure import marching_cubes
    except ImportError:
        marching_cubes = None

    if grid_resolution_m <= 0.0 or max_cloud_points <= 0 or heatmap_limit <= 0.0:
        raise ValueError("grid resolution, cloud limit and heatmap limit must be positive")
    model = HermiteGPIS.load(model_directory)
    with np.load(cloud_npz, allow_pickle=False) as arrays:
        if array_name not in arrays.files:
            raise ValueError(f"cloud archive has no {array_name!r} array")
        cloud_base = np.asarray(arrays[array_name], dtype=float)
    if (
        cloud_base.ndim != 2
        or cloud_base.shape[1] != 3
        or len(cloud_base) == 0
        or not np.all(np.isfinite(cloud_base))
    ):
        raise ValueError("cloud array must contain finite points with shape (N, 3)")

    transform = model.to_arrays()["base_from_model"]
    model_from_base = np.linalg.inv(transform)
    cloud_model = (
        np.column_stack((cloud_base, np.ones(len(cloud_base)))) @ model_from_base.T
    )[:, :3]
    if len(cloud_model) > max_cloud_points:
        shown_indices = np.linspace(
            0, len(cloud_model) - 1, max_cloud_points, dtype=int
        )
        shown_model = cloud_model[shown_indices]
        shown_base = cloud_base[shown_indices]
    else:
        shown_model = cloud_model
        shown_base = cloud_base

    field_values = np.empty(len(shown_model), dtype=float)
    for start in range(0, len(shown_model), 2048):
        end = min(start + 2048, len(shown_model))
        field_values[start:end] = model.evaluate_mean_many(shown_model[start:end])
    residuals = np.abs(field_values)
    normalized = np.clip(residuals / heatmap_limit, 0.0, 1.0)
    colors = np.column_stack(
        (
            np.clip(2.0 * normalized, 0.0, 1.0),
            np.clip(2.0 * (1.0 - normalized), 0.0, 1.0),
            np.zeros(len(normalized)),
        )
    )
    support_points = np.asarray([item.point_m for item in model.observations])
    cloud_support = cKDTree(support_points).query(shown_model, k=1)[0] < model.support_radius_m
    colors[~cloud_support] = (0.35, 0.35, 0.35)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(shown_base)
    point_cloud.colors = o3d.utility.Vector3dVector(colors)

    lower = np.min(support_points, axis=0) - grid_resolution_m
    upper = np.max(support_points, axis=0) + grid_resolution_m
    axes = tuple(
        np.arange(lower[axis], upper[axis] + 0.5 * grid_resolution_m, grid_resolution_m)
        for axis in range(3)
    )
    grid_count = int(np.prod([len(axis) for axis in axes]))
    print(
        f"[INFO] {len(model.observations)} Hermite observations, "
        f"{len(shown_model)} plotted returns; evaluating {grid_count:,} grid points."
    )
    field_volume = _field_volume(model, axes)
    if marching_cubes is not None:
        vertices, triangles, _, _ = marching_cubes(
            field_volume,
            level=0.0,
            spacing=(grid_resolution_m,) * 3,
        )
        vertices += np.array([axis[0] for axis in axes])
    else:
        vertices, triangles = _marching_tetrahedra(axes, field_volume)
    vertices, triangles = _filter_supported_surface(
        vertices,
        triangles,
        support_points,
        model.support_radius_m,
    )

    geometries = [point_cloud]
    if len(triangles):
        mesh = o3d.geometry.TriangleMesh()
        vertices_base = (
            np.column_stack((vertices, np.ones(len(vertices)))) @ transform.T
        )[:, :3]
        mesh.vertices = o3d.utility.Vector3dVector(vertices_base)
        mesh.triangles = o3d.utility.Vector3iVector(triangles)
        mesh.compute_vertex_normals()
        if solid_mesh:
            mesh.paint_uniform_color((0.15, 0.8, 0.8))
            geometries.append(mesh)
        else:
            wireframe = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
            wireframe.paint_uniform_color((0.1, 0.9, 0.9))
            geometries.append(wireframe)
        print(f"[INFO] Extracted zero level with {len(triangles):,} triangles.")
    else:
        print("[WARN] The sampled model field contains no zero-level surface in its support bounds.")

    if not hide_primitives:
        zero_value_samples = [
            item for item in model.observations
            if item.kind == "value" and abs(item.value) <= 1e-12
        ]
        derivative_samples = [item for item in model.observations if item.kind == "derivative"]
        for observation in zero_value_samples:
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.0025)
            point_base = transform[:3, :3] @ np.asarray(observation.point_m) + transform[:3, 3]
            sphere.translate(point_base)
            sphere.paint_uniform_color((0.2, 0.5, 1.0))
            geometries.append(sphere)
        for observation in derivative_samples:
            point_base = (
                transform[:3, :3] @ np.asarray(observation.point_m) + transform[:3, 3]
            )
            normal_base = transform[:3, :3] @ np.asarray(observation.direction)
            geometries.append(
                _create_arrow(
                    o3d,
                    point_base,
                    normal_base,
                    length=0.02,
                )
            )

    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.12)
    geometries.append(frame)
    visualizer = o3d.visualization.Visualizer()
    if not visualizer.create_window(
        window_name="HGW: superficie cero y nube en base_link",
        width=1280,
        height=800,
    ):
        raise RuntimeError("Open3D could not create a visualization window")
    for geometry in geometries:
        visualizer.add_geometry(geometry)
    options = visualizer.get_render_option()
    options.background_color = np.array((0.06, 0.06, 0.06))
    options.point_size = 3.0
    print(
        f"[STATS] Residuo medio |f|={np.mean(residuals):.6g}; "
        f"p95={np.percentile(residuals, 95):.6g}; "
        f"heatmap_limit={heatmap_limit:.6g} (unidades de campo, no distancia)."
    )
    print(
        "Leyenda: cian = superficie f(x)=0; verde = residuo bajo; "
        "amarillo/rojo = residuo alto; azul = muestras de superficie; "
        "naranja = normales estimadas."
    )
    visualizer.run()
    visualizer.destroy_window()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        type=Path,
        default=Path(
            "artifacts/models/opponent_gpis/static_1m_robot_hgw_20261002/"
            "surface_points_100"
        ),
    )
    parser.add_argument(
        "--input-npz",
        type=Path,
        default=Path("data/static_1m_robot.npz"),
    )
    parser.add_argument("--array", default="points_base_link")
    parser.add_argument("--grid-res", type=float, default=0.015, help="Grid spacing in metres.")
    parser.add_argument("--max-cloud-points", type=int, default=8000)
    parser.add_argument(
        "--heatmap-limit",
        type=float,
        default=0.015,
        help="Residual scale in implicit-field units; it is not a distance tolerance.",
    )
    parser.add_argument("--solid-mesh", action="store_true")
    parser.add_argument("--hide-primitives", action="store_true")
    args = parser.parse_args()
    visualize(
        args.model,
        args.input_npz,
        array_name=args.array,
        grid_resolution_m=args.grid_res,
        max_cloud_points=args.max_cloud_points,
        heatmap_limit=args.heatmap_limit,
        hide_primitives=args.hide_primitives,
        solid_mesh=args.solid_mesh,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
