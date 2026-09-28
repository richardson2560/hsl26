"""Validated orthogonal maze fixtures for P6 SIL software tests only."""

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

from hsl_core.topology import EdgeState, NodeKind, TopologyEdge, TopologyGraph, TopologyNode

from .common import Segment, WorldGeometry


_SPLITS = frozenset({"training", "validation", "held_out"})
@dataclass(frozen=True, slots=True)
class MazeFixture:
    scenario_id: str
    split: str
    map_bank_id: str
    opponent_bank_id: str
    seed_bank_id: str
    seed: int
    rows: tuple[str, ...]
    cell_size_m: float
    guardian_start_rc: tuple[int, int]
    explorer_start_rc: tuple[int, int]
    synthetic_goal_rc: tuple[int, int]
    geometry: WorldGeometry
    topology: TopologyGraph

    def cell_pose(self, cell_rc: tuple[int, int]) -> tuple[float, float]:
        row, col = cell_rc
        return (
            (col + 0.5) * self.cell_size_m,
            (len(self.rows) - row - 0.5) * self.cell_size_m,
        )

    @property
    def policy_goal_authorized(self) -> bool:
        return False


@dataclass(frozen=True, slots=True)
class MazeBank:
    bank_id: str
    fixtures: tuple[MazeFixture, ...]
    held_out_policy: str

    def for_split(self, split: str) -> tuple[MazeFixture, ...]:
        if split not in _SPLITS:
            raise ValueError(f"unsupported scenario split: {split}")
        return tuple(fixture for fixture in self.fixtures if fixture.split == split)

    def training_fixtures(self) -> tuple[MazeFixture, ...]:
        return self.for_split("training")

    def validation_fixtures(self) -> tuple[MazeFixture, ...]:
        return self.for_split("validation")

    def held_out_fixtures(self) -> tuple[MazeFixture, ...]:
        return self.for_split("held_out")


def load_maze_bank(path: str | Path) -> MazeBank:
    with Path(path).open("r", encoding="utf-8") as stream:
        raw: Any = json.load(stream)
    if not isinstance(raw, dict):
        raise ValueError("maze bank must be a JSON object")
    if raw.get("schema") != "hsl26.phase6.synthetic-maze-bank.v1":
        raise ValueError("unsupported maze bank schema")
    if raw.get("purpose") != "software_fixture_only":
        raise ValueError("maze bank must be explicitly marked as software fixtures")
    if raw.get("physical_authority") is not False or raw.get("competition_map") is not False:
        raise ValueError("synthetic maze fixtures cannot claim physical or competition authority")
    bank_id = _required_text(raw.get("bank_id"), "bank_id")
    cell_size = _finite_positive(raw.get("cell_size_m"), "cell_size_m")
    held_out_policy = _required_text(raw.get("held_out_policy"), "held_out_policy")
    entries = raw.get("scenarios")
    if not isinstance(entries, list) or not entries:
        raise ValueError("maze bank requires a non-empty scenarios list")
    fixtures = tuple(_parse_fixture(entry, cell_size) for entry in entries)
    if len({fixture.scenario_id for fixture in fixtures}) != len(fixtures):
        raise ValueError("maze scenario IDs must be unique")
    for split in _SPLITS:
        if not any(fixture.split == split for fixture in fixtures):
            raise ValueError(f"maze bank lacks {split} fixtures")
    _validate_independent_banks(fixtures)
    return MazeBank(bank_id, fixtures, held_out_policy)


def _parse_fixture(raw: Any, cell_size_m: float) -> MazeFixture:
    if not isinstance(raw, dict):
        raise ValueError("each maze scenario must be a JSON object")
    scenario_id = _required_text(raw.get("scenario_id"), "scenario_id")
    split = raw.get("split")
    if not isinstance(split, str) or split not in _SPLITS:
        raise ValueError("scenario split must be training, validation, or held_out")
    bank_ids = tuple(
        _required_text(raw.get(field), field)
        for field in ("map_bank_id", "opponent_bank_id", "seed_bank_id")
    )
    seed = raw.get("seed")
    if not isinstance(seed, int) or isinstance(seed, bool) or seed < 0:
        raise ValueError("scenario seed must be a non-negative integer")
    rows_raw = raw.get("grid")
    if (
        not isinstance(rows_raw, list)
        or not rows_raw
        or any(not isinstance(row, str) or not row for row in rows_raw)
    ):
        raise ValueError("maze grid must be a non-empty array of row strings")
    rows = tuple(rows_raw)
    width = len(rows[0])
    if any(len(row) != width or set(row) - {".", "#"} for row in rows):
        raise ValueError("maze grid must be rectangular and contain only '.' and '#'")
    starts = {
        "guardian": _cell(raw.get("guardian_start_rc"), "guardian_start_rc"),
        "explorer": _cell(raw.get("explorer_start_rc"), "explorer_start_rc"),
    }
    goal = _cell(raw.get("synthetic_goal_rc"), "synthetic_goal_rc")
    all_cells = (*starts.values(), goal)
    if any(not _is_open(rows, cell) for cell in all_cells):
        raise ValueError("all starts and synthetic goal must be open grid cells")
    if starts["guardian"] == starts["explorer"]:
        raise ValueError("role starts must be distinct cells")
    if not isinstance(raw.get("synthetic_goal_rc"), list):
        raise ValueError("synthetic_goal_rc must be a row/column array")
    geometry = _build_geometry(rows, cell_size_m)
    topology = _build_topology(rows, cell_size_m, f"maze:{scenario_id}:localization")
    _require_connected(topology, starts["guardian"], starts["explorer"], goal, width)
    return MazeFixture(
        scenario_id,
        split,
        bank_ids[0],
        bank_ids[1],
        bank_ids[2],
        seed,
        rows,
        cell_size_m,
        starts["guardian"],
        starts["explorer"],
        goal,
        geometry,
        topology,
    )


def _build_geometry(rows: tuple[str, ...], cell_size_m: float) -> WorldGeometry:
    height, width = len(rows), len(rows[0])
    walls: dict[tuple[tuple[float, float], tuple[float, float]], Segment] = {}
    for row in range(height):
        for col in range(width):
            if rows[row][col] != ".":
                continue
            x0, x1 = col * cell_size_m, (col + 1) * cell_size_m
            y0 = (height - row - 1) * cell_size_m
            y1 = y0 + cell_size_m
            neighbors = (
                (row - 1, col, ((x0, y1), (x1, y1))),
                (row + 1, col, ((x0, y0), (x1, y0))),
                (row, col - 1, ((x0, y0), (x0, y1))),
                (row, col + 1, ((x1, y0), (x1, y1))),
            )
            for next_row, next_col, endpoints in neighbors:
                if not _is_open(rows, (next_row, next_col)):
                    key = tuple(sorted(endpoints))
                    walls[key] = Segment(*endpoints)
    return WorldGeometry(tuple(walls[key] for key in sorted(walls)))


def _build_topology(
    rows: tuple[str, ...], cell_size_m: float, localization_epoch: str
) -> TopologyGraph:
    height, width = len(rows), len(rows[0])
    open_cells = [
        (row, col)
        for row in range(height)
        for col in range(width)
        if rows[row][col] == "."
    ]
    node_ids = {
        cell: cell[0] * width + cell[1]
        for cell in open_cells
    }
    nodes = tuple(
        TopologyNode(
            node_id=node_ids[(row, col)],
            x_m=(col + 0.5) * cell_size_m,
            y_m=(height - row - 0.5) * cell_size_m,
            kind=NodeKind.PORTAL,
            clearance_radius_m=cell_size_m / 2.0,
        )
        for row, col in open_cells
    )
    edges: list[TopologyEdge] = []
    for row, col in open_cells:
        for neighbor in ((row, col + 1), (row + 1, col)):
            if not _is_open(rows, neighbor):
                continue
            start_id, end_id = node_ids[(row, col)], node_ids[neighbor]
            start = next(node for node in nodes if node.node_id == start_id)
            end = next(node for node in nodes if node.node_id == end_id)
            edges.append(
                TopologyEdge(
                    edge_id=len(edges),
                    from_node=start_id,
                    to_node=end_id,
                    polyline_xy_m=((start.x_m, start.y_m), (end.x_m, end.y_m)),
                    min_clearance_radius_m=cell_size_m / 2.0,
                    min_width_m=cell_size_m,
                    width_valid=True,
                    state=EdgeState.OPEN,
                )
            )
    return TopologyGraph(
        map_version=1,
        topology_version=1,
        localization_epoch=localization_epoch,
        nodes=nodes,
        edges=tuple(edges),
    )


def _require_connected(
    topology: TopologyGraph,
    guardian_start: tuple[int, int],
    explorer_start: tuple[int, int],
    goal: tuple[int, int],
    width: int,
) -> None:
    node_ids = {
        row * width + col for row, col in (guardian_start, explorer_start, goal)
    }
    start = next(iter(node_ids))
    visited = {start}
    stack = [start]
    adjacency = topology.adjacency()
    while stack:
        current = stack.pop()
        for edge in adjacency[current]:
            if edge.state is EdgeState.OPEN and edge.to_node not in visited:
                visited.add(edge.to_node)
                stack.append(edge.to_node)
    if not node_ids <= visited:
        raise ValueError("maze role starts and goal must share an open connected component")


def _validate_independent_banks(fixtures: tuple[MazeFixture, ...]) -> None:
    for field in ("map_bank_id", "opponent_bank_id", "seed_bank_id"):
        bank_splits: dict[str, str] = {}
        for fixture in fixtures:
            bank_id = getattr(fixture, field)
            previous = bank_splits.setdefault(bank_id, fixture.split)
            if previous != fixture.split:
                raise ValueError(f"{field} leaks across dataset splits")
    map_ids = [fixture.map_bank_id for fixture in fixtures]
    if len(map_ids) != len(set(map_ids)):
        raise ValueError("every maze fixture must use an independent map bank")


def _is_open(rows: tuple[str, ...], cell: tuple[int, int]) -> bool:
    row, col = cell
    return (
        0 <= row < len(rows)
        and 0 <= col < len(rows[0])
        and rows[row][col] == "."
    )


def _cell(value: Any, field: str) -> tuple[int, int]:
    if (
        not isinstance(value, list)
        or len(value) != 2
        or any(not isinstance(item, int) or isinstance(item, bool) for item in value)
    ):
        raise ValueError(f"{field} must contain integer row and column")
    return value[0], value[1]


def _required_text(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field} must be a non-empty string")
    return value


def _finite_positive(value: Any, field: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or value <= 0.0
    ):
        raise ValueError(f"{field} must be finite and positive")
    return float(value)
