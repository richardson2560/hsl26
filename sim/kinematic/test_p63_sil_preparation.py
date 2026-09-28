"""P6.3 SIL fixture-bank and readiness boundary tests."""

from copy import deepcopy
import json
from pathlib import Path

import pytest

from hsl_core.planning import astar
from sim.kinematic.maze_bank import MazeFixture, load_maze_bank


_BANK_PATH = (
    Path(__file__).parent / "scenarios" / "phase6_maze_bank.json"
)


@pytest.fixture
def maze_bank():
    return load_maze_bank(_BANK_PATH)


def test_maze_bank_has_independent_training_validation_and_reserved_heldout():
    bank = load_maze_bank(_BANK_PATH)
    assert len(bank.training_fixtures()) == 2
    assert len(bank.validation_fixtures()) == 1
    assert len(bank.held_out_fixtures()) == 1
    assert all(item.split == "training" for item in bank.training_fixtures())
    assert all(item.split == "validation" for item in bank.validation_fixtures())
    held_out = bank.held_out_fixtures()[0]
    assert held_out.scenario_id == "maze_grid_4x4_held_out"
    assert held_out not in bank.training_fixtures()
    assert held_out not in bank.validation_fixtures()
    assert held_out.policy_goal_authorized is False

    for field in ("map_bank_id", "opponent_bank_id", "seed_bank_id"):
        split_by_bank = {}
        for fixture in bank.fixtures:
            prior = split_by_bank.setdefault(getattr(fixture, field), fixture.split)
            assert prior == fixture.split
    assert len({fixture.map_bank_id for fixture in bank.fixtures}) == len(bank.fixtures)


@pytest.mark.parametrize("fixture_index", range(4))
def test_maze_conversion_matches_open_cells_and_orthogonal_topology(maze_bank, fixture_index):
    fixture = maze_bank.fixtures[fixture_index]
    assert isinstance(fixture, MazeFixture)
    open_cells = {
        (row, col)
        for row, line in enumerate(fixture.rows)
        for col, value in enumerate(line)
        if value == "."
    }
    assert len(fixture.topology.nodes) == len(open_cells)
    node_by_rc = {
        (node.node_id // len(fixture.rows[0]), node.node_id % len(fixture.rows[0])): node
        for node in fixture.topology.nodes
    }
    assert set(node_by_rc) == open_cells
    for edge in fixture.topology.edges:
        assert edge.state.name == "OPEN"
        assert edge.width_valid
        assert edge.min_width_m == pytest.approx(fixture.cell_size_m)
        a = next(node for node in fixture.topology.nodes if node.node_id == edge.from_node)
        b = next(node for node in fixture.topology.nodes if node.node_id == edge.to_node)
        delta = (abs(a.x_m - b.x_m), abs(a.y_m - b.y_m))
        assert sorted(delta) == pytest.approx([0.0, fixture.cell_size_m])
        assert len(edge.polyline_xy_m) == 2

    guardian_node = (
        fixture.guardian_start_rc[0] * len(fixture.rows[0])
        + fixture.guardian_start_rc[1]
    )
    goal_node = (
        fixture.synthetic_goal_rc[0] * len(fixture.rows[0])
        + fixture.synthetic_goal_rc[1]
    )
    path = astar(fixture.topology, guardian_node, goal_node)
    assert path.map_version == fixture.topology.map_version
    assert path.topology_version == fixture.topology.topology_version
    assert path.cost >= 0.0


def test_wall_geometry_is_finite_nonempty_and_cell_scale_consistent(maze_bank):
    for fixture in maze_bank.fixtures:
        assert fixture.geometry.static_segments
        assert all(
            segment.start_xy != segment.end_xy
            and (
                segment.start_xy[0] == pytest.approx(segment.end_xy[0])
                or segment.start_xy[1] == pytest.approx(segment.end_xy[1])
            )
            for segment in fixture.geometry.static_segments
        )
        assert all(
            (abs(segment.start_xy[0] - segment.end_xy[0]) == pytest.approx(fixture.cell_size_m))
            or (abs(segment.start_xy[1] - segment.end_xy[1]) == pytest.approx(fixture.cell_size_m))
            for segment in fixture.geometry.static_segments
        )


@pytest.mark.parametrize(
    "mutation, message",
    [
        (lambda raw: raw.update({"physical_authority": True}), "cannot claim"),
        (lambda raw: raw.update({"competition_map": True}), "cannot claim"),
        (lambda raw: raw["scenarios"][0].update({"split": "held_out"}), "leaks across"),
        (lambda raw: raw["scenarios"][0].update({"guardian_start_rc": [1, 1]}), "open grid cells"),
        (lambda raw: raw["scenarios"][0].update({"seed": True}), "seed"),
    ],
)
def test_maze_bank_loader_rejects_claims_leakage_and_invalid_inputs(
    tmp_path, mutation, message
):
    raw = json.loads(_BANK_PATH.read_text(encoding="utf-8"))
    raw = deepcopy(raw)
    mutation(raw)
    file_path = tmp_path / "bad-bank.json"
    file_path.write_text(json.dumps(raw), encoding="utf-8")
    with pytest.raises(ValueError, match=message):
        load_maze_bank(file_path)


def test_maze_loader_rejects_disconnected_scenario_and_nonrectangular_grid(tmp_path):
    raw = json.loads(_BANK_PATH.read_text(encoding="utf-8"))
    raw["scenarios"][0]["grid"] = [".....", ".###.", ".#.#.", ".###.", "##.##"]
    raw["scenarios"][0]["guardian_start_rc"] = [0, 0]
    raw["scenarios"][0]["explorer_start_rc"] = [4, 2]
    raw["scenarios"][0]["synthetic_goal_rc"] = [4, 2]
    raw["scenarios"][0]["map_bank_id"] = "map-train-disconnected"
    path = tmp_path / "disconnected.json"
    path.write_text(json.dumps(raw), encoding="utf-8")
    with pytest.raises(ValueError, match="connected component"):
        load_maze_bank(path)

    raw = json.loads(_BANK_PATH.read_text(encoding="utf-8"))
    raw["scenarios"][0]["grid"][0] = "...."
    path.write_text(json.dumps(raw), encoding="utf-8")
    with pytest.raises(ValueError, match="rectangular"):
        load_maze_bank(path)
