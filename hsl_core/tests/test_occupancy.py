# hsl_core/tests/test_occupancy.py
"""Adversarial SIL tests for the Phase-3 layered occupancy mapper."""

import math

import numpy as np
import pytest

from hsl_core.mapping import OCCUPIED, OBSERVED_FREE, UNKNOWN
from hsl_core.occupancy import (
    GridSnapshot,
    OccupancyMapConfig,
    OccupancyMapper,
    RayEvidence,
    RayKind,
)


def _mapper() -> OccupancyMapper:
    return OccupancyMapper(
        OccupancyMapConfig(
            origin_xy_m=(-1.0, -1.0),
            resolution_m=0.1,
            width=30,
            height=20,
            max_batch_abs_update=1.5,
            dynamic_decay_tau_s=1.0,
        )
    )


def _ray(x: float, stamp: float, *, kind=RayKind.STATIC, hit=True) -> RayEvidence:
    return RayEvidence((0.0, 0.0), (x, 0.0), stamp, kind, hit)


def test_initial_map_is_unknown_and_version_zero():
    snapshot = _mapper().snapshot
    assert snapshot.map_version == 0
    assert np.all(snapshot.structural == UNKNOWN)
    assert np.all(snapshot.collision == UNKNOWN)
    assert np.all(snapshot.semantic == UNKNOWN)
    assert np.all(snapshot.observed_free == UNKNOWN)


def test_static_ray_separates_observed_free_from_terminal_occupied_cell():
    snapshot = _mapper().update((_ray(1.0, 1.0),))
    assert snapshot.structural[10, 20] == OCCUPIED
    assert snapshot.collision[10, 20] == OCCUPIED
    assert snapshot.observed_free[10, 19] == OBSERVED_FREE


def test_hit_does_not_clear_cells_behind_terminal_return():
    snapshot = _mapper().update((_ray(0.5, 1.0),))
    assert snapshot.collision[10, 15] == OCCUPIED
    assert snapshot.collision[10, 20] == UNKNOWN


def test_dynamic_hit_is_collision_occupied_but_not_structural():
    snapshot = _mapper().update((_ray(0.5, 1.0, kind=RayKind.DYNAMIC),))
    assert snapshot.semantic[10, 15] == OCCUPIED
    assert snapshot.collision[10, 15] == OCCUPIED
    assert snapshot.structural[10, 15] != OCCUPIED


def test_dynamic_decay_removes_transient_occupancy_without_certifying_free():
    mapper = _mapper()
    mapper.update((_ray(0.5, 1.0, kind=RayKind.DYNAMIC),))
    snapshot = mapper.decay(3.0)
    assert snapshot.semantic[10, 15] == UNKNOWN
    assert snapshot.collision[10, 15] == UNKNOWN
    assert snapshot.observed_free[10, 15] == UNKNOWN


def test_dynamic_decay_never_removes_persistent_static_wall():
    mapper = _mapper()
    mapper.update((_ray(0.5, 1.0),))
    mapper.update((_ray(0.5, 2.0, kind=RayKind.DYNAMIC),))
    snapshot = mapper.decay(5.0)
    assert snapshot.structural[10, 15] == OCCUPIED
    assert snapshot.collision[10, 15] == OCCUPIED


def test_correlated_scan_evidence_is_capped_per_cell():
    mapper = _mapper()
    snapshot = mapper.update(tuple(_ray(0.5, 1.0) for _ in range(100)))
    assert snapshot.structural[10, 15] == OCCUPIED
    assert mapper._static_log_odds[10, 15] <= 1.5


def test_free_evidence_can_reduce_static_belief_but_not_dynamic_semantics():
    mapper = _mapper()
    mapper.update((_ray(0.5, 1.0),))
    before = mapper.snapshot.structural[10, 15]
    mapper.update((_ray(0.5, 2.0, hit=False),))
    after = mapper.snapshot.structural[10, 15]
    assert before == OCCUPIED
    assert after != OCCUPIED


def test_out_of_grid_endpoint_keeps_in_grid_ray_free_and_no_terminal_hit():
    snapshot = _mapper().update((_ray(5.0, 1.0),))
    assert np.count_nonzero(snapshot.observed_free == OBSERVED_FREE) > 0
    assert np.count_nonzero(snapshot.collision == OCCUPIED) == 0


def test_map_versions_are_atomic_and_empty_update_is_noop():
    mapper = _mapper()
    first = mapper.update((_ray(0.5, 1.0),))
    empty = mapper.update(())
    assert empty.map_version == first.map_version
    with pytest.raises(ValueError, match="timestamps"):
        mapper.update((_ray(0.4, 0.5),))
    assert mapper.snapshot.map_version == first.map_version
    assert np.array_equal(mapper.snapshot.collision, first.collision)


def test_snapshot_is_immutable_from_input_and_output_arrays():
    mapper = _mapper()
    snapshot = mapper.update((_ray(0.5, 1.0),))
    copied = snapshot.collision.copy()
    copied[10, 15] = UNKNOWN
    assert snapshot.collision[10, 15] == OCCUPIED
    with pytest.raises(ValueError, match="shape"):
        GridSnapshot(0, 0.0, np.zeros((2, 2)), np.zeros((1, 2)), np.zeros((2, 2)), np.zeros((2, 2)), np.zeros((2, 2)))
    with pytest.raises(ValueError, match="invalid state"):
        GridSnapshot(
            0,
            0.0,
            np.full((2, 2), 1.5),
            np.zeros((2, 2)),
            np.zeros((2, 2)),
            np.zeros((2, 2)),
            np.zeros((2, 2)),
        )


@pytest.mark.parametrize(
    "kwargs, message",
    [
        ({"p_free": 0.5}, "straddle"),
        ({"p_occupied": 0.5}, "straddle"),
        ({"min_probability": 0.6}, "thresholds"),
        ({"resolution_m": 0.0}, "resolution"),
        ({"max_batch_abs_update": 0.0}, "max_batch_abs_update"),
        ({"width": True}, "width"),
    ],
)
def test_configuration_rejects_non_conservative_parameters(kwargs, message):
    with pytest.raises(ValueError, match=message):
        OccupancyMapConfig(**kwargs)


@pytest.mark.parametrize(
    "factory, message",
    [
        (lambda: RayEvidence((0.0, 0.0), (1.0, 0.0), math.nan), "stamp"),
        (lambda: RayEvidence((0.0, 0.0), (1.0, 0.0), 1.0, 99), "99"),
    ],
)
def test_ray_contract_rejects_invalid_values(factory, message):
    with pytest.raises(ValueError, match=message):
        factory()
