# hsl_core/tests/test_p43_tracking.py
"""Adversarial P4.3 tests for CV tracking, association and lifecycle."""

import math

import numpy as np
import pytest

from hsl_core.perception.ekf_opponent import (
    FilterConfig,
    OpponentDetection,
    OpponentFilter,
)
from hsl_core.types import TrackState


def _detection(x, y, stamp, source="synthetic", epoch="epoch-0", map_version=1):
    return OpponentDetection(
        (x, y), (0.01, 0.0, 0.0, 0.01), stamp, source, map_version, epoch
    )


def _filter(**kwargs):
    return OpponentFilter(FilterConfig(**kwargs))


def test_initialization_requires_confirmation_and_updates_state():
    tracker = _filter(confirmation_count=2)
    first = tracker.initialize(_detection(1.0, 2.0, 0.0))
    assert first.state == TrackState.SEARCHING
    second = tracker.update([_detection(1.05, 2.0, 0.1)], 0.1)
    assert second.state == TrackState.TRACKED
    assert second.association.accepted


def test_prediction_uses_cv_model_and_separates_timestamps():
    tracker = _filter(confirmation_count=1)
    tracker.initialize(_detection(0.0, 0.0, 0.0))
    tracker.update([_detection(1.0, 0.0, 1.0)], 1.0)
    output = tracker.predict(1.5)
    assert output.prediction_stamp_s == pytest.approx(1.5)
    assert output.publication_stamp_s == pytest.approx(1.5)
    assert output.last_measurement_s == pytest.approx(1.0)
    assert output.state_vector[0] > 1.0


def test_gated_outlier_does_not_refresh_measurement_stamp_or_state():
    tracker = _filter(confirmation_count=1)
    tracker.initialize(_detection(0.0, 0.0, 0.0))
    before = tracker.state
    output = tracker.update([_detection(100.0, 100.0, 0.1)], 0.1)
    assert output.association.reason == "GATED_OUT"
    assert output.last_measurement_s == pytest.approx(0.0)
    assert np.allclose(tracker.state, tracker.state)
    assert not np.allclose(tracker.state[:2], (100.0, 100.0))


def test_ambiguous_candidates_are_rejected_not_fused():
    tracker = _filter(confirmation_count=1, ambiguity_margin=1.0)
    tracker.initialize(_detection(0.0, 0.0, 0.0))
    output = tracker.update(
        [_detection(0.1, 0.0, 0.1, "a"), _detection(0.1001, 0.0, 0.1, "b")], 0.1
    )
    assert output.association.ambiguous
    assert output.association.accepted is False
    assert output.last_measurement_s == pytest.approx(0.0)


def test_stale_versions_are_not_associated():
    tracker = _filter(confirmation_count=1)
    tracker.initialize(_detection(0.0, 0.0, 0.0))
    output = tracker.update([_detection(0.01, 0.0, 0.1, map_version=2)], 0.1)
    assert output.association.reason == "GATED_OUT"
    assert output.last_measurement_s == pytest.approx(0.0)


def test_joseph_update_remains_symmetric_psd():
    tracker = _filter(confirmation_count=1)
    tracker.initialize(_detection(0.0, 0.0, 0.0))
    tracker.update([_detection(0.2, -0.1, 0.1)], 0.1)
    covariance = tracker.covariance
    assert np.allclose(covariance, covariance.T, atol=1e-12)
    assert np.min(np.linalg.eigvalsh(covariance)) >= -1e-10


def test_lifecycle_transitions_and_lost_reset():
    tracker = _filter(
        confirmation_count=1, coasting_timeout_s=0.5, belief_timeout_s=1.0, lost_timeout_s=2.0
    )
    tracker.initialize(_detection(0.0, 0.0, 0.0))
    assert tracker.predict(0.2).state == TrackState.COASTING
    assert tracker.update([], 0.7).state == TrackState.OCCLUDED_BELIEF
    assert tracker.update([], 1.2).state == TrackState.LOST
    assert tracker.update([], 2.1).state == TrackState.SEARCHING


def test_invalid_detection_and_time_order_are_rejected():
    with pytest.raises(ValueError):
        _detection(0.0, 0.0, 0.0, map_version=-1)
    with pytest.raises(ValueError):
        OpponentDetection((0.0, 0.0), (1.0, 0.0, 0.0, -1.0), 0.0, "s", 1, "e")
    tracker = _filter(confirmation_count=1)
    tracker.initialize(_detection(0.0, 0.0, 1.0))
    with pytest.raises(ValueError):
        tracker.predict(0.9)
    with pytest.raises(ValueError):
        tracker.update([], 2.5)


def test_config_rejects_invalid_gate_and_timeout_order():
    with pytest.raises(ValueError):
        FilterConfig(innovation_gate=0.0)
    with pytest.raises(ValueError):
        FilterConfig(coasting_timeout_s=2.0, belief_timeout_s=1.0)
