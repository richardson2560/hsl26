# hsl_core/hsl_core/perception/runtime_codec.py
"""Strict transport-neutral codecs for versioned P4 opponent outputs."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math
from typing import Mapping, Sequence

import numpy as np

from ..types import TrackState
from .ekf_opponent import FilterOutput
from .topological_belief import BeliefCell, TopologicalBelief


class ContractValidity(IntEnum):
    INVALID = 0
    VALID = 1
    DEGRADED = 2


class SensorProfile(IntEnum):
    REAL_CLOUD = 0
    SIM_CLOUD = 1
    SYNTHETIC_DETECTION = 2
    REPLAY_CLOUD = 3


@dataclass(frozen=True)
class TrackEnvelope:
    track_id: str
    state: TrackState
    x: float
    y: float
    vx: float
    vy: float
    covariance: tuple[float, ...]
    position_valid: bool
    yaw: float
    yaw_variance: float
    yaw_valid: bool
    last_measurement_stamp: float
    sensor_profile: SensorProfile
    model_id: str
    association_status: str
    source_id: str
    source_session: str
    seq: int
    stage_id: str
    clock_epoch: str
    localization_epoch: str
    frame_id: str
    observation_stamp: float
    state_stamp: float
    publication_stamp: float
    valid_until: float
    map_version: int
    topology_version: int
    validity: ContractValidity

    def __post_init__(self) -> None:
        for field in ("track_id", "source_id", "source_session", "stage_id",
                      "clock_epoch", "localization_epoch", "frame_id",
                      "model_id", "association_status"):
            if not getattr(self, field):
                raise ValueError(f"{field} must not be empty")
        for field in (
            "x", "y", "vx", "vy", "yaw", "yaw_variance",
            "last_measurement_stamp", "observation_stamp", "state_stamp",
            "publication_stamp", "valid_until",
        ):
            if not math.isfinite(float(getattr(self, field))):
                raise ValueError(f"{field} must be finite")
        for field in (
            "last_measurement_stamp", "observation_stamp", "state_stamp",
            "publication_stamp", "valid_until",
        ):
            if getattr(self, field) < 0.0:
                raise ValueError(f"{field} must be non-negative")
        if not isinstance(self.position_valid, bool) or not isinstance(self.yaw_valid, bool):
            raise ValueError("position_valid and yaw_valid must be booleans")
        for field in ("position_valid", "yaw_valid"):
            if not isinstance(getattr(self, field), bool):
                raise ValueError(f"{field} must be boolean")
        if self.yaw_variance < 0.0:
            raise ValueError("yaw_variance must be non-negative")
        if not (
            self.observation_stamp <= self.state_stamp <= self.publication_stamp <= self.valid_until
        ):
            raise ValueError("track observation/state/publication/expiry stamps are inconsistent")
        if self.last_measurement_stamp > self.observation_stamp:
            raise ValueError("last_measurement_stamp must not follow observation_stamp")
        for field in ("seq", "map_version", "topology_version"):
            value = getattr(self, field)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{field} must be a non-negative integer")
        if any(not isinstance(getattr(self, name), str) for name in (
            "track_id", "source_id", "source_session", "stage_id", "clock_epoch",
            "localization_epoch", "frame_id", "model_id", "association_status",
        )):
            raise ValueError("text contract fields must be strings")
        covariance = np.asarray(self.covariance, dtype=float)
        if covariance.shape != (16,) or not np.all(np.isfinite(covariance)):
            raise ValueError("track covariance must contain 16 finite values")
        covariance = covariance.reshape(4, 4)
        if not np.allclose(covariance, covariance.T, atol=1e-9):
            raise ValueError("track covariance must be symmetric")
        if np.min(np.linalg.eigvalsh(covariance)) < -1e-9:
            raise ValueError("track covariance must be positive semidefinite")
        if not self.yaw_valid and self.yaw != 0.0:
            raise ValueError("invalid yaw must use the canonical zero payload")
        state = _strict_enum(TrackState, self.state, "state")
        profile = _strict_enum(SensorProfile, self.sensor_profile, "sensor_profile")
        validity = _strict_enum(ContractValidity, self.validity, "validity")
        if state in (TrackState.SEARCHING, TrackState.LOST) and self.position_valid:
            raise ValueError("SEARCHING and LOST tracks cannot publish a valid position")
        if validity == ContractValidity.INVALID and (self.position_valid or self.yaw_valid):
            raise ValueError("INVALID track cannot claim valid position or yaw")
        object.__setattr__(self, "state", state)
        object.__setattr__(self, "sensor_profile", profile)
        object.__setattr__(self, "validity", validity)
        object.__setattr__(self, "covariance", tuple(float(v) for v in covariance.reshape(-1)))


@dataclass(frozen=True)
class BeliefEnvelope:
    track_id: str
    cells: tuple[BeliefCell, ...]
    unknown_mass: float
    last_measurement_stamp: float
    transition_model_id: str
    visibility_model_id: str
    sensor_profile: SensorProfile
    source_id: str
    source_session: str
    seq: int
    stage_id: str
    clock_epoch: str
    localization_epoch: str
    frame_id: str
    observation_stamp: float
    state_stamp: float
    publication_stamp: float
    valid_until: float
    map_version: int
    topology_version: int
    validity: ContractValidity

    def __post_init__(self) -> None:
        for field in (
            "track_id", "transition_model_id", "visibility_model_id", "source_id",
            "source_session", "stage_id", "clock_epoch", "localization_epoch", "frame_id",
        ):
            if not getattr(self, field):
                raise ValueError(f"{field} must not be empty")
        for field in (
            "unknown_mass", "last_measurement_stamp", "observation_stamp",
            "state_stamp", "publication_stamp", "valid_until",
        ):
            if not math.isfinite(float(getattr(self, field))):
                raise ValueError(f"{field} must be finite")
        if any(getattr(self, field) < 0.0 for field in (
            "last_measurement_stamp", "observation_stamp", "state_stamp",
            "publication_stamp", "valid_until",
        )):
            raise ValueError("belief timestamps must be non-negative")
        if self.observation_stamp > self.state_stamp or self.state_stamp > self.publication_stamp:
            raise ValueError("belief observation/state/publication stamps are inconsistent")
        if not 0.0 <= self.unknown_mass <= 1.0:
            raise ValueError("unknown_mass must be in [0, 1]")
        total = self.unknown_mass + sum(cell.mass for cell in self.cells)
        if abs(total - 1.0) > 1e-9:
            raise ValueError("belief cell masses plus unknown_mass must sum to one")
        if self.last_measurement_stamp > self.state_stamp:
            raise ValueError("last_measurement_stamp must not follow state_stamp")
        if self.publication_stamp > self.valid_until:
            raise ValueError("valid_until must not precede publication_stamp")
        for field in ("seq", "map_version", "topology_version"):
            value = getattr(self, field)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{field} must be a non-negative integer")
        if any(not isinstance(getattr(self, name), str) for name in (
            "track_id", "transition_model_id", "visibility_model_id", "source_id",
            "source_session", "stage_id", "clock_epoch", "localization_epoch", "frame_id",
        )):
            raise ValueError("text contract fields must be strings")
        cells = tuple(self.cells)
        if any(not isinstance(cell, BeliefCell) for cell in cells):
            raise ValueError("cells must contain BeliefCell values")
        object.__setattr__(self, "cells", cells)
        object.__setattr__(
            self, "sensor_profile", _strict_enum(SensorProfile, self.sensor_profile, "sensor_profile")
        )
        object.__setattr__(
            self, "validity", _strict_enum(ContractValidity, self.validity, "validity")
        )


def track_from_filter_output(
    output: FilterOutput,
    *,
    track_id: str,
    sensor_profile: SensorProfile,
    model_id: str,
    source_id: str,
    source_session: str,
    seq: int,
    stage_id: str,
    clock_epoch: str,
    localization_epoch: str,
    frame_id: str,
    map_version: int,
    topology_version: int,
    publication_stamp: float,
    valid_until: float,
    validity: ContractValidity,
) -> TrackEnvelope:
    """Bind P4.3 state to the revision-2 track wire contract."""
    x, y, vx, vy = output.state_vector
    position_valid = output.state not in (TrackState.SEARCHING, TrackState.LOST)
    return TrackEnvelope(
        track_id=track_id,
        state=output.state,
        x=x,
        y=y,
        vx=vx,
        vy=vy,
        covariance=output.covariance,
        position_valid=position_valid,
        yaw=0.0,
        yaw_variance=0.0,
        yaw_valid=False,
        last_measurement_stamp=output.last_measurement_s,
        sensor_profile=sensor_profile,
        model_id=model_id,
        association_status=output.association.reason,
        source_id=source_id,
        source_session=source_session,
        seq=seq,
        stage_id=stage_id,
        clock_epoch=clock_epoch,
        localization_epoch=localization_epoch,
        frame_id=frame_id,
        observation_stamp=output.last_measurement_s,
        state_stamp=output.prediction_stamp_s,
        publication_stamp=publication_stamp,
        valid_until=valid_until,
        map_version=map_version,
        topology_version=topology_version,
        validity=validity,
    )


def _stamp(seconds: float) -> dict[str, int]:
    if not math.isfinite(seconds) or seconds < 0.0:
        raise ValueError("stamp must be finite and non-negative")
    sec = math.floor(seconds)
    nanosec = int(round((seconds - sec) * 1_000_000_000))
    if nanosec == 1_000_000_000:
        sec += 1
        nanosec = 0
    return {"sec": int(sec), "nanosec": nanosec}


def _seconds(value: object, name: str) -> float:
    if not isinstance(value, Mapping) or set(value) != {"sec", "nanosec"}:
        raise ValueError(f"{name} must contain exactly sec and nanosec")
    sec, nanosec = value["sec"], value["nanosec"]
    if isinstance(sec, bool) or not isinstance(sec, int) or sec < 0:
        raise ValueError(f"{name}.sec must be a non-negative integer")
    if isinstance(nanosec, bool) or not isinstance(nanosec, int) or not 0 <= nanosec < 1_000_000_000:
        raise ValueError(f"{name}.nanosec is out of range")
    return sec + nanosec * 1e-9


def _require_bool(value: object, name: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be boolean")
    return value


def _require_nonnegative_int(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _strict_enum(enum_type, value: object, name: str):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer enum value")
    try:
        return enum_type(value)
    except ValueError as exc:
        raise ValueError(f"{name} has an unknown enum value") from exc


def encode_track(track: TrackEnvelope) -> dict:
    return {
        "meta": {
            "schema_version": 2,
            "source_id": track.source_id,
            "source_session": track.source_session,
            "seq": track.seq,
            "stage_id": track.stage_id,
            "clock_epoch": track.clock_epoch,
            "localization_epoch": track.localization_epoch,
            "frame_id": track.frame_id,
            "observation_stamp": _stamp(track.observation_stamp),
            "state_stamp": _stamp(track.state_stamp),
            "publication_stamp": _stamp(track.publication_stamp),
            "valid_until": _stamp(track.valid_until),
            "map_version": track.map_version,
            "topology_version": track.topology_version,
            "validity": int(track.validity),
        },
        "track_id": track.track_id,
        "state": int(track.state),
        "x": track.x,
        "y": track.y,
        "vx": track.vx,
        "vy": track.vy,
        "covariance": list(track.covariance),
        "position_valid": track.position_valid,
        "yaw": track.yaw,
        "yaw_variance": track.yaw_variance,
        "yaw_valid": track.yaw_valid,
        "last_measurement_stamp": _stamp(track.last_measurement_stamp),
        "sensor_profile": int(track.sensor_profile),
        "model_id": track.model_id,
        "association_status": track.association_status,
    }


def decode_track(message: Mapping[str, object]) -> TrackEnvelope:
    if set(message) != {
        "meta", "track_id", "state", "x", "y", "vx", "vy", "covariance",
        "position_valid", "yaw", "yaw_variance", "yaw_valid",
        "last_measurement_stamp", "sensor_profile", "model_id", "association_status",
    }:
        raise ValueError("OpponentTrack fields do not match the revision-2 schema")
    meta = message["meta"]
    meta_fields = {
        "schema_version", "source_id", "source_session", "seq", "stage_id",
        "clock_epoch", "localization_epoch", "frame_id", "observation_stamp",
        "state_stamp", "publication_stamp", "valid_until", "map_version",
        "topology_version", "validity",
    }
    if not isinstance(meta, Mapping) or set(meta) != meta_fields:
        raise ValueError("ContractHeader fields do not match revision 2")
    if meta["schema_version"] != 2:
        raise ValueError("unsupported contract schema version")
    return TrackEnvelope(
        track_id=message["track_id"],
        state=_strict_enum(TrackState, message["state"], "state"),
        x=message["x"],
        y=message["y"],
        vx=message["vx"],
        vy=message["vy"],
        covariance=tuple(message["covariance"]),
        position_valid=_require_bool(message["position_valid"], "position_valid"),
        yaw=message["yaw"],
        yaw_variance=message["yaw_variance"],
        yaw_valid=_require_bool(message["yaw_valid"], "yaw_valid"),
        last_measurement_stamp=_seconds(message["last_measurement_stamp"], "last_measurement_stamp"),
        sensor_profile=_strict_enum(SensorProfile, message["sensor_profile"], "sensor_profile"),
        model_id=message["model_id"],
        association_status=message["association_status"],
        source_id=meta["source_id"],
        source_session=meta["source_session"],
        seq=_require_nonnegative_int(meta["seq"], "seq"),
        stage_id=meta["stage_id"],
        clock_epoch=meta["clock_epoch"],
        localization_epoch=meta["localization_epoch"],
        frame_id=meta["frame_id"],
        observation_stamp=_seconds(meta["observation_stamp"], "observation_stamp"),
        state_stamp=_seconds(meta["state_stamp"], "state_stamp"),
        publication_stamp=_seconds(meta["publication_stamp"], "publication_stamp"),
        valid_until=_seconds(meta["valid_until"], "valid_until"),
        map_version=_require_nonnegative_int(meta["map_version"], "map_version"),
        topology_version=_require_nonnegative_int(meta["topology_version"], "topology_version"),
        validity=_strict_enum(ContractValidity, meta["validity"], "validity"),
    )


def _write_ros_stamp(target: object, seconds: float) -> None:
    value = _stamp(seconds)
    setattr(target, "sec", value["sec"])
    setattr(target, "nanosec", value["nanosec"])


def _write_ros_header(target: object, header: Mapping[str, object]) -> None:
    for name, value in header.items():
        if name in ("observation_stamp", "state_stamp", "publication_stamp", "valid_until"):
            stamp = getattr(target, name)
            setattr(stamp, "sec", value["sec"])
            setattr(stamp, "nanosec", value["nanosec"])
        else:
            setattr(target, name, value)


def populate_ros_track(message: object, track: TrackEnvelope) -> None:
    """Populate a generated ROS OpponentTrack instance without importing ROS."""
    encoded = encode_track(track)
    _write_ros_header(getattr(message, "meta"), encoded["meta"])
    for name, value in encoded.items():
        if name == "meta":
            continue
        if name == "last_measurement_stamp":
            _write_ros_stamp(getattr(message, name), track.last_measurement_stamp)
        else:
            setattr(message, name, value)


def _read_ros_stamp(value: object, name: str) -> dict[str, int]:
    return {
        "sec": getattr(value, "sec"),
        "nanosec": getattr(value, "nanosec"),
    }


def _read_ros_header(value: object) -> dict:
    names = (
        "schema_version", "source_id", "source_session", "seq", "stage_id",
        "clock_epoch", "localization_epoch", "frame_id", "observation_stamp",
        "state_stamp", "publication_stamp", "valid_until", "map_version",
        "topology_version", "validity",
    )
    result = {}
    for name in names:
        field = getattr(value, name)
        result[name] = _read_ros_stamp(field, name) if name.endswith("_stamp") or name == "valid_until" else field
    return result


def decode_ros_track(message: object) -> TrackEnvelope:
    """Decode a generated ROS OpponentTrack instance using strict schema checks."""
    names = (
        "track_id", "state", "x", "y", "vx", "vy", "covariance",
        "position_valid", "yaw", "yaw_variance", "yaw_valid",
        "last_measurement_stamp", "sensor_profile", "model_id", "association_status",
    )
    payload = {name: getattr(message, name) for name in names}
    payload["meta"] = _read_ros_header(getattr(message, "meta"))
    payload["last_measurement_stamp"] = _read_ros_stamp(
        getattr(message, "last_measurement_stamp"), "last_measurement_stamp"
    )
    return decode_track(payload)


def encode_belief(
    belief: TopologicalBelief,
    *,
    track_id: str,
    transition_model_id: str,
    visibility_model_id: str,
    sensor_profile: SensorProfile,
    source_id: str,
    source_session: str,
    seq: int,
    stage_id: str,
    clock_epoch: str,
    frame_id: str,
    publication_stamp: float,
    valid_until: float,
    validity: ContractValidity,
) -> dict:
    envelope = BeliefEnvelope(
        track_id, belief.cells, belief.unknown_mass, belief.last_measurement_stamp_s,
        transition_model_id, visibility_model_id, sensor_profile, source_id,
        source_session, seq, stage_id, clock_epoch, belief.localization_epoch,
        frame_id, belief.last_measurement_stamp_s, belief.current_stamp_s,
        publication_stamp, valid_until, belief.map_version, belief.topology_version,
        validity,
    )
    return {
        "meta": {
            "schema_version": 2,
            "source_id": envelope.source_id,
            "source_session": envelope.source_session,
            "seq": envelope.seq,
            "stage_id": envelope.stage_id,
            "clock_epoch": envelope.clock_epoch,
            "localization_epoch": envelope.localization_epoch,
            "frame_id": envelope.frame_id,
            "observation_stamp": _stamp(envelope.observation_stamp),
            "state_stamp": _stamp(envelope.state_stamp),
            "publication_stamp": _stamp(envelope.publication_stamp),
            "valid_until": _stamp(envelope.valid_until),
            "map_version": envelope.map_version,
            "topology_version": envelope.topology_version,
            "validity": int(envelope.validity),
        },
        "track_id": envelope.track_id,
        "cells": [
            {
                "edge_id": cell.edge_id,
                "s_begin_m": cell.s_begin_m,
                "s_end_m": cell.s_end_m,
                "lateral_bound_m": cell.lateral_bound_m,
                "mass": cell.mass,
            }
            for cell in envelope.cells
        ],
        "unknown_mass": envelope.unknown_mass,
        "last_measurement_stamp": _stamp(envelope.last_measurement_stamp),
        "transition_model_id": envelope.transition_model_id,
        "visibility_model_id": envelope.visibility_model_id,
        "sensor_profile": int(envelope.sensor_profile),
    }


def populate_ros_belief(
    message: object,
    belief: TopologicalBelief,
    *,
    belief_cell_factory: type,
    **metadata,
) -> None:
    """Populate generated OpponentBelief and BeliefCell ROS message objects."""
    encoded = encode_belief(belief, **metadata)
    _write_ros_header(getattr(message, "meta"), encoded["meta"])
    for name, value in encoded.items():
        if name == "meta":
            continue
        if name == "cells":
            ros_cells = []
            for payload in value:
                cell = belief_cell_factory()
                for field_name, field_value in payload.items():
                    setattr(cell, field_name, field_value)
                ros_cells.append(cell)
            setattr(message, name, ros_cells)
        elif name == "last_measurement_stamp":
            _write_ros_stamp(getattr(message, name), belief.last_measurement_stamp_s)
        else:
            setattr(message, name, value)


def decode_ros_belief(message: object) -> BeliefEnvelope:
    """Decode generated ROS belief messages into the validated envelope."""
    meta = _read_ros_header(getattr(message, "meta"))
    cells = []
    for cell in getattr(message, "cells"):
        cells.append(
            {
                "edge_id": getattr(cell, "edge_id"),
                "s_begin_m": getattr(cell, "s_begin_m"),
                "s_end_m": getattr(cell, "s_end_m"),
                "lateral_bound_m": getattr(cell, "lateral_bound_m"),
                "mass": getattr(cell, "mass"),
            }
        )
    payload = {
        "meta": meta,
        "track_id": getattr(message, "track_id"),
        "cells": cells,
        "unknown_mass": getattr(message, "unknown_mass"),
        "last_measurement_stamp": _read_ros_stamp(
            getattr(message, "last_measurement_stamp"), "last_measurement_stamp"
        ),
        "transition_model_id": getattr(message, "transition_model_id"),
        "visibility_model_id": getattr(message, "visibility_model_id"),
        "sensor_profile": getattr(message, "sensor_profile"),
    }
    return decode_belief(payload)


def decode_belief(message: Mapping[str, object]) -> BeliefEnvelope:
    expected = {
        "meta", "track_id", "cells", "unknown_mass", "last_measurement_stamp",
        "transition_model_id", "visibility_model_id", "sensor_profile",
    }
    if set(message) != expected:
        raise ValueError("OpponentBelief fields do not match revision-2 schema")
    meta = message["meta"]
    if not isinstance(meta, Mapping) or meta.get("schema_version") != 2:
        raise ValueError("unsupported or missing ContractHeader schema version")
    cell_payloads = message["cells"]
    if not isinstance(cell_payloads, Sequence) or isinstance(cell_payloads, (str, bytes)):
        raise ValueError("cells must be a sequence")
    cells = []
    for payload in cell_payloads:
        if not isinstance(payload, Mapping) or set(payload) != {
            "edge_id", "s_begin_m", "s_end_m", "lateral_bound_m", "mass"
        }:
            raise ValueError("BeliefCell fields do not match revision-2 schema")
        cells.append(BeliefCell(**payload))
    meta_fields = {
        "schema_version", "source_id", "source_session", "seq", "stage_id",
        "clock_epoch", "localization_epoch", "frame_id", "observation_stamp",
        "state_stamp", "publication_stamp", "valid_until", "map_version",
        "topology_version", "validity",
    }
    if set(meta) != meta_fields:
        raise ValueError("ContractHeader fields do not match revision 2")
    return BeliefEnvelope(
        track_id=message["track_id"],
        cells=tuple(cells),
        unknown_mass=message["unknown_mass"],
        last_measurement_stamp=_seconds(message["last_measurement_stamp"], "last_measurement_stamp"),
        transition_model_id=message["transition_model_id"],
        visibility_model_id=message["visibility_model_id"],
        sensor_profile=_strict_enum(SensorProfile, message["sensor_profile"], "sensor_profile"),
        source_id=meta["source_id"],
        source_session=meta["source_session"],
        seq=_require_nonnegative_int(meta["seq"], "seq"),
        stage_id=meta["stage_id"],
        clock_epoch=meta["clock_epoch"],
        localization_epoch=meta["localization_epoch"],
        frame_id=meta["frame_id"],
        observation_stamp=_seconds(meta["observation_stamp"], "observation_stamp"),
        state_stamp=_seconds(meta["state_stamp"], "state_stamp"),
        publication_stamp=_seconds(meta["publication_stamp"], "publication_stamp"),
        valid_until=_seconds(meta["valid_until"], "valid_until"),
        map_version=_require_nonnegative_int(meta["map_version"], "map_version"),
        topology_version=_require_nonnegative_int(meta["topology_version"], "topology_version"),
        validity=_strict_enum(ContractValidity, meta["validity"], "validity"),
    )
