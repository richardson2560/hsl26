# tools/calibrate_braking.py
"""Prepare and execute HSL26 braking calibration trials.

The dry-run profile is deterministic and never publishes a velocity command.
The real profile is intentionally explicit about its unavailable ROS/hardware
authority on this host; it must be connected to an authorized driver adapter
before physical motion is enabled.
"""

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import sys
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class BrakingReport:
    schema_version: int
    profile: str
    status: str
    speed_mps: float
    sample_rate_hz: float
    response_delay_s: float
    stopping_distance_m: float
    measured_stop_time_s: float
    measured_b_min_mps2: float
    trial_count: int
    source: str
    seed: int | None
    uncertainty: dict[str, float]


def _positive(value: float, name: str) -> float:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


def _nonnegative(value: float, name: str) -> float:
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return float(value)


def estimate_b_min(speed_mps: float, stopping_distance_m: float, response_delay_s: float) -> float:
    """Estimate constant minimum deceleration from measured trial values."""

    speed_mps = _positive(speed_mps, "speed_mps")
    stopping_distance_m = _positive(stopping_distance_m, "stopping_distance_m")
    response_delay_s = _nonnegative(response_delay_s, "response_delay_s")
    braking_distance = stopping_distance_m - speed_mps * response_delay_s
    if braking_distance <= 0.0:
        raise ValueError("stopping distance does not contain a positive braking phase")
    return speed_mps * speed_mps / (2.0 * braking_distance)


def _simulate_trial(
    speed_mps: float,
    deceleration_mps2: float,
    response_delay_s: float,
    sample_rate_hz: float,
    noise_std_m: float,
    seed: int,
) -> tuple[float, float]:
    speed_mps = _positive(speed_mps, "speed_mps")
    deceleration_mps2 = _positive(deceleration_mps2, "deceleration_mps2")
    response_delay_s = _nonnegative(response_delay_s, "response_delay_s")
    sample_rate_hz = _positive(sample_rate_hz, "sample_rate_hz")
    noise_std_m = _nonnegative(noise_std_m, "noise_std_m")
    rng = np.random.default_rng(seed)
    dt = 1.0 / sample_rate_hz
    stop_duration = response_delay_s + speed_mps / deceleration_mps2
    stamps = np.arange(0.0, stop_duration + dt * 0.5, dt)
    distances = np.where(
        stamps <= response_delay_s,
        speed_mps * stamps,
        speed_mps * response_delay_s
        + speed_mps * (stamps - response_delay_s)
        - 0.5 * deceleration_mps2 * (stamps - response_delay_s) ** 2,
    )
    distances = np.maximum.accumulate(distances)
    distances += rng.normal(0.0, noise_std_m, distances.shape)
    distances = np.maximum.accumulate(distances)
    measured_distance = float(distances[-1])
    measured_stop_time = float(stamps[-1])
    return measured_distance, measured_stop_time


def run_dry_run(
    *,
    speed_mps: float,
    sample_rate_hz: float,
    response_delay_s: float,
    deceleration_mps2: float,
    noise_std_m: float,
    trials: int,
    seed: int,
) -> BrakingReport:
    if not isinstance(trials, int) or isinstance(trials, bool) or trials <= 0:
        raise ValueError("trials must be a positive integer")
    distances = []
    stop_times = []
    estimates = []
    for trial in range(trials):
        distance, stop_time = _simulate_trial(
            speed_mps,
            deceleration_mps2,
            response_delay_s,
            sample_rate_hz,
            noise_std_m,
            seed + trial,
        )
        distances.append(distance)
        stop_times.append(stop_time)
        estimates.append(estimate_b_min(speed_mps, distance, response_delay_s))
    estimates_array = np.asarray(estimates, dtype=float)
    return BrakingReport(
        schema_version=1,
        profile="kinematic",
        status="PASS_SYNTHETIC_ONLY",
        speed_mps=float(speed_mps),
        sample_rate_hz=float(sample_rate_hz),
        response_delay_s=float(response_delay_s),
        stopping_distance_m=float(np.mean(distances)),
        measured_stop_time_s=float(np.mean(stop_times)),
        measured_b_min_mps2=float(np.min(estimates_array)),
        trial_count=trials,
        source="deterministic synthetic odometry; not hardware calibration",
        seed=seed,
        uncertainty={
            "distance_std_m": float(np.std(distances, ddof=0)),
            "b_min_std_mps2": float(np.std(estimates_array, ddof=0)),
            "noise_std_m": float(noise_std_m),
        },
    )


def write_report(report: BrakingReport, target_file: Path) -> None:
    target_file.parent.mkdir(parents=True, exist_ok=True)
    target_file.write_text(
        json.dumps(asdict(report), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="HSL26 braking calibration tool")
    profile = parser.add_mutually_exclusive_group(required=True)
    profile.add_argument("--dry-run", action="store_true", help="run deterministic synthetic trials")
    profile.add_argument("--real", action="store_true", help="request authorized hardware mode")
    parser.add_argument("--speed", type=float, default=0.40, help="initial speed in m/s")
    parser.add_argument("--sample-rate", type=float, default=50.0, help="odometry rate in Hz")
    parser.add_argument("--response-delay", type=float, default=0.075, help="synthetic response delay in s")
    parser.add_argument("--deceleration", type=float, default=0.85, help="synthetic deceleration in m/s^2")
    parser.add_argument("--noise-std", type=float, default=0.001, help="synthetic odometry noise in m")
    parser.add_argument("--trials", type=int, default=3, help="number of synthetic trials")
    parser.add_argument("--seed", type=int, default=20260925, help="deterministic synthetic seed")
    parser.add_argument(
        "--target-file",
        type=Path,
        default=Path("artifacts/calibration/braking_report.json"),
        help="JSON report destination",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.real:
        print(
            "REAL_BLOCKED_FOR_HARDWARE: connect an authorized ROS/Kobuki odometry "
            "adapter before enabling physical braking trials",
            file=sys.stderr,
        )
        return 2
    report = run_dry_run(
        speed_mps=args.speed,
        sample_rate_hz=args.sample_rate,
        response_delay_s=args.response_delay,
        deceleration_mps2=args.deceleration,
        noise_std_m=args.noise_std,
        trials=args.trials,
        seed=args.seed,
    )
    write_report(report, args.target_file)
    print(json.dumps(asdict(report), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
