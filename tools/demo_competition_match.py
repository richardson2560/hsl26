"""Run an explicitly synthetic SIL self-play visualization of a P6.3 policy."""

from __future__ import annotations

import argparse
from html import escape
import hashlib
from pathlib import Path
import sys
from typing import Any

ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))
CORE_DIR = ROOT_DIR / "hsl_core"
if str(CORE_DIR) not in sys.path:
    sys.path.insert(0, str(CORE_DIR))

from sim.kinematic.benchmark import BenchmarkConfig, SILBenchmarkRunner  # noqa: E402
from sim.kinematic.maze_bank import load_maze_bank  # noqa: E402
from tools.p63_policy import (  # noqa: E402
    EVIDENCE_CLASS,
    build_default_baseline,
    canonical_json,
    policy_payload,
    load_development_policy,
)


_DEFAULT_BANK = ROOT_DIR / "sim" / "kinematic" / "scenarios" / "phase6_maze_bank.json"


def run_ideal_competition_match(
    *,
    scenario_id: str = "maze_multiring_7x7_train_a",
    max_episode_duration_s: float = 30.0,
    seed: int = 101,
    bank_path: str | Path = _DEFAULT_BANK,
    policy_path: str | Path | None = None,
    output_dir: str | Path,
) -> dict[str, Any]:
    bank = load_maze_bank(bank_path)
    fixture_by_id = {fixture.scenario_id: fixture for fixture in bank.fixtures}
    if scenario_id not in fixture_by_id:
        raise ValueError(f"scenario_id is not present in the maze bank: {scenario_id}")
    fixture = fixture_by_id[scenario_id]
    if fixture.split == "held_out":
        raise ValueError("demo cannot consume reserved held-out fixtures")
    if policy_path is None:
        genome = build_default_baseline()
        policy_payload_data = policy_payload(
            genome,
            selection_reason="demo_baseline_no_policy_supplied",
            baseline_sha256=genome.sha256,
            run_id="baseline-demo",
        )
    else:
        genome, policy_payload_data = load_development_policy(policy_path)

    runner = SILBenchmarkRunner(
        BenchmarkConfig(
            episode_duration_s=max_episode_duration_s,
            control_period_s=0.05,
            discount_rate_per_s=0.01,
            lidar_beam_count=360,
            lidar_max_range_m=6.0,
            lidar_noise_std_m=0.0,
            robot_radius_m=0.15,
            robot_max_speed_mps=0.20,
        )
    )
    episode = runner.run_episode(
        genome,
        fixture,
        seed=seed,
        collect_telemetry=True,
    )
    distances = [
        ((sample.guardian_xy_m[0] - sample.explorer_xy_m[0]) ** 2
         + (sample.guardian_xy_m[1] - sample.explorer_xy_m[1]) ** 2) ** 0.5
        for sample in episode.telemetry
    ]
    report = {
        "schema": "hsl26.p63-sil-match-demo.v1",
        "evidence_class": EVIDENCE_CLASS,
        "promotion_eligible": False,
        "official_score_available": False,
        "physical_authority": False,
        "sensor_profile": {
            "description": "synthetic 360-beam full-circle raycast, zero range noise, no configured blind sectors",
            "hardware_or_sensor_validation": False,
        },
        "scenario_id": episode.scenario_id,
        "scenario_sha256": episode.scenario_sha256,
        "split": episode.split.value,
        "evaluation_profile_id": episode.evaluation_profile_id,
        "benchmark_config": {
            "episode_duration_s": max_episode_duration_s,
            "control_period_s": 0.05,
            "discount_rate_per_s": 0.01,
            "lidar_beam_count": 360,
            "lidar_max_range_m": 6.0,
            "lidar_noise_std_m": 0.0,
            "robot_radius_m": 0.15,
            "robot_max_speed_mps": 0.20,
        },
        "maze_bank_sha256": hashlib.sha256(
            Path(bank_path).read_bytes()
        ).hexdigest(),
        "seed": seed,
        "policy_sha256": genome.sha256,
        "policy_selection_reason": policy_payload_data["selection_reason"],
        "terminal_kind": episode.terminal_kind.name,
        "active_duration_s": episode.active_duration_s,
        "step_count": episode.step_count,
        "guardian_wall_contacts": episode.guardian_wall_contacts,
        "explorer_wall_contacts": episode.explorer_wall_contacts,
        "minimum_inter_robot_distance_m": min(distances) if distances else None,
        "capture_threshold_m": 0.45,
        "capture_truth_is_offline_evaluation_only": True,
        "role_results": {
            result.role.name: {
                "policy_sha256": result.policy_id,
                "surrogate_return": result.training_return,
                "safety_overrides": result.safety_overrides,
                "safety_violations": result.safety_violations,
                "collisions": result.collisions,
                "completed": result.completed,
            }
            for result in episode.episode_results
        },
        "telemetry_samples": len(episode.telemetry),
        "trajectory": [
            {
                "stamp_ns": sample.stamp_ns,
                "guardian_xy_m": sample.guardian_xy_m,
                "explorer_xy_m": sample.explorer_xy_m,
                "guardian_command": sample.guardian_command,
                "explorer_command": sample.explorer_command,
                "guardian_option": sample.guardian_option,
                "explorer_option": sample.explorer_option,
            }
            for sample in episode.telemetry
        ],
    }

    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    report_path = destination / "match_report.json"
    svg_path = destination / "match.svg"
    if report_path.exists() or svg_path.exists():
        raise FileExistsError(
            f"refusing to overwrite existing demo outputs in {destination}"
        )
    report_path.write_text(canonical_json(report), encoding="utf-8")
    svg_path.write_text(_render_svg(fixture.geometry.static_segments, episode), encoding="utf-8")
    report["report_path"] = str(report_path)
    report["visualization_path"] = str(svg_path)
    return report


def _render_svg(walls, episode) -> str:
    width, height = 1200, 660
    left = (40, 50, 560, 560)
    right = (640, 50, 520, 560)
    points = [
        point
        for wall in walls
        for point in (wall.start_xy, wall.end_xy)
    ]
    points.extend(
        point
        for sample in episode.telemetry
        for point in (sample.guardian_xy_m, sample.explorer_xy_m)
    )
    min_x = min(point[0] for point in points) - 0.25
    max_x = max(point[0] for point in points) + 0.25
    min_y = min(point[1] for point in points) - 0.25
    max_y = max(point[1] for point in points) + 0.25
    scale = min(left[2] / (max_x - min_x), left[3] / (max_y - min_y))

    def map_xy(point: tuple[float, float]) -> tuple[float, float]:
        x = left[0] + (point[0] - min_x) * scale
        y = left[1] + left[3] - (point[1] - min_y) * scale
        return x, y

    chunks = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" role="img">',
        "<rect width='100%' height='100%' fill='white'/>",
        "<text x='40' y='30' font-family='sans-serif' font-size='20'>SIL fixture match — not competition evidence</text>",
        "<text x='40' y='635' font-family='sans-serif' font-size='13'>Guardian (red), Explorer (blue); trajectory and options come from policy/runtime telemetry.</text>",
    ]
    for wall in walls:
        x1, y1 = map_xy(wall.start_xy)
        x2, y2 = map_xy(wall.end_xy)
        chunks.append(
            f"<line x1='{x1:.2f}' y1='{y1:.2f}' x2='{x2:.2f}' y2='{y2:.2f}' stroke='#222' stroke-width='3'/>"
        )
    for role, color in (
        ("guardian", "#c62828"),
        ("explorer", "#1565c0"),
    ):
        trajectory = [
            map_xy(getattr(sample, f"{role}_xy_m"))
            for sample in episode.telemetry
        ]
        if trajectory:
            path = " ".join(
                f"{'M' if index == 0 else 'L'}{x:.2f},{y:.2f}"
                for index, (x, y) in enumerate(trajectory)
            )
            chunks.append(
                f"<path d='{path}' fill='none' stroke='{color}' stroke-width='2' opacity='0.8'/>"
            )
            for x, y in (trajectory[0], trajectory[-1]):
                chunks.append(
                    f"<circle cx='{x:.2f}' cy='{y:.2f}' r='4' fill='{color}'/>"
                )

    times = [sample.stamp_ns / 1_000_000_000 for sample in episode.telemetry]
    distances = [
        ((sample.guardian_xy_m[0] - sample.explorer_xy_m[0]) ** 2
         + (sample.guardian_xy_m[1] - sample.explorer_xy_m[1]) ** 2) ** 0.5
        for sample in episode.telemetry
    ]
    px, py, pw, ph = right
    chunks.extend(
        [
            f"<rect x='{px}' y='{py}' width='{pw}' height='{ph}' fill='#fafafa' stroke='#777'/>",
            f"<text x='{px}' y='{py - 12}' font-family='sans-serif' font-size='16'>Inter-robot distance; capture threshold is diagnostic</text>",
        ]
    )
    if times and distances:
        max_time = max(times[-1], 1e-9)
        max_distance = max(max(distances), 0.5)
        coords = [
            (
                px + time / max_time * pw,
                py + ph - min(distance, max_distance) / max_distance * ph,
            )
            for time, distance in zip(times, distances)
        ]
        path = " ".join(
            f"{'M' if index == 0 else 'L'}{x:.2f},{y:.2f}"
            for index, (x, y) in enumerate(coords)
        )
        threshold_y = py + ph - min(0.45, max_distance) / max_distance * ph
        chunks.append(
            f"<line x1='{px}' y1='{threshold_y:.2f}' x2='{px + pw}' y2='{threshold_y:.2f}' stroke='#d32f2f' stroke-dasharray='7 5'/>"
        )
        chunks.append(
            f"<path d='{path}' fill='none' stroke='#7b1fa2' stroke-width='2'/>"
        )
    terminal = escape(episode.terminal_kind.name)
    chunks.append(
        f"<text x='{px}' y='{py + ph + 28}' font-family='sans-serif' font-size='14'>Terminal: {terminal}; fixture-only / no official score</text>"
    )
    chunks.append("</svg>")
    return "\n".join(chunks) + "\n"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario-id", default="maze_multiring_7x7_train_a")
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--seed", type=int, default=101)
    parser.add_argument("--bank", type=Path, default=_DEFAULT_BANK)
    parser.add_argument("--policy", type=Path)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT_DIR / "artifacts" / "reports" / "phase6" / "p63_demo",
    )
    return parser.parse_args()


if __name__ == "__main__":
    arguments = _parse_args()
    result = run_ideal_competition_match(
        scenario_id=arguments.scenario_id,
        max_episode_duration_s=arguments.duration_s,
        seed=arguments.seed,
        bank_path=arguments.bank,
        policy_path=arguments.policy,
        output_dir=arguments.output_dir,
    )
    print(
        f"SIL fixture demo ended {result['terminal_kind']} at "
        f"{result['active_duration_s']:.3f}s; evidence={result['evidence_class']}; "
        f"visualization={result['visualization_path']}"
    )
