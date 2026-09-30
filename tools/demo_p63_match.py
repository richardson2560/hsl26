"""Run and render an interactive, fixture-only P6.3 match (top view + Doom view)."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Any

ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))
CORE_DIR = ROOT_DIR / "hsl_core"
if str(CORE_DIR) not in sys.path:
    sys.path.insert(0, str(CORE_DIR))

from hsl_core.types import Pose2D  # noqa: E402
from sim.kinematic.common import CircleTarget, WorldGeometry  # noqa: E402
from sim.kinematic.maze_bank import MazeFixture, load_maze_bank  # noqa: E402
from sim.kinematic.raycaster import FirstHitRaycaster  # noqa: E402
from sim.kinematic.sensors import LidarSensor  # noqa: E402
from tools.demo_competition_match import run_ideal_competition_match  # noqa: E402
from tools.p63_policy import canonical_json  # noqa: E402


_DEFAULT_BANK = (
    ROOT_DIR / "sim" / "kinematic" / "scenarios"
    / "phase6_interior_tactics_bank.json"
)
_ROBOT_RADIUS_M = 0.15


def _scan_frames(
    report: dict[str, Any],
    fixture: MazeFixture,
    *,
    frame_stride: int,
) -> list[dict[str, Any]]:
    trajectory = report["trajectory"]
    selected_indices = list(range(0, len(trajectory), frame_stride))
    if selected_indices[-1] != len(trajectory) - 1:
        selected_indices.append(len(trajectory) - 1)
    sensors = {
        role: LidarSensor(
            raycaster=FirstHitRaycaster(max_range_m=6.0),
            beam_count=360,
            max_range_m=6.0,
            noise_std_m=0.0,
            seed=fixture.seed,
        )
        for role in ("guardian", "explorer")
    }
    frames = []
    for index in selected_indices:
        sample = trajectory[index]
        scans = {}
        for role, opponent in (("guardian", "explorer"), ("explorer", "guardian")):
            x, y = sample[f"{role}_xy_m"]
            opponent_xy = sample[f"{opponent}_xy_m"]
            pose = Pose2D(x, y, sample[f"{role}_heading_rad"])
            observation = sensors[role].observe(
                pose,
                sample["stamp_ns"] / 1_000_000_000.0,
                WorldGeometry(
                    fixture.geometry.static_segments,
                    (CircleTarget(opponent_xy, _ROBOT_RADIUS_M, opponent),),
                ),
            )
            scans[role] = {
                "ranges": [round(value, 4) for value in observation.ranges_m],
                "valid": list(observation.valid_mask),
            }
        frames.append(
            {
                "stamp_ns": sample["stamp_ns"],
                "guardian": sample["guardian_xy_m"],
                "explorer": sample["explorer_xy_m"],
                "guardian_heading": sample["guardian_heading_rad"],
                "explorer_heading": sample["explorer_heading_rad"],
                "guardian_option": sample["guardian_option"],
                "explorer_option": sample["explorer_option"],
                "scans": scans,
            }
        )
    return frames


def _html_document(report: dict[str, Any], fixture: MazeFixture, frame_stride: int) -> str:
    data = {
        "scenario_id": fixture.scenario_id,
        "evidence_class": report["evidence_class"],
        "terminal_kind": report["terminal_kind"],
        "active_duration_s": report["active_duration_s"],
        "step_count": report["step_count"],
        "seed": report["seed"],
        "policy_sha256": report["policy_sha256"],
        "promotion_eligible": False,
        "cell_size_m": fixture.cell_size_m,
        "width_m": len(fixture.rows[0]) * fixture.cell_size_m,
        "height_m": len(fixture.rows) * fixture.cell_size_m,
        "walls": [
            [segment.start_xy, segment.end_xy]
            for segment in fixture.geometry.static_segments
        ],
        "frames": _scan_frames(report, fixture, frame_stride=frame_stride),
    }
    encoded_data = canonical_json(data).replace("<", "\\u003c")
    document = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>HSL26 P6.3 synthetic match viewer</title>
<style>
body{font:15px system-ui,sans-serif;margin:1rem;background:#f2f4f7;color:#18212b}
h1{font-size:1.35rem;margin:.25rem 0}.warning{color:#8b2600;font-weight:700}
.controls{display:flex;gap:.8rem;align-items:center;flex-wrap:wrap;margin:.8rem 0}
.views{display:grid;grid-template-columns:minmax(420px,1.2fr) minmax(320px,1fr);gap:1rem}
canvas{width:100%;height:auto;background:#fff;border:1px solid #8793a0;border-radius:5px}
.panel{min-width:0}.caption{font-size:.9rem;color:#43505c}
@media(max-width:900px){.views{grid-template-columns:1fr}}
</style>
</head>
<body>
<h1>Fixture match: <span id="scenario"></span></h1>
<div class="warning">SYNTHETIC SIL VISUALIZATION — NOT COMPETITION EVIDENCE OR A PHYSICAL SENSOR VIEW</div>
<p id="summary"></p>
<div class="controls">
  <button id="play">Play</button>
  <label>View robot <select id="role"><option value="guardian">Guardian</option><option value="explorer">Explorer</option></select></label>
  <label>Frame <input id="frame" type="range" min="0" value="0"></label>
  <span id="clock"></span>
</div>
<div class="views">
  <section class="panel"><canvas id="top" width="900" height="650"></canvas><div class="caption">Top view: walls, both trajectories, current poses and heading.</div></section>
  <section class="panel"><canvas id="doom" width="640" height="360"></canvas><div class="caption">Doom-style 2.5D projection from a synthetic 360-beam LiDAR scan; not a 3D renderer.</div></section>
</div>
<script id="scene-data" type="application/json">__SCENE_DATA__</script>
<script>
const scene=JSON.parse(document.getElementById("scene-data").textContent);
const topCanvas=document.getElementById("top"),tc=topCanvas.getContext("2d");
const doomCanvas=document.getElementById("doom"),dc=doomCanvas.getContext("2d");
const slider=document.getElementById("frame"),role=document.getElementById("role");
const frameList=scene.frames; slider.max=String(frameList.length-1);
document.getElementById("scenario").textContent=scene.scenario_id;
document.getElementById("summary").textContent=
  `Terminal ${scene.terminal_kind} at ${scene.active_duration_s.toFixed(2)} s; seed ${scene.seed}; `+
  `policy ${scene.policy_sha256.slice(0,16)}…; ${frameList.length} rendered frames.`;
function px(p){const m=38,s=Math.min((topCanvas.width-2*m)/scene.width_m,(topCanvas.height-2*m)/scene.height_m);
  return [m+p[0]*s,topCanvas.height-m-p[1]*s]}
function drawRobot(ctx,p,heading,color,label){const [x,y]=p;ctx.fillStyle=color;ctx.beginPath();ctx.arc(x,y,9,0,Math.PI*2);ctx.fill();
  ctx.strokeStyle="#111";ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(x,y);
  ctx.lineTo(x+19*Math.cos(heading),y-19*Math.sin(heading));ctx.stroke();
  ctx.fillStyle="#111";ctx.font="bold 14px sans-serif";ctx.fillText(label,x+11,y-10)}
function drawTop(index){const f=frameList[index],m=38;tc.fillStyle="#f8fafc";tc.fillRect(0,0,topCanvas.width,topCanvas.height);
  tc.strokeStyle="#202830";tc.lineWidth=3;
  for(const w of scene.walls){const a=px(w[0]),b=px(w[1]);tc.beginPath();tc.moveTo(...a);tc.lineTo(...b);tc.stroke()}
  for(const [name,color] of [["guardian","#d53032"],["explorer","#1769aa"]]){
    tc.strokeStyle=color;tc.lineWidth=3;tc.beginPath();
    for(let j=0;j<=index;j++){const q=px(frameList[j][name]);if(j===0)tc.moveTo(...q);else tc.lineTo(...q)}tc.stroke()
  }
  drawRobot(tc,px(f.guardian),f.guardian_heading,"#d53032","G");
  drawRobot(tc,px(f.explorer),f.explorer_heading,"#1769aa","E");
  const name=role.value,op=name==="guardian"?"explorer":"guardian";
  tc.fillStyle="#17212b";tc.font="14px sans-serif";
  tc.fillText(`${name.toUpperCase()} option: ${f[name+"_option"]}`,m,22);
  tc.fillText(`Opponent: ${op.toUpperCase()}  |  map is a software fixture`,m,topCanvas.height-12)
}
function drawDoom(index){const f=frameList[index],name=role.value,scan=f.scans[name],w=doomCanvas.width,h=doomCanvas.height,mid=h/2;
  dc.fillStyle="#6488aa";dc.fillRect(0,0,w,mid);dc.fillStyle="#555";dc.fillRect(0,mid,w,h-mid);
  const fov=Math.PI/3,count=scan.ranges.length;
  for(let x=0;x<w;x++){const rel=-fov/2+(x+.5)/w*fov;
    const bi=Math.min(count-1,Math.max(0,Math.floor(((rel+Math.PI)/(2*Math.PI))*count)));
    if(!scan.valid[bi])continue;
    const d=Math.max(.001,scan.ranges[bi]*Math.cos(rel));
    const wall=Math.min(h,1.0*320/d),y0=Math.max(0,Math.round(mid-wall/2)),y1=Math.min(h,Math.round(mid+wall/2));
    const shade=Math.max(35,Math.min(220,Math.round(220/(1+d/6))));
    dc.fillStyle=`rgb(${shade},${shade},${shade})`;dc.fillRect(x,y0,1,y1-y0)
  }
  dc.fillStyle="#fff";dc.font="14px sans-serif";dc.fillText(`${name.toUpperCase()} synthetic LiDAR`,12,20)
}
function show(){const i=Number(slider.value),f=frameList[i];drawTop(i);drawDoom(i);
  document.getElementById("clock").textContent=`t=${(f.stamp_ns/1e9).toFixed(2)} s  ${f[role.value+"_option"]}`}
slider.addEventListener("input",show);role.addEventListener("change",show);
let playing=false,timer=null;document.getElementById("play").addEventListener("click",()=>{
  playing=!playing;document.getElementById("play").textContent=playing?"Pause":"Play";
  if(playing){timer=setInterval(()=>{slider.value=String((Number(slider.value)+1)%frameList.length);show()},200)}
  else clearInterval(timer)});
show();
</script>
</body>
</html>
"""
    return document.replace("__SCENE_DATA__", encoded_data)


def run_p63_match_viewer(
    *,
    policy_path: str | Path,
    output_dir: str | Path,
    bank_path: str | Path = _DEFAULT_BANK,
    scenario_id: str = "maze_interior_loops_train_c",
    seed: int = 101,
    duration_s: float = 60.0,
    frame_stride: int = 4,
) -> dict[str, Any]:
    if (
        not isinstance(frame_stride, int)
        or isinstance(frame_stride, bool)
        or frame_stride < 1
    ):
        raise ValueError("frame_stride must be a positive integer")
    bank = load_maze_bank(bank_path)
    fixture = next(
        (item for item in bank.fixtures if item.scenario_id == scenario_id),
        None,
    )
    if fixture is None:
        raise ValueError(f"scenario_id is not present in the maze bank: {scenario_id}")
    if fixture.split == "held_out":
        raise ValueError("visualization cannot consume reserved held-out fixtures")
    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    report = run_ideal_competition_match(
        scenario_id=scenario_id,
        max_episode_duration_s=duration_s,
        seed=seed,
        bank_path=bank_path,
        policy_path=policy_path,
        output_dir=destination,
    )
    viewer_path = destination / "match_viewer.html"
    if viewer_path.exists():
        raise FileExistsError(f"refusing to overwrite existing viewer: {viewer_path}")
    viewer_path.write_text(
        _html_document(report, fixture, frame_stride),
        encoding="utf-8",
    )
    report["viewer_path"] = str(viewer_path)
    report["rendered_frame_stride"] = frame_stride
    return report


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--bank", type=Path, default=_DEFAULT_BANK)
    parser.add_argument("--scenario-id", default="maze_interior_loops_train_c")
    parser.add_argument("--seed", type=int, default=101)
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--frame-stride", type=int, default=4)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


if __name__ == "__main__":
    arguments = _parse_args()
    result = run_p63_match_viewer(
        policy_path=arguments.policy,
        output_dir=arguments.output_dir,
        bank_path=arguments.bank,
        scenario_id=arguments.scenario_id,
        seed=arguments.seed,
        duration_s=arguments.duration_s,
        frame_stride=arguments.frame_stride,
    )
    print(
        f"Fixture match {result['terminal_kind']} at "
        f"{result['active_duration_s']:.3f}s; "
        f"viewer={result['viewer_path']}; "
        f"promotion_eligible={result['promotion_eligible']}"
    )
