# HSL26 — Repository Structure and Implementation (Professional Software Engineering Report)

**Role:** software engineering / repository architecture.
**Inherited base:** the `hackathon/2025` repository (StarLine GitLab), Docker + `colcon` + ROS2 pattern, with `kobuki/docker/{build,run,into,stop}.bash`, `kobuki/scripts/udev_rules`, `kobuki/scripts/connect_to_new_wifi.bash`, and `kobuki/workspace/src/{cmd_vel_mux, kobuki_core, kobuki_ros, kobuki_ros_interfaces, livox_ros_driver2}`.
**Hardware context confirmed by the 2025 repo:** Kobuki (differential base) + Livox MID-360 + Intel NUC (BOXNUC7I7BNH) + MikroTik RB951G-2HND router (robot's local subnet, `eth2`→NUC, `eth3`→Livox) + Rombica NEO PRO 280 auxiliary battery. SSH over this subnet is the only preparation channel (R8).
**Goal of this document:** define the directory tree, the Docker contract, the ROS2 package layout, the real/simulation/replay policy, the concrete mechanisms that close the 4 blind spots flagged by the external audit (§3 of the attached dictamen): LiDAR blind zone, LiDAR-odometry degeneration, Python GIL blocking the supervisor, and the Kobuki driver watchdog — and, in this revision, the `kobuki_core` dependency details and the GitHub repository creation procedure.

---

## 0. Software engineering decisions (why this structure and not another)

1. **`kobuki/` is not rewritten.** It is the already-proven vendor/hardware layer from HSL25 (Kobuki and Livox drivers, `cmd_vel_mux`, udev rules, Docker scripts). It is treated as a frozen dependency pinned to a specific upstream commit/tag; all of HSL26's own development lives in new packages that **depend on it** and never modify it in place. This reduces the risk of breaking something that already worked last year.
2. **The existing `cmd_vel_mux` is reused as the final physical arbitration layer**, instead of reimplementing a custom mux: the Supervisor (Layer 6 of the design) publishes on an autonomous-priority topic, and a higher-priority topic is kept for teleop/emergency stop over SSH during testing — so a human can always take control without touching code.
3. **Strict separation between core (pure Python, no ROS) and ROS2 adapters (thin).** Everything mathematical (`rules.py`, `kinematics.py`, `topology.py`, EKF, A*, FSM/ASG) is implemented as a `pytest`-testable Python library. The `ros_ws/src/hsl_*` packages are thin *wrappers*: they receive ROS2 messages, call the `hsl_core` library, and publish ROS2 messages. This allows running **thousands** of unit tests without bringing up ROS or Docker, and reusing exactly the same code in the lightweight kinematic simulator.
4. **Real, simulation (MVSim), and bag replay share the same ROS2 node graph** via a profiled bringup; only the origin of `/sensors/points`, `/sensors/imu`, and `/wheel_odom` changes, along with who runs the physics. No decision node knows whether it is in sim or real — this is what makes the sim-to-real transition valid without rewriting tactics/control.
5. **A single, isolated system process for the Supervisor**, directly addressing audit point 3.3 (GIL): the Supervisor runs in its own ROS2 node/process, with its own dedicated single-thread executor, and does not import heavy NumPy/SciPy, DBSCAN, or the A* planner in that process. It communicates via topics with `depth=1`, `reliability=BEST_EFFORT` for sensor data and `RELIABLE` for `MatchState`.
6. **Anything the rulebook forbids hardcoding (R9) lives in `config/`, never in code.** No `.py`/`.cpp` file contains maze, obstacle, or opponent coordinates.

---

## 1. Proposed directory tree (extends the 2025 repo)

```
hsl26/
├── .gitignore
├── .gitlab-ci.yml
├── .github/                                  # if mirrored to GitHub — see §7
│   ├── workflows/ci.yml
│   ├── CODEOWNERS
│   └── ISSUE_TEMPLATE/
├── README.md
├── LICENSE
├── docs/
│   ├── HSL26_FINAL_ARCHITECTURE.md            # this design document (normative source)
│   ├── REPO_STRUCTURE.md                      # this report
│   ├── Регламент_HSL26_-_v06092026.pdf
│   ├── runbook_competition.md
│   └── media/
│
├── kobuki/                                    # ← inherited from HSL25, pinned by commit, NOT edited
│   ├── docker/{Dockerfile,build.bash,run.bash,into.bash,stop.bash}
│   ├── scripts/{udev_rules/, connect_to_new_wifi.bash}
│   └── workspace/src/
│       ├── cmd_vel_mux/                       # reused as the final teleop > autonomous arbiter
│       ├── kobuki_core/                       # C++ core libraries — see §2 for pinning strategy
│       ├── kobuki_ros/
│       ├── kobuki_ros_interfaces/
│       └── livox_ros_driver2/
│
├── hsl_core/                                  # pure Python library, NO ROS, pytest-testable
│   ├── pyproject.toml
│   ├── hsl_core/
│   │   ├── __init__.py
│   │   ├── types.py                           # EgoState, OpponentTrack, WorldSnapshot, OptionGoal...
│   │   ├── kinematics.py                      # Eq. (1)-(6): integration, deskew, footprint
│   │   ├── rules.py                           # Eq. (15)-(18): capture, arrival, geometric feasibility
│   │   ├── mapping.py                         # Eq. (7)-(8): log-odds, inflation, background veto
│   │   ├── topology.py                        # portal graph, bridge/articulation-point DFS
│   │   ├── perception/
│   │   │   ├── implicit_surface.py            # GPIS-W (reused/adapted from the 2025 tracker)
│   │   │   ├── registration.py                # damped Gauss-Newton, Eq. (9)-(10)
│   │   │   ├── segmenter.py                   # DBSCAN + shape/volume cascade filter
│   │   │   ├── ekf_opponent.py                # Eq. (11)-(14): CV filter + gating
│   │   │   └── topological_belief.py          # Eq. (§6.3-bis): occlusion + anti-flanking belief tracker
│   │   ├── planning/
│   │   │   ├── astar.py                       # Eq. (19)-(20): costs ≥0, admissible heuristic
│   │   │   └── intercept.py                   # Eq. (22)-(24): quadratic + topological route times
│   │   ├── tactics/
│   │   │   ├── fsm.py                         # base deterministic FSM (initial policy)
│   │   │   ├── options.py                     # option contracts: precond./effect/termination
│   │   │   └── utility.py                     # U_E, U_G (Appendix A of the design)
│   │   ├── control/
│   │   │   ├── regulated_pursuit.py           # Eq. (25)
│   │   │   ├── dwa_local.py                   # Eq. (26)
│   │   │   └── braking.py                     # Eq. (27)-(28): braking envelope with latency
│   │   └── learning/                          # Layer 7 — never imported by critical runtime nodes
│   │       ├── dirichlet.py                   # Eq. (33)-(34)
│   │       ├── option_value.py                # Eq. (32)
│   │       └── evolution.py                   # bounded search over 5-10 parameters
│   └── tests/
│       ├── test_rules.py                      # T01-T04, T14 of the design's verification table
│       ├── test_braking.py                    # T12
│       ├── test_astar.py                      # T11
│       ├── test_ekf_gating.py                 # T08, T09
│       ├── test_topological_belief.py         # reachable-set bound, negative-information collapse
│       └── test_kinematics.py                 # T06 deskew, sinc for |ω|≤ε
│
├── ros_ws/
│   └── src/
│       ├── hsl_interfaces/                    # ONLY .msg/.action/.srv definitions, no logic
│       │   ├── msg/{EgoState,OpponentTrack,WorldSnapshot,OptionGoal,OptionFeedback,
│       │   │        MotionCandidate,SafetyStatus,MatchState}.msg
│       │   ├── action/ExecuteOption.action
│       │   └── srv/ResetStage.srv
│       │
│       ├── hsl_perception/                    # adapter: real/sim Livox cloud → hsl_core.perception
│       │   ├── hsl_perception/
│       │   │   ├── ego_state_node.py          # deskew + TF + EgoState (Layer 0)
│       │   │   ├── local_obstacles_node.py    # fast obstacle path (Layer 1, high priority)
│       │   │   └── opponent_tracker_node.py   # Layer 2
│       │   └── launch/perception.launch.py
│       │
│       ├── hsl_world/                         # Layer 1: structural map + topology
│       │   ├── hsl_world/{map_server_node.py, topology_node.py, odom_fusion_node.py}
│       │   └── launch/world.launch.py
│       │
│       ├── hsl_decision/                      # Layer 3: per-role FSM/ASG
│       │   ├── hsl_decision/{tactics_node.py, option_lifecycle.py}
│       │   └── launch/decision.launch.py
│       │
│       ├── hsl_navigation/                    # Layer 4: A* + regulated pursuit/DWA
│       │   ├── hsl_navigation/{global_planner_node.py, local_control_node.py}
│       │   └── launch/navigation.launch.py
│       │
│       ├── hsl_safety/                        # Layer 6: ISOLATED PROCESS, sole producer of cmd_vel_final
│       │   ├── hsl_safety/
│       │   │   ├── supervisor_node.py         # dedicated single-thread executor, no SciPy/DBSCAN
│       │   │   ├── watchdog.py                # publishes zeros ≥20 Hz, closes §3.4 of the dictamen
│       │   │   └── braking_envelope.py        # thin wrapper over hsl_core.control.braking
│       │   └── launch/safety.launch.py
│       │
│       ├── hsl_match/                         # Layer 5: freeze/active/timeout/score_profile
│       │   ├── hsl_match/stage_manager_node.py
│       │   └── launch/match.launch.py
│       │
│       ├── hsl_bringup/                       # SINGLE entry point; real/sim/replay profiles
│       │   ├── launch/
│       │   │   ├── hsl26.launch.py            # orchestrates every hsl_* node per `mode:=`
│       │   │   ├── real_hardware.launch.py    # includes real kobuki + livox_ros_driver2
│       │   │   ├── sim_mvsim.launch.py        # includes sim/mvsim/*
│       │   │   └── replay_bag.launch.py       # ros2 bag play + remaps
│       │   └── config/
│       │       ├── hardware.yaml              # extrinsics, physical limits, measured b_min
│       │       ├── perception.yaml
│       │       ├── tactics.yaml
│       │       ├── score_profile.yaml         # NO official values until announced (R13)
│       │       └── frames.yaml                # map/odom/base_link/lidar_frame
│       │
│       └── hsl_diagnostics/                   # R16 terminal-indication requirement
│           └── hsl_diagnostics/terminal_hud_node.py
│
├── sim/
│   ├── kinematic/                             # lightweight tactical simulator (NumPy), reuses hsl_core
│   │   ├── engine.py                          # integrates (3), no ROS, thousands of episodes/sec
│   │   ├── referee.py                         # private ground truth + official events
│   │   └── scenarios/                         # corridor, T, crossroads, cycle, dead end, etc. (design §8)
│   └── mvsim/
│       ├── worlds/*.world.xml
│       ├── vehicles/turtlebot2_mid360.vehicle.xml
│       └── adapter/simulation_adapter.py      # real reset/step/observe/ground_truth_for_referee
│
├── config/
│   └── schema/                                # JSON-Schema validation for every YAML above
│
├── tools/
│   ├── preflight_check.py                     # G0: TF, topics, time, command, clean shutdown;
│   │                                           #     `--blind-zone-sweep` mode for §5.1 below
│   ├── bag_replay_eval.py
│   ├── benchmark_latency.py                   # end-to-end p50/p95/p99 (design §9)
│   └── release_freeze.py                      # commit hash + manifest (R10, R18)
│
├── tests/
│   └── integration/                           # colcon test: startup, TF tree, remaps, freeze→active,
│                                               # watchdog rate (§5.4 below)
│
└── artifacts/
    ├── policies/                              # frozen policies/parameters + hash manifest
    └── calibration/                           # blind_zone_report.yaml and similar measured constants
```

---

## 2. The `kobuki_core` dependency

`kobuki_core` (folder `kobuki/workspace/src/kobuki_core/` in the tree above) is not an internal HSL package: it is the **C++ library and utility stack** for talking to a Kobuki robot base, maintained upstream at `kobuki-base/kobuki_core` (`http://kobuki.yujinrobot.com`). Two release lines are documented upstream:

| `kobuki_core` branch | Documentation |
|:---:|:---:|
| [`devel`](https://github.com/kobuki-base/kobuki_core/tree/devel) | https://kobuki.readthedocs.io/en/devel/ |
| [`release/1.2.x`](https://github.com/kobuki-base/kobuki_core/tree/release/1.2.x) | https://kobuki.readthedocs.io/en/release-1.0.x/ |

Consequences for HSL26's repository structure:

- **Pin, don't fork.** Since `kobuki_core` is a real upstream project with its own branches, it should be tracked with `git submodule` (or `git subtree` if the team prefers a single-clone workflow) pointed at a specific commit on `release/1.2.x` — the stable line — rather than a loose copy-pasted snapshot as currently sits under `kobuki/workspace/src/`. This makes upstream bug fixes/security updates a deliberate `git submodule update` instead of an untracked diff.
- **`devel` is for reference only.** It is useful to consult the `devel` docs when debugging a driver-level issue, but HSL26 competition builds must resolve to a `release/1.2.x` commit, matching the C++ ABI the rest of `kobuki/workspace/src/{kobuki_ros, kobuki_ros_interfaces}` (already vendored from 2025) was built against.
- **Version manifest.** `tools/release_freeze.py` (§1 tree) records the exact `kobuki_core` commit hash alongside the HSL26 commit hash in `artifacts/policies/*.yaml`, so a competition-day build is fully reproducible even if upstream moves.
- **Language boundary.** `kobuki_core` and `kobuki_ros` are C++; everything HSL26 adds (`hsl_core`, `ros_ws/src/hsl_*`) is Python. The only interface between them is standard ROS2 topics/services/actions — `hsl_perception`/`hsl_safety` never link against `kobuki_core` directly, they only depend on the topics `kobuki_ros` already exposes. This keeps the build graph simple (`colcon build` handles the mixed C++/Python workspace without custom glue) and keeps the audit's GIL argument (§5.3 below) valid: the C++ driver layer is not subject to Python's GIL at all.

---

## 3. Docker strategy

### 3.1 Reusing the 2025 base image

The 2025 `kobuki/docker/Dockerfile` already solves ROS2 + Kobuki/Livox drivers + `colcon`. It is kept **as is** as the base image (`hsl26/kobuki:2025-frozen`, tagged by the inherited commit hash) and HSL26 is built as a second stage **on top of it**, not replacing it:

```dockerfile
# hsl26/docker/Dockerfile
ARG BASE_IMAGE=hsl26/kobuki:2025-frozen
FROM ${BASE_IMAGE} AS hsl26-base

# Dependencies new and exclusive to HSL26 (do not touch kobuki/'s own deps)
COPY hsl_core/pyproject.toml /tmp/hsl_core/pyproject.toml
RUN pip install --no-cache-dir -e /tmp/hsl_core \
    && pip install --no-cache-dir numpy scipy open3d pytest

# Combined ROS2 workspace: kobuki/workspace (inherited) + ros_ws (own)
COPY kobuki/workspace /workspace_base
COPY ros_ws /workspace_hsl26
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /workspace_base && colcon build --symlink-install && \
    source install/setup.bash && \
    cd /workspace_hsl26 && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

ENTRYPOINT ["/hsl26/docker/entrypoint.bash"]
```

`entrypoint.bash` chain-sources `/opt/ros/.../setup.bash` → `/workspace_base/install/setup.bash` → `/workspace_hsl26/install/setup.bash`, then runs the `CMD` (by default, an interactive shell for `into.bash`).

### 3.2 Scripts, same pattern as 2025 (operational consistency for the team)

| Script | Change relative to 2025 |
|---|---|
| `hsl26/docker/build.bash` | Same pattern; adds a `MODE` build-arg (`real`/`sim`) to install MVSim dependencies only when needed |
| `hsl26/docker/run.bash` | Mounts `ros_ws/`, `hsl_core/`, `sim/`, `config/`, `artifacts/` as development volumes (`--symlink-install` already supports hot editing); exposes `--network host` only on the real robot (required for Livox/Kobuki on the MikroTik subnet) |
| `hsl26/docker/into.bash` | Same |
| `hsl26/docker/stop.bash` | Same |
| `hsl26/docker/run_sim.bash` (**new**) | No `--network host`, no USB/serial device access, for laptop development without a robot |

### 3.3 Why not a single monolithic image

Separating the "frozen kobuki base" from the "HSL26 layer" lets the team rebuild the own layer in seconds during development (Docker cache over the heavy driver stage), and it also lets the team **exactly reproduce the HSL25 environment** to debug regressions without contaminating it with new dependencies (NumPy/SciPy/Open3D specific to opponent perception).

---

## 4. `hsl_interfaces`: contracts as code, not prose

Every message described in the architecture document maps 1:1 to a `.msg`. Example (`OpponentTrack.msg`):

```
# ros_ws/src/hsl_interfaces/msg/OpponentTrack.msg
std_msgs/Header header
uint32 seq
builtin_interfaces/Time observation_stamp
builtin_interfaces/Time valid_until
uint32 map_version
uint32 localization_epoch
string status                    # SEARCHING | TRACKED | COASTING | OCCLUDED_BELIEF | LOST
float64 x
float64 y
float64 vx
float64 vy
float64[16] covariance           # 4x4 over [x,y,vx,vy]
bool yaw_valid
float64 yaw                      # only valid if yaw_valid=true
builtin_interfaces/Time t_last_measurement
```

`status` includes `OCCLUDED_BELIEF` to expose the phase-2/3 belief state from `hsl_core/perception/topological_belief.py` (design §6.3-bis) to the tactics layer, so `hsl_decision` can distinguish "actively tracked" from "diffusing belief over the topological graph" when weighing `INTERCEPT_PORTAL` vs. `FALLBACK_DEFEND_BASE`.

`MotionCandidate.msg` and `SafetyStatus.msg` follow the same principle: **every temporal field explicit**, never an isolated `float confidence` (rule fixed in the design's §4). This package has **no logic**, only definitions — so `hsl_core` can have its own mirrored Python dataclasses without depending on `rclpy`, and ROS2 nodes only convert at the boundary.

---

## 5. Real / Simulation / Replay: one graph, three data origins

`hsl_bringup/launch/hsl26.launch.py` accepts a `mode:={real,sim,replay}` argument and composes:

```
                         ┌─────────────────────────────┐
mode:=real   ──────────► │ kobuki_node + livox_ros_     │
                         │ driver2 (real hardware)       │──┐
                         └─────────────────────────────┘  │
                                                            │  /sensors/points
mode:=sim    ──────────► ┌─────────────────────────────┐  │  /sensors/imu
                         │ sim/mvsim adapter (or the     │──┤  /wheel_odom
                         │ lightweight kinematic engine)  │  │  (logical channels
                         └─────────────────────────────┘  │   identical)
                                                            │
mode:=replay ──────────► ┌─────────────────────────────┐  │
                         │ ros2 bag play (real           │──┘
                         │ preparation bags)              │
                         └─────────────────────────────┘
                                       │
                                       ▼
        (identical in all 3 modes) hsl_perception → hsl_world → hsl_decision →
        hsl_navigation → hsl_safety → cmd_vel_mux → real or sim driver
```

Hard rules of this design (already fixed in the architecture document, §14.3, and enforced in code):

- **No node in `hsl_decision`, `hsl_navigation`, or `hsl_safety` imports anything from `sim/`.** They only consume logical topics (`/state/ego`, `/world/opponent`, etc.), never the simulator's API. This is verified by a static import check in CI (`tools/check_no_sim_imports.py`).
- **`hsl_safety` is the only node allowed to write to `cmd_vel_mux`'s autonomous-priority input.** A second, higher-priority topic is reserved for manual teleop (`teleop_twist_keyboard`, already used in 2025) — so during field tests a human operator can always take control without stopping the process.
- **In `mode:=sim`, the opponent's true pose never reaches `hsl_decision`.** The simulation adapter publishes `ground_truth_for_referee` on a separate topic, consumed *only* by `sim/kinematic/referee.py` or the MVSim referee — never by `hsl_perception`. This closes the "guard against simulation cheating" that the audit highlighted as a strength, here turned into a verifiable ROS2 namespacing rule (separate namespaces `/referee/*` vs. `/world/*`).
- **`mode:=replay` remaps `/sensors/points` and `/sensors/imu`** to the topics stored in the bag, and forces `use_sim_time:=true` with the bag's clock — wall clock and bag clock are never mixed (the "single epoch" rule from the design, §14.4).

---

## 6. How the repository closes the 4 blind spots flagged by the audit, in code

### 6.1 MID-360 short-range blind zone (dictamen §3.1)

- `tools/preflight_check.py` has a `--blind-zone-sweep` mode that at G1 places both robots at 0.50/0.40/0.30 m and logs the point count returned by `hsl_perception/opponent_tracker_node.py`; the result is stored in `artifacts/calibration/blind_zone_report.yaml`.
- `hsl_core/perception/ekf_opponent.py` implements an explicit **short-range** `COASTING` state (distinct from the general occlusion `OCCLUDED_BELIEF` of §6.3-bis in the design), activated when `d̂ < d_blind_measured` and the track was `TRACKED` with low relative velocity: it coasts inertially without declaring `LOST`, with a short `valid_until`. The `d_blind_measured` threshold is loaded from `blind_zone_report.yaml`, **never hardcoded**.

### 6.2 LiDAR-odometry degeneration in homogeneous corridors (dictamen §3.2)

- `hsl_bringup/config/hardware.yaml` explicitly declares the odometry source: `odom_source: wheel_imu_fused` by default (via a `robot_localization` EKF fusing `kobuki_core`/`kobuki_ros` wheel encoders + the NUC/Livox IMU), with `lio_source: fastlio2` only as slow-drift correction or relocalization, never as the sole source of longitudinal advance.
- New package `hsl_world/hsl_world/odom_fusion_node.py` (or, if time is short, a direct `robot_localization` configuration) is the sole producer of `odom→base_link`; `hsl_perception/ego_state_node.py` consumes its output and does **not** publish a second transform on the same link (the "one producer per transform" rule, design §4).

### 6.3 Python GIL and the 50 Hz Supervisor (dictamen §3.3)

- `hsl_safety` is its **own ROS2 package with its own process** (`supervisor_node.py` launched as an independent node, not as a *component* loaded into a shared container). `hsl_safety/launch/safety.launch.py` launches it with its own `ExecuteProcess`, never inside the same `ComposableNodeContainer` as perception/planning.
- `hsl_safety/hsl_safety/watchdog.py` runs in the same process as a high-priority timer of the single-thread executor; that process **does not import** `open3d`, `scipy.spatial`, or the A* planner — only `hsl_core.control.braking` and lightweight types. This is what guarantees a 40 ms DBSCAN call in `hsl_perception` cannot steal the GIL from the supervisor: they are separate OS processes, not threads of the same interpreter.
- Supervisor input QoS: `depth=1`, `history=KEEP_LAST` on every topic it consumes, so it never processes a stale queued sample.

### 6.4 Kobuki driver watchdog (dictamen §3.4)

- `hsl_safety/hsl_safety/watchdog.py` publishes on `/commands/velocity` (or whichever topic the installed `kobuki_ros` version exposes) at **≥20 Hz unconditionally**, even in `FREEZE` or with `cmd_vel_final` at explicit zero — it never stops publishing. Verified by an integration test (`tests/integration/test_watchdog_rate.py`) that measures the actual published frequency over 30 s of tactical inactivity.
- `tools/preflight_check.py` at G0 includes an explicit test: kill the decision/planning node and confirm the robot still receives zero commands at the Kobuki driver's minimum frequency (avoids the ~0.6 s motor cutoff documented by the manufacturer).

---

## 7. Creating and structuring the GitHub repository

The team's 2025 work lives on GitLab (`StarLine / hackathon / 2025`); this section covers standing up HSL26 as its own **GitHub** repository (or a GitHub mirror of the same GitLab project), matching the structure above.

### 7.1 Initial creation

```bash
# 1. Create the repo on GitHub first (empty, no README/gitignore/license — avoids a merge conflict
#    with the local history below), e.g. via `gh repo create StarLine/hsl26 --private --source=. `
#    or through the GitHub web UI.

# 2. Locally, start from the existing 2025 layout so history/config are not lost
git clone <gitlab-url>/hackathon/2025.git hsl26
cd hsl26
git remote rename origin gitlab-2025          # keep the 2025 remote for reference/cherry-picks
git remote add origin git@github.com:StarLine/hsl26.git

# 3. Pin kobuki_core to the real upstream project instead of a loose copy (see §2)
git rm -r --cached kobuki/workspace/src/kobuki_core
git submodule add -b release/1.2.x https://github.com/kobuki-base/kobuki_core.git \
    kobuki/workspace/src/kobuki_core
git commit -m "Pin kobuki_core to upstream release/1.2.x as a submodule"

# 4. Scaffold the new layers from §1
mkdir -p hsl_core/hsl_core hsl_core/tests \
         ros_ws/src/{hsl_interfaces,hsl_perception,hsl_world,hsl_decision,hsl_navigation,hsl_safety,hsl_match,hsl_bringup,hsl_diagnostics} \
         sim/kinematic/scenarios sim/mvsim/{worlds,vehicles,adapter} \
         config/schema tools tests/integration artifacts/{policies,calibration} docs/media

git add -A
git commit -m "HSL26: scaffold repository structure on top of the HSL25 base"
git push -u origin main
```

### 7.2 Branch strategy

| Branch | Purpose | Protection |
|---|---|---|
| `main` | Always buildable; every commit passes `unit` + `integration` CI | Protected, PR-only, required status checks |
| `develop` | Integration branch for in-progress features | Protected, PR-only |
| `feature/<layer>-<short-desc>` | One branch per layer/task (e.g. `feature/hsl_safety-braking-envelope`) | Deleted after merge |
| `release/hsl26-<tag>` | Cut before a competition day, only bugfixes on top | Protected |

### 7.3 Repository-root files to add alongside the tree in §1

- **`README.md`** — mirrors the 2025 README's structure (hardware description, power-on sequence, Docker usage, launch commands) updated for `hsl_bringup`'s `mode:=` argument, plus a link to `docs/HSL26_FINAL_ARCHITECTURE.md` and this report.
- **`.gitignore`** — extends the 2025 one with `hsl_core/**/__pycache__/`, `ros_ws/**/build/`, `ros_ws/**/install/`, `ros_ws/**/log/`, `artifacts/calibration/*.yaml` (measured, machine-specific — not committed by default; only the schema is), `*.bag`.
- **`.gitmodules`** — created automatically by `git submodule add` in §7.1; also add any other vendored upstream package the team decides to pin the same way.
- **`.github/CODEOWNERS`** — maps each `ros_ws/src/hsl_*` package and `hsl_core/hsl_core/<area>/` to the teammate(s) responsible for that layer, so PRs auto-request the right reviewer.
- **`.github/workflows/ci.yml`** — GitHub Actions mirror of the `.gitlab-ci.yml` stages in §8 below, if the team wants CI on both platforms (or only on GitHub, if it fully migrates).
- **`LICENSE`** — match whatever license the 2025 repo/StarLine organization uses; `kobuki_core` and `livox_ros_driver2` keep their own upstream licenses under `kobuki/`.

### 7.4 Keeping GitLab and GitHub in sync (if both are used)

If GitLab remains the org's primary platform and GitHub is only a mirror (or vice versa), configure a simple two-remote push:

```bash
git remote set-url --add --push origin git@github.com:StarLine/hsl26.git
git remote set-url --add --push origin <gitlab-url>/hackathon/hsl26.git
git push origin main   # pushes to both remotes in one command
```

For automated mirroring instead of a manual dual-push, GitLab's built-in "Mirroring repositories" (push mirror to GitHub) or a scheduled `git push --mirror` CI job are both simpler to maintain than keeping two independent histories by hand.

---

## 8. Testing and CI (`.gitlab-ci.yml`, GitLab already being the team's platform)

| Stage | What runs | Where |
|---|---|---|
| `lint` | `ruff`/`flake8` + `mypy` on `hsl_core/` | Lightweight runner, no robot Docker |
| `unit` | `pytest hsl_core/tests` (T01-T09, T11, T12, T14 of the design's verification table) | Same runner, seconds |
| `build` | `docker build` of `hsl26/docker/Dockerfile`, full `colcon build` (submodule checked out) | Runner with Docker-in-Docker |
| `integration` | `tests/integration/` against `mode:=sim` (startup, TF tree, freeze→active, watchdog rate) | Inside the built container |
| `no-sim-leak` | `tools/check_no_sim_imports.py` (fails the pipeline if `hsl_decision`/`hsl_safety` import anything from `sim/`) | Static, fast |
| `release` (manual, tags only) | `tools/release_freeze.py`: generates a hash + manifest (including the pinned `kobuki_core` commit) in `artifacts/policies/` | Only before competing |

The `unit` stage catches regressions fastest because it **does not depend on ROS2 or Docker** — the main reason `hsl_core` is kept separate from `ros_ws/`.

---

## 9. Short usage runbook (consistent with the design's §18.1)

```bash
# Normal development (laptop, no robot)
bash hsl26/docker/run_sim.bash
bash hsl26/docker/into.bash
ros2 launch hsl_bringup hsl26.launch.py mode:=sim role:=explorer

# Validation with real preparation bags
ros2 launch hsl_bringup hsl26.launch.py mode:=replay bag:=/data/bags/prep_day1

# On the real robot, over SSH, during preparation (R8)
bash hsl26/docker/run.bash        # --network host, access to Kobuki/Livox
bash hsl26/docker/into.bash
ros2 launch hsl_bringup hsl26.launch.py mode:=real role:=guardian

# Before competing (release freeze, R10/R18)
python3 tools/release_freeze.py --tag hsl26-final --profile config/score_profile.yaml
```

This flow reuses exactly the same verbs the team already used in HSL25 (`build.bash`, `run.bash`, `into.bash`, `stop.bash`, `colcon build --symlink-install`), minimizing the learning curve for the new repository under time pressure.
