# HSL26

StarLine Hackathon 2026 (HSL26) - Autonomous Pursuit and Evasion System.

This repository combines a pure-Python mathematical and training core with a
ROS 2 Humble hardware/integration layer inherited from HSL25. The layers are
deliberately developed and executed with different tools:

| Layer | Preferred environment | Purpose |
| --- | --- | --- |
| `hsl_core`, `sim/kinematic` | Windows Conda/venv or Linux Python | Fast unit tests, mathematical simulation, offline training |
| `ros_ws`, `kobuki`, `sim/mvsim` | Docker Desktop with WSL2 or Linux | ROS 2 builds, drivers, MVSim, integration tests |
| Physical robot | Linux NUC and hardware | Competition deployment and diagnostics |
| Replay | Docker/Linux ROS 2 | rosbag2 playback and sensor/filter calibration |

## Repository map

```text
hsl_core/       ROS-independent domain types, mathematics, and learning
ros_ws/src/     HSL26 ROS 2 interfaces and node packages
kobuki/         Frozen/inherited HSL25 Kobuki and Livox hardware layer
sim/kinematic/  Lightweight NumPy simulation
sim/mvsim/      MVSim integration adapter
docker/         Pinned multi-stage ROS 2 build and environment entrypoint
docs/           Architecture, Phase 1 design, roadmap, and audit records
```

The safety boundary is mandatory: navigation publishes a `MotionCandidate`,
`hsl_safety` validates it, and only the supervisor publishes
`/hsl/cmd_vel_final`. The physical mux publishes `/commands/velocity`.

## Quick start: local mathematical development

Use this path for `hsl_core` and the lightweight simulator. It does not
require ROS 2, Docker, Kobuki, or Linux device access.

### Windows PowerShell with Conda

```powershell
conda create -n hsl26 python=3.10 -y
conda activate hsl26
python -m pip install numpy scipy open3d matplotlib pytest
python -m pip install -e .\hsl_core
python -m pytest .\hsl_core\tests -v
python .\sim\kinematic\engine.py
```

If Conda is not available, a Python 3.10+ virtual environment can be used
instead. The simulator must remain usable in headless mode; visualization is
optional and should not be a prerequisite for tests or training.

### Linux, WSL2, or a POSIX shell

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install numpy scipy open3d matplotlib pytest
python -m pip install -e ./hsl_core
python -m pytest ./hsl_core/tests -v
python ./sim/kinematic/engine.py
```

The current host validation may also run the dependency-free contract checks
from the Phase 1 task. Full pytest coverage is required before P1.1 can be
marked `DONE`.

## Docker and ROS 2 workflow

Docker is the supported environment for ROS 2 Humble, the inherited drivers,
MVSim, and integration testing. The Dockerfile intentionally starts from the
pinned HSL25 hardware image
`nickodema/kobuki:humble-22.04-100625`; replacing it with a clean ROS image
requires rebuilding and validating Livox-SDK2 and Kobuki dependencies.

From the repository root:

```bash
docker build -f docker/Dockerfile -t hsl26:latest .
docker run --rm -it hsl26:latest bash
```

Inside the container:

```bash
colcon test --packages-select hsl_interfaces hsl_safety hsl_bringup
colcon test-result --verbose
```

The complete workspace build is performed by the image build. When iterating
on source code, rebuild the image or mount a development checkout explicitly;
do not assume a running container sees unmounted host changes.

## Execution modes

The repository defines four operational modes:

1. **Lightweight kinematic simulation**: NumPy-based, fast, suitable for
   rules, FSM behavior, parameter sweeps, and Layer 7 offline training.
2. **MVSim dynamic simulation**: ROS 2 integration path for LiDAR, wheel,
   latency, and complete graph validation.
3. **Real hardware**: Kobuki and Livox on the competition NUC.
4. **Replay**: rosbag2-driven calibration and regression testing.

The bringup launch files are currently integration scaffolds. Before claiming
an end-to-end mode is operational, verify that the relevant launch file starts
the required nodes and passes the topic-graph and safety acceptance tests.
Intended command forms are:

```bash
ros2 launch hsl_bringup hsl26.launch.py mode:=sim role:=guardian
ros2 launch hsl_bringup hsl26.launch.py mode:=sim role:=explorer
```

For headless operation, use the launch/configuration path without GUI
forwarding and monitor diagnostics from a second shell. GUI forwarding through
WSLg or X11 is a development convenience, not a competition requirement.

## Offline training

Training is deliberately separated from ROS 2 and MVSim. The lightweight
kinematic simulator is the intended high-throughput environment:

```bash
python -m hsl_core.learning.evolution --episodes 5000 --workers 4
```

Training outputs must be treated as reviewed artifacts. A policy candidate
must include its configuration, provenance, and SHA-256 integrity value before
it is copied into `artifacts/policies/` and referenced by
`ros_ws/src/hsl_bringup/config/tactics.yaml`. Online adaptation is disabled
for the competition runtime.

## Development order

1. Run local `hsl_core` tests and the kinematic simulator.
2. Build the pinned Docker image and run ROS package tests.
3. Complete Phase 1: domain contracts, kinematics, braking, rules,
   supervisor, watchdog, and end-to-end command authority.
4. Validate simulation and replay paths.
5. Only then implement higher-level perception, navigation, tactics, and
   offline learning integration.

Read [`docs/HSL26_PHASE1_TECHNICAL_DESIGN.md`](docs/HSL26_PHASE1_TECHNICAL_DESIGN.md)
for normative APIs, algorithms, timing, and safety behavior. Read
[`docs/HSL26_IMPLEMENTATION_ROADMAP.md`](docs/HSL26_IMPLEMENTATION_ROADMAP.md)
for task status and acceptance evidence.
