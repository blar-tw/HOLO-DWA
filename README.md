# Holonomic Dynamic Window Approach in a PX4 SITL + Gazebo Environment

> A holonomic take on the Dynamic Window Approach: reactive, LiDAR-driven
> obstacle avoidance for a simulated multirotor, using Gazebo to sim.

## Demo

<!-- Replace with a real recording (GIF or an uploaded MP4 link). -->
_Demo video: coming soon._

## Table of Contents

- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Documentation](#documentation)
  - [What each file does](#what-each-file-does)
  - [Configuration](#configuration)
  - [Structs / Classes](#structs--classes)
  - [Functions](#functions)
- [Comparison with Other 3D Obstacle-Avoidance Algorithms](#comparison-with-other-3d-obstacle-avoidance-algorithms)
- [References](#references)

## Requirements

- WSL2 + Ubuntu 22.04 (or native Ubuntu 22.04)
- ROS 2 Humble
- PX4-Autopilot v1.14.4 + Gazebo (Garden or Harmonic — must match the bridge)
- `ros_gz` bridge **built for your Gazebo version** — check `gz sim --version`
  first. The default `ros-humble-ros-gz-bridge` targets Fortress and will
  silently drop every message; this was a real bug, see [bug.md](archive/bug.md).
- Micro-XRCE-DDS-Agent, `px4_msgs`, `tmux`
- Python 3.10 with `numpy` (live node); `pybullet` for the offline prototype

## Installation

Full step-by-step setup (WSL2, ROS 2, PX4, the version-matched `ros_gz`
bridge, the workspace, and the extra Gazebo models) is in
**[installation.md](docs/installation.md)**.

The `x500_lidar_2d` / `lidar_2d_v2` models and the `dwa_test` world are missing
from PX4 v1.14.4; [`gz_extra/install.sh`](gz_extra/install.sh) copies them in:

```bash
cd ~/ws/src/HOLO-DWA
./gz_extra/install.sh ~/PX4-Autopilot
```

## Usage

One-shot launch (opens a 4-pane tmux session: PX4 SITL + Gazebo, XRCE-DDS
Agent, `ros_gz_bridge`, and the DWA node):

```bash
cd ~/ws/src/HOLO-DWA
./run.sh                # default goal (12.0, 0.0)
./run.sh 8.0 -3.0       # custom goal_x goal_y  (Gazebo world coords)
./run.sh kill           # tear the session down
```

Before the first run, PX4 needs Offboard-without-RC enabled once
(`NAV_DLL_ACT`, `COM_RCL_EXCEPT`, ...); the exact params and the manual
4-terminal launch are in [installation.md](docs/installation.md).

## Documentation

### What each file does

| Path | Role |
|------|------|
| [`scanner.py`](scanner.py) | Live ROS 2 node — PX4 Offboard control + LiDAR-driven DWA navigation (state machine: INIT → TAKEOFF → NAVIGATE). |
| [`dwa_core.py`](dwa_core.py) | The holonomic DWA algorithm: the `(vx, vy)` dynamic-window search + scoring. No ROS / PyBullet deps, so it is imported directly by the node. |
| [`run.sh`](run.sh) | One-shot tmux launcher for the full stack (PX4 SITL+Gazebo / XRCE-DDS agent / `ros_gz` bridge / DWA node). |
| [`gz_extra/`](gz_extra/) | The `x500_lidar_2d` model, airframe, and `dwa_test` world missing from PX4 v1.14.4; [`install.sh`](gz_extra/install.sh) copies them into PX4-Autopilot. |
| [`holo_lab/`](holo_lab/) | **Instrumented experiment harness + the optimized planner.** Logs every control tick, batches runs, and carries a tuned `dwa_core.py` that takes obstacle avoidance from **1/5 to 15/15 runs with zero collisions**. Start here for the results — see its [README](holo_lab/README.md). |
| [`docs/`](docs/) | [architecture.md](docs/architecture.md) (frames, state machine, DWA loop, params), [documentation.md](docs/documentation.md) (class/function reference), [installation.md](docs/installation.md), [discussion.md](docs/discussion.md), [modify.md](docs/modify.md). |
| [`archive/`](archive/) | [`dwa_logic.py`](archive/dwa_logic.py) (offline PyBullet prototype) and [bug.md](archive/bug.md) (the `ros_gz` version-mismatch debugging log). |

> The root `scanner.py` / `dwa_core.py` are the **baseline** planner. The tuned
> version and the full before→after study (every failure mode, root cause, and
> fix) live in [`holo_lab/`](holo_lab/) and
> [holo_lab/EXPERIMENTS.md](holo_lab/EXPERIMENTS.md).

### Configuration

All planner knobs are fields on `dwa_core.Config`, overridden for the live
drone in the `dwa_config` block near the top of `scanner.py`. Edit there and
relaunch (`./run.sh`) — no rebuild needed.

| Parameter | Default (live) | What it does |
|-----------|---------------:|--------------|
| `v_max`, `vx/vy_min/max` | 1.5, ±1.5 | speed and per-axis velocity limits (m/s) |
| `a_max`, `brake_a_max` | 1.0, 1.0 | dynamic-window accel bound / braking limit (m/s²) |
| `predict_time`, `predict_dt` | 3.0, 0.2 | how far / how finely each candidate is rolled out (s) |
| `vx/vy_resolution` | 0.1 | velocity-grid step — finer = smoother paths, more compute |
| `robot_radius` | 0.2 | obstacle inflation radius (m); raise for more clearance |
| `goal_threshold` | 0.5 | arrival radius (m) |
| `heading / clearance / velocity_weight` | 0.2 / 0.2 / 0.6 | relative weight of aiming at the goal / staying clear / going fast |
| `velocity_mode` | `scalar` | `scalar` \| `component` \| `blend` velocity reward |

ROS 2 node parameters: `goal_x`, `goal_y` (Gazebo world coords, default
`12.0, 0.0`) — pass as `./run.sh <goal_x> <goal_y>`.

How each weight and mode changes behaviour (wall deadlock, the doorway
problem, open-space drift), and the tuned values that fixed them, are in
[docs/discussion.md](docs/discussion.md) and
[holo_lab/EXPERIMENTS.md](holo_lab/EXPERIMENTS.md).

### Structs / Classes

- **`dwa_core.Config`** — every tunable planner parameter (see [Configuration](#configuration)).
- **`scanner.DroneLidarScanner(Node)`** — the ROS 2 flight node (Offboard heartbeat, LiDAR intake, control loop).

Full fields and methods: [docs/documentation.md → Structs / Classes](docs/documentation.md#structs--classes).

### Functions

- **`dwa_core.scan_to_world_points(...)`** — convert a body-frame LiDAR scan into a world-frame obstacle point cloud.
- **`dwa_core.dwa_control(state, goal_xy, obstacle_points, config)`** — one DWA step over the dynamic window → `(vx, vy, ok)`.

Full signatures and semantics: [docs/documentation.md → Functions](docs/documentation.md#functions).

## Comparison with Other 3D Obstacle-Avoidance Algorithms

> **Thesis work in progress.** This is the central study of the thesis: a
> controlled benchmark of the tuned holonomic DWA here against a baseline of
> comparable compute cost. The framework below is fixed; the results table is
> pending.

**Algorithms compared**

1. **Original DWA** (Fox et al., 1997) — the untuned baseline this project
   started from, to quantify what the scoring changes bought.
2. **A reactive planner of comparable per-tick compute** *(candidate: VFH+ /
   APF — to be fixed)* — matched on compute budget so the comparison isolates
   *behaviour*, not hardware/compute advantage.

**Method** *(to be finalized)*

- Identical setup for every algorithm: the same `dwa_test` arena, start pose,
  goal, LiDAR (1080 rays, 270° FOV, 30 Hz), and 20 Hz control rate.
- N runs each on fresh simulator stacks, with fixed / logged initial
  conditions for repeatability (same harness as [`holo_lab/`](holo_lab/)).
- Compute held comparable across algorithms — reported per-tick in ms — so
  differences reflect the planner, not the budget.

**Metrics**

- success rate (reached / N) and collision count / episodes
- minimum obstacle clearance (m)
- path length and path efficiency (straight-line / actual)
- time to goal (s)
- per-tick compute (ms) — the fairness control

**Limitations** *(to be expanded)*

- Simulation only (PX4 SITL + Gazebo); no real-flight validation yet.
- 2D scan plane at a held altitude — obstacles are effectively vertical; true
  3D avoidance is out of scope for this comparison.
- Static obstacles only; moving-obstacle cases are future work.

**Results**

_Coming soon — the benchmark tables and trajectory plots will land here._

## References

- D. Fox, W. Burgard, S. Thrun, *The Dynamic Window Approach to Collision
  Avoidance*, IEEE Robotics & Automation Magazine, 1997. — the original DWA;
  the heading / clearance / velocity scoring used here follows this.
- O. Brock, O. Khatib, *High-Speed Navigation Using the Global Dynamic Window
  Approach*, ICRA, 1999. — adds a global connectivity check to escape the
  local minima plain DWA gets stuck in (the wall/doorway traps in
  [discussion.md](docs/discussion.md)).
- P. Ögren, N. E. Leonard, *A Convergent Dynamic Window Approach to Obstacle
  Avoidance*, IEEE Transactions on Robotics, 2005. — conditions under which DWA
  provably reaches the goal.
- M. Seder, I. Petrović, *Dynamic Window Based Approach to Mobile Robot Motion
  Control in the Presence of Moving Obstacles*, ICRA, 2007. — extends DWA to
  dynamic obstacles.
- J. Borenstein, Y. Koren, *The Vector Field Histogram — Fast Obstacle
  Avoidance for Mobile Robots*, IEEE T-RA, 1991. — a reactive-avoidance
  predecessor for comparison.
- [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide)
- [Gazebo Harmonic + ROS installation](https://gazebosim.org/docs/harmonic/ros_installation/)
- [gazebosim/ros_gz](https://github.com/gazebosim/ros_gz)
- [PX4/PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models)
- [eProsima Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent)
- Related notes in this repo: [installation.md](docs/installation.md),
  [architecture.md](docs/architecture.md), [documentation.md](docs/documentation.md),
  [discussion.md](docs/discussion.md), [bug.md](archive/bug.md), and the
  obstacle-adding guide [gz_extra/usage.md](gz_extra/usage.md).
