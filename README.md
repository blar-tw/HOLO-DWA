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
- [Comparison](#comparison)

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
| [`scanner.py`](scanner.py) | Live ROS 2 node: PX4 Offboard control + LiDAR-driven DWA navigation. |
| [`dwa_core.py`](dwa_core.py) | Holonomic DWA algorithm — the `(vx, vy)` window search + scoring (no ROS deps). |
| [`run.sh`](run.sh) | One-shot tmux launcher for the full stack. |
| [`gz_extra/`](gz_extra/) | `x500_lidar_2d` model, airframe, and `dwa_test` world missing from PX4 v1.14.4 (`install.sh` copies them in); obstacle guide in [usage.md](gz_extra/usage.md). |
| [`holo_lab/`](holo_lab/) | Experiment harness + tuned planner (1/5 → 15/15 runs, zero collisions). **Start here for results** — [README](holo_lab/README.md). |
| [`docs/`](docs/) | architecture, documentation, installation, discussion. |
| [`archive/`](archive/) | `dwa_logic.py` (offline PyBullet prototype), `bug.md` (`ros_gz` debug log). |

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

## Comparison

> **Thesis work in progress.** This is the central study of the thesis: a
> controlled benchmark of the tuned holonomic DWA here against a baseline of
> comparable compute cost. The framework is fixed; the results table is pending.
