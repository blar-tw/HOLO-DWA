# Holonomic Dynamic Window Approach in a PX4 SITL + Gazebo Environment

> A holonomic take on the Dynamic Window Approach: reactive, LiDAR-driven
> obstacle avoidance for a simulated multirotor, using Gazebo to sim.

Status: simulation-only, actively developed. The full pipeline
(takeoff -> LiDAR obstacle avoidance -> reach goal) runs end-to-end in Gazebo.
The drone plans holonomically in the horizontal plane at a held altitude
(2D LiDAR + constant Z); true 3D avoidance is on the roadmap.

Source: https://github.com/blar-tw/HOLO-DWA.git

## Demo

<!-- Replace with a real recording (GIF or an uploaded MP4 link). -->
_Demo video: coming soon._

## Table of Contents

- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Documentation](#documentation)
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
  silently drop every message; this was a real bug, see [bug.md](docs/bug.md).
- Micro-XRCE-DDS-Agent, `px4_msgs`, `tmux`
- Python 3.10 with `numpy` (live node); `pybullet` for the offline prototype

Exact, checkpoint-by-checkpoint versions are pinned in
[installation.md](docs/installation.md).

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

The system is one custom ROS 2 node ([`scanner.py`](scanner.py)) driving PX4
Offboard control, fed by a version-matched Gazebo->ROS LiDAR bridge, with the
planner factored into a ROS-free module ([`dwa_core.py`](dwa_core.py)).

- **Architecture, data flow, coordinate frames, state machine, and the
  simulation world** -> [architecture.md](docs/architecture.md)
- **Full API reference** -> [documentation.md](docs/documentation.md)
- **Algorithm design trade-offs** (local minima, velocity reward, doorway
  problem) -> [discussion.md](docs/discussion.md)

### Structs / Classes

| Name | Where | Role |
|------|-------|------|
| `Config` | `dwa_core.py` | DWA parameters and drone limits. |
| `DroneLidarScanner` | `scanner.py` | Live node: Offboard control + DWA navigation state machine. |
| `X550Drone`, `Barrier`, `Environment`, `Goal` | `dwa_logic.py` | Offline PyBullet prototype. |

### Functions

| Name | Where | Role |
|------|-------|------|
| `scan_to_world_points(...)` | `dwa_core.py` | Turn a 2D LiDAR scan into a local-frame obstacle point cloud. |
| `dwa_control(...)` | `dwa_core.py` | Vectorized holonomic DWA search over the `(vx, vy)` window. |
| `run_dwa_navigation()` | `scanner.py` | Per-tick: build cloud -> plan -> publish velocity setpoint. |

See [documentation.md](docs/documentation.md) for the complete field/method tables.

## Comparison with Other 3D Obstacle-Avoidance Algorithms

HOLO-DWA is a **local, reactive** planner: it searches velocity space every
tick with no map and no global plan. That makes it cheap and map-free, at the
cost of local minima. Where it sits among comparable methods:

| Algorithm | Class | Searches | Needs a map / global plan | Notes vs. HOLO-DWA |
|-----------|-------|----------|---------------------------|--------------------|
| **HOLO-DWA** (this) | Local reactive | `(vx, vy)` velocity window (holonomic) | No | Planar; no yaw coupling; local minima (see discussion.md). |
| Classic DWA (Fox 1997) | Local reactive | `(v, ω)` window | No | Non-holonomic; heading = yaw. HOLO-DWA drops yaw as a planning variable. |
| VFH / VFH+ / 3DVFH+ | Local reactive | Steering direction from an obstacle histogram | No | No velocity dynamics / braking model; 3DVFH+ extends to MAV altitude. |
| Artificial Potential Fields | Local reactive | Force = attractive + repulsive | No | Cheapest; notorious local minima and oscillation. |
| ORCA / RVO2 | Local reactive | Velocity obstacles | No | Strong for many moving agents; assumes reciprocal behavior. |
| TEB / Fast-Planner / EGO-Planner | Optimization | Smooth trajectory over a horizon | Usually yes | Near-optimal, smooth 3D paths; heavier compute and infrastructure. |

Takeaway: HOLO-DWA's niche is a low-compute, map-free reactive layer for a
holonomic multirotor. Its weakness is the local-minimum family analyzed in
[discussion.md](docs/discussion.md); the intended fix is a global planner (A* / RRT)
feeding waypoints on top (see the roadmap in [architecture.md](docs/architecture.md)).

## References

- D. Fox, W. Burgard, S. Thrun, *The Dynamic Window Approach to Collision
  Avoidance*, IEEE Robotics & Automation Magazine, 1997.
- J. Borenstein, Y. Koren, *The Vector Field Histogram — Fast Obstacle
  Avoidance for Mobile Robots*, IEEE T-RA, 1991.
- [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide)
- [Gazebo Harmonic + ROS installation](https://gazebosim.org/docs/harmonic/ros_installation/)
- [gazebosim/ros_gz](https://github.com/gazebosim/ros_gz)
- [PX4/PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models)
- [eProsima Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent)
- Related notes in this repo: [installation.md](docs/installation.md),
  [architecture.md](docs/architecture.md), [documentation.md](docs/documentation.md),
  [discussion.md](docs/discussion.md), [bug.md](docs/bug.md), [note.md](docs/note.md).
