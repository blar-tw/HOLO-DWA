# Code Documentation

Reference for the classes and functions in the repo. For how they fit together
at runtime, see [architecture.md](architecture.md).

## Module map

| File | Role |
|------|------|
| [`dwa_core.py`](../dwa_core.py) | Holonomic DWA algorithm. No ROS / PyBullet deps — imported by both the live node and (in spirit) the prototype. |
| [`scanner.py`](../scanner.py) | Live ROS 2 node: PX4 Offboard control + LiDAR-driven DWA navigation. |
| [`dwa_logic.py`](../archive/dwa_logic.py) | Offline PyBullet prototype / visualizer. Same DWA idea against analytic obstacle shapes. |
| [`run.sh`](../run.sh) | One-shot tmux launcher for the full simulation stack. |
| [`gz_extra/`](../gz_extra/) | Gazebo models, airframe, and the `dwa_test` world missing from PX4 v1.14.4; `install.sh` copies them into PX4-Autopilot. |

> Note: earlier docs referred to the prototype as `dwa.py`; the file is
> actually `dwa_logic.py`. `dwa_core.py` is the ROS-free extraction shared by
> the live pipeline.

## Structs / Classes

### `dwa_core.Config`

Plain container of DWA parameters and drone limits. See the parameter table in
[architecture.md](architecture.md#5-key-parameters) for the values used live.

| Field | Default | Meaning |
|-------|---------|---------|
| `v_max` | 10.0 | max speed magnitude (m/s) |
| `vx_min/max`, `vy_min/max` | ±10.0 | per-axis velocity limits (m/s) |
| `a_max` | 5.0 | dynamic-window acceleration limit (m/s^2) |
| `brake_a_max` | 5.0 | braking limit for the admissible-velocity test (m/s^2) |
| `vx_resolution`, `vy_resolution` | 0.1 | velocity-grid step (m/s) |
| `control_dt` | 0.2 | horizon used for the acceleration bound (s) |
| `predict_time`, `predict_dt` | 2.0, 0.1 | trajectory rollout horizon / step (s) |
| `heading_weight` | 0.2 | weight on the heading (goal-cosine) score |
| `clearance_weight` | 0.2 | weight on the obstacle-clearance score |
| `velocity_weight` | 0.6 | weight on the speed score |
| `velocity_mode` | `"scalar"` | `scalar` / `component` / `blend` (see below) |
| `blend_alpha` | 0.5 | scalar floor used by `blend` mode |
| `clearance_lookahead` | 0.0 | >0 switches clearance to a direction probe of this length (m); 0 = legacy path-based |
| `clearance_norm` | 1.0 | clearance saturation: safe margin (m) that scores 1.0 |
| `robot_radius` | 0.2 | inflation radius (m) |
| `goal_threshold` | 0.3 | arrival radius (m) |
| `goal_capture` | 2.0 | terminal-basin radius (m): candidates passing this close to the goal use the basin score |
| `goal_approach_a` | 0.5 | deceleration (m/s^2) for the terminal approach speed curve |

`velocity_mode` trade-offs: `scalar` rewards raw speed (drifts diagonally in
open space, but escapes walls by sliding); `component` rewards only
goal-directed speed (flies straight, but can deadlock at walls); `blend` is
`max(component, blend_alpha * scalar)`. Full analysis in
[discussion.md](discussion.md) section 1.

### `scanner.DroneLidarScanner(Node)`

The live rclpy node. Owns the PX4 publishers/subscribers, the 20 Hz control
timer, the navigation state machine, and one `dwa_core.Config`.

- **Publishers**: `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`,
  `/fmu/in/vehicle_command`.
- **Subscribers**: `/lidar` (and the raw gz scan topic) -> `lidar_callback`;
  `/fmu/out/vehicle_odometry` -> `odom_callback`.
- **Parameters**: `goal_x`, `goal_y` (Gazebo world coords, default `12.0, 0.0`).
- **State**: `nav_state` in `INIT -> TAKEOFF -> NAVIGATE -> GOAL_REACHED`.

### Prototype classes (`dwa_logic.py`)

Only used by the offline PyBullet visualizer:

- `X550Drone` — simplified holonomic point-mass; `step(vx, vy)` integrates a
  world-frame velocity command, yaw is visual only.
- `Barrier` — analytic obstacle (`cylinder` / `box`) with a signed
  `distance(point)` used instead of a LiDAR point cloud.
- `Environment` — collection of `Barrier`s; `min_distance(point)` over all.
- `Goal` — target position with a `reached(pos)` threshold test.

## Functions

### `dwa_core.scan_to_world_points(ranges, angle_min, angle_increment, range_min, range_max, robot_x, robot_y, yaw, stride=1, flip_y=False) -> (N, 2) array`

Convert a body-frame 2D LiDAR scan into local/world-frame obstacle points.
Angle 0 is straight ahead (sensor +x); `yaw` rotates the body frame into the
robot's frame; `stride` subsamples rays to keep the cloud small. `flip_y=True`
negates the scan angle to fix the gz `gpu_lidar` z-up (+angle = left) vs PX4
NED/FRD (+angle = right) handedness — the live node passes it. Invalid /
out-of-range returns are dropped; returns `(0, 2)` if nothing is seen.

### `dwa_core.dwa_control(state, goal_xy, obstacle_points, config, return_debug=False) -> (vx, vy, ok[, debug])`

The vectorized holonomic DWA search.

- `state`: dict with `x`, `y`, `vx`, `vy` in the local frame.
- `goal_xy`: `(x, y)` target in the same frame.
- `obstacle_points`: `(K, 2)` cloud, or `(0, 2)` if empty.
- Returns `(vx, vy, ok)`. `ok` is `False` when no admissible velocity survives
  (every candidate hits an obstacle or exceeds safe braking speed) — the caller
  should hold/brake. With `return_debug=True`, also returns a score breakdown
  for logging (sub-scores + weighted terms), or `None` when `ok` is `False`.

### `scanner.DroneLidarScanner` methods

| Method | What it does |
|--------|--------------|
| `timer_callback()` | 20 Hz loop: heartbeat + state machine. |
| `odom_callback(msg)` | Update position / velocity / yaw / altitude from `VehicleOdometry`. |
| `lidar_callback(msg)` | Store latest scan; compute nearest-threat and front distance for status logging. |
| `get_goal_ned()` | Map the Gazebo-world goal (ENU) into PX4 NED (axis swap). |
| `yaw_to_goal(goal_n, goal_e)` | NED heading from current position to the goal. |
| `run_dwa_navigation()` | Build the obstacle cloud, call `dwa_control`, publish the velocity setpoint, throttle a status line. |
| `publish_offboard_control_heartbeat(use_velocity)` | Offboard heartbeat; toggles velocity control for NAVIGATE. |
| `publish_position_setpoint(x, y, z, yaw)` | Position-controlled setpoint. |
| `publish_velocity_setpoint(vx, vy, z, yaw)` | XY velocity + Z position (per-axis NaN passthrough). |
| `publish_vehicle_command(command, p1, p2)` | Low-level `VehicleCommand` (mode switch, arm). |

### `dwa_logic.py` functions (prototype)

`predict_trajectory`, `calculate_traj_distance`, `compute_score`,
`dwa_control` — the same DWA structure as `dwa_core`, but scoring against
analytic `Environment` distances instead of a LiDAR cloud, and using a
progress-ratio heading term (the older formulation `dwa_core` replaced with the
distance-independent angle cosine).
