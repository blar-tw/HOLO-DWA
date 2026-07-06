# Architecture

How the pieces fit together, what runs where, and which coordinate frame each
value lives in. For the algorithm-level design trade-offs (local minima, the
velocity-reward tug-of-war, the doorway problem) see [discussion.md](discussion.md).

## 1. System overview

The stack is four processes talking over ROS 2 / Gazebo Transport. Only
[`scanner.py`](../scanner.py) is our own node; everything else is off-the-shelf
PX4 / Gazebo / ROS tooling, wired together by [`run.sh`](../run.sh).

```
 Gazebo (PX4 SITL, world: dwa_test)
   x500_lidar_2d model  ──►  gpu_lidar sensor (lidar_2d_v2)
        │  gz.msgs.LaserScan on
        │  /world/<world>/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan
        ▼
 ros_gz_bridge  ──►  sensor_msgs/msg/LaserScan  (remapped to ROS topic /lidar)
        │
        ▼
 scanner.py  (rclpy node "scanner", 20 Hz control loop)
        │   subscribes: /lidar (LaserScan), /fmu/out/vehicle_odometry
        │   runs:       dwa_core.dwa_control() over the (vx, vy) window
        │   publishes:  /fmu/in/offboard_control_mode
        │               /fmu/in/trajectory_setpoint
        │               /fmu/in/vehicle_command
        ▼
 Micro XRCE-DDS Agent  ◄──►  PX4 SITL (uORB)
        │
        ▼
 PX4 flight controller  ──►  actuates the multirotor in Gazebo
```

The XRCE-DDS Agent is the uORB <-> ROS 2 shuttle: it is how PX4's
`/fmu/in/*` and `/fmu/out/*` topics reach `scanner.py`.

## 2. Coordinate frames

Two frames are in play, and getting them wrong sends the drone the wrong way.

| Frame | Used by | Axes |
|-------|---------|------|
| Gazebo world (ENU) | `dwa_test.sdf`, the `goal_x`/`goal_y` params | x = East, y = North |
| PX4 local (NED) | `scanner.py` internals, `vehicle_odometry` | x = North, y = East |

The goal is entered in **Gazebo world coordinates** (same frame as the
obstacles in the SDF). Assuming the drone spawns at the world origin, the two
frames share an origin and the mapping is a pure axis swap:

```
North = gz_y      East = gz_x
```

This is `DroneLidarScanner.get_goal_ned()`. The LiDAR point cloud and the
odometry are already in the same local frame, so DWA does all of its search in
that one frame.

## 3. Navigation state machine

`scanner.py` runs a small state machine inside the 20 Hz timer callback. Every
tick it also republishes the Offboard heartbeat (PX4 disarms without it).

```
INIT ──(20 heartbeats sent)──► TAKEOFF ──(reached takeoff_alt)──► NAVIGATE ──(within goal_threshold)──► GOAL_REACHED
```

- **INIT** — send heartbeats; after 20, switch to Offboard mode and arm.
- **TAKEOFF** — climb in place to `takeoff_alt` (-2.0 m NED = 2 m up) while
  yawing to face the goal, so navigation starts pointed at the target (also
  aligns the LiDAR's forward coverage).
- **NAVIGATE** — run one DWA search per tick; command XY velocity, hold Z by
  position, keep the nose on the goal.
- **GOAL_REACHED** — hold the position/heading captured at arrival (instead of
  chasing live odometry, which would slowly drift).

Altitude is always position-controlled; only XY is velocity-controlled, and
only during NAVIGATE (`OffboardControlMode.velocity` is toggled accordingly).
XY velocity setpoints use PX4's per-axis NaN passthrough: position is
`[NaN, NaN, z]` and velocity is `[vx, vy, NaN]`.

## 4. The DWA loop

Each NAVIGATE tick, `run_dwa_navigation()`:

1. Converts the latest LiDAR scan into a local-frame obstacle point cloud
   (`dwa_core.scan_to_world_points`, subsampled by `lidar_stride = 6`, so
   ~180 of the 1080 rays).
2. Builds the **dynamic window**: a grid of `(vx, vy)` candidates centered on
   the current velocity, bounded by acceleration limits (`a_max * control_dt`)
   and the absolute velocity limits.
3. Rolls each candidate forward `predict_time` seconds at constant velocity
   (holonomic straight-line trajectories — no yaw coupling).
4. Rejects candidates that are **infeasible**:
   - trajectory passes within `robot_radius` of an obstacle, or
   - speed exceeds the safe braking speed `sqrt(2 * safe_dist * brake_a_max)`.
5. Scores the survivors and takes the argmax:

   ```
   score = heading_weight   * heading      (cosine of velocity vs. goal direction, [-1, 1])
         + clearance_weight  * clearance    (min obstacle clearance, clipped to [0, 1])
         + velocity_weight   * velocity     (speed reward, [0, 1])
   ```

   The whole search is vectorized with NumPy so it fits inside the 20 Hz loop.

If no candidate is feasible, the node brakes/holds. The `heading` term is
distance-independent (an angle cosine, not a progress ratio) — this is a
deliberate fix for a runaway failure mode documented in
[discussion.md](discussion.md) section 3.

### Velocity-reward modes

`Config.velocity_mode` selects what "velocity" rewards. It is the single
biggest behavioral knob and the subject of [discussion.md](discussion.md)
section 1:

| Mode | Rewards | Open space | In front of a wall |
|------|---------|-----------|--------------------|
| `scalar` (default) | raw speed `\|v\|` | slight diagonal drift | slides along, finds the gap |
| `component` | goal-directed speed only | flies straight | can deadlock (sideways escape scores 0) |
| `blend` | `max(component, α·scalar)` | straight | keeps a motion floor when blocked |

## 5. Key parameters

`scanner.py` overrides the `dwa_core.Config` defaults for the live drone
(slower and more conservative than the offline prototype):

| Parameter | Live (`scanner.py`) | Meaning |
|-----------|---------------------|---------|
| `v_max`, `vx/vy_min/max` | 1.5, ±1.5 | speed / per-axis velocity limits (m/s) |
| `a_max`, `brake_a_max` | 1.0, 1.0 | window accel limit / braking limit (m/s^2) |
| `control_dt` | 0.2 | window horizon for the accel bound (s) |
| `predict_time`, `predict_dt` | 3.0, 0.2 | trajectory rollout horizon / step (s) |
| `vx/vy_resolution` | 0.1 | velocity-grid resolution (m/s) |
| `robot_radius` | 0.2 | inflation radius (m) |
| `goal_threshold` | 0.5 | arrival radius (m) |
| `heading/clearance/velocity_weight` | 0.2 / 0.5 / 0.3 | score weights (defaults, not overridden) |
| `velocity_mode` | `scalar` | velocity-reward mode (default) |

Node parameters (ROS 2): `goal_x`, `goal_y` (Gazebo world coords, default
`12.0, 0.0`).

LiDAR sensor (`lidar_2d_v2`): 1080 rays, 270 deg FOV (±135 deg), 0.1–30 m range,
30 Hz.

## 6. Simulation world (`dwa_test.sdf`)

The test arena is a corridor the drone crosses along +x, meeting three obstacle
types in sequence before the goal at `(12, 0)`:

```
 x=-4  back / boundary wall
 x=3   wall with a gap at y=0            (obstacle type 1)
 x=6   three staggered cylinders (slalom) (obstacle type 2)
 x=9   two box pillars with a gap at y=0  (obstacle type 3)
 x=12  goal marker (visual only)
 x=16  front boundary wall
        arena enclosed in y ∈ [-9, 9]
```

Adding your own obstacles (copy-paste SDF blocks, or spawn at runtime with
`gz service`) is described in [note.md](note.md).

## 7. Roadmap

- [x] DWA velocity planning wired into `scanner.py` (replaced hover-only logic).
- [x] Holonomic DWA algorithm consolidated into `dwa_core.py` with notes.
- [ ] `package.xml` + launch file so the whole stack comes up with
      `ros2 launch`, replacing the tmux script.
- [ ] Hardware / real-world validation: LiDAR mounting offset (currently
      assumed coincident with the body origin), and the goal-to-world
      coordinate mapping.
- [ ] Add a global planner (A* / RRT) above DWA to remove the local-minimum
      failure modes in [discussion.md](discussion.md).
