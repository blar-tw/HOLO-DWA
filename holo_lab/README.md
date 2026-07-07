# holo_lab — tuning a holonomic DWA obstacle-avoidance planner

`holo_lab` is the **experiment harness and optimized planner** for the
[HOLO-DWA](https://github.com/blar-tw/HOLO-DWA) project: a simulated PX4
multirotor that flies from a start point to a goal through a cluttered arena
using nothing but a **2D LiDAR** and a **holonomic Dynamic Window Approach**
local planner — no global map, no pre-planned path, purely reactive.

This folder is self-contained. It carries its own copy of the planner
(`dwa_core.py`) and flight node (`scanner_lab.py`) plus tooling to fly the
drone in Gazebo, log every control tick, and iterate on the planner's scoring
function **without ever touching the repo-root originals**.

## What problem it solves

The drone must cross an arena and reach the goal at `(12, 0)` while weaving
through three obstacle types in sequence — a **wall with a 1.5 m gap**, a
**three-cylinder slalom**, and a **two-pillar gate** — all inside a boundary
wall. It only ever sees the current LiDAR scan, so it has to decide the next
velocity from local information alone.

```
   y (North)
   +9 ┌────────────────────────────────────────────────┐
      │        ###        ######                        │   # = obstacle
      │        ###        ######                        │   S = start (0,0)
    0 │ S..........  ..........  ..............  E G     │   . = flown path
      │        ###o ###          ######                 │   G = goal (12,0)
      │        wall  cylinders    pillar-gate            │   E = end
   -9 └────────────────────────────────────────────────┘
      x=-4      x=3    x=5-6      x=9        x=12   (East)
```
*(schematic — real trajectory maps are rendered by [`report.py`](report.py))*

## Result

Starting from the repo-root planner as **baseline**, the scoring function was
rewritten over five iterations. Verified across **three independent 5-run
batches, each on a fresh simulator stack**:

| | reached | collisions | min. obstacle distance | avg. time |
|---|---:|---:|---:|---:|
| **baseline** | 1 / 5 | 44+ episodes | contact (0.0 m) | 90 s (mostly timeout) |
| **optimized** | **15 / 15** | **0** | **0.66 m** | **18 s** |

Every control tick fits the 20 Hz budget with ~1.7 ms of compute (3 % of
50 ms). The full before→after story — each failure mode, its root cause, and
the fix — is in **[EXPERIMENTS.md](EXPERIMENTS.md)**.

## How the planner scores a velocity

Each 20 Hz tick, DWA samples the reachable `(vx, vy)` velocities (the "dynamic
window", bounded by acceleration limits), rolls each one forward, discards any
that would hit an obstacle or exceed a safe braking speed, and scores the rest:

```
score = heading_weight   · heading      (cosine of angle to the goal)
      + clearance_weight  · clearance    (obstacle distance along the heading)
      + velocity_weight   · velocity     (goal-directed speed, with a floor)
```

Trajectories that pass near the goal instead get a dominating **terminal
basin** score that steers to the center at a braking-curve speed. The four
changes that took it from 1/5 to 15/15 (planner structure — the `(vx, vy)`
window, feasibility mask, and forward prediction — is untouched):

1. **LiDAR de-mirroring** *(the real root cause)*: a Gazebo `gpu_lidar` scans
   z-up (`+angle` = body **left**) but the planner's frame is NED/FRD
   (`+angle` = body **right**). Consuming the scan raw mirrors every obstacle
   across the body axis, so the drone dodged phantom obstacles straight into
   the real cylinders. Fixed with `flip_y=True` in `scan_to_world_points`.
2. **Direction-based clearance**: the clearance term probes a fixed distance
   along the candidate's heading instead of along its (speed-dependent)
   predicted arc, so it no longer rewards creeping and stops the drone inching
   into obstacles.
3. **Continuous terminal basin + braking-curve arrival**: replaces a binary
   "passed within 0.5 m" bonus that gave zero pull on a fast fly-by, which had
   the drone orbiting the goal forever.
4. **Blended velocity reward**: goal-directed component in open space (kills
   the diagonal drift a raw-speed reward causes) with a scalar floor so the
   drone still slides along walls to find gaps instead of deadlocking.

> ⚠️ The **repo-root `scanner.py` / `dwa_core.py` still carry the LiDAR mirror
> bug** (fix #1). It was left untouched here by scope; port `flip_y` over there
> too.

## Quick start

```bash
cd ~/ws/src/HOLO-DWA/holo_lab

# bring the whole stack up ONCE (default goal 12,0), watch it fly in Gazebo
./run_lab.sh
./run_lab.sh 8.0 -3.0           # or a custom goal_x goal_y

# fly a fresh logged run without relaunching PX4/gz (picks up code edits)
./run_lab.sh reset

# read the results
./analyze.py                    # per-run table (reached / collisions / time)
./report.py                     # ASCII trajectory map of the newest run

# tear everything down
./run_lab.sh kill
```

Prerequisites are the parent project's (ROS 2 Humble, PX4 v1.14.4 + Gazebo,
the `ros_gz` bridge, `tmux`, `numpy`) — see the
[root README](../README.md) and [docs/installation.md](../docs/installation.md).

## Reproduce the verification

```bash
./exp.sh up                     # fresh detached headless stack, N_RUNS=5
EXP_TIMEOUT=700 ./exp.sh collect myrun
cat logs/exp/myrun/report.txt   # per-run table + ASCII trajectory maps
```

Each experiment archives everything (session CSV, summary, report, config
snapshot) under `logs/exp/<name>/`; the twelve committed archives
(`baseline`, `iter1`–`iter5c`, `verifyA/B`, `final5`) are the evidence behind
the results table.

## What each file does

The pieces fit together as: **`dwa_core.py`** (the algorithm) runs inside
**`scanner_lab.py`** (the flight node), driven by **`run_lab.sh`** / **`exp.sh`**
(execution), producing **`logs/`** (data), read back with **`analyze.py`** /
**`report.py`** / **`sim_offline.py`** (analysis). The reasoning is logged in
**`EXPERIMENTS.md`**.

### Core algorithm (the heart of the project)

- **`dwa_core.py`** — **the optimized planner.** Each control tick it samples
  every reachable `(vx, vy)` in the dynamic window, rolls each forward,
  discards any that would hit an obstacle or exceed a safe braking speed, and
  scores the rest with the three weighted terms (heading / clearance /
  velocity) plus the terminal-basin score near the goal. All four winning
  fixes live here (LiDAR `flip_y`, direction-based clearance, terminal basin,
  blended velocity). Pure math, no ROS — so the offline sim can import it
  directly.
- **`scanner_lab.py`** — **the flight node** (largest file). Wires `dwa_core`
  to the real drone: PX4 offboard control (takeoff / navigate / land),
  converting LiDAR scans into obstacle points, the state machine
  (`INIT → TAKEOFF → NAVIGATE → RETURN_HOME → HOLD`), and **all logging**
  (per-tick CSV, per-run JSON). The `dwa_config` block near the top is where
  you set the parameters (`v_max`, the weights, `robot_radius`, …).

### Run / experiment flow

- **`run_lab.sh`** — **launcher.** Brings up a 4-pane tmux stack (PX4+Gazebo /
  XRCE agent / LiDAR bridge / flight node). Subcommands: `reset` (restart just
  the node — drone flies home and starts a fresh run, picking up code edits),
  `kill`, `gui` (attach a Gazebo window to watch). Env vars: `HEADLESS=1`,
  `NO_ATTACH=1`, `N_RUNS`, `RUN_TIMEOUT`.
- **`exp.sh`** — **experiment driver** (for batches). Sits on top of
  `run_lab.sh`: `up` brings up a fresh stack, `collect <name>` waits for the
  batch to finish and archives everything into `logs/exp/<name>/`. The whole
  baseline→iter5 verification was run through this.

### Analysis tools (judge a run without opening the GUI)

- **`analyze.py`** — **results table.** Prints `summary.jsonl` as one row per
  run (reached / collisions / time / min distance), or tick stats for a
  session CSV. Zero dependencies.
- **`report.py`** — **ASCII trajectory map.** Renders a session CSV as a text
  map of the arena: `S` start, `.` path, `X` collision, `!` infeasible,
  `G` goal, plus collision / stall diagnostics. The main tool for telling a
  good flight from a bad one without watching Gazebo.
- **`sim_offline.py`** — **offline sim (seconds, not minutes).** No ROS / no
  Gazebo — a ray-cast LiDAR + first-order velocity model running `dwa_core` in
  the same arena. `--sweep` compares several parameter sets at once, `--noise`
  stress-tests robustness. Pre-screen a scoring change here before spending
  minutes verifying it in Gazebo. Not a substitute for Gazebo.
- **`verify_frame.py`** — **LiDAR handedness check.** Grabs one real scan,
  converts it both ways, and scores each against the known arena geometry.
  This is what proved the mirror bug (fix #1): 0.08 m vs 0.32 m.

### Data & docs

- **`EXPERIMENTS.md`** — the full iteration log (motivation → change → result →
  decision). Read this to understand *why* each change was made.
- **`README.md`** — this file.
- **`logs/exp/<name>/`** — the twelve committed experiment archives
  (`baseline`, `iter1`–`iter5c`, `verifyA/B`, `final5`), each with its session
  CSV, summary, report, and config snapshot — the evidence behind the results
  table. Loose `logs/*.csv` / `summary.jsonl` are scratch and git-ignored.

## Unattended batches

Set `N_RUNS` (and optionally `RUN_TIMEOUT`) at launch to fly several runs
back-to-back with no keystrokes — the drone auto-returns home and re-navigates
between runs, logging each one:

```bash
N_RUNS=5 RUN_TIMEOUT=60 ./run_lab.sh      # 5 runs, abort any run stuck >60 s
N_RUNS=0 ./run_lab.sh                      # loop forever (Ctrl-C / kill to stop)
COLLISION_DIST=0.25 ./run_lab.sh           # tighter collision threshold (m)
```

- `N_RUNS<=0` → loop forever.
- `RUN_TIMEOUT=0` → never time out (a run stuck in a local minimum blocks the
  batch; give it a timeout for unattended use).
- A manual `./run_lab.sh reset` works at any time, including mid-run.

## How reset works (drone returns to origin, PX4/gz/bridge stay up)

### `./run_lab.sh reset` — restart the scanner node (recommended)

Kills just `scanner_lab.py` and relaunches it in its tmux pane; PX4, Gazebo and
the bridge keep running (no ~30 s relaunch). The fresh node re-arms, and because
the drone is still airborne out at the goal, its startup logic **flies it home
via DWA first**, then starts a new logged run:

```
(fresh node)  INIT → TAKEOFF(climb in place) ──not at origin?──► RETURN_HOME
                                    already at origin │            │ (DWA home)
                                                      ▼            ▼
                                                   NAVIGATE ◄─ back at origin
```

A brand-new process can't get stuck in a bad state, and edits to
`dwa_core.py` / `scanner_lab.py` are picked up on restart (the new process
re-imports them) — no PX4/gz relaunch needed. While the scanner is down for
~1 s, PX4's offboard-loss failsafe just holds the drone in place.

### `./run_lab.sh reset-soft` — in-node `/holo_lab/reset` service

Sends `ros2 service call /holo_lab/reset std_srvs/srv/Trigger`. The running node
switches to a `RETURN_HOME` state (DWA toward the origin) and, on arrival, starts
a new run. No process restart, but it can get stuck returning; prefer `reset`.

## What gets logged

`logs/session_<timestamp>.csv` — one row per control tick:

`t, run_id, state, pos_n, pos_e, alt, vel_n, vel_e, yaw, cmd_vx, cmd_vy, speed,
target_n, target_e, dist_to_target, front_dist, min_threat, min_threat_angle,
collision, ok, n_obs, compute_ms, heading_score, clearance_score,
velocity_score, heading_term, clearance_term, velocity_term, total, safe_dist`

`logs/summary.jsonl` — one JSON object per run:

`run_id, outcome (reached/timeout/reset), goal_ned, start_wall, duration_s,
ticks, infeasible_ticks, path_length_m, straight_line_m, path_efficiency,
min_clearance_m, max_speed_mps, compute_ms_mean, compute_ms_max, end_pos_ned,
end_dist_to_goal_m, collided, collisions, min_dist_m, first_collision`

`compute_ms` is the wall time of each `dwa_control()` call — handy for checking
the search still fits the 20 Hz (50 ms) budget.

### Collision recording

Every run records whether it hit anything. The signal is the **nearest LiDAR
return**: when it drops below `collision_dist` (default **0.3 m**, roughly the
drone body), that tick is flagged (`collision=1`) and, on the rising edge,
counted as one collision **episode**. Per run you get `collided`, `collisions`
(episode count), `min_dist_m` (closest the drone ever got), and
`first_collision`. This is a **proxy**, not a Gazebo contact sensor: the 2D
LiDAR sits at body height, so it catches walls/cylinders/pillars well but only
sees the scan plane.

## Tuning

Edit `dwa_core.py` (the scoring/`Config` defaults) or the `dwa_config` block in
`scanner_lab.py`, then `./run_lab.sh reset` to fly a fresh run with the change
— or pre-screen it offline first with `./sim_offline.py --sweep`. The goal and
`n_runs` are baked into the launch args, so changing those needs a full
`./run_lab.sh` relaunch.
