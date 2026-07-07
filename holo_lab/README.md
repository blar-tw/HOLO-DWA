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
snapshot) under `logs/exp/<name>/`; the ten committed archives
(`baseline`, `iter1`–`iter5c`, `verifyA/B`, `final5`) are the evidence behind
the results table.

## Files

| File | Role |
|------|------|
| `dwa_core.py` | **The optimized planner.** Vectorized holonomic DWA search + scoring. Frame-agnostic, no ROS. |
| `scanner_lab.py` | Flight node: PX4 offboard control, LiDAR intake, the `dwa_config` block, per-tick CSV + per-run JSON logging, `RETURN_HOME` / batch automation. |
| `run_lab.sh` | Launcher (4-pane tmux: PX4+gz / agent / bridge / node) + `reset` / `kill` / `gui` / `play` subcommands. Env: `HEADLESS=1`, `NO_ATTACH=1`, `N_RUNS`, `RUN_TIMEOUT`, `RECORD=1`. |
| `exp.sh` | Experiment driver: `up` / `collect <name>` / `go <name>` / `status` / `down`. Archives each batch to `logs/exp/`. |
| `analyze.py` | Dependency-free reader: per-run table from a `summary.jsonl`, or tick stats from a session CSV. |
| `report.py` | Session CSV → ASCII arena map with the flown path + diagnostics (collisions, infeasible spans, stalls, score stats). No GUI needed. |
| `sim_offline.py` | No-ROS/no-Gazebo sim of the same arena (ray-cast LiDAR + velocity tracking) to pre-screen scoring changes in **seconds** (`--sweep`, `--noise`, `--map`). Not a substitute for Gazebo. |
| `verify_frame.py` | One-shot check that the LiDAR handedness (fix #1) is correct against the known arena geometry. |
| `replay_demo.py` + `replay_pose.cpp` | Puppet-replays a logged flight in a Gazebo GUI for demo recording — see [Recording a demo video](#recording-a-demo-video-wsl2). |
| `EXPERIMENTS.md` | The iteration log: motivation → change → result → decision, one entry per experiment. |
| `logs/exp/<name>/` | Committed experiment archives. Loose `logs/*.csv` / `summary.jsonl` are scratch and git-ignored. |

---

## Recording a demo video (WSL2)

Measured on this box: attaching **anything** to the live sim degrades the
flight — the integrated GUI, a `gz sim -g` client, or even `--record` state
logging all turn the clean 18 s run into 55–90 s of wandering (same-day
control: pure headless 18.0 s / 11.9 m). So never record the flight live;
replay it afterwards, where render load can't affect the already-flown path:

```bash
# 1. fly ONE clean run, headless (the per-tick CSV is the recording)
NO_ATTACH=1 HEADLESS=1 N_RUNS=1 ./run_lab.sh
./analyze.py                 # wait for "reached", sanity-check the run
./run_lab.sh kill

# 2. replay the trajectory in a Gazebo GUI
./replay_demo.py --loop      # newest CSV; add --speed 0.5 for slow-mo
# in the GUI: right-click the drone -> Follow, then screen-record (Win+G)
```

`RECORD=1 ./run_lab.sh` + `./run_lab.sh play` (native gz state-log playback)
also exist, but the recording overhead itself spoils the flight being
recorded — prefer `replay_demo.py`.

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
