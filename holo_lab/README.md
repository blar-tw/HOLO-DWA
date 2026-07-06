# holo_lab — instrumented + automated HOLO-DWA runner

A self-contained harness for iterating on the **holonomic DWA** config. It
wraps the normal `scanner.py` + `dwa_core.py` flight stack with two things:

1. **Instrumentation** — every 20 Hz control tick is logged to a CSV, and each
   navigation run gets a one-line JSON summary. All data lands in
   [`logs/`](logs/), nothing is written outside this folder.
2. **Automation** — one command flies the drone home and starts a **fresh
   logged run without restarting PX4 / Gazebo / the bridge**. Bringing that
   heavy stack up takes ~30 s; a reset takes a few seconds.

Everything the harness needs is copied in here (`dwa_core.py`,
`scanner_lab.py`, `run_lab.sh`), so it runs independently of the repo-root
`scanner.py` / `run.sh` and never modifies them.

> Scope: this is only the `holonomic DWA` case. No control groups / A-B configs
> yet — just make this one instrumented and repeatable.

## Files

| File | Role |
|------|------|
| `scanner_lab.py` | Instrumented copy of `scanner.py`: per-tick CSV, per-run JSON, `/holo_lab/reset` service, `RETURN_HOME` state, `n_runs` batch loop. |
| `dwa_core.py` | Copy of the planner (imported by `scanner_lab.py`). Edit *this* copy to tune the lab without touching the original. |
| `run_lab.sh` | Launcher (same 4-pane tmux stack as `../run.sh`) + `reset` (restart scanner, drone flies home) / `reset-soft` / `kill` subcommands. Env: `NO_ATTACH=1` (script-friendly), `HEADLESS=1` (no gz GUI). |
| `exp.sh` | Experiment driver on top of `run_lab.sh`: `up` (fresh stack) / `collect <name>` (wait for the batch, archive to `logs/exp/<name>/`) / `go <name>` (reset + collect) / `status` / `down`. |
| `analyze.py` | Dependency-free log reader (per-run table from a `summary.jsonl`, or per-run tick stats from a session CSV). |
| `report.py` | Session CSV → ASCII trajectory map over the arena + per-run diagnostics (collision ticks, infeasible spans, stalls, score stats). No GUI needed. |
| `sim_offline.py` | No-ROS/no-Gazebo offline sim of the same arena (ray-cast LiDAR + first-order velocity tracking) for pre-screening scoring changes in seconds (`--sweep`, `--noise`, `--map`). Not a substitute for Gazebo. |
| `EXPERIMENTS.md` | Iteration log: motivation → change → result → decision, one entry per experiment. |
| `logs/` | All output. `session_*.csv` (per-tick timeseries) + `summary.jsonl` (one line per run) + `exp/<name>/` archives. Git-ignored. |

## Quick start

```bash
cd ~/ws/src/HOLO-DWA/holo_lab

# 1. bring the whole stack up ONCE (default goal 12,0)
./run_lab.sh
./run_lab.sh 8.0 -3.0          # or a custom goal_x goal_y

# 2. iterate: fly home + start a new logged run, stack stays up
./run_lab.sh reset

# 3. read the results
./analyze.py                    # table of all runs from summary.jsonl
./analyze.py logs/session_*.csv # per-run tick stats from one session

# 4. tear everything down
./run_lab.sh kill
```

`reset` **restarts only the scanner node** (PX4 / gz / bridge keep running) and
the fresh node flies the drone home before the next run — see below.

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

There are two ways to start a fresh run without touching the heavy stack:

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

This is the sturdy option: a brand-new process can't get stuck in a bad state,
and the session log is preserved (each restart writes a new `session_*.csv`).
While the scanner is down for ~1 s, PX4's offboard-loss failsafe just holds the
drone in place; the new node then re-engages offboard and brings it home.

**Why not the in-node service?** The obstacles are 5 m tall and the drone flies
at 2 m, so returning means threading back through the same gaps with DWA — which
can stall in a local minimum (see the repo's `docs/discussion.md`). A process
restart sidesteps that failure mode. If you still want the lighter, no-restart
path, it's there as:

### `./run_lab.sh reset-soft` — in-node `/holo_lab/reset` service

Sends `ros2 service call /holo_lab/reset std_srvs/srv/Trigger`. The running node
switches to a `RETURN_HOME` state (DWA toward the origin) and, on arrival, starts
a new run. No process restart, but it can get stuck returning; prefer `reset`.

### Unattended batches vs. reset

`N_RUNS>1` still auto-returns home between runs **inside the one node** (same
`RETURN_HOME` mechanism as `reset-soft`), which is fine when runs succeed. For
hand-driven iteration where a run may end badly, `./run_lab.sh reset` is more
reliable.

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
drone body), that tick is flagged (`collision=1` in the CSV) and, on the rising
edge, counted as one collision **episode**. Per run you get `collided` (bool),
`collisions` (episode count), `min_dist_m` (closest the drone ever got), and
`first_collision` (`{t, pos_ned, min_dist_m}`). Live `[COLLISION] ...` warnings
also print to the scanner pane.

Tune the threshold at launch:

```bash
COLLISION_DIST=0.25 ./run_lab.sh      # stricter contact threshold
```

This is a **proxy**, not ground truth: the 2D LiDAR sits at body height, so it
catches the arena's walls/cylinders/pillars well, but (a) it can miss a hit if
the drone ends up *inside* an obstacle past the sensor's `range_min` (0.1 m),
and (b) it only sees the scan plane. If you later want true contacts, add a
Gazebo `contact` sensor to the `x500_lidar_2d` model and bridge its topic — a
heavier change, deliberately skipped here to keep the instrumentation simple.

## Tuning

Edit `holo_lab/dwa_core.py` (the copy) or the `dwa_config` block near the top of
`scanner_lab.py`, then `./run_lab.sh reset` to fly a fresh run with the change.
Because `reset` restarts the scanner *process*, edits to `dwa_core.py` /
`scanner_lab.py` are picked up (the new process re-imports them) — no PX4/gz
relaunch needed. The goal and `n_runs` are baked into the launch args (and into
the reset helper), so changing those needs a full `./run_lab.sh` relaunch; that
is why `reset` re-uses the same goal.
