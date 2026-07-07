# holo_lab — tuning a holonomic DWA planner

The **experiment harness and optimized planner** for the
[HOLO-DWA](https://github.com/blar-tw/HOLO-DWA) project. Self-contained: its own
copy of the planner (`dwa_core.py`) and flight node (`scanner_lab.py`), plus
tooling to fly the PX4 multirotor in Gazebo, log every control tick, and iterate
on the scoring function **without touching the repo-root originals**.

The drone crosses a cluttered arena to a goal at `(12, 0)` on **2D LiDAR only**
— no map, no global path — through a wall (1.5 m gap), a three-cylinder slalom,
and a two-pillar gate:

```
   y (North)
   +9 ┌────────────────────────────────────────────────┐
      │        ###        ######                        │   # = obstacle
      │        ###        ######                        │   S = start (0,0)
    0 │ S..........  ..........  ..............  E G     │   . = flown path
      │        ###o ###          ######                 │   G = goal (12,0)
   -9 └────────────────────────────────────────────────┘
      x=-4      x=3    x=5-6      x=9        x=12   (East)
```
*(schematic — real trajectory maps come from [`report.py`](report.py))*

## Result — baseline vs. tuned

From the repo-root planner as **baseline**, the scoring was rewritten over five
iterations and verified across **three independent 5-run batches, each on a
fresh simulator stack**:

| | reached | collisions | min. obstacle dist. | avg. time |
|---|---:|---:|---:|---:|
| **baseline** | 1 / 5 | 44+ episodes | contact (0.0 m) | 90 s (mostly timeout) |
| **tuned** | **15 / 15** | **0** | **0.66 m** | **18 s** |

Each 20 Hz tick, DWA samples the reachable `(vx, vy)`, rolls each forward, drops
any that would hit an obstacle or exceed a safe braking speed, and scores the
rest:

```
score = heading_weight   · heading    (cosine of angle to the goal)
      + clearance_weight · clearance  (obstacle distance along the heading)
      + velocity_weight  · velocity   (goal-directed speed, with a floor)
```

plus a dominating **terminal-basin** score near the goal. The `(vx, vy)` window,
feasibility mask, and forward prediction are **identical to the baseline**
(~1.7 ms/tick, 3 % of the 50 ms budget) — only the scoring, one sensing fix, and
a few limits differ:

| | baseline (root) | tuned (`holo_lab`) |
|---|---|---|
| LiDAR frame | mirror bug (no `flip_y`) | `flip_y=True` — gz z-up/+left vs PX4 NED/+right |
| Clearance | min dist along the *predicted path* (speed-coupled → rewards creeping) | 1.5 m probe along the *direction*, speed-independent (`clearance_norm=0.5`) |
| Terminal | binary bonus + raw speed → fly-by orbits | continuous basin (`goal_capture=2.0`) + braking-curve speed (`goal_approach_a=0.5`) |
| `robot_radius` | 0.2 m (< 0.3 m collision proxy → guaranteed hit) | 0.30 m |
| Weights H/C/V, mode | 0.2/0.2/0.6, `scalar` | 0.3/0.3/0.4, `blend` |

The four fixes, in the order they were found: **(1) LiDAR de-mirroring** — the
real root cause; a raw scan mirrors every obstacle, so the drone dodged phantoms
straight into the real cylinders. **(2) direction-based clearance** — stops it
inching into obstacles. **(3) continuous terminal basin** — a binary "within
0.5 m" bonus gave zero pull on a fast fly-by, so it orbited the goal forever.
**(4) blended velocity** — goal component in the open kills diagonal drift; a
scalar floor keeps wall-sliding so it doesn't deadlock. Full before→after story:
**[EXPERIMENTS.md](EXPERIMENTS.md)**.

> **Note:** the repo-root `scanner.py` / `dwa_core.py` still carry the LiDAR
> mirror bug (fix 1), left untouched by scope — port `flip_y` over there too.

## Quick start

```bash
cd ~/ws/src/HOLO-DWA/holo_lab
./run_lab.sh                # bring the stack up once, fly to goal (12,0) in Gazebo
./run_lab.sh 8.0 -3.0       # custom goal_x goal_y
./run_lab.sh reset          # fresh logged run without relaunching PX4/gz (picks up edits)
./analyze.py                # per-run table (reached / collisions / time)
./report.py                 # ASCII trajectory map of the newest run
./run_lab.sh kill           # tear down
```

Prerequisites are the parent project's (ROS 2 Humble, PX4 v1.14.4 + Gazebo, the
`ros_gz` bridge, `tmux`, `numpy`) — see the [root README](../README.md) and
[docs/installation.md](../docs/installation.md).

Reproduce the verification (detached headless batch):

```bash
./exp.sh up                       # fresh stack, N_RUNS=5
EXP_TIMEOUT=700 ./exp.sh collect myrun
cat logs/exp/myrun/report.txt     # per-run table + ASCII maps
```

## Files

| File | Role |
|------|------|
| `dwa_core.py` | The optimized planner — `(vx, vy)` window + scoring (all four fixes). Pure math, no ROS. |
| `scanner_lab.py` | Flight node: PX4 offboard state machine (`INIT→TAKEOFF→NAVIGATE→RETURN_HOME→HOLD`), LiDAR→points, all logging. `dwa_config` block sets the params. |
| `run_lab.sh` | Launcher — 4-pane tmux stack. Subcommands `reset` / `kill` / `gui`; env `HEADLESS`, `N_RUNS`, `RUN_TIMEOUT`. |
| `exp.sh` | Batch driver on top of `run_lab.sh`: `up` + `collect <name>` archives to `logs/exp/<name>/`. |
| `analyze.py` | Per-run results table from `summary.jsonl`. |
| `report.py` | ASCII trajectory map from a session CSV (`S`/`.`/`X`/`!`/`G` + diagnostics). |
| `sim_offline.py` | ROS-free ray-cast sim (seconds); `--sweep` / `--noise` to pre-screen a change before Gazebo. |
| `verify_frame.py` | LiDAR handedness check — proved the mirror bug (0.08 m vs 0.32 m). |
| `EXPERIMENTS.md` | Full iteration log (motivation → change → result → decision). |
| `logs/exp/<name>/` | Twelve committed archives (`baseline`, `iter1`–`iter5c`, `verifyA/B`, `final5`) — the evidence. Loose `logs/*.csv` are git-ignored scratch. |

## Running notes

- **Batches:** `N_RUNS=5 RUN_TIMEOUT=60 ./run_lab.sh` flies 5 runs back-to-back
  (auto-returns home between runs); `N_RUNS=0` loops forever; `RUN_TIMEOUT=0`
  never times out.
- **Reset:** `./run_lab.sh reset` restarts just the flight node (PX4/gz/bridge
  stay up, ~1 s); the fresh process flies home via DWA, then starts a new run,
  picking up any `dwa_core.py` / `scanner_lab.py` edits.
- **Collision proxy:** nearest LiDAR return < `collision_dist` (0.3 m) flags a
  tick; rising edges are counted as episodes. A 2D-scan-plane proxy, not a
  Gazebo contact sensor.
- **Tuning:** edit `dwa_core.py` or the `dwa_config` block, then `./run_lab.sh
  reset` — or pre-screen offline with `./sim_offline.py --sweep`. Goal / `n_runs`
  are launch args, so changing those needs a full relaunch.
- **Logged data:** per-tick `logs/session_*.csv` (pose, cmd, scores,
  `compute_ms`, collision) and per-run `logs/summary.jsonl` (outcome, path
  efficiency, `min_dist_m`, collisions).
