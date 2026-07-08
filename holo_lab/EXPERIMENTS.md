# HOLO-DWA scoring system iteration log

> **TL;DR (final result)**: baseline 1/5 reached, 44+ collisions → **iter4:
> 5/5 reached, 0 collisions, min distance 0.66m, average 16.8s**. Four key fixes:
> ① velocity switched to blend (fixes open-area diagonal drift) ② clearance switched to
> directional 1.5m probing (fixes the creeping trap) ③ **LiDAR point-cloud mirroring bug**
> (gz FLU vs PX4 FRD, flip_y=True — the real root cause behind all the earlier failures)
> ④ goal bonus switched to a braking curve (fixes orbiting around the goal).
> DWA's (vx,vy) dynamic window, feasibility mask, and prediction mechanism were all left untouched.

Goal: optimize the scoring system so the drone **clearly avoids all obstacles** (0 collisions) and can be verified repeatably.
Constraint: don't change DWA's (vx, vy) dynamic-window search structure; only touch the scoring core / parameters / scanner-side config.
Scenario: `dwa_test.sdf` — wall at x=3 (1.5m gap @y=0) → three-cylinder slalom at x=6 → boxed gate at x=9 (2m gap) → goal (12, 0).
Verification: `./exp.sh up` + `./exp.sh go <name>`, N_RUNS=5 per round, RUN_TIMEOUT=90s, check
`logs/exp/<name>/report.txt` (run statistics + ASCII trajectory plot).

Judgment order (comparing good vs. bad):
1. Number of runs with collisions (most important, must be 0)
2. Number reached (5/5 counts as passing)
3. min_dist_m (minimum lidar distance across the whole batch, >0.4m to feel safe)
4. Average time / path_efficiency (secondary)

## Tools

- `exp.sh up|go|collect|status|down` — experiment driver (see the file header for details).
- `report.py` — session CSV → ASCII trajectory plot + collision/infeasible/stall diagnostics.
- Each run is archived under `logs/exp/<name>/`: summary.jsonl, session CSV, report.txt,
  dwa_core.py.snap, node_config.txt, pane.txt.

## Baseline configuration (baseline, 2026-07-06)

`dwa_core.py`: heading = angular cosine (distance-independent), clearance = clip(safe_dist,0,1),
velocity = scalar mode; weights H/C/V = 0.2/0.2/0.6.
`scanner_lab.py`: v_max=1.5, a_max=1.0, predict 3.0s@0.2s, robot_radius=0.2,
goal_threshold=0.5, collision_dist=0.3, lidar_stride=6.

Known going in (from docs/discussion.md and earlier logs):
- The scalar velocity reward drifts diagonally during acceleration; heading has already been
  switched to the angular form, which should theoretically suppress this — to be verified.
- Previously observed: getting stuck in the cylinder area (repeated infeasible) → rushing
  north → hitting the north boundary wall @gz(5.1, 8.0).
- robot_radius (0.2) < collision_dist (0.3): the planner is allowed to get as close as 0.2m,
  but anything within 0.3m is logged as a collision — this inconsistency alone will manufacture
  "collision" records. A candidate fix.

---

## Experiment log

(Each round: motivation → change → result → decision)

### exp 0 — baseline (2026-07-06 21:30)

Config: same as "Baseline configuration." `logs/exp/baseline/`.

Result: **major failure (as expected)** — 5 runs: reached 1 (with 4 collisions), timeout 4;
44+ collision events; max_speed hit 12.4 m/s (v_max=1.5 → hit the wall and got bounced by
physics); infeasible-tick ratio 15–30%.

Mechanisms found by per-tick analysis (session CSV):
1. **Square-window corner degeneracy**: during acceleration, the |v| bonus of a corner
   candidate (0.6·Δ|v|/1.5) ≈ the heading-cosine penalty (0.2·Δcos), so the argmax flips
   from noise → random diagonal drift (see t=12.1 turning from straight to diagonal). Once
   off-axis, the drone faces a wall segment instead of the gap → gets stuck at the wall →
   hugs the wall → collides.
2. **min_clearance=0.007m**: the clip(safe,0,1)·0.2 penalty is too weak to counter the 0.6
   incentive from velocity, so the planner actively chooses zero-margin trajectories.
3. **radius(0.2) < collision_dist(0.3)**: structurally guaranteed collision. The very first
   collision happens while going through the gap, hugging the edge at 0.279m (t=5.5,
   gz(2.57, 0.21)).
4. After being bounced off a wall, |v| > v_max → all candidates become infeasible → brake
   command, PX4 slowly pulls it back, but it can't recover within 90s → timeout. The real
   fix is not hitting the wall in the first place.

Decision: iter1 = {robot_radius 0.30, velocity_mode blend (α0.5),
clearance_norm 0.5, weights H/C/V 0.3/0.3/0.4} (each addressing mechanisms 1–3).

### exp 1 — iter1: blend + norm0.5 + r0.3 + weights 334 (2026-07-06 21:43)

`logs/exp/iter1/` (only collected 2 runs before stopping early — the mechanism was already clear).

Result: partial success, new failure mode emerged.
- ✅ Open-area diagonal drift gone: the path from start to the first gap is a straight line,
  cleanly through the 1.5m gap.
- ❌ run1 reached at 75s but with 12 collisions: **creeps** in front of the cylinder area
  (0.2 m/s), then with heading=1.0 rubs straight into the face of cylinder_2 (t+20.6s,
  gz(6.03,0.19), 0.29m) → grazes it → gets bounced by physics (6.9 m/s) → chaos.
- Reproduced the creeping offline in sim sync: iter1 19.4s vs baseline 12.1s.

Root cause (verified via per-tick scoring): **time-based clearance (minimum distance over
the whole 3-second ray) is coupled with speed** — a slow candidate has a short ray → naturally
high clearance → the C term rewards slowness (ΔC=0.3×0.24 > ΔV=0.4×0.2/1.5), and a slow, short
ray also "can't see" a cylinder 1m away → feasibility can't stop it either → it inches its way
into a dead end step by step. The "creeping trap."

Decision: iter2 = switch clearance to **directional**: along the candidate's unit direction,
take the fixed minimum obstacle distance within 1.5m, independent of speed. Direction quality
(H+C) and speed selection (V + braking cap) are completely decoupled. Offline sweep: iter2
13.5s, min_surf 0.63m, strictly better than iter1 across the board; worst_surf 0.60m across
8 noise seeds is also best. L=1.5 beats 1.0/2.0.

### exp 2 — iter2: directional clearance (2026-07-06 21:52)

`logs/exp/iter2/` (stopped after run 1 — found the actual root cause).

Result: creeping is gone (reaches the cylinder area at 10.7s, vs. 20.6s for iter1), goes
straight through the first gap, but **run 1 still has 7 collisions**, and the first collision
is again straight into the face of cylinder_2.

Tick-level investigation (t+17.5~21.3) found an unexplainable contradiction: candidates
heading toward the cylinder were logged with clearance_score=0.84, but recomputing offline
with the same state and correct field geometry gives all-infeasible. **The point cloud the
node sees doesn't match the real world.**

### 🐛 Root cause: LiDAR scan coordinate-frame handedness mirroring (a sensing bug, not a scoring issue)

- gz's `gpu_lidar` scan angle convention is z-up: **positive angle = toward the body's left** (FLU).
- `scan_to_world_points`'s rotation assumes NED/FRD: **positive angle = toward the body's right**.
- Combined, this **mirrors the entire point cloud across the body's x-axis** (left and right swapped).

Why it didn't blow up worse before this: the wall gap, box gate, and boundaries in the field
are all **left-right symmetric**, so mirroring them barely changes anything; **only the three
cylinders are an asymmetric layout** — so every run's first collision happened precisely
during the slalom: dodging a "phantom cylinder to the south" → turning north → hitting the
real cylinder_2. The unexplained infeasible braking in open areas (15–30% of ticks) was also
caused by phantom walls.

Fix: `scan_to_world_points(..., flip_y=True)` (in the holo_lab copy); the logged threat angle
is converted to FRD accordingly.

Verification: `verify_frame.py` — captures one real scan, transforms it into world coordinates
both with and without the flip, and computes surface-distance statistics against the known
field geometry; the correct handedness should be accurate to centimeter level.

### exp 3 — iter3 = iter2 scoring + flip_y fix (2026-07-06 22:12)

`logs/exp/iter3/` (stopped after run 1 — caught yet another new mechanism).

Result: **obstacle avoidance fully succeeds** — run 1 has 0 collisions throughout, minimum
lidar distance 0.633m, cleanly through all three obstacles — but **timeout: orbits around the
goal, never enters the 0.5m capture circle**.

Root cause: the goal-reached bonus `10000 + speed` rewards the "fastest" trajectory through
the circle → rushes at the goal at v_max → tracking error causes the actual path to graze past
the circle → the ±0.2/tick window takes ~15 ticks to turn around → big loop, retry → an
endless orbit (multiple large loops visible on the map).

Fix history:
- `10000 − speed` (slowest through the circle): offline 20.7s — crawls for the last 4.5m the
  whole way, too conservative.
- **`10000 − |speed − v_target|`, v_target = min(v_max, √(2·brake_a·(d−r)))**
  (a braking curve): full speed far away, naturally slows to ~1.0 m/s before entering the
  circle (PX4 can brake to a stop within 0.17m). Offline 14.1s (vs. 13.5s for the reckless
  version — only 0.6s slower), margin unchanged at 0.63m.

### exp 4 — iter4 = iter3 + braking-curve arrival (2026-07-06 22:09) ✅

`logs/exp/iter4/`.

| run | outcome | dur_s | path_m | eff | col | min_dist | infeasible |
|----:|---------|------:|-------:|----:|----:|---------:|-----------:|
| 1 | reached | 17.6 | 11.9 | 1.01 | 0 | 0.661 | 0/351 |
| 2 | reached | 17.5 | 19.7 | 0.60 | 0 | 0.787 | 0/349 |
| 3 | reached | 16.0 | 19.8 | 0.61 | 0 | 0.754 | 0/318 |
| 4 | reached | 16.4 | 19.8 | 0.62 | 0 | 0.800 | 0/327 |
| 5 | reached | 16.5 | 19.9 | 0.62 | 0 | 0.785 | 0/330 |

**5/5 reached, 0 collisions, minimum lidar distance across the whole batch 0.661m, average
16.8s, 0 infeasible ticks (not a single emergency brake), compute ~1.7ms.**

Two valid route types:
- run 1 (cold start, zero initial velocity): **straight through all three obstacles** — gap →
  the c1–c2 corridor going south around cylinder_2 → box gate → goal, path 11.9m, near-optimal.
- runs 2-5 (after RETURN_HOME, starting with residual velocity): **wide arc to the north**,
  taking the 3m open corridor beyond the wall's north end (y∈[6,9], already part of the field)
  around all obstacles. From a northward-leaning starting condition, the H+C+V score for the
  wide corridor beats realigning to the 1.5m gap.

### Repeat verification of iter4 (verifyA / verifyB, 2026-07-06 22:2x)

Two independent batches, each a fresh stack, 5 runs each. `logs/exp/verifyA|verifyB/`.

- verifyA: 4/5 reached, **0 collisions**, min_dist ≥ 0.649m. run 1 timeout: **flies past and
  orbits the goal**, circling for 90 seconds (radius 1.5–2.4m), never entering the 0.5m circle.
- verifyB: 5/5 reached, **0 collisions**, min_dist ≥ 0.676m. run 2 slow (59.6s / 67m): flew
  past and circled all the way around once before being captured.

iter4 combined across three batches: **15 runs, 0 collisions, 14/15 reached** — safety target
met, terminal capture has a fly-past defect in 2/15.

Tick-level + offline-reproduction diagnosis (verifyA run 1, t=35.6):
- bonus triggered = **0/1800 ticks**, closest approach 1.478m.
- The drone is "already turning as hard as the window allows" (vy can only go −0.23→−0.43);
  at a 1.5 m/s tangential speed, the turning radius ~4m ≫ the 0.5m capture circle → a stable
  limit cycle.
- Root cause: the goal bonus is a **binary switch** (only triggers if the ray passes through
  the 0.5m circle). For fly-past geometry, the ray never passes through the circle → zero
  terminal attraction at all.

### exp 5 — iter5 = continuous terminal attraction basin (2026-07-06 22:4x)

Change (dwa_core.py): for candidates whose ray's closest point is < `goal_capture` (2.0m),
switch to scoring
`10000 − 10·min_goal_dist − |speed − √(2·brake_a·(d−r))|` —
millimeter-level proximity takes priority (×10 so "distance" always dominates "speed match"),
with matching the braking curve as secondary. Any close pass near the goal is now continuously
pulled toward the center, arriving at a speed it can actually stop at.

Offline: nominal 13.9s (unchanged), four tangential fly-past stress tests (from offset starting
points like (10,4), (14,−4), (8,−5)) all converge directly in 3.6–5.2s with no orbiting.

Final verification protocol: 3 independent batches × 5 runs, fresh stack each time.
Passing criteria: 15/15 reached, 0 collisions, min_dist ≥ 0.4m.

Results:
- **iter5a: 5/5 reached, 0 collisions, min_dist ≥ 0.681m, 17.9–19.3s**
- iter5b: ❌ invalid batch — simulator RTF crashed (TAKEOFF climb took 92 wall-clock seconds,
  position nearly frozen relative to wall clock), all 5 runs timed out in place. **Even so,
  still 0 collisions.** This is infrastructure flake (WSL2 + repeated gz/gpu_lidar restarts),
  not the algorithm.
- **iter5c: 5/5 reached, 0 collisions, min_dist ≥ 0.658m, 17.0–22.1s**
- **iter5b2 (replacement batch): 5/5 reached, 0 collisions, min_dist ≥ 0.691m, 16.6–20.6s**

### ✅ Final verdict (2026-07-06 23:0x)

**15/15 reached, 0 collisions, minimum lidar distance across all 45 healthy-run ticks 0.658m
(threshold 0.4m), average 18.2s, compute ~1.7ms (3% of the 50ms budget).**

Compared to baseline: reached 1/5 → 15/15; collision events 44+ → 0; the previous norm of
90s timeouts → average 18.2s. The goal (clearly avoiding all obstacles, repeatably verifiable)
has been met.

How to reproduce:
```bash
cd ~/ws/src/HOLO-DWA/holo_lab
./exp.sh up                    # fresh stack, N_RUNS=5, RUN_TIMEOUT=90
EXP_TIMEOUT=700 ./exp.sh collect <name>
cat logs/exp/<name>/report.txt # table + ASCII trajectory plot
```

---

## Follow-up (2026-07-07): hardening terminal approach + "why is the GUI slower?"

### Controlled comparison for "running with GUI is slower"

Manually running `./run_lab.sh` (with GUI) produced runs that meandered for 50–90s, raising
suspicion that run_lab was worse than exp. Compared side by side (same day, same machine, same
code):

| Mode | Result |
|------|------|
| Integrated GUI (`./run_lab.sh`) | 59.9s / 41m, tracking ratio 0.61 |
| External GUI (`./run_lab.sh gui`) | 90s timeout / 78m |
| **Pure headless (control group)** | **18.0s / 11.9m / eff 1.019 — perfect** |

Conclusion: **attaching GUI rendering to a live simulation on WSL2 degrades PX4's velocity
tracking** (tracking ratio 0.61 vs 0.87), causing the path to wander and loop more. Not an
algorithm problem; exp.sh and run_lab.sh share the same logic, the only difference is
HEADLESS=1. Use `./run_lab.sh gui` if you need to see it visually, just know it'll be messier
than headless.

### Hardening terminal approach: goal_approach_a = 0.5

The original braking curve used brake_a_max=1.0 → stayed at full speed until 1.6m from the
goal, then braked hard in the last 1m; with enough environmental noise it would graze past the
circle and loop around again. Added `Config.goal_approach_a=0.5`: starts decelerating from
~2.5m out, entering the circle at ~0.7 m/s. Offline nominal only 0.5s slower (14.4s), 10/10
across ten noise seeds at noise level 0.1. Re-verified in Gazebo: see `logs/exp/final5/`.

Known infrastructure risk (unrelated to the algorithm): occasional gz RTF crashes. Healthy
batches have an easy tell: TAKEOFF should complete within ~10 wall-clock seconds; if it takes
> 30 seconds, restart the stack.

## Final configuration (iter5)

Changes in `holo_lab/dwa_core.py` relative to the original (DWA's window/feasibility/prediction
completely untouched):
1. `scan_to_world_points(..., flip_y=True)` — **fixes the LiDAR mirroring bug**
   (gz z-up scan vs. NED/FRD handedness; the primary cause behind all the early failures).
2. Switched the clearance term to **directional probing**: takes the minimum obstacle distance
   along the candidate's unit direction at a fixed 1.5m (`clearance_lookahead=1.5`), decoupled
   from the candidate's speed → the creeping trap is gone; normalization
   `clearance_norm=0.5` (a 0.5m margin now scores full marks).
3. Terminal attraction basin: within `goal_capture=2.0`,
   `10000 − 10·min_goal_dist − |speed − √(2·brake_a·(d−r))|`
   (continuous attraction + braking-curve arrival; fixes the fly-past orbiting).
4. velocity term switched to `blend` (α=0.5): uses the goal-directed component in open areas
   (fixes the random diagonal drift from square-window corner degeneracy), keeps an α·|v| floor
   when blocked (fixes freezing in front of walls).

`holo_lab/scanner_lab.py`'s dwa_config:
`robot_radius 0.30` (≥ collision_dist 0.3; the original 0.2 was a "guaranteed collision"
design), weights H/C/V = 0.3/0.3/0.4, everything else (v_max 1.5, a_max 1.0, predict 3.0s,
resolution 0.1, goal_threshold 0.5) unchanged.

Task complete ✅
Final results: 3 independent batches × 5 runs = 15/15 reached, 0 collisions, closest approach
to any obstacle throughout 0.658m, average 18.2 seconds (baseline: 1/5 reached, 44+ collisions,
90-second timeout as the norm). The algorithm is repeatably verifiable, all data archived under
holo_lab/logs/exp/, and the full iteration log is in holo_lab/EXPERIMENTS.md.

Iteration process (each round: run the simulation → look at the data → find the mechanism →
fix it → verify again)

| Round | Problem found (mechanism) | Fix |
|---|---|---|
| baseline | Square-window corner degeneracy: during acceleration, a corner candidate's \|v\| bonus ≈ the heading-cosine penalty, direction gets flipped randomly by noise → random diagonal drift into the wall area | velocity switched to blend: goal-component dominant + an α·\|v\| floor as a backstop (fixes drift while still allowing escape in front of walls) |
| iter1 | Creeping trap: clearance takes "minimum distance over the whole 3-second ray," slow = short ray = high score → rewards crawling, rubs into the face of a cylinder at 0.2 m/s | Switched clearance to directional: fixed 1.5m probe along the unit direction, completely decoupled from speed |
| iter2 | Per-tick recomputation found the point cloud didn't match the world → LiDAR mirroring bug: gz scan +angle = body left (z-up), but the conversion assumes +angle = body right (FRD) → obstacles all mirrored left-right. Only the cylinder array in the field is asymmetric, so it hit a cylinder every single time | flip_y=True; wrote verify_frame.py to validate a real scan against the field geometry (0.08m vs 0.32m, conclusive proof) |
| iter3 | Obstacle avoidance perfect but orbits the goal: the goal bonus 10000+speed rewards rushing the circle at full speed, grazing past and missing means looping around and retrying | Switched the bonus to a braking curve 10000−\|v−√(2·a·d)\| |
| iter4 | All 15 runs 0 collisions, but 2/15 fly-past: the binary bonus gives zero terminal attraction when the ray doesn't pass through the 0.5m circle → a stable limit cycle | Switched to a continuous attraction basin: within 2m, 10000 − 10·miss − \|v−braking target\| |
| iter5 | — | Final verification, 3×5, all passed |

Deliverables (all within holo_lab)
- Algorithm: `dwa_core.py` (scoring-core rewrite; DWA's vx/vy window, feasibility mask, and
  prediction mechanism completely untouched) + `scanner_lab.py` config (robot_radius 0.2→0.3,
  fixing the "planner allowed at 0.2m but collision judged at 0.3m" guaranteed-collision design)
- Toolchain: `exp.sh` (one-shot batch run + archiving), `report.py` (CSV → ASCII trajectory
  plot, so you can see the path/collision points without a GUI), `sim_offline.py` (ROS-free
  offline pre-screening, second-scale), `verify_frame.py` (sensor handedness verification)
- Docs: `EXPERIMENTS.md` (includes TL;DR, per-round mechanism analysis, reproduction commands)

One more thing you need to know: hit one simulator flake during verification (iter5b: gz
real-time factor crashed, takeoff climb took 92 seconds, all 5 runs timed out in place —
even while frozen, still 0 collisions): WSL2 occasionally does this on repeated gz restarts;
telling a healthy batch apart is simple — if TAKEOFF takes more than 30 seconds, restart
the stack.
