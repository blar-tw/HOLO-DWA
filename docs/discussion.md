# DWA algorithm issue discussion

Records two algorithm-level problems currently encountered with the holonomic DWA: the design trade-off for the velocity reward, and the doorway dilemma. Both problems belong to the classic local-minimum family for planners like DWA that "only look locally and pick one step at a time." This records the phenomena, root causes, and candidate fixes first; experiments will follow one by one.

---

## 1. velocity_score: goal-direction component vs. raw speed magnitude

### Background

DWA's scoring function is a weighted sum of three terms:

```
score = heading_weight * progress      (how much progress was made toward the goal)
      + clearance_weight * clearance   (distance from obstacles)
      + velocity_weight * velocity     (how fast it's flying)
```

The `velocity` term has two possible designs:

- **Raw magnitude**: `|v| / v_max`, regardless of direction — flying fast scores well. This is what the original `dwa.py` did.
- **Goal-directed component**: project `(vx, vy)` onto the unit vector from "current position → goal"; only velocity that actually moves toward the goal is rewarded.

### Problem A: raw-magnitude reward → diagonal drift in open space

Tested in simulation (fully open space, goal 12m straight ahead): with the raw-magnitude reward, after takeoff the drone drifts diagonally off course, with lateral offset reaching up to about 2.3m, only correcting back as it nears the goal.

Cause: the dynamic window is a **square** region centered on the current velocity (vx, vy each ±a·dt); the "corners" of that box have a larger velocity magnitude than the straight-ahead direction (√2 times, diagonally). When the goal is far away, a small angular deviation barely affects `progress` (a second-order effect via the Pythagorean theorem) — the two are nearly indistinguishable. So the raw-velocity reward ends up dominating, the score always favors the corners, and the drone accelerates away along the diagonal.

### Problem B: goal-component reward → freezes in front of walls

After switching velocity_score to the goal-directed component, open-space flight does become a straight line to the goal. But testing in Gazebo showed the drone would **stop dead in front of a wall**.

Cause: escaping a blocking wall requires "sidestepping along the wall to find a gap." But while sidestepping:

- The velocity component toward the goal ≈ 0 → velocity_score gets 0
- Temporarily moving away from the goal → progress goes slightly negative
- Hugging the wall → clearance is also low

The result is that "sidestepping to explore" scores about the same as "hovering in place," so DWA has no incentive to move at all — it freezes. The raw-magnitude reward does the opposite: "just moving at all" earns points, so the drone slides along the wall; once it's lined up with the gap, "flying straight through the gap" becomes the high-progress option — this is exactly the mechanism that lets the raw-magnitude reward escape this kind of local minimum.

### Trade-off summary

| | Raw magnitude `\|v\|` | Component (projected onto goal direction) |
|---|---|---|
| Open space | Diagonal drift during acceleration (box-corner effect) | Straight to the goal |
| Wall blocking path | Slides along the wall, may find the gap | Freezes (no reward for sidestepping) |
| Nature | Rewards "moving" itself | Rewards "getting closer to the goal" |

### Current choice

**Reverted to raw magnitude** (current state of `dwa_core.py`), accepting slight diagonal drift in open space in exchange for not freezing at walls. Rationale: drift is just an ugly path that still converges to the goal eventually; freezing is an outright mission failure — the severity isn't comparable.

### Directions for future experiments

1. **Hybrid reward**: `velocity_score = max(component, α·magnitude)`, with α around 0.3–0.5 — the component dominates in open space for straight-line flight, and when blocked by a wall (component drops to zero) it falls back to the magnitude reward to keep the incentive to move.
2. **Classic DWA heading term**: add a separate "cosine of the angle between velocity direction and goal direction" term, splitting "is the direction right" from "how fast is it going" into two independent scoring terms instead of mixing them into one.
3. **A higher-level global planner**: the fundamental fix. DWA would only handle local obstacle avoidance, with a global planner like A*/RRT supplying waypoints, making the local-minimum problem disappear naturally.

---

## 2. The doorway / narrow-gap problem

### Symptom

Simulation test (wall at x=5, gap width 1.5m, `robot_radius=0.4`): the drone correctly slides up to right in front of the gap, but once facing the gap head-on it **stops and oscillates slightly (vx bouncing between ±0.1), afraid to go through**.

### Root cause analysis

With a 1.5m gap and a 0.4m body radius, flying straight through the center leaves only 0.75m to each side wall, and after subtracting the radius `safe_dist` is only 0.35m. This brings a triple penalty:

1. **Low clearance**: the trajectory through the gap has clearance_score ≈ 0.35, while loitering outside the gap can maintain close to 1.0.
2. **Braking speed limit**: the admissible-velocity condition `speed ≤ √(2·safe_dist·brake_a)` at safe_dist=0.35, brake_a=1.0 only allows ≈ 0.84 m/s, so velocity_score is also suppressed.
3. **Small geometric margin**: DWA's candidate trajectories are straight lines; cutting diagonally into a 1.5m gap from a position off the gap's axis, the endpoint easily lands within 0.4m of the wall edge and gets ruled infeasible outright.

Summed together, "going through" loses to "loitering at the doorway" on score, even though going through is actually safe. This isn't a bug — the scoring function is faithfully executing the value judgment that "being close to an obstacle is bad." The problem is that this value judgment doesn't account for "this closeness is necessary and temporary."

### Relationship to problem 1

Both share the same root: DWA only evaluates the immediate score of "this one step," with no concept of "take a short-term loss for a long-term gain." Freezing in front of a wall is "sidestepping temporarily away from the goal" being penalized; hesitating at the doorway is "temporarily hugging an obstacle" being penalized.

### Possible fixes (to be tested)

1. **Lower `clearance_weight`**, or lower the clearance normalization ceiling from 1.0m (e.g. to 0.5m): so trajectories that are "already safe enough" stop losing points just for not being far enough.
2. **Lower `robot_radius`** to match the actual body size more closely (the x500's wheelbase is about 0.5m, so a radius of 0.25–0.3 is more reasonable; the current 0.4 is conservative).
3. **Widen the gap** (see next section): not an algorithmic fix, but an engineering workaround to get the end-to-end test passing first.
4. **Global planner**: same as problem 1, the fundamental fix.

---

## 3. Running away after moving off-goal and never coming back (heading term vanishes with distance)

### Symptom

A real log from a gap-scenario test (goal at NED (N=0, E=12)) shows three phases:

1. **Diagonal drift**: flies diagonally from (0,0) to (5.4, 4.5). The goal is due east, yet it drifts toward +N at the same time (scalar velocity reward's diagonal wander, plus the goal being hidden behind the wall with the gap on the +N side).
2. **Wall collision and bounce**: at (5.4, 4.5), 0.52m from the wall ahead, DWA repeatedly reports "no feasible velocity found, braking," yet the position lurches 4–7 m/s within 1 second (far exceeding v_max=1.5) — the command was to brake to (0,0) yet it flew that fast, almost certainly because it actually hit the wall in Gazebo and got bounced by physics.
3. **Runaway**: after being bounced to (5, −9), DWA keeps commanding full speed in the direction **away from the goal**, with distance-to-goal monotonically growing from 22m to 42m, never coming back.

### Root cause: the heading term gets diluted by start_dist

Scores during the runaway phase: heading stays at a small constant negative value (≈ −0.07), clearance is saturated at 1.0, velocity ≈ 0.98. The problem lies in the original heading (progress) definition:

```
progress = (start_dist − final_dist) / start_dist
```

At 30m from the goal, a 3m look-ahead prediction, even if pointed dead at the goal, only yields progress = 3/30 = 0.1, which after weighting is `0.28 × 0.1 ≈ 0.028`. **The farther from the goal, the more the score gap between "toward the goal" and "away from the goal" gets diluted by start_dist, down to only ±0.03**, while the velocity reward (scalar, rewarding ≈ 0.3 just for being fast) is nearly the same for every full-speed direction. As a result, "full speed in the wrong direction" doesn't lose to "decelerate and turn around," and combined with the dynamic window only allowing ±0.2 m/s change per step (turning is slow), the drone just flies off.

### Why "capping clearance" doesn't fix it (an intuitive dead end)

One earlier hypothesis was "clearance has no ceiling, letting it fly farther and farther." But:

- `clearance_score = clip(safe_dist, 0, 1.0)` **already has a ceiling** — beyond about 1.2m from an obstacle it already saturates at 1.0.
- More importantly: the runaway happens in open space, where **every candidate direction has clearance = 1.0**. A term that's equal across all candidates cancels out in the argmax and has zero effect on which one gets picked. So clearance is "neutral" in open areas — it isn't what pushed the drone away from the goal.

It's not clearance pushing it away — it's that **heading is too weak to pull it back**. Capping clearance wouldn't change the open-area runaway at all.

### Fix: switch to a distance-independent, angle-based heading (already applied)

Replaced heading with the classic DWA (Fox 1997) angular definition — the **cosine of the angle** between velocity direction and "direction toward the goal":

```
heading_score = (vx·ux + vy·uy) / |v|          # ux, uy is the unit vector pointing at the goal
```

Fixed range [−1, 1] (+1 = directly at the goal, −1 = directly away), so **no matter how far the goal is, the score gap between toward vs. away from the goal is always the full ±weight**. Compared to the old progress ratio (which vanishes at distance), this directly restores the steering force at long range, and "facing away from the goal" becomes an explicit negative score, actively suppressing runaway behavior. The change is confined to the heading term in `dwa_core.py`; clearance / velocity / the dynamic window are untouched.

### Verification (offline, not yet re-run in Gazebo)

- **Long-range steering force**: starting 32m from the goal in open space, the new heading directly selects `heading_score = 1.0` (pointed straight at the goal); the old version only got ≈ 0.1 here.
- **Runaway recovery**: reproducing the state of full speed facing away from the goal at (5, −9), the new version starts turning back toward the goal within the allowed dynamic window (vy −0.45 → −0.25, gradually flipping positive).
- **Open-space straight flight**: (0,0)→(0,12), N-direction drift stays within ±0.16m over the whole run (the old scalar version drifted to ≈ 2.3m), arriving at t≈9s.

### Not yet resolved (a separate thread)

The "wall collision and bounce" from phase 2 is a separate problem: braking on infeasible didn't actually hold, and hugging the wall too closely led to a physics bounce. Fixing heading means it at least flies back after being bounced, but **not hitting the wall in the first place** still needs separate handling (controlled braking on infeasible, or not letting it get pushed this close to the wall in the first place; plus the local-minimum issues from problems 1 and 2).

---

## 4. Temporary world-file adjustment (to test end-to-end first)

To first validate that the whole "takeoff → obstacle avoidance → reach the destination" pipeline works, the gap in `dwa_test.sdf` was temporarily widened from 1.5m to **3m** (y = 0.75 ~ 3.75, center kept at y=2.25):

- Flying through the gap center now gives safe_dist = 1.5 − 0.4 = 1.1m, so clearance_score hits 1.0 outright and the braking speed limit is lifted, eliminating all three penalties from problem 2.
- Once the pipeline is verified, the gap will be narrowed step by step (3m → 2m → 1.5m) to go back and run the parameter experiments for problem 2.

Remember to resync the world to PX4 after changing it:

```bash
cd ~/ws/src/HOLO-DWA
./gz_extra/install.sh ~/PX4-Autopilot
```
