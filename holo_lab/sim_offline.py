#!/usr/bin/env python3
# Offline (no ROS / no Gazebo) sanity sim for holo_lab/dwa_core.py.
#
# Same arena geometry as dwa_test.sdf (see report.py), a ray-cast 2D LiDAR
# (360 rays, 12 m range, occlusion-correct), first-order velocity tracking
# for the vehicle, and the 20 Hz control loop of scanner_lab.py. This is NOT
# a substitute for the Gazebo run - PX4 dynamics, EKF noise and physics
# contact are missing - but it reproduces the scoring/feasibility behavior
# of the planner faithfully enough to pre-screen scoring changes in
# milliseconds instead of minutes.
#
#   ./sim_offline.py                 # one run, current scanner_lab config
#   ./sim_offline.py --map           # ... plus ASCII trajectory map
#   ./sim_offline.py --sweep         # compare a few hardcoded config variants
import math
import argparse
import numpy as np

import dwa_core

# gz world frame: x = NED East, y = NED North. Sim runs in gz frame with
# goal (12, 0); dwa_core is frame-agnostic.
OBSTACLE_RECTS = [
    (-4.1, -9.1, -3.9, 9.1),
    (15.9, -9.1, 16.1, 9.1),
    (-4.1, -9.1, 16.1, -8.9),
    (-4.1, 8.9, 16.1, 9.1),
    (2.9, -6.0, 3.1, -0.75),
    (2.9, 0.75, 3.1, 6.0),
    (8.4, -2.2, 9.6, -1.0),
    (8.4, 1.0, 9.6, 2.2),
]
OBSTACLE_CIRCLES = [
    (5.5, -1.5, 0.3),
    (6.5, 0.3, 0.3),
    (5.5, 2.0, 0.3),
]
GOAL = (12.0, 0.0)
BODY_RADIUS = 0.25          # physical contact radius (x500-ish)
LIDAR_RANGE = 12.0
N_RAYS = 360                # node uses 1080/6 = 180 points; 360 is safe


def _rect_segments():
    segs = []
    for (x0, y0, x1, y1) in OBSTACLE_RECTS:
        segs += [(x0, y0, x1, y0), (x1, y0, x1, y1),
                 (x1, y1, x0, y1), (x0, y1, x0, y0)]
    return np.array(segs)          # (S, 4)

_SEGS = _rect_segments()
_CIRC = np.array(OBSTACLE_CIRCLES)  # (C, 3)


def lidar_scan(px, py):
    """Occlusion-correct 360-ray scan from (px, py). Returns (N,2) hit points."""
    ang = np.linspace(-math.pi, math.pi, N_RAYS, endpoint=False)
    dx, dy = np.cos(ang), np.sin(ang)
    t_best = np.full(N_RAYS, LIDAR_RANGE)

    # segments: solve p + t*d = a + u*(b-a)
    for (ax, ay, bx, by) in _SEGS:
        ex, ey = bx - ax, by - ay
        denom = dx * ey - dy * ex               # (N,)
        ok = np.abs(denom) > 1e-12
        t = np.where(ok, ((ax - px) * ey - (ay - py) * ex) / np.where(ok, denom, 1.0), np.inf)
        # Cramer on [d, -e][t,u]^T = a-p gives u = ((a-p) x d) / (d x e),
        # with x the 2D cross product and denom = d x e computed above.
        u_num = (ax - px) * dy - (ay - py) * dx
        u = np.where(ok, u_num / np.where(ok, denom, 1.0), np.inf)
        hit = ok & (t > 1e-9) & (u >= 0.0) & (u <= 1.0)
        t_best = np.where(hit & (t < t_best), t, t_best)

    # circles: |p + t d - c|^2 = r^2
    for (cx, cy, r) in _CIRC:
        fx, fy = px - cx, py - cy
        b = fx * dx + fy * dy
        c = fx * fx + fy * fy - r * r
        disc = b * b - c
        ok = disc >= 0.0
        sq = np.sqrt(np.where(ok, disc, 0.0))
        t = np.where(ok, -b - sq, np.inf)
        t = np.where(t > 1e-9, t, np.inf)
        t_best = np.minimum(t_best, t)

    hitmask = t_best < LIDAR_RANGE
    return np.stack([px + t_best[hitmask] * dx[hitmask],
                     py + t_best[hitmask] * dy[hitmask]], axis=1)


def surface_distance(px, py):
    """Distance from (px,py) to the nearest obstacle surface."""
    best = float("inf")
    for (x0, y0, x1, y1) in OBSTACLE_RECTS:
        ddx = max(x0 - px, 0.0, px - x1)
        ddy = max(y0 - py, 0.0, py - y1)
        best = min(best, math.hypot(ddx, ddy))
    for (cx, cy, r) in _CIRC:
        best = min(best, math.hypot(px - cx, py - cy) - r)
    return best


def make_config():
    """Mirror scanner_lab.py's dwa_config block."""
    cfg = dwa_core.Config()
    cfg.v_max = 1.5
    cfg.vx_min = cfg.vy_min = -1.5
    cfg.vx_max = cfg.vy_max = 1.5
    cfg.a_max = 1.0
    cfg.brake_a_max = 1.0
    cfg.vx_resolution = cfg.vy_resolution = 0.1
    cfg.control_dt = 0.2
    cfg.predict_time = 3.0
    cfg.predict_dt = 0.2
    cfg.robot_radius = 0.30
    cfg.goal_threshold = 0.5
    cfg.velocity_mode = "blend"
    cfg.blend_alpha = 0.5
    cfg.clearance_norm = 0.5
    cfg.clearance_lookahead = 1.5
    cfg.heading_weight = 0.3
    cfg.clearance_weight = 0.3
    cfg.velocity_weight = 0.4
    return cfg


def simulate(cfg, start=(0.0, 0.0), goal=GOAL, t_max=90.0, dt=0.05,
             track_a=3.0, seed=0, collect_path=False, noise=0.0):
    """Run one navigation. Velocity tracks the command at <= track_a m/s^2.

    noise > 0 adds zero-mean Gaussian noise (std = noise, m/s) to the
    velocity the planner sees and a small random initial drift velocity,
    mimicking EKF jitter / takeoff transients that tip near-degenerate
    argmax decisions in the real stack.
    """
    rng = np.random.default_rng(seed)
    px, py = start
    vx = vy = 0.0
    if noise > 0.0:
        vx, vy = rng.normal(0.0, 0.3, 2)   # takeoff residual drift
    t = 0.0
    path_len = 0.0
    min_surf = float("inf")
    collisions = 0
    in_contact = False
    infeasible = 0
    ticks = 0
    path = []
    replan_every = 1     # planner runs every tick, like the node
    while t < t_max:
        ticks += 1
        obs = lidar_scan(px, py)
        mvx, mvy = vx, vy
        if noise > 0.0:
            mvx += rng.normal(0.0, noise)
            mvy += rng.normal(0.0, noise)
        state = {"x": px, "y": py, "vx": mvx, "vy": mvy}
        cvx, cvy, ok = dwa_core.dwa_control(state, goal, obs, cfg)
        if not ok:
            cvx = cvy = 0.0
            infeasible += 1
        # first-order velocity tracking with accel cap
        for _ in range(replan_every):
            ax = np.clip((cvx - vx) / dt, -track_a, track_a)
            ay = np.clip((cvy - vy) / dt, -track_a, track_a)
            vx += ax * dt
            vy += ay * dt
            nx, ny = px + vx * dt, py + vy * dt
            path_len += math.hypot(nx - px, ny - py)
            px, py = nx, ny
            t += dt
        d = surface_distance(px, py)
        min_surf = min(min_surf, d)
        if d < BODY_RADIUS:
            if not in_contact:
                collisions += 1
                in_contact = True
        else:
            in_contact = False
        if collect_path:
            path.append((t, px, py))
        if math.hypot(goal[0] - px, goal[1] - py) < cfg.goal_threshold:
            return {"outcome": "reached", "t": t, "path_len": path_len,
                    "min_surf": min_surf, "collisions": collisions,
                    "infeasible": infeasible, "ticks": ticks, "end": (px, py),
                    "path": path}
    return {"outcome": "timeout", "t": t, "path_len": path_len,
            "min_surf": min_surf, "collisions": collisions,
            "infeasible": infeasible, "ticks": ticks, "end": (px, py),
            "path": path}


def ascii_map(path):
    X_MIN, X_MAX, Y_MIN, Y_MAX = -4.5, 16.5, -9.5, 9.5
    CPM, RPM = 4.6, 2.3
    ncols = int((X_MAX - X_MIN) * CPM) + 1
    nrows = int((Y_MAX - Y_MIN) * RPM) + 1
    grid = [[" "] * ncols for _ in range(nrows)]

    def put(gx, gy, ch, force=False):
        c = int((gx - X_MIN) * CPM)
        r = int((Y_MAX - gy) * RPM)
        if 0 <= r < nrows and 0 <= c < ncols:
            if force or grid[r][c] == " ":
                grid[r][c] = ch

    step = 0.5 / CPM
    for (x0, y0, x1, y1) in OBSTACLE_RECTS:
        gy = y0
        while gy <= y1 + 1e-9:
            gx = x0
            while gx <= x1 + 1e-9:
                put(gx, gy, "#", force=True)
                gx += step
            gy += step / 2
    for (cx, cy, rad) in OBSTACLE_CIRCLES:
        gy = cy - rad
        while gy <= cy + rad + 1e-9:
            gx = cx - rad
            while gx <= cx + rad + 1e-9:
                if (gx - cx) ** 2 + (gy - cy) ** 2 <= rad * rad:
                    put(gx, gy, "#", force=True)
                gx += step
            gy += step / 2
    put(*GOAL, "G", force=True)
    for (t, gx, gy) in path:
        put(gx, gy, ".")
    if path:
        put(path[0][1], path[0][2], "S", force=True)
        put(path[-1][1], path[-1][2], "E", force=True)
    return "\n".join("".join(row).rstrip() for row in grid)


def fmt(r):
    return (f"{r['outcome']:8s} t={r['t']:5.1f}s path={r['path_len']:5.1f}m "
            f"min_surf={r['min_surf']:.2f}m col={r['collisions']} "
            f"infeas={r['infeasible']}/{r['ticks']} end=({r['end'][0]:+.1f},{r['end'][1]:+.1f})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", action="store_true")
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--goal", nargs=2, type=float, default=list(GOAL))
    ap.add_argument("--noise", type=float, default=0.0,
                    help="velocity measurement noise std (m/s); also enables random initial drift")
    ap.add_argument("--seeds", type=int, default=8, help="seeds per variant when --noise > 0")
    args = ap.parse_args()

    if args.sweep:
        variants = {
            "baseline(scalar,r.2,norm1,226)": dict(velocity_mode="scalar", robot_radius=0.2,
                                                   clearance_norm=1.0, heading_weight=0.2,
                                                   clearance_weight=0.2, velocity_weight=0.6,
                                                   clearance_lookahead=0.0),
            "iter1(time-based clearance)": dict(clearance_lookahead=0.0),
            "iter2(dir clearance L1.5)": dict(),
            "iter2+L1.0": dict(clearance_lookahead=1.0),
            "iter2+L2.0": dict(clearance_lookahead=2.0),
            "iter2+scalar": dict(velocity_mode="scalar"),
            "iter2+H.2C.3V.5": dict(heading_weight=0.2, clearance_weight=0.3, velocity_weight=0.5),
            "iter2+H.4C.4V.2": dict(heading_weight=0.4, clearance_weight=0.4, velocity_weight=0.2),
        }
        for name, over in variants.items():
            cfg = make_config()
            for k, v in over.items():
                setattr(cfg, k, v)
            if args.noise > 0:
                results = [simulate(cfg, goal=tuple(args.goal), noise=args.noise, seed=s)
                           for s in range(args.seeds)]
                ok = sum(1 for r in results if r["outcome"] == "reached")
                col = sum(r["collisions"] for r in results)
                worst = min(r["min_surf"] for r in results)
                avg_t = sum(r["t"] for r in results if r["outcome"] == "reached") / max(ok, 1)
                print(f"{name:36s} reached {ok}/{len(results)}  collisions {col}  "
                      f"worst_surf {worst:.2f}m  avg_t {avg_t:.1f}s")
            else:
                r = simulate(cfg, goal=tuple(args.goal))
                print(f"{name:36s} {fmt(r)}")
        return

    cfg = make_config()
    r = simulate(cfg, goal=tuple(args.goal), collect_path=args.map)
    print(fmt(r))
    if args.map:
        print(ascii_map(r["path"]))


if __name__ == "__main__":
    main()
