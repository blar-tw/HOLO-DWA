#!/usr/bin/env python3
# Trajectory + diagnostics report for holo_lab session CSVs. Stdlib only.
#
# Renders each navigate run as an ASCII map of the arena (obstacles from
# gz_extra/worlds/dwa_test.sdf, hardcoded below) so behavior is visible
# without the Gazebo GUI, and prints per-run diagnostics: collisions,
# infeasible clusters, stalls, score statistics.
#
#   ./report.py                        # newest session CSV in logs/
#   ./report.py logs/session_X.csv     # specific session(s)
#   ./report.py --all                  # every session CSV in logs/
import os
import sys
import csv
import glob
import math

LAB_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(LAB_DIR, "logs")

# ---------------------------------------------------------------- arena map
# Gazebo world frame: x forward (East in NED), y left (North in NED).
# CSV logs NED: pos_n = gz_y, pos_e = gz_x.
X_MIN, X_MAX = -4.5, 16.5
Y_MIN, Y_MAX = -9.5, 9.5
COLS_PER_M = 4.6   # horizontal chars per meter
ROWS_PER_M = 2.3   # vertical chars per meter (chars are ~2x tall)

# (x0, y0, x1, y1) axis-aligned obstacle rectangles in gz world coords
OBSTACLE_RECTS = [
    (-4.1, -9.1, -3.9, 9.1),      # boundary back
    (15.9, -9.1, 16.1, 9.1),      # boundary front
    (-4.1, -9.1, 16.1, -8.9),     # boundary right (south)
    (-4.1, 8.9, 16.1, 9.1),       # boundary left (north)
    (2.9, -6.0, 3.1, -0.75),      # wall_gap right segment
    (2.9, 0.75, 3.1, 6.0),        # wall_gap left segment
    (8.4, -2.2, 9.6, -1.0),       # box_right
    (8.4, 1.0, 9.6, 2.2),         # box_left
]
OBSTACLE_CIRCLES = [
    (5.5, -1.5, 0.3),             # cylinder_1
    (6.5, 0.3, 0.3),              # cylinder_2
    (5.5, 2.0, 0.3),              # cylinder_3
]
GOAL_XY = (12.0, 0.0)


def make_grid():
    ncols = int((X_MAX - X_MIN) * COLS_PER_M) + 1
    nrows = int((Y_MAX - Y_MIN) * ROWS_PER_M) + 1
    grid = [[" "] * ncols for _ in range(nrows)]

    def put(gx, gy, ch):
        c = int((gx - X_MIN) * COLS_PER_M)
        r = int((Y_MAX - gy) * ROWS_PER_M)
        if 0 <= r < nrows and 0 <= c < ncols:
            grid[r][c] = ch

    step = 0.5 / COLS_PER_M
    for (x0, y0, x1, y1) in OBSTACLE_RECTS:
        gy = y0
        while gy <= y1 + 1e-9:
            gx = x0
            while gx <= x1 + 1e-9:
                put(gx, gy, "#")
                gx += step
            gy += step / 2
    for (cx, cy, rad) in OBSTACLE_CIRCLES:
        gy = cy - rad
        while gy <= cy + rad + 1e-9:
            gx = cx - rad
            while gx <= cx + rad + 1e-9:
                if (gx - cx) ** 2 + (gy - cy) ** 2 <= rad * rad:
                    put(gx, gy, "#")
                gx += step
            gy += step / 2
    put(*GOAL_XY, "G")
    return grid


def render(grid):
    lines = []
    lines.append("    y=+9 (gz +y / NED North up on map top)")
    for row in grid:
        lines.append("  " + "".join(row).rstrip())
    lines.append(f"    x: {X_MIN:+.0f} .. {X_MAX:+.0f} (gz +x / NED East, left->right) "
                 f"| S start, E end, . path, X collision, ! infeasible, * stall, G goal")
    return "\n".join(lines)


def fnum(v, default=None):
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def load_runs(path):
    """Group NAVIGATE ticks by run_id (>=1). Returns {run_id: [row, ...]}."""
    runs = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            if r["state"] != "NAVIGATE":
                continue
            rid = int(r["run_id"])
            if rid >= 1:
                runs.setdefault(rid, []).append(r)
    return runs


def run_report(rid, rows):
    out = [f"--- run {rid}: {len(rows)} navigate ticks ---"]
    if not rows:
        return "\n".join(out)

    grid = make_grid()
    t0, t1 = fnum(rows[0]["t"]), fnum(rows[-1]["t"])

    # stall detection: window of >=3s where the drone moved < 0.3 m
    stalls = []
    stall_cells = set()
    i = 0
    while i < len(rows):
        j = i
        xi, yi = fnum(rows[i]["pos_e"]), fnum(rows[i]["pos_n"])
        while j + 1 < len(rows):
            xj, yj = fnum(rows[j + 1]["pos_e"]), fnum(rows[j + 1]["pos_n"])
            if math.hypot(xj - xi, yj - yi) > 0.3:
                break
            j += 1
        dur = fnum(rows[j]["t"]) - fnum(rows[i]["t"])
        if dur >= 3.0:
            stalls.append((fnum(rows[i]["t"]) - t0, dur, xi, yi))
            for k in range(i, j + 1):
                stall_cells.add(k)
            i = j + 1
        else:
            i += 1

    # paint path (later marks overwrite earlier ones; X > ! > * > .)
    def put(gx, gy, ch):
        c = int((gx - X_MIN) * COLS_PER_M)
        r = int((Y_MAX - gy) * ROWS_PER_M)
        if 0 <= r < len(grid) and 0 <= c < len(grid[0]):
            grid[r][c] = ch

    prio = {".": 0, "*": 1, "!": 2, "X": 3, "S": 4, "E": 4}

    def put_prio(gx, gy, ch):
        c = int((gx - X_MIN) * COLS_PER_M)
        r = int((Y_MAX - gy) * ROWS_PER_M)
        if 0 <= r < len(grid) and 0 <= c < len(grid[0]):
            cur = grid[r][c]
            if cur == "#" or prio.get(ch, 0) >= prio.get(cur, -1):
                grid[r][c] = ch

    collisions = []
    infeas_spans = []
    infeas_open = None
    min_threat_run = float("inf")
    min_threat_pos = None
    totals = []
    for k, r in enumerate(rows):
        gx, gy = fnum(r["pos_e"]), fnum(r["pos_n"])   # gz x = East, gz y = North
        ch = "."
        if k in stall_cells:
            ch = "*"
        if r["ok"] == "0":
            ch = "!"
            if infeas_open is None:
                infeas_open = [fnum(r["t"]) - t0, fnum(r["t"]) - t0, gx, gy]
            else:
                infeas_open[1] = fnum(r["t"]) - t0
        else:
            if infeas_open is not None:
                infeas_spans.append(tuple(infeas_open))
                infeas_open = None
        if r["collision"] == "1":
            ch = "X"
            collisions.append((fnum(r["t"]) - t0, gx, gy, fnum(r["min_threat"])))
        mt = fnum(r["min_threat"])
        if mt is not None and mt < min_threat_run:
            min_threat_run = mt
            min_threat_pos = (gx, gy)
        tot = fnum(r["total"])
        if tot is not None and tot < 100:   # skip the 10000+ goal bonus rows
            totals.append(tot)
        put_prio(gx, gy, ch)

    if infeas_open is not None:
        infeas_spans.append(tuple(infeas_open))
    put_prio(fnum(rows[0]["pos_e"]), fnum(rows[0]["pos_n"]), "S")
    put_prio(fnum(rows[-1]["pos_e"]), fnum(rows[-1]["pos_n"]), "E")

    out.append(render(grid))
    out.append(f"  time {t0:.1f} -> {t1:.1f}s ({t1 - t0:.1f}s), "
               f"end at gz({fnum(rows[-1]['pos_e']):+.2f}, {fnum(rows[-1]['pos_n']):+.2f}), "
               f"dist_to_target={rows[-1]['dist_to_target']}")
    if min_threat_pos:
        out.append(f"  closest lidar return: {min_threat_run:.2f} m at gz({min_threat_pos[0]:+.1f}, {min_threat_pos[1]:+.1f})")
    if totals:
        out.append(f"  score total: mean={sum(totals)/len(totals):.3f} min={min(totals):.3f} max={max(totals):.3f}")
    if collisions:
        out.append(f"  COLLISION ticks: {len(collisions)}")
        for (tt, gx, gy, mt) in collisions[:8]:
            out.append(f"    t+{tt:6.1f}s gz({gx:+.2f}, {gy:+.2f}) min_dist={mt:.2f} m")
    else:
        out.append("  collisions: none")
    if infeas_spans:
        out.append(f"  infeasible spans: {len(infeas_spans)}")
        for (ta, tb, gx, gy) in infeas_spans[:8]:
            out.append(f"    t+{ta:6.1f}..{tb:6.1f}s at gz({gx:+.2f}, {gy:+.2f})")
    if stalls:
        out.append(f"  stalls (>=3s, <0.3m moved): {len(stalls)}")
        for (tt, dur, gx, gy) in stalls[:8]:
            out.append(f"    t+{tt:6.1f}s for {dur:5.1f}s at gz({gx:+.2f}, {gy:+.2f})")
    return "\n".join(out)


def main():
    args = [a for a in sys.argv[1:]]
    if "--all" in args:
        paths = sorted(glob.glob(os.path.join(LOG_DIR, "session_*.csv")))
    elif args:
        paths = args
    else:
        paths = sorted(glob.glob(os.path.join(LOG_DIR, "session_*.csv")))[-1:]
    if not paths:
        # Fresh clone / after cleanup there are no loose sessions yet - point
        # at the committed experiment archives instead of a dead end.
        archived = sorted(glob.glob(os.path.join(LOG_DIR, "exp", "*", "session_*.csv")))
        print("no loose session CSVs in logs/ - fly one with ./run_lab.sh, "
              "or view an archived run:")
        for a in archived:
            print(f"  ./report.py {os.path.relpath(a, LAB_DIR)}")
        return
    for p in paths:
        print(f"===== {os.path.basename(p)} =====")
        runs = load_runs(p)
        if not runs:
            print("  (no NAVIGATE ticks - run never started)")
        for rid in sorted(runs):
            print(run_report(rid, runs[rid]))
        print()


if __name__ == "__main__":
    main()
