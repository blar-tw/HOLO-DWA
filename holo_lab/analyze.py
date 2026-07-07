#!/usr/bin/env python3
# Quick, dependency-free reader for the lab logs in holo_lab/logs/.
# Stdlib only (no pandas/numpy needed) so it runs anywhere python3 does.
#
#   ./analyze.py                       # table of every run in summary.jsonl
#   ./analyze.py logs/session_X.csv    # per-run stats from one session CSV
#   ./analyze.py some/other.jsonl      # table of runs from a specific jsonl
import os
import sys
import csv
import json
import math

LAB_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(LAB_DIR, "logs")


def show_summary(path=None):
    if path is None:
        path = os.path.join(LOG_DIR, "summary.jsonl")
    if not os.path.exists(path):
        print(f"no summary yet at {path}")
        return
    rows = [json.loads(l) for l in open(path) if l.strip()]
    if not rows:
        print("summary.jsonl is empty")
        return
    hdr = ("run", "outcome", "dur_s", "path_m", "eff",
           "col", "min_dist", "min_clr", "infeas", "cmp_ms")
    fmt = "{:>3} {:<8} {:>6} {:>7} {:>5} {:>4} {:>8} {:>7} {:>8} {:>7}"
    print(fmt.format(*hdr))
    print("-" * 74)
    for r in rows:
        print(fmt.format(
            r.get("run_id", "?"),
            str(r.get("outcome", "?")),
            r.get("duration_s", "-"),
            r.get("path_length_m", "-"),
            str(r.get("path_efficiency", "-")),
            r.get("collisions", "-"),
            str(r.get("min_dist_m", "-")),
            str(r.get("min_clearance_m", "-")),
            f"{r.get('infeasible_ticks','-')}/{r.get('ticks','-')}",
            r.get("compute_ms_mean", "-"),
        ))
    reached = [r for r in rows if r.get("outcome") == "reached"]
    collided = [r for r in rows if r.get("collided")]
    print("-" * 74)
    if reached:
        n = len(reached)
        avg_t = sum(r["duration_s"] for r in reached) / n
        effs = [r["path_efficiency"] for r in reached if r.get("path_efficiency")]
        avg_eff = sum(effs) / len(effs) if effs else 0.0
        print(f"reached: {n}/{len(rows)}  avg_time={avg_t:.1f}s  avg_efficiency={avg_eff:.2f}")
    print(f"runs with collision: {len(collided)}/{len(rows)}"
          + (f"  (runs {[r.get('run_id') for r in collided]})" if collided else ""))


def show_session(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print("empty session csv")
        return
    runs = {}
    for r in rows:
        rid = r["run_id"]
        runs.setdefault(rid, []).append(r)
    print(f"{path}: {len(rows)} ticks across run_ids {sorted(runs)}")
    for rid in sorted(runs, key=lambda x: int(x) if str(x).lstrip('-').isdigit() else 0):
        rr = runs[rid]
        states = {}
        for r in rr:
            states[r["state"]] = states.get(r["state"], 0) + 1
        cmps = [float(r["compute_ms"]) for r in rr if r.get("compute_ms") not in ("", None)]
        cmp_mean = sum(cmps) / len(cmps) if cmps else 0.0
        cmp_max = max(cmps) if cmps else 0.0
        print(f"  run {rid}: {len(rr)} ticks  states={states}  "
              f"compute mean/max={cmp_mean:.2f}/{cmp_max:.2f}ms")


def main():
    if len(sys.argv) > 1:
        if sys.argv[1].endswith(".jsonl"):
            show_summary(sys.argv[1])
        else:
            show_session(sys.argv[1])
    else:
        show_summary()


if __name__ == "__main__":
    main()
