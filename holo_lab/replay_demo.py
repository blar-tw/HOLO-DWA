#!/usr/bin/env python3
# Puppet-replay a logged flight inside a Gazebo GUI for demo recording.
#
# Why this exists: on this WSL2 box, attaching ANYTHING extra to the live
# simulation (the integrated GUI, a `gz sim -g` client, or even gz state
# recording) degrades the flight badly - measured control experiments:
# pure headless run 18.0 s / 11.9 m vs 55-90 s wandering with any client
# attached. So the demo path is: fly headless (clean), then REPLAY the
# logged trajectory here by puppeting a static drone model with set_pose.
# Render load during replay cannot affect the already-flown trajectory.
#
#   ./replay_demo.py                       # newest session CSV, first run, 1x
#   ./replay_demo.py --run 2 --speed 0.5   # slow motion
#   ./replay_demo.py --csv logs/exp/iter5b2/session_*.csv --loop
#
# Workflow for a clean 18 s video:
#   NO_ATTACH=1 HEADLESS=1 N_RUNS=1 ./run_lab.sh   (fly; wait for "reached")
#   ./run_lab.sh kill
#   ./replay_demo.py --loop                        (GUI opens, drone replays)
#   -> in the GUI: right-click the drone -> Follow, then screen-record.
import argparse
import csv
import glob
import math
import os
import shutil
import signal
import subprocess
import sys
import time

LAB_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(LAB_DIR, "logs")
BIN = os.path.join(LOG_DIR, ".replay_pose")
SRC = os.path.join(LAB_DIR, "replay_pose.cpp")
PX4_DIR = os.environ.get("PX4_DIR", os.path.expanduser("~/PX4-Autopilot"))
WORLD = "dwa_test"
WORLD_FILE = os.path.join(PX4_DIR, "Tools/simulation/gz/worlds", WORLD + ".sdf")
MODEL = "demo_drone"

# Visual-only static quad: dark hub, four arms, red front / black rear rotor
# disks so the heading reads clearly on video. Static => physics never
# fights set_pose.
ARM = 0.18
DRONE_SDF = f"""<sdf version='1.9'><model name='{MODEL}'><static>true</static>
<link name='body'>
<visual name='hub'><geometry><box><size>0.20 0.20 0.08</size></box></geometry>
<material><ambient>0.12 0.12 0.12 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material></visual>
<visual name='arm_fl'><pose>{ARM/2} {ARM/2} 0 0 0 0.7854</pose><geometry><box><size>0.26 0.03 0.02</size></box></geometry>
<material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.25 0.25 0.25 1</diffuse></material></visual>
<visual name='arm_fr'><pose>{ARM/2} {-ARM/2} 0 0 0 -0.7854</pose><geometry><box><size>0.26 0.03 0.02</size></box></geometry>
<material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.25 0.25 0.25 1</diffuse></material></visual>
<visual name='arm_rl'><pose>{-ARM/2} {ARM/2} 0 0 0 -0.7854</pose><geometry><box><size>0.26 0.03 0.02</size></box></geometry>
<material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.25 0.25 0.25 1</diffuse></material></visual>
<visual name='arm_rr'><pose>{-ARM/2} {-ARM/2} 0 0 0 0.7854</pose><geometry><box><size>0.26 0.03 0.02</size></box></geometry>
<material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.25 0.25 0.25 1</diffuse></material></visual>
<visual name='rot_fl'><pose>{ARM} {ARM} 0.05 0 0 0</pose><geometry><cylinder><radius>0.13</radius><length>0.012</length></cylinder></geometry>
<material><ambient>0.8 0.1 0.1 1</ambient><diffuse>0.9 0.1 0.1 1</diffuse></material></visual>
<visual name='rot_fr'><pose>{ARM} {-ARM} 0.05 0 0 0</pose><geometry><cylinder><radius>0.13</radius><length>0.012</length></cylinder></geometry>
<material><ambient>0.8 0.1 0.1 1</ambient><diffuse>0.9 0.1 0.1 1</diffuse></material></visual>
<visual name='rot_rl'><pose>{-ARM} {ARM} 0.05 0 0 0</pose><geometry><cylinder><radius>0.13</radius><length>0.012</length></cylinder></geometry>
<material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.08 0.08 0.08 1</diffuse></material></visual>
<visual name='rot_rr'><pose>{-ARM} {-ARM} 0.05 0 0 0</pose><geometry><cylinder><radius>0.13</radius><length>0.012</length></cylinder></geometry>
<material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.08 0.08 0.08 1</diffuse></material></visual>
</link></model></sdf>"""


def build_streamer():
    if os.path.exists(BIN) and os.path.getmtime(BIN) >= os.path.getmtime(SRC):
        return
    print("building replay_pose streamer...")
    flags = subprocess.check_output(
        ["pkg-config", "--cflags", "--libs", "gz-transport12", "gz-msgs9"]).decode().split()
    subprocess.check_call(["g++", "-O2", SRC, "-o", BIN] + flags)


def load_trajectory(path, run_id):
    """CSV NED -> gz frame (t, x=E, y=N, z=-alt, yaw_gz = pi/2 - yaw_ned)."""
    out = []
    with open(path) as f:
        for r in csv.DictReader(f):
            if r["state"] != "NAVIGATE" or r["run_id"] != str(run_id):
                continue
            try:
                t = float(r["t"])
                n, e = float(r["pos_n"]), float(r["pos_e"])
                z = max(0.0, -float(r["alt"]))
                yaw = math.pi / 2.0 - float(r["yaw"])
                out.append((t, e, n, z, yaw))
            except (KeyError, ValueError):
                continue
    return out


def gz_service(args_list, timeout=10):
    return subprocess.run(["gz", "service"] + args_list,
                          capture_output=True, text=True, timeout=timeout)


def world_running():
    try:
        r = subprocess.run(["gz", "service", "-l"], capture_output=True, text=True, timeout=6)
        return f"/world/{WORLD}/set_pose" in r.stdout
    except subprocess.TimeoutExpired:
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=None, help="session CSV (default: newest in logs/)")
    ap.add_argument("--run", type=int, default=None, help="run_id (default: first with data)")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed factor")
    ap.add_argument("--loop", action="store_true", help="replay forever (Ctrl-C to stop)")
    args = ap.parse_args()

    path = args.csv or max(glob.glob(os.path.join(LOG_DIR, "session_*.csv")),
                           key=os.path.getmtime)
    run_id = args.run
    if run_id is None:
        for cand in range(1, 20):
            if load_trajectory(path, cand):
                run_id = cand
                break
        else:
            sys.exit(f"no NAVIGATE data in {path}")
    traj = load_trajectory(path, run_id)
    if not traj:
        sys.exit(f"no NAVIGATE ticks for run {run_id} in {path}")
    dur = traj[-1][0] - traj[0][0]
    print(f"replaying {os.path.basename(path)} run {run_id}: "
          f"{len(traj)} ticks, {dur:.1f} s at {args.speed}x")

    build_streamer()
    if shutil.which("gz") is None:
        sys.exit("gz not found in PATH")

    gui_proc = None
    if not world_running():
        print("starting Gazebo GUI world (no PX4/lidar - render load is harmless here)...")
        env = dict(os.environ)
        gui_proc = subprocess.Popen(["gz", "sim", "-r", WORLD_FILE], env=env,
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for _ in range(60):
            if world_running():
                break
            time.sleep(1)
        else:
            sys.exit("gz world did not come up in 60 s")

    # spawn the puppet (ignore 'already exists' on re-runs)
    t0 = traj[0]
    req = (f'sdf: "{DRONE_SDF.replace(chr(34), chr(92)+chr(34))}", '
           f'pose: {{position: {{x: {t0[1]}, y: {t0[2]}, z: {t0[3]}}}}}, '
           f'name: "{MODEL}", allow_renaming: false')
    gz_service(["-s", f"/world/{WORLD}/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "3000", "--req", req])

    lines = "".join(f"{t} {x} {y} {z} {yaw}\n" for (t, x, y, z, yaw) in traj)
    print("streaming poses (Ctrl-C to stop). Tip: right-click the drone in "
          "the GUI -> Follow, then screen-record.")
    try:
        while True:
            subprocess.run([BIN, WORLD, MODEL, str(args.speed)],
                           input=lines, text=True)
            if not args.loop:
                break
            time.sleep(1.5)
    except KeyboardInterrupt:
        pass
    finally:
        if gui_proc is not None and gui_proc.poll() is None:
            print("\n(leaving the Gazebo window open - close it yourself, "
                  "or 'pkill -f gz sim')")


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.default_int_handler)
    main()
