# demo/ — replay a logged flight in a Gazebo GUI for video recording

On this WSL2 box, any live rendering (GUI or `gz sim -g`) degrades the
flight itself (measured: headless 18 s clean vs 55-90 s wandering with a
GUI attached). So demo footage is captured by **replaying** a logged flight:
fly headless first (the per-tick session CSV *is* the recording), then this
tool puppets a static drone model along that trajectory inside a Gazebo GUI
world. Render load during replay cannot affect the already-flown path.

```bash
cd ~/ws/src/HOLO-DWA/holo_lab

# 1. fly one clean run, headless
NO_ATTACH=1 HEADLESS=1 N_RUNS=1 ./run_lab.sh     # wait for "reached", then:
./run_lab.sh kill

# 2. replay it in a GUI (loops until Ctrl-C)
./demo/replay_demo.py --loop                     # newest CSV
./demo/replay_demo.py --csv logs/exp/final5/session_*.csv --loop   # archived run

# 3. in the GUI: right-click the drone -> Follow, then screen-record
```

Options: `--run N` (pick a run inside the CSV), `--speed 0.5` (slow motion),
`--fps 60` (interpolation rate, default 60).

`replay_pose.cpp` is the pose streamer (one persistent gz-transport node;
spawning `gz service` per pose would cost ~280 ms each). It is compiled
automatically on first use to `demo/.replay_pose`.
