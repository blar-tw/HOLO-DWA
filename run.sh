#!/usr/bin/env bash
#
# run.sh - one-shot launcher for the HOLO-DWA gz simulation stack.
#
# Brings up a 4-pane tmux session (session name: holo-dwa) with:
#   1. PX4 SITL + Gazebo   (gz_x500_lidar_2d, world dwa_test)
#   2. Micro XRCE-DDS Agent (udp4 :8888)
#   3. ros_gz_bridge        (LiDAR scan topic -> /lidar)
#   4. scanner.py           (DWA navigation node)
#
# Usage:
#   ./run.sh                       # defaults (goal 12.0, 0.0)
#   ./run.sh 8.0 -3.0              # goal_x=8.0  goal_y=-3.0
#   PX4_DIR=/path/to/PX4-Autopilot ./run.sh
#   ./run.sh kill                  # tear the session down
#
# Environment overrides:
#   PX4_DIR       PX4-Autopilot checkout        (default: ~/PX4-Autopilot)
#   ROS_SETUP     ROS 2 setup script            (default: /opt/ros/humble/setup.bash)
#   PX4_GZ_WORLD  Gazebo world                  (default: dwa_test)
#   SESSION       tmux session name             (default: holo-dwa)

set -euo pipefail

# --- resolve paths ----------------------------------------------------------
# WS_DIR is the colcon workspace root (holds install/, src/), independent of
# where this script lives. Override with WS_DIR=... if your workspace moves.
WS_DIR="${WS_DIR:-${HOME}/ws}"

PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WS_SETUP="${WS_DIR}/install/setup.bash"
SCANNER="${WS_DIR}/src/HOLO-DWA/scanner.py"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-dwa_test}"
SESSION="${SESSION:-holo-dwa}"

# The gz sensor topic exposed by the x500_lidar_2d model, bridged to /lidar.
# The namespace tracks the world name, so it must match PX4_GZ_WORLD.
LIDAR_TOPIC="/world/${PX4_GZ_WORLD}/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan"

# --- teardown shortcut ------------------------------------------------------
if [[ "${1:-}" == "kill" || "${1:-}" == "stop" ]]; then
  tmux kill-session -t "${SESSION}" 2>/dev/null && echo "Killed tmux session '${SESSION}'." \
    || echo "No tmux session '${SESSION}' running."
  exit 0
fi

# --- navigation goal (optional positional args) -----------------------------
GOAL_X="${1:-12.0}"
GOAL_Y="${2:-0.0}"

# --- sanity checks ----------------------------------------------------------
fail() { echo "ERROR: $*" >&2; exit 1; }

command -v tmux >/dev/null 2>&1 || fail "tmux not found (sudo apt install tmux)."
[[ -d "${PX4_DIR}" ]]     || fail "PX4-Autopilot not found at '${PX4_DIR}' (set PX4_DIR)."
[[ -f "${ROS_SETUP}" ]]   || fail "ROS setup not found at '${ROS_SETUP}' (set ROS_SETUP)."
[[ -f "${WS_SETUP}" ]]    || fail "workspace not built: '${WS_SETUP}' missing (run 'colcon build' in ${WS_DIR})."
[[ -f "${SCANNER}" ]]     || fail "scanner.py not found at '${SCANNER}'."

if tmux has-session -t "${SESSION}" 2>/dev/null; then
  fail "tmux session '${SESSION}' already running. Attach with 'tmux attach -t ${SESSION}' or './run.sh kill' first."
fi

# Source line reused by every ROS pane.
SOURCE_ROS="source '${ROS_SETUP}'; source '${WS_SETUP}'"

echo "Launching '${SESSION}':"
echo "  PX4_DIR       = ${PX4_DIR}"
echo "  PX4_GZ_WORLD  = ${PX4_GZ_WORLD}"
echo "  goal          = (${GOAL_X}, ${GOAL_Y})"

# --- build the tmux layout --------------------------------------------------
# Track panes by their unique tmux pane-id (%N), NOT positional indices:
# split-window renumbers positional indices by on-screen position, which would
# otherwise send commands to the wrong pane.

# Pane A: PX4 SITL + Gazebo
P_SIM="$(tmux new-session -d -P -F '#{pane_id}' -s "${SESSION}" -n sim -c "${PX4_DIR}")"
tmux send-keys -t "${P_SIM}" \
  "PX4_GZ_WORLD=${PX4_GZ_WORLD} make px4_sitl gz_x500_lidar_2d" C-m

# Pane B: Micro XRCE-DDS Agent
P_AGENT="$(tmux split-window -t "${P_SIM}" -h -P -F '#{pane_id}' -c "${WS_DIR}")"
tmux send-keys -t "${P_AGENT}" \
  "${SOURCE_ROS}; MicroXRCEAgent udp4 -p 8888" C-m

# Pane C: ros_gz_bridge (wait for gz to come up first)
P_BRIDGE="$(tmux split-window -t "${P_AGENT}" -v -P -F '#{pane_id}' -c "${WS_DIR}")"
tmux send-keys -t "${P_BRIDGE}" \
  "${SOURCE_ROS}; sleep 12; ros2 run ros_gz_bridge parameter_bridge \
'${LIDAR_TOPIC}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
--ros-args -r '${LIDAR_TOPIC}':=/lidar" C-m

# Pane D: scanner.py (DWA navigation) — wait for bridge + PX4 boot
P_SCAN="$(tmux split-window -t "${P_SIM}" -v -P -F '#{pane_id}' -c "${WS_DIR}")"
tmux send-keys -t "${P_SCAN}" \
  "${SOURCE_ROS}; sleep 18; python3 '${SCANNER}' --ros-args -p goal_x:=${GOAL_X} -p goal_y:=${GOAL_Y}" C-m

tmux select-layout -t "${SESSION}":sim tiled
tmux select-pane   -t "${P_SIM}"

echo "Attaching... (detach: Ctrl-b d   |   teardown: ./run.sh kill)"
tmux attach-session -t "${SESSION}"
