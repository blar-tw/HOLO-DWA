#!/usr/bin/env bash
#
# run_lab.sh - launcher for the INSTRUMENTED + AUTOMATED HOLO-DWA lab.
#
# Same 4-pane tmux stack as ../run.sh (PX4 SITL+gz / XRCE agent / ros_gz_bridge
# / scanner), except the DWA pane runs holo_lab/scanner_lab.py, which logs every
# tick into holo_lab/logs/ and exposes a /holo_lab/reset service.
#
# The whole point of the lab: bring the heavy stack up ONCE, then iterate with
#   ./run_lab.sh reset
# which flies the drone home and starts a fresh logged run - no PX4/gz/bridge
# restart. Everything this script needs and produces stays inside holo_lab/.
#
# Usage:
#   ./run_lab.sh                 # launch, default goal (12.0, 0.0), 1 run
#   ./run_lab.sh 8.0 -3.0        # launch with goal_x=8.0 goal_y=-3.0
#   ./run_lab.sh reset           # RESTART just the scanner: drone flies home
#                                #   via DWA, then a fresh logged run. PX4/gz/
#                                #   bridge stay up. Sturdiest way to repeat.
#   ./run_lab.sh reset-soft      # in-node reset via service (no restart; can
#                                #   stall if DWA gets stuck returning)
#   ./run_lab.sh gui             # attach a Gazebo GUI window to a RUNNING
#                                #   headless stack (physics keeps running if
#                                #   the window is closed). Note: on this WSL2
#                                #   box live rendering degrades PX4 tracking,
#                                #   so a GUI run wanders more than headless.
#   ./run_lab.sh kill            # tear the whole session down
#
# Batch / automation env vars (read at launch):
#   N_RUNS        navigate runs to fly unattended (<=0 = loop forever)  [1]
#   RUN_TIMEOUT   abort a stuck run after N seconds (0 = never)         [0]
#   NO_ATTACH=1   do not attach to the tmux session after launch (for
#                 driving the lab from scripts / non-interactive shells)
#   HEADLESS=1    run Gazebo server-only (no GUI) - faster, works without
#                 a display; passed through to PX4's sim launcher
#
# Path env overrides (same as ../run.sh):
#   PX4_DIR   ROS_SETUP   PX4_GZ_WORLD   SESSION   WS_DIR

set -euo pipefail
tmux set-option -g mouse on 2>/dev/null || true   # no-op when no tmux server yet
LAB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --- resolve paths ----------------------------------------------------------
WS_DIR="${WS_DIR:-${HOME}/ws}"
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WS_SETUP="${WS_DIR}/install/setup.bash"
SCANNER="${LAB_DIR}/scanner_lab.py"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-dwa_test}"
SESSION="${SESSION:-holo-dwa-lab}"

N_RUNS="${N_RUNS:-1}"
RUN_TIMEOUT="${RUN_TIMEOUT:-0}"
COLLISION_DIST="${COLLISION_DIST:-0.3}"   # LiDAR min-dist below this = collision
NO_ATTACH="${NO_ATTACH:-0}"               # 1 = leave the tmux session detached
HEADLESS="${HEADLESS:-0}"                 # 1 = gz server only (no GUI)

LIDAR_TOPIC="/world/${PX4_GZ_WORLD}/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan"

# --- process cleanup --------------------------------------------------------
kill_sim_procs() {
  local sig="${1:-TERM}"
  pkill -"${sig}" -f 'gz sim'           2>/dev/null || true
  pkill -"${sig}" -x  px4               2>/dev/null || true
  pkill -"${sig}" -f 'MicroXRCEAgent'   2>/dev/null || true
  pkill -"${sig}" -f 'parameter_bridge' 2>/dev/null || true
  pkill -"${sig}" -f 'scanner_lab\.py'  2>/dev/null || true
  pkill -"${sig}" -f 'make px4_sitl'    2>/dev/null || true
}

SIM_PATTERN='gz sim|MicroXRCEAgent|[p]x4|parameter_bridge|scanner_lab\.py'

cleanup_sim() {
  kill_sim_procs TERM
  for _ in $(seq 1 8); do
    pgrep -f "${SIM_PATTERN}" >/dev/null 2>&1 || break
    sleep 0.5
  done
  kill_sim_procs KILL
}

# --- subcommands ------------------------------------------------------------
if [[ "${1:-}" == "kill" || "${1:-}" == "stop" ]]; then
  tmux kill-session -t "${SESSION}" 2>/dev/null && echo "Killed tmux session '${SESSION}'." \
    || echo "No tmux session '${SESSION}' running."
  cleanup_sim
  echo "Cleaned up sim processes (gz / px4 / agent / bridge / scanner_lab)."
  exit 0
fi

if [[ "${1:-}" == "reset" || "${1:-}" == "rerun" ]]; then
  # RESTART ONLY the scanner_lab node - PX4 / gz / bridge keep running. The
  # fresh node re-arms and, if the drone is airborne away from the origin,
  # flies HOME via DWA before starting a new logged run. This is the robust way
  # to repeat a trial: no stuck state machine, and no ~30 s stack relaunch.
  tmux has-session -t "${SESSION}" 2>/dev/null || {
    echo "ERROR: no tmux session '${SESSION}' - launch first with ./run_lab.sh" >&2; exit 1; }
  PANE_FILE="${LAB_DIR}/logs/.scan_pane"
  RESTART="${LAB_DIR}/logs/.restart_scanner.sh"
  [[ -f "${PANE_FILE}" && -f "${RESTART}" ]] || {
    echo "ERROR: no saved scanner pane - relaunch once with ./run_lab.sh" >&2; exit 1; }
  PANE="$(cat "${PANE_FILE}")"
  echo "Restarting scanner_lab in pane ${PANE} (PX4/gz/bridge stay up)..."
  pkill -f 'scanner_lab\.py' 2>/dev/null || true   # stop the old node cleanly
  sleep 1
  tmux respawn-pane -k -t "${PANE}" "bash '${RESTART}'" || {
    echo "ERROR: could not respawn pane ${PANE} - is the session still alive?" >&2; exit 1; }
  echo "Done. Watch the scanner pane: drone returns to origin, then navigates."
  exit 0
fi

if [[ "${1:-}" == "gui" ]]; then
  # Attach a Gazebo GUI client to the already-running (headless) server, to
  # watch the drone. Rendering runs in this separate process; note that on
  # WSL2 the extra render load still costs enough real-time factor that PX4
  # velocity tracking degrades (measured: displacement/commanded speed ratio
  # 0.61 with GUI vs 0.87 headless), so a watched run wanders more.
  pgrep -f 'gz sim' >/dev/null 2>&1 || {
    echo "ERROR: no gz server running - launch first: HEADLESS=1 ./run_lab.sh" >&2; exit 1; }
  echo "Attaching Gazebo GUI to the running server (close the window to detach)..."
  exec gz sim -g
fi

if [[ "${1:-}" == "reset-soft" ]]; then
  # In-node reset via the /holo_lab/reset service (no process restart). Lighter,
  # but can stall if DWA gets stuck returning through the obstacle field;
  # './run_lab.sh reset' (a scanner restart) is the sturdier option.
  set +eu
  source "${ROS_SETUP}"
  [[ -f "${WS_SETUP}" ]] && source "${WS_SETUP}"
  set -eu
  if ! ros2 service list 2>/dev/null | grep -qx '/holo_lab/reset'; then
    echo "ERROR: service /holo_lab/reset not found (is the sim running?)." >&2
    exit 1
  fi
  echo "Calling /holo_lab/reset ..."
  ros2 service call /holo_lab/reset std_srvs/srv/Trigger
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
[[ -f "${SCANNER}" ]]     || fail "scanner_lab.py not found at '${SCANNER}'."

if tmux has-session -t "${SESSION}" 2>/dev/null; then
  fail "tmux session '${SESSION}' already running. Attach with 'tmux attach -t ${SESSION}' or './run_lab.sh kill' first."
fi

if pgrep -f "${SIM_PATTERN}" >/dev/null 2>&1; then
  echo "Found leftover sim processes from a previous run - cleaning up first..."
  cleanup_sim
fi

SOURCE_ROS="source '${ROS_SETUP}'; source '${WS_SETUP}'"

mkdir -p "${LAB_DIR}/logs"

echo "Launching '${SESSION}' (lab):"
echo "  PX4_DIR       = ${PX4_DIR}"
echo "  PX4_GZ_WORLD  = ${PX4_GZ_WORLD}"
echo "  goal          = (${GOAL_X}, ${GOAL_Y})"
echo "  N_RUNS        = ${N_RUNS}   RUN_TIMEOUT = ${RUN_TIMEOUT}s"
echo "  COLLISION_DIST= ${COLLISION_DIST}m"
echo "  logs          -> ${LAB_DIR}/logs/"

# --- build the tmux layout --------------------------------------------------
# Pane A: PX4 SITL + Gazebo
P_SIM="$(tmux new-session -d -P -F '#{pane_id}' -s "${SESSION}" -n sim -c "${PX4_DIR}")"
SIM_ENV="PX4_GZ_WORLD=${PX4_GZ_WORLD}"
[[ "${HEADLESS}" == "1" ]] && SIM_ENV="${SIM_ENV} HEADLESS=1"
tmux send-keys -t "${P_SIM}" \
  "${SIM_ENV} make px4_sitl gz_x500_lidar_2d" C-m

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

# Shared scanner CLI args (also baked into the restart helper below).
SCAN_ARGS="-p goal_x:=${GOAL_X} -p goal_y:=${GOAL_Y} \
-p n_runs:=${N_RUNS} -p run_timeout:=${RUN_TIMEOUT} -p collision_dist:=${COLLISION_DIST}"

# Restart helper used by './run_lab.sh reset' to relaunch JUST the scanner
# (no gz-boot sleep needed - the stack is already up).
RESTART="${LAB_DIR}/logs/.restart_scanner.sh"
cat > "${RESTART}" <<EOF
#!/usr/bin/env bash
set +u
source '${ROS_SETUP}'
[ -f '${WS_SETUP}' ] && source '${WS_SETUP}'
exec python3 '${SCANNER}' --ros-args ${SCAN_ARGS}
EOF
chmod +x "${RESTART}"

# Pane D: scanner_lab.py (DWA navigation + instrumentation) — wait for boot
P_SCAN="$(tmux split-window -t "${P_SIM}" -v -P -F '#{pane_id}' -c "${LAB_DIR}")"
printf '%s' "${P_SCAN}" > "${LAB_DIR}/logs/.scan_pane"   # for ./run_lab.sh reset
tmux send-keys -t "${P_SCAN}" \
  "${SOURCE_ROS}; sleep 18; python3 '${SCANNER}' --ros-args ${SCAN_ARGS}" C-m

tmux select-layout -t "${SESSION}":sim tiled
tmux select-pane   -t "${P_SIM}"

if [[ "${NO_ATTACH}" == "1" || ! -t 0 ]]; then
  echo "Launched detached (attach: tmux attach -t ${SESSION}  |  new run: ./run_lab.sh reset  |  teardown: ./run_lab.sh kill)"
  exit 0
fi
echo "Attaching... (detach: Ctrl-b d   |   new run: ./run_lab.sh reset   |   teardown: ./run_lab.sh kill)"
tmux attach-session -t "${SESSION}"
