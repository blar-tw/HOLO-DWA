#!/usr/bin/env bash
set -euo pipefail

SESSION_NAME="holo-dwa"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_DIR="$HOME/ws"
PX4_DIR="$HOME/PX4-Autopilot"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCANNER="$SCRIPT_DIR/scanner.py"

LIDAR_TOPIC_PATTERN="lidar_2d_v2/scan"

need_file() {
  if [ ! -e "$1" ]; then
    echo "找不到：$1"
    exit 1
  fi
}

need_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "找不到指令：$1"
    exit 1
  fi
}

need_file "$ROS_SETUP"
need_file "$WS_DIR/src/px4_msgs/package.xml"
need_file "$PX4_DIR/Makefile"
need_file "$SCANNER"
need_command tmux
need_command colcon
need_command MicroXRCEAgent
need_command gz
need_command ros2

if ! ros2 pkg executables ros_gz_bridge 2>/dev/null | grep -q "parameter_bridge"; then
  echo "找不到 ros_gz_bridge parameter_bridge，請先安裝："
  echo "  sudo apt update"
  echo "  sudo apt install ros-humble-ros-gz-bridge"
  exit 1
fi

if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
  echo "tmux session '$SESSION_NAME' 已經存在，直接進入。"
  tmux attach -t "$SESSION_NAME"
  exit 0
fi

if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "第一次執行：正在 build ROS 2 workspace..."
  cd "$WS_DIR"
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  colcon build
fi

tmux new-session -d -s "$SESSION_NAME" -n "holo-dwa"

tmux send-keys -t "$SESSION_NAME:0.0" \
  "cd '$PX4_DIR' && make px4_sitl gz_x500_lidar_2d" C-m

tmux split-window -h -t "$SESSION_NAME:0.0"
tmux send-keys -t "$SESSION_NAME:0.1" \
  "MicroXRCEAgent udp4 -p 8888" C-m

tmux split-window -v -t "$SESSION_NAME:0.0"
tmux send-keys -t "$SESSION_NAME:0.2" \
  "source '$ROS_SETUP' && echo '等待 Gazebo LiDAR topic...' && until LIDAR_GZ_TOPIC=\$(gz topic -l | grep -m1 '$LIDAR_TOPIC_PATTERN'); do sleep 1; done && echo \"使用 LiDAR topic: \$LIDAR_GZ_TOPIC\" && ros2 run ros_gz_bridge parameter_bridge \"\$LIDAR_GZ_TOPIC@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan\" --ros-args -r \"\$LIDAR_GZ_TOPIC:=/lidar\"" C-m

tmux split-window -v -t "$SESSION_NAME:0.1"
tmux send-keys -t "$SESSION_NAME:0.3" \
  "cd '$WS_DIR' && source '$ROS_SETUP' && source '$WS_DIR/install/setup.bash' && sleep 8 && python3 '$SCANNER'" C-m

tmux select-layout -t "$SESSION_NAME:0" tiled >/dev/null

cat <<EOF
已啟動 tmux session：$SESSION_NAME

窗格內容：
  1. PX4 + Gazebo: make px4_sitl gz_x500_lidar_2d
  2. Micro XRCE-DDS Agent
  3. Gazebo LiDAR -> ROS /lidar bridge
  4. scanner.py

離開 tmux 但保持程式執行：按 Ctrl-b，再按 d
結束全部：在 tmux 裡按 Ctrl-b，再輸入 :kill-session
EOF

tmux attach -t "$SESSION_NAME"
