#!/usr/bin/env bash
# 把 PX4-Autopilot v1.14.4 pin 住的 PX4-gazebo-models submodule 版本裡缺少的東西
# （x500_lidar_2d / lidar_2d_v2 model、對應 airframe、dwa_test world）補進 PX4-Autopilot。
#
# 用法：
#   ./install.sh [PX4_DIR]
# PX4_DIR 預設 ~/PX4-Autopilot
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${1:-$HOME/PX4-Autopilot}"

if [ ! -f "$PX4_DIR/Makefile" ]; then
  echo "找不到 PX4-Autopilot：$PX4_DIR（先 clone + checkout v1.14.4 再跑這個 script）"
  exit 1
fi

MODELS_DIR="$PX4_DIR/Tools/simulation/gz/models"
WORLDS_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
AIRFRAMES_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
CMAKELISTS="$AIRFRAMES_DIR/CMakeLists.txt"

echo "複製 gz model：x500_lidar_2d, lidar_2d_v2"
cp -r "$SCRIPT_DIR/models/x500_lidar_2d" "$MODELS_DIR/"
cp -r "$SCRIPT_DIR/models/lidar_2d_v2" "$MODELS_DIR/"

echo "複製 airframe：4013_gz_x500_lidar_2d"
cp "$SCRIPT_DIR/airframes/4013_gz_x500_lidar_2d" "$AIRFRAMES_DIR/"

echo "複製 world：dwa_test.sdf"
cp "$SCRIPT_DIR/worlds/dwa_test.sdf" "$WORLDS_DIR/"

if grep -q "4013_gz_x500_lidar_2d" "$CMAKELISTS"; then
  echo "CMakeLists.txt 已經有 4013_gz_x500_lidar_2d，跳過"
else
  echo "在 CMakeLists.txt 加入 4013_gz_x500_lidar_2d"
  # 插在 4006_gz_px4vision 那一行後面
  sed -i '/4006_gz_px4vision/a\	4013_gz_x500_lidar_2d' "$CMAKELISTS"
fi

cat <<EOF

完成。接下來可以：
  cd $PX4_DIR
  make px4_sitl gz_x500_lidar_2d                      # 預設 world
  PX4_GZ_WORLD=dwa_test make px4_sitl gz_x500_lidar_2d # 帶障礙物的測試 world
EOF
