#!/usr/bin/env bash
# Add the pieces missing from the PX4-gazebo-models submodule version pinned by
# PX4-Autopilot v1.14.4 (x500_lidar_2d / lidar_2d_v2 model, matching airframe,
# dwa_test world) into PX4-Autopilot.
#
# Usage:
#   ./install.sh [PX4_DIR]
# PX4_DIR defaults to ~/PX4-Autopilot
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${1:-$HOME/PX4-Autopilot}"

if [ ! -f "$PX4_DIR/Makefile" ]; then
  echo "PX4-Autopilot not found: $PX4_DIR (clone + checkout v1.14.4 first, then run this script)"
  exit 1
fi

MODELS_DIR="$PX4_DIR/Tools/simulation/gz/models"
WORLDS_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
AIRFRAMES_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
CMAKELISTS="$AIRFRAMES_DIR/CMakeLists.txt"

echo "Copying gz models: x500_lidar_2d, lidar_2d_v2"
cp -r "$SCRIPT_DIR/models/x500_lidar_2d" "$MODELS_DIR/"
cp -r "$SCRIPT_DIR/models/lidar_2d_v2" "$MODELS_DIR/"

echo "Copying airframe: 4013_gz_x500_lidar_2d"
cp "$SCRIPT_DIR/airframes/4013_gz_x500_lidar_2d" "$AIRFRAMES_DIR/"

echo "Copying world: dwa_test.sdf"
cp "$SCRIPT_DIR/worlds/dwa_test.sdf" "$WORLDS_DIR/"

if grep -q "4013_gz_x500_lidar_2d" "$CMAKELISTS"; then
  echo "CMakeLists.txt already has 4013_gz_x500_lidar_2d, skipping"
else
  echo "Adding 4013_gz_x500_lidar_2d to CMakeLists.txt"
  # Insert right after the 4006_gz_px4vision line
  sed -i '/4006_gz_px4vision/a\	4013_gz_x500_lidar_2d' "$CMAKELISTS"
fi

cat <<EOF

Done. Next steps:
  cd $PX4_DIR
  make px4_sitl gz_x500_lidar_2d                      # default world
  PX4_GZ_WORLD=dwa_test make px4_sitl gz_x500_lidar_2d # test world with obstacles
EOF
