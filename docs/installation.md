# Installation steps (WSL2 + Ubuntu 22.04 + ROS 2 Humble + PX4 v1.14.x + Gazebo)

## 0. WSL2

```bash
wsl --install
wsl --install -d Ubuntu-22.04
```

## 1. Base tools + locale

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y git curl wget tmux python3-pip

sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo apt update
```

## 2. ROS 2 Humble

```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update

sudo apt install -y ros-dev-tools python3-pip
sudo apt install -y ros-humble-desktop
```

## 3. PX4-Autopilot + Gazebo

There's no bare tag for `v1.14`, so pin to an actual existing version (using the latest in that series, `v1.14.4`, here):

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout v1.14.4
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh
```

**Confirm the Gazebo version that actually got installed** (this step matters — the next step needs to install the matching bridge for this version):

```bash
gz sim --version
```

- `Gazebo Sim, version 8.x.x` → Gazebo **Harmonic**
- `Gazebo Sim, version 7.x.x` → Gazebo **Garden**

## 4. Install the matching ros_gz bridge (⚠️ a version mismatch here is the root cause of the issue documented earlier in archive/bug.md)

The default `ros-humble-ros-gz-bridge` for ROS 2 Humble is built against **Gazebo Fortress** (the `ignition-*` package series). Neither Garden nor Harmonic can use this default package — installing it will "succeed" and topics will even show up, but messages never actually arrive (`/clock` won't tick either), because the `gz-msgs`/`gz-transport` (or `ignition-*` from the Fortress era) versions are incompatible and decoding fails. You need to install the meta-package matching the version actually installed, and they conflict with each other — only one can be installed:

```bash
# If the previous step confirmed Harmonic (8.x):
sudo apt remove -y ros-humble-ros-gz-bridge ros-humble-ros-gz 2>/dev/null || true
sudo apt install -y ros-humble-ros-gzharmonic

# If the previous step confirmed Garden (7.x):
sudo apt remove -y ros-humble-ros-gz-bridge ros-humble-ros-gz 2>/dev/null || true
sudo apt install -y ros-humble-ros-gzgarden
```

In practice, PX4 v1.14.4's `Tools/setup/ubuntu.sh` actually installs **Garden (7.x)** on Ubuntu 22.04, not Harmonic as initially assumed at the start of this doc — so always run `gz sim --version` to confirm before picking the matching package; don't just assume Harmonic.

### Minimal verification (test immediately after installing — don't wait until the whole pipeline is wired up to discover it's broken)

Start a standalone Gazebo world and manually bridge `/clock` to confirm data is actually flowing:

```bash
gz sim -r shapes.sdf
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

And in yet another terminal, check:

```bash
ros2 topic echo /clock
```

**You must see a continuously changing timestamp here — that's what confirms the bridge version is correctly installed.** If there's no data at this step, don't proceed further; go back and confirm that `gz sim --version` and the installed bridge package actually match.

## 5. ROS 2 workspace

```bash
mkdir -p ~/ws/src
cd ~/ws/src
git clone https://github.com/PX4/px4_msgs.git

cd ~/ws
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig

cd ~/ws/src
git clone https://github.com/blar-tw/HOLO-DWA.git

cd ~/ws
source /opt/ros/humble/setup.bash
colcon build
```

Add the commonly used source commands to `.bashrc` so you don't have to retype them every time you open a new terminal:

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/ws/install/setup.bash' >> ~/.bashrc
```

## 6. Add the gz models missing from PX4-Autopilot (x500_lidar_2d / lidar_2d_v2)

PX4-Autopilot's `Tools/simulation/gz` is a separate submodule (pointing at [`PX4/PX4-gazebo-models`](https://github.com/PX4/PX4-gazebo-models)). The submodule commit pinned by the `v1.14.4` tag **does not yet have** the `x500_lidar_2d` and `lidar_2d_v2` models, and the required airframe config file (`4013_gz_x500_lidar_2d`) hasn't landed in ROMFS either. Skipping this step makes `make px4_sitl gz_x500_lidar_2d` fail outright (`ninja: error: unknown target 'gz_x500_lidar_2d'`).

These files have already been collected into this repo's [`gz_extra/`](../gz_extra/) (copied from the latest upstream `main` branch of `PX4-gazebo-models` and `PX4-Autopilot` — the SDF/mesh files themselves are independent of the PX4 firmware version, so they can be safely applied to `v1.14.4`). Run the setup script once to copy them into PX4-Autopilot (safe to re-run; it won't add the same line twice):

```bash
cd ~/ws/src/HOLO-DWA
./gz_extra/install.sh ~/PX4-Autopilot
```

`gz_extra/` also includes a test world with a wall and cylinder obstacles (`worlds/dwa_test.sdf`); once installed, you can use:

```bash
PX4_GZ_WORLD=dwa_test make px4_sitl gz_x500_lidar_2d
```

to run and test DWA obstacle avoidance (the drone flies from the origin toward +X, first meets the gapped wall at x=5, then the two cylinders at x=8).

## 7. PX4 parameters: allow offboard mode to work without an RC transmitter

After entering the PX4 SITL console, run:

```
param set NAV_DLL_ACT 0
param set COM_DLL_EXCEPT 4
param set COM_RCL_EXCEPT 4
param save
```

After that you can go straight to:

```
commander mode offboard
commander arm -f
```

## 8. Running it

Drone model reference: `PX4-Autopilot/Tools/simulation/gz/models/x500_lidar_2d/model.sdf`
LiDAR model (always on): `PX4-Autopilot/Tools/simulation/gz/models/lidar_2d_v2/model.sdf`

### Option 1: one-shot launch (recommended)

```bash
cd ~/ws/src/HOLO-DWA
./run.sh
```

### Option 2: manually, across 4 terminals

```bash
# terminal 1
cd ~/PX4-Autopilot
make px4_sitl gz_x500_lidar_2d
```

```bash
# terminal 2
MicroXRCEAgent udp4 -p 8888
```

```bash
# terminal 3
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  --ros-args \
  -r /world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan:=/lidar
```

```bash
# terminal 4
cd ~/ws
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
python3 ~/ws/src/HOLO-DWA/scanner.py
```
