# 安裝步驟 (WSL2 + Ubuntu 22.04 + ROS 2 Humble + PX4 v1.14.x + Gazebo)

## 0. WSL2

```bash
wsl --install
wsl --install -d Ubuntu-22.04
```

## 1. 基礎工具 + locale

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

`v1.14` 沒有對應的裸 tag，要 pin 到實際存在的版本（這裡用該系列最新的 `v1.14.4`）：

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout v1.14.4
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh
```

**確認實際裝到的 Gazebo 版本**（這一步很重要，下一步要對應這個版本裝正確的 bridge）：

```bash
gz sim --version
```

- `Gazebo Sim, version 8.x.x` → Gazebo **Harmonic**
- `Gazebo Sim, version 7.x.x` → Gazebo **Garden**

## 4. 安裝對應版本的 ros_gz bridge（⚠️ 版本不對就是先前 bug.md 那個問題的根因）

ROS 2 Humble 預設的 `ros-humble-ros-gz-bridge` 是對應 **Gazebo Fortress**（`ignition-*` 系列套件）編譯的，Garden／Harmonic 都不能用這個預設套件，裝下去雖然會成功、topic 也看得到，但訊息完全收不到（`/clock` 也不會動），因為 `gz-msgs`/`gz-transport`（或 Fortress 時代的 `ignition-*`）版本互不相容、解碼失敗。要照實際裝到的版本裝對應的 meta-package，而且它們會互相衝突、只能裝一個：

```bash
# 若上一步確認是 Harmonic (8.x)：
sudo apt remove -y ros-humble-ros-gz-bridge ros-humble-ros-gz 2>/dev/null || true
sudo apt install -y ros-humble-ros-gzharmonic

# 若上一步確認是 Garden (7.x)：
sudo apt remove -y ros-humble-ros-gz-bridge ros-humble-ros-gz 2>/dev/null || true
sudo apt install -y ros-humble-ros-gzgarden
```

實測下來，PX4 v1.14.4 的 `Tools/setup/ubuntu.sh` 在 Ubuntu 22.04 上裝的其實是 **Garden (7.x)**，不是本文一開始猜測的 Harmonic，所以務必先跑 `gz sim --version` 確認再挑對應套件，不要直接假設是 Harmonic。

### 最小驗證（裝完立刻測試，不要等到接上整條 pipeline 才發現壞掉）

獨立開一個 Gazebo world，手動 bridge `/clock`，確認真的有資料流出來：

```bash
gz sim -r shapes.sdf
```

另開一個 terminal：

```bash
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

再開一個 terminal 檢查：

```bash
ros2 topic echo /clock
```

**這裡必須要能看到持續變動的時間戳，才代表 bridge 版本裝對了。** 如果這步就沒資料，先別往下走，回頭確認 `gz sim --version` 跟裝的 bridge 套件是否真的對應。

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

把常用的 source 指令寫進 `.bashrc`，避免每次開新 terminal 都要重打：

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/ws/install/setup.bash' >> ~/.bashrc
```

## 6. 補齊 PX4-Autopilot 缺少的 gz model（x500_lidar_2d / lidar_2d_v2）

`PX4-Autopilot` 的 `Tools/simulation/gz` 是一個獨立 submodule（指向 [`PX4/PX4-gazebo-models`](https://github.com/PX4/PX4-gazebo-models)），`v1.14.4` 這個 tag 當時 pin 住的 submodule commit **還沒有** `x500_lidar_2d` 跟 `lidar_2d_v2` 這兩個 model，需要的 airframe 設定檔（`4013_gz_x500_lidar_2d`）也還沒進到 ROMFS。如果跳過這步，`make px4_sitl gz_x500_lidar_2d` 會直接失敗（`ninja: error: unknown target 'gz_x500_lidar_2d'`）。

這幾個檔案已經整理進本 repo 的 [`gz_extra/`](../gz_extra/)（從 `PX4-gazebo-models` 和 `PX4-Autopilot` 上游最新 `main` branch 複製，SDF/mesh 本身跟 PX4 韌體版本無關，可以安全套用到 `v1.14.4`）。跑一次 setup script 把它們複製進 PX4-Autopilot（可重複執行，不會重複加同一行）：

```bash
cd ~/ws/src/HOLO-DWA
./gz_extra/install.sh ~/PX4-Autopilot
```

`gz_extra/` 裡也附了一個帶牆跟圓柱障礙物的測試 world（`worlds/dwa_test.sdf`），裝完之後可以用：

```bash
PX4_GZ_WORLD=dwa_test make px4_sitl gz_x500_lidar_2d
```

跑起來測 DWA 避障（無人機從原點往 +X 飛，會先遇到 x=5 帶缺口的牆，再遇到 x=8 的兩根圓柱）。

## 7. PX4 參數：允許 offboard 模式在沒有 RC 遙控器的情況下運作

進到 PX4 SITL console 後執行：

```
param set NAV_DLL_ACT 0
param set COM_DLL_EXCEPT 4
param set COM_RCL_EXCEPT 4
param save
```

之後就能直接：

```
commander mode offboard
commander arm -f
```

## 8. 執行

無人機模型參考：`PX4-Autopilot/Tools/simulation/gz/models/x500_lidar_2d/model.sdf`
LiDAR 模型（常駐開啟）：`PX4-Autopilot/Tools/simulation/gz/models/lidar_2d_v2/model.sdf`

### 方式一：一鍵啟動（推薦）

```bash
cd ~/ws/src/HOLO-DWA
./run.sh
```

### 方式二：手動分 4 個 terminal

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

## 已知問題

見 [bug.md](bug.md) — 記錄了「bridge 建立成功但完全收不到訊息」的完整除錯過程與根因（就是第 4 節那個版本問題）。
