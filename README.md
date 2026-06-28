# HOLO-DWA

Gazebo + PX4 + ROS 2 Humble 的 DWA 模擬專案。

## Installation

以下流程以 **WSL2 + Ubuntu 22.04 + ROS 2 Humble + PX4 SITL/Gazebo** 為主。

### 1. 安裝 WSL 與 Ubuntu 22.04

在 Windows PowerShell 或 Windows Terminal 執行：

```powershell
wsl --install
wsl --install -d Ubuntu-22.04
```

完成 Ubuntu 初次設定後，進入 WSL 終端機。

### 2. 安裝基本工具

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y git curl wget tmux python3-pip
```

### 3. 設定 locale

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 4. 安裝 ROS 2 Humble

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo apt update && sudo apt install -y curl

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update

sudo apt install -y ros-dev-tools python3-pip
sudo apt install -y ros-humble-desktop
```

安裝完成後，可以先 source ROS 環境：

```bash
source /opt/ros/humble/setup.bash
```

如果希望每次開啟 WSL 都自動載入 ROS：

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 5. 安裝 PX4 與 Gazebo

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
```

執行 PX4 官方 Ubuntu 安裝腳本，安裝 PX4 SITL 需要的依賴與對應版本的 Gazebo：

```bash
bash ./Tools/setup/ubuntu.sh
```

安裝完成後，建議重新開啟 WSL 終端機，或重新載入 shell 環境。

### 6. 建立 ROS 2 workspace

```bash
mkdir -p ~/ws/src
cd ~/ws/src
```

下載 PX4 官方 ROS 2 訊息定義：

```bash
git clone https://github.com/PX4/px4_msgs.git
```

### 7. 安裝 Micro XRCE-DDS Agent

```bash
cd ~/ws
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
```

### 8. 下載本專案

```bash
cd ~/ws/src
git clone https://github.com/blar-tw/HOLO-DWA.git
```

如果你已經在 `~/ws/src/HOLO-DWA` 裡面，就可以略過這一步。

### 9. 建置 workspace

```bash
cd ~/ws
source /opt/ros/humble/setup.bash
colcon build
```

建置完成後載入 workspace：

```bash
source ~/ws/install/setup.bash
```

如果希望每次開啟 WSL 都自動載入此 workspace：

```bash
echo "source ~/ws/install/setup.bash" >> ~/.bashrc
```
