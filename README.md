# HOLO-DWA

在 **PX4 SITL + Gazebo + ROS 2 Humble** 上模擬 holonomic 無人機的 Dynamic Window Approach (DWA) 避障。

程式碼：https://github.com/blar-tw/HOLO-DWA.git

## 目前進度

- ✅ PX4 offboard 控制骨架：arm → takeoff → hover
- ✅ 2D LiDAR 讀取（`/lidar` → `LaserScan`），可算出最近障礙物距離與正前方距離
- ⬜ DWA 速度規劃尚未接上 `scanner.py`，目前只有 hover + 障礙物回報，還不會自動避障

換句話說：整合骨架（PX4 ↔ ROS2 ↔ Gazebo LiDAR）已經打通，DWA 演算法本身還沒接進這個迴圈。這是下一步要做的事。

## 架構 / 資料流

```
Gazebo Sensor Plugin
        │
        ▼
gz.msgs.LaserScan
        │
        ▼
/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan
        │
        ▼
ros_gz_bridge
        │
        ▼
sensor_msgs/msg/LaserScan  (ROS 2 topic: /lidar)
        │
        ▼
scanner.py (DroneLidarScanner node)
        │
        ▼
lidar_callback() → 最近障礙物 / 正前方距離
        │
        ▼
（下一步）DWA 速度規劃 → TrajectorySetpoint → PX4 offboard
```

無人機模型：`x500_lidar_2d`（PX4-Autopilot 內建），LiDAR sensor 用自訂的 `lidar_2d_v2`（常駐開啟，不需額外觸發）。

## 安裝

完整步驟見 [installation.md](installation.md)。安裝文件裡包含一個曾經踩過的版本相容性問題（Gazebo 版本對不上 ROS 2 bridge，見下方 Troubleshooting），照著文件的檢查點做可以避開它。

## 執行

```bash
cd ~/ws/src/HOLO-DWA
./run.sh
```

`run.sh` 會開一個 4-pane tmux（`holo-dwa`），依序啟動：PX4 SITL + Gazebo（`gz_x500_lidar_2d`）、Micro XRCE-DDS Agent、`ros_gz_bridge`（bridge LiDAR topic 到 `/lidar`）、`scanner.py`。

也可以手動分開跑，步驟同樣列在 [installation.md](installation.md) 的「執行」章節。

## Troubleshooting

**Bridge 建立成功、topic 存在，但完全收不到訊息（包括 `/clock`）**：這是 ROS 2 Humble 預設的 `ros-humble-ros-gz-bridge`（對應 Gazebo Fortress）跟 PX4 在 Ubuntu 22.04 上實際裝的 Gazebo Harmonic 版本不相容造成的，topic 看起來有連上但訊息解碼失敗、靜默掉包。正確做法是改裝 `ros-humble-ros-gzharmonic`。完整除錯過程與根因分析見 [bug.md](bug.md)。

## Roadmap

- [ ] 把 DWA 速度規劃接進 `scanner.py`，取代目前的 hover-only 邏輯
- [ ] 把改良版 DWA 演算法（holonomic 相關改動）整理進 repo 並補上說明
- [ ] 補齊 `package.xml` / launch file，讓整個流程可以用 `ros2 launch` 一次帶起來，取代 tmux script
