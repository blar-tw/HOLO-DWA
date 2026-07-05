# HOLO-DWA

在 **PX4 SITL + Gazebo + ROS 2 Humble** 上模擬 holonomic 無人機的 Dynamic Window Approach (DWA) 避障。

程式碼：https://github.com/blar-tw/HOLO-DWA.git

## 目前進度

- ✅ PX4 offboard 控制骨架：arm → takeoff → hover
- ✅ 2D LiDAR 讀取（`/lidar` → `LaserScan`），可算出最近障礙物距離與正前方距離
- ✅ DWA 速度規劃已接上 `scanner.py`（`dwa_core.py`），取代原本的 hover-only 邏輯，起飛到高度後會自動朝目標點導航並用 LiDAR 點雲避障

`dwa_core.py` 是從 `dwa.py`（PyBullet 離線原型）抽出來、去掉 pybullet 依賴的共用演算法：一樣的 holonomic DWA 速度視窗搜尋與評分邏輯，差別只在障礙物來源改成 LiDAR 掃描轉出來的 2D 點雲，並整個向量化以應付 20Hz 控制迴圈。`dwa.py` 本身維持不動，繼續當作可視化/離線測試用的原型。

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
        ├─ lidar_callback() → 儲存最新掃描 + 最近障礙物/正前方距離回報
        ├─ odom_callback()  → 目前位置 / 速度 / yaw
        │
        ▼
run_dwa_navigation() → dwa_core.dwa_control() → (vx, vy)
        │
        ▼
publish_velocity_setpoint() → TrajectorySetpoint (x/y 速度控制、z 位置控制) → PX4 offboard
```

導航目標點透過 ROS 2 參數 `goal_x` / `goal_y` 設定（預設 `12.0, 0.0`）。**座標是 Gazebo world 座標**（跟 `dwa_test.sdf` 裡障礙物的座標同一個 frame），程式內部會轉成 PX4 NED（北=gz_y、東=gz_x）；無人機起飛時就會轉向目標、飛行途中機頭也會持續朝著目標。例如：

```bash
python3 ~/ws/src/HOLO-DWA/scanner.py --ros-args -p goal_x:=12.0 -p goal_y:=0.0
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

## 演算法討論

DWA 目前遇到的演算法層級問題（速度獎勵的分量/純量取捨、門口困境）與候選解法整理在 [discussion.md](discussion.md)。

## Roadmap

- [x] 把 DWA 速度規劃接進 `scanner.py`，取代目前的 hover-only 邏輯
- [x] 把改良版 DWA 演算法（holonomic 相關改動）整理進 repo 並補上說明（見 `dwa_core.py`）
- [ ] 補齊 `package.xml` / launch file，讓整個流程可以用 `ros2 launch` 一次帶起來，取代 tmux script
- [ ] 實機/實測驗證：LiDAR 安裝偏移量（目前假設 LiDAR 與機身原點重合）、goal 座標與 Gazebo world 座標的對應關係
