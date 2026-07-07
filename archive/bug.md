## ✅ Resolved — Root Cause & Fix

**根因**：`ros-humble-ros-gz-bridge`（ROS 2 Humble 的預設 apt 套件）是對應 **Gazebo Fortress** 編譯的，但 PX4 v1.14.x 的 `Tools/setup/ubuntu.sh` 在 Ubuntu 22.04 上裝的是 **Gazebo Harmonic**（本篇一路記錄下來的 `gz-msgs 10.3.2` / `gz-transport 13.5.0` 就是 Harmonic 的版本序列，不是 Fortress 的）。兩邊的 `gz-msgs`/`gz-transport` protobuf ABI 不相容，所以：

- Gazebo Transport 層本身完全正常（`gz topic -e` 看得到資料）
- ROS 端的 topic 與 bridge node「看起來」建立成功（topic discovery 本來就不檢查訊息內容）
- 但實際訊息完全解碼失敗、靜默掉包 → `/clock`、`/scan` 全部沒資料，並出現本篇第 7 節那個 `Unknown message type [8]/[9]`（這是 gz-msgs 版本不同時的典型症狀）

第 12 節當時猜測的「PX4 啟動時把 Gazebo Transport runtime 隔離了」是錯誤方向 —— 這不是 runtime 隔離問題，是單純的套件版本不匹配。

**修法**：`ros-humble-ros-gzharmonic` 與 `ros-humble-ros-gz-bridge` / `ros-humble-ros-gz` 互相衝突、不能共存，改裝前者：

```bash
sudo apt remove -y ros-humble-ros-gz-bridge ros-humble-ros-gz 2>/dev/null || true
sudo apt install -y ros-humble-ros-gzharmonic
```

裝完之後用 `/clock` 做最小驗證（獨立跑 `gz sim`，手動 bridge `/clock`，`ros2 topic echo /clock` 應該要有持續變動的時間戳），確認 bridge 真的在動之後，再接上完整的 LiDAR pipeline。完整安裝流程已經寫進 [installation.md](installation.md) 的第 3、4 節。

參考資料：
- [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide)
- [Gazebo Harmonic + ROS installation](https://gazebosim.org/docs/harmonic/ros_installation/)
- [PX4-Autopilot#22180 — Gazebo SITL does not work if Gazebo Harmonic is installed](https://github.com/PX4/PX4-Autopilot/issues/22180)
- [gazebosim/ros_gz](https://github.com/gazebosim/ros_gz)

---

# PX4 + Gazebo Sim 8 + ros_gz_bridge Debug Log

## 📌 Project Context

This log documents a debugging session involving:

- PX4 SITL (`make px4_sitl gz_x500_lidar_2d`)
- Gazebo Sim 8.14.0
- ROS 2 Humble
- ros_gz_bridge (apt version 0.244.24)
- GPU LiDAR sensor (`lidar_2d_v2`)
- Goal: Bridge Gazebo LiDAR → ROS 2 `sensor_msgs/msg/LaserScan`
- Eventually support DWA-based UAV navigation

---

# 1. 🧨 Initial Problem

## Symptom

User expected ROS topic:


/scan


to receive LiDAR data from Gazebo, but:

- `ros2 topic echo /scan` → **no output**
- `ros2 topic info /scan` → **no publisher or inconsistent state**
- Bridge appeared to start successfully

---

# 2. 🔍 Initial Observations

## Gazebo side confirmed working:


gz topic -e -t /world/.../lidar_2d_v2/scan


✔ LiDAR data streaming correctly  
✔ angle_min / angle_max / ranges present  
✔ simulation running at real-time factor ~1.0  

---

## ROS side:

- `/scan` existed but no data
- bridge node existed but no message flow
- echo command stuck or silent

---

# 3. 🔧 First Hypothesis: Topic mismatch

### Issue suspected:
Incorrect Gazebo topic name due to PX4 instance suffix

### Discovery:

Gazebo actual topic:


/world/default/model/x500_lidar_2d_0/...


but user initially used:


/world/default/model/x500_lidar_2d/...


### Fix applied:
Corrected topic to include `_0`

✔ This fixed topic resolution issues  
❌ Still no ROS data

---

# 4. 🔧 Second Hypothesis: ros_gz_bridge QoS mismatch

### Test:

ros2 topic echo --qos-reliability best_effort


### Result:

- Still no data

### Conclusion:

❌ QoS mismatch NOT the root cause

---

# 5. 🔧 Third Observation: Bridge appears connected


ros2 topic info /scan -v


Result:


Publisher count: 1 (ros_gz_bridge)
Subscription count: 0


### Interpretation:

✔ ROS topic created  
✔ Bridge node exists  
❌ No message flow

---

# 6. 🔧 Gazebo Transport verification


gz topic -i -t scan


Confirmed:

- Publisher exists
- Subscriber (ros_gz_bridge) connected at TCP level

BUT:

❗ No actual ROS messages received

---

# 7. 🔥 Critical Anomaly Detected

During bridge startup:


Unknown message type [8]
Unknown message type [9]


### Meaning:

This indicates:

- internal Gazebo message decoding failure OR
- ros_gz_bridge binary mismatch with gz-msgs runtime

---

# 8. 🔧 System Version Check

## Gazebo


gz sim 8.14.0


## ROS bridge


ros-humble-ros-gz-bridge 0.244.24


## Gazebo messaging stack


gz-msgs 10.3.2
gz-transport 13.5.0
protobuf 3.12


✔ Versions appear compatible on paper  
❗ but runtime mismatch suspected

---

# 9. 🔧 ros_gz_bridge capability check


ros2 run ros_gz_bridge bridge_types | grep Laser


Result:


(no output)


### Interpretation:

❗ LaserScan conversion mapping not registered OR bridge runtime not fully initialized

---

# 10. 🔧 Minimal test: /clock bridge

Test:


ros2 run ros_gz_bridge parameter_bridge
/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock


Result:


ros2 topic echo /clock → no output


### Critical conclusion:

❗ ros_gz_bridge is NOT receiving ANY Gazebo Transport messages

---

# 11. 🧪 Isolation testing

User confirmed:

- Gazebo runs independently ✔
- LiDAR publishes correctly ✔
- ros_gz_bridge node runs ✔
- ROS topics exist ✔

BUT:

> No ROS message flow at all

---

# 12. 💣 Final Root Cause Analysis

After eliminating:

- ❌ topic name mismatch
- ❌ QoS mismatch
- ❌ ROS subscriber issues
- ❌ LiDAR SDF problems
- ❌ PX4 startup errors (partial)

The remaining root cause:

## 🚨 PX4 Gazebo launch isolation of transport domain

PX4 SITL launch introduces a **separate Gazebo Transport runtime context**, causing:

- Gazebo CLI sees topics ✔
- PX4 sees sensors ✔
- ros_gz_bridge attaches ✔
- BUT does NOT receive message callbacks ❌

### Effect:


Gazebo publishes data
↓
ros_gz_bridge sees topic existence
↓
BUT message event loop never triggers
↓
ROS receives nothing (/clock included)


---

# 13. 🧠 Key Insight

This is NOT a ROS issue.

This is NOT a Gazebo sensor issue.

This is:

> ❗ Gazebo Transport runtime isolation caused by PX4 SITL launch environment

---

# 14. 🧪 Final Diagnostic Proof

### Working:


gz topic -e → data exists


### Broken:


ros2 topic echo /clock → no output
ros2 topic echo /scan → no output
bridge_types → empty


### Interpretation:

✔ Gazebo layer healthy  
❌ ros_gz_bridge not receiving transport events

---

# 15. 🛠️ Final Resolution Strategy

## Option A (Recommended for debugging)

Run Gazebo independently:

```bash
gz sim -r <world>

Then manually attach bridge:

ros2 run ros_gz_bridge parameter_bridge \
/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
/world/.../scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan

✔ Fully functional pipeline

Option B (PX4 integration fix)

Requires:

alignment of Gazebo instance namespace
explicit GZ_PARTITION / ROS_DOMAIN_ID sync
PX4 launch configuration adjustment
Option C (Long-term robust solution)
rebuild ros_gz from source
align Gazebo Sim + PX4 + ROS2 versions explicitly
avoid apt hybrid stack mismatches
16. 📌 Final Conclusion
Root Cause:

PX4 SITL Gazebo launch environment isolates Gazebo Transport runtime, preventing ros_gz_bridge from receiving message callbacks, even though topics are visible.

Impact:
No ROS LiDAR data
No /clock synchronization
No bridge streaming
Resolution Path:
bypass PX4 launch OR
fix Gazebo transport domain alignment OR
rebuild ros_gz stack consistently
17. 🚀 Outcome

After diagnosis:

topic naming issue resolved
bridge configuration validated
Gazebo sensor confirmed healthy
root cause isolated to transport runtime architecture

This reduces debugging space from:

“ROS / Gazebo / PX4 / LiDAR issue”

to:

“Gazebo Transport runtime isolation problem caused by PX4 launch system”
