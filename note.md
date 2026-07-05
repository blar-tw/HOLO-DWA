怎麼自己加障礙物，複製貼上這兩個 block、貼在 dwa_test.sdf 裡 </world> 之前，改 <pose> 跟 name 就好：

牆（長方體，<box><size>厚度 長度 高度</size></box>）：


<model name='wall_3'>
  <static>true</static>
  <pose>X Y Z 0 0 0</pose>
  <link name='link'>
    <collision name='collision'>
      <geometry><box><size>0.2 4.5 1</size></box></geometry>
    </collision>
    <visual name='visual'>
      <geometry><box><size>0.2 4.5 1</size></box></geometry>
      <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>
    </visual>
  </link>
</model>

圓柱：

<model name='cylinder_3'>
  <static>true</static>
  <pose>X Y Z 0 0 0</pose>
  <link name='link'>
    <collision name='collision'>
      <geometry><cylinder><radius>0.3</radius><length>1</length></cylinder></geometry>
    </collision>
    <visual name='visual'>
      <geometry><cylinder><radius>0.3</radius><length>1</length></cylinder></geometry>
      <material><ambient>0.8 0.2 0.2 1</ambient><diffuse>0.8 0.2 0.2 1</diffuse></material>
    </visual>
  </link>
</model>

要點：

pose 的 Z 通常設成 <length>/2（高度的一半），讓底部貼地
name 每個 model 要唯一，重複會被 gz 忽略或報錯
牆的 <box><size> 三個數字依序是「厚度（垂直牆面方向）、長度（沿牆延伸方向）、高度」，圓柱同理用 radius/length（length 是圓柱的高度，不是半徑）
如果 sim 已經在跑，不想改檔重開，可以用 gz service 即時生成一個障礙物，不用重啟整個模擬（world 名稱要換成你實際跑的那個，例如 dwa_test）：


gz service -s /world/dwa_test/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf: "<sdf version=\"1.9\"><model name=\"runtime_cyl\"><static>true</static><pose>6 0 0.5 0 0 0</pose><link name=\"link\"><collision name=\"c\"><geometry><cylinder><radius>0.3</radius><length>1</length></cylinder></geometry></collision><visual name=\"v\"><geometry><cylinder><radius>0.3</radius><length>1</length></cylinder></geometry></visual></link></model></sdf>"'

# terminal 1 — PX4 SITL + Gazebo(帶障礙物的 dwa_test world)
cd ~/PX4-Autopilot
PX4_GZ_WORLD=dwa_test make px4_sitl gz_x500_lidar_2d

# terminal 2 — XRCE-DDS Agent
MicroXRCEAgent udp4 -p 8888

# terminal 3 — LiDAR bridge(world 是 dwa_test,跟你選取的筆記一樣)
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/dwa_test/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  --ros-args \
  -r /world/dwa_test/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan:=/lidar

# terminal 4 — DWA 導航節點(goal 預設 12,0,可用參數改)
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
python3 ~/ws/src/HOLO-DWA/scanner.py --ros-args -p goal_x:=12.0 -p goal_y:=0.0


# refresh sdf
cd ~/ws/src/HOLO-DWA
./gz_extra/install.sh ~/PX4-Autopilot

