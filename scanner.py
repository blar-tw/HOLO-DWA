#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import math

# 引入 PX4 通訊格式
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry
# 引入雷達通訊格式
from sensor_msgs.msg import LaserScan

class DroneLidarScanner(Node):
    def __init__(self):
        super().__init__('scanner')

        # PX4 要求的 QoS 設定 (通訊品質設定)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 1. 建立 Publisher (對飛控下指令)
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # 2. 建立 Subscriber (讀取飛控狀態與雷達資料)
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile_sensor_data)
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.lidar_sub = self.create_subscription(LaserScan, '/lidar', self.lidar_callback, lidar_qos)
        self.gz_lidar_sub = self.create_subscription(
            LaserScan,
            '/world/default/model/x500_lidar_2d/link/link/sensor/lidar_2d_v2/scan',
            self.lidar_callback,
            lidar_qos
        )

        # 3. 建立計時器 (20Hz，用於發送心跳訊號與控制迴圈)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # 狀態變數
        self.nav_state = "INIT"
        self.heartbeat_counter = 0
        self.current_alt = 0.0
        self.takeoff_alt = -2.0  # PX4 使用 NED 座標系 (Z軸朝下)，所以 -2.0 代表往上飛 2 公尺
        self.last_lidar_time = None
        self.last_lidar_warning_time = 0.0

    def odom_callback(self, msg):
        """即時更新無人機的高度"""
        self.current_alt = msg.position[2] 

    def lidar_callback(self, msg):
        """處理 2D LiDAR 掃描數據"""
        # 只有在抵達指定高度懸停時，才開始回報雷達數據
        if self.nav_state != "HOVER_AND_SCAN":
            return 

        self.last_lidar_time = self.get_clock().now()
        ranges = np.array(msg.ranges)
        
        # 過濾掉無效值 (inf 或超過最大範圍的點)
        valid_indices = np.where((ranges > msg.range_min) & (ranges < msg.range_max))[0]
        
        if len(valid_indices) == 0:
            self.get_logger().info("👀 環顧四周：非常空曠，雷達範圍內無障礙物！")
            return

        # 1. 找出四周「最近」的障礙物
        min_dist = np.min(ranges[valid_indices])
        min_idx = valid_indices[np.argmin(ranges[valid_indices])]
        
        # 計算該障礙物的實際角度 (度數)
        min_angle_rad = msg.angle_min + min_idx * msg.angle_increment
        min_angle_deg = math.degrees(min_angle_rad)

        # 2. 讀取「正前方」的距離，取 0 度左右約 10 度範圍
        front_center = int(round((0.0 - msg.angle_min) / msg.angle_increment))
        front_half_width = max(1, int(round(math.radians(10.0) / msg.angle_increment)))
        front_start = max(0, front_center - front_half_width)
        front_end = min(len(ranges), front_center + front_half_width + 1)
        front_slice = ranges[front_start:front_end]
        valid_front = front_slice[(front_slice > msg.range_min) & (front_slice < msg.range_max)]
        front_dist = np.min(valid_front) if len(valid_front) > 0 else float('inf')

        # 終端機漂亮輸出
        self.get_logger().info(
            f"🎯 [掃描回報] 正前方距離: {front_dist:.2f} m | 🚨 全向最近威脅: {min_dist:.2f} m (方位: {min_angle_deg:.1f}°)"
        )

    def timer_callback(self):
        """主控制迴圈 (20Hz)"""
        # 1. 永遠保持發送 Offboard 心跳 (PX4 安全機制)
        self.publish_offboard_control_heartbeat()

        # 2. 狀態機控制
        if self.nav_state == "INIT":
            # 先發送 20 次心跳，確保通訊穩定後再解鎖
            if self.heartbeat_counter == 20:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # 切換 Offboard
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0) # 馬達解鎖
                self.nav_state = "TAKEOFF"
                self.get_logger().info("Arming")
            
        elif self.nav_state == "TAKEOFF":
            # 給定目標點 (原地起飛至 2m 高)
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_alt)
            
            # 判斷是否抵達高度 (誤差 0.2m 內)
            if self.current_alt < (self.takeoff_alt + 0.2): 
                self.nav_state = "HOVER_AND_SCAN"
                self.get_logger().info("🚁 到達目標高度！開始懸停並啟動 LiDAR 掃描...")

        elif self.nav_state == "HOVER_AND_SCAN":
            # 維持在 2m 高度懸停 (此時 lidar_callback 會開始瘋狂印出資料)
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_alt)
            now = self.get_clock().now()
            now_sec = now.nanoseconds / 1e9

            if self.last_lidar_time is None and now_sec - self.last_lidar_warning_time > 2.0:
                self.get_logger().warn("尚未收到 LiDAR 資料，請確認 Gazebo scan topic 與 ros_gz_bridge 是否正在發布。")
                self.last_lidar_warning_time = now_sec

        self.heartbeat_counter += 1

    # --- PX4 底層發布函式 ---
    def publish_offboard_control_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0 # 面向北方
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneLidarScanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KBinterrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
