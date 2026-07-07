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

import dwa_core

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

        # LiDAR readouts stored by lidar_callback, printed by the throttled
        # DWA status line (so the whole set of numbers comes out together)
        self.front_dist = float('inf')
        self.min_threat = float('inf')
        self.min_threat_angle = 0.0
        # Wall-clock throttle for the consolidated DWA status print (~1 Hz)
        self.last_status_print_sec = 0.0
        self.status_print_period = 1.0

        # 里程計狀態 (與 LiDAR 同一個 local frame，供 DWA 使用)
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.yaw = 0.0
        self.latest_scan = None
        self.lidar_stride = 6  # 1080 條光束降採樣到 ~180 個障礙點，加快 DWA 搜尋

        # Goal is given in Gazebo world coordinates (ENU: x East, y North),
        # the same frame the obstacles in dwa_test.sdf are placed in.
        # PX4's local frame is NED (x North, y East), so the goal is axis-
        # swapped into NED right before use — see get_goal_ned().
        self.declare_parameter('goal_x', 12.0)
        self.declare_parameter('goal_y', 0.0)

        # Hold target captured at the moment the goal is reached
        self.hold_x = 0.0
        self.hold_y = 0.0
        self.hold_yaw = 0.0

        self.dwa_config = dwa_core.Config()
        self.dwa_config.v_max = 1.5
        self.dwa_config.vx_min = -1.5
        self.dwa_config.vx_max = 1.5
        self.dwa_config.vy_min = -1.5
        self.dwa_config.vy_max = 1.5
        self.dwa_config.a_max = 1.0
        self.dwa_config.brake_a_max = 1.0
        self.dwa_config.vx_resolution = 0.1
        self.dwa_config.vy_resolution = 0.1
        self.dwa_config.control_dt = 0.2
        self.dwa_config.predict_time = 3.0
        self.dwa_config.predict_dt = 0.2
        # Planner keep-out must be >= the 0.3 m collision proxy (the x500 body is
        # ~0.25 m); 0.2 let the planner fly through what counts as a hit.
        self.dwa_config.robot_radius = 0.30
        self.dwa_config.goal_threshold = 0.5
        # Tuned scoring (see holo_lab/EXPERIMENTS.md): blend velocity reward,
        # direction-based clearance (speed-independent 1.5 m probe), a continuous
        # terminal basin, and heading/clearance rebalanced against velocity.
        self.dwa_config.velocity_mode = "blend"
        self.dwa_config.blend_alpha = 0.5
        self.dwa_config.clearance_norm = 0.5
        self.dwa_config.clearance_lookahead = 1.5
        self.dwa_config.heading_weight = 0.3
        self.dwa_config.clearance_weight = 0.3
        self.dwa_config.velocity_weight = 0.4

    @staticmethod
    def _yaw_from_quaternion(q):
        """從 (w, x, y, z) 四元數取出繞 Z 軸的 yaw"""
        w, x, y, z = q[0], q[1], q[2], q[3]
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def odom_callback(self, msg):
        """即時更新無人機的高度、位置、速度與朝向，供 DWA 使用"""
        self.current_alt = msg.position[2]

        if np.all(np.isfinite(msg.position[:2])):
            self.pos_x = float(msg.position[0])
            self.pos_y = float(msg.position[1])

        if np.all(np.isfinite(msg.velocity[:2])):
            self.vel_x = float(msg.velocity[0])
            self.vel_y = float(msg.velocity[1])

        if np.all(np.isfinite(msg.q)):
            self.yaw = self._yaw_from_quaternion(msg.q)

    def get_goal_ned(self):
        """Map the Gazebo-world goal (ENU) into the PX4 local NED frame.

        Assumes the drone spawned at the Gazebo origin, so the two frames
        share an origin and the mapping is a pure axis swap:
        North = gz_y, East = gz_x.
        """
        gz_x = self.get_parameter('goal_x').value
        gz_y = self.get_parameter('goal_y').value
        return gz_y, gz_x

    def yaw_to_goal(self, goal_n, goal_e):
        """NED heading (0 = North, positive toward East) from current position to the goal."""
        dn = goal_n - self.pos_x
        de = goal_e - self.pos_y
        if math.hypot(dn, de) < 1e-6:
            return self.yaw
        return math.atan2(de, dn)

    def lidar_callback(self, msg):
        """處理 2D LiDAR 掃描數據"""
        self.last_lidar_time = self.get_clock().now()
        self.latest_scan = msg  # 保留給 timer_callback 的 DWA 控制迴圈使用

        # 只有在導航階段才印出詳細回報，避免起飛過程洗版
        if self.nav_state != "NAVIGATE":
            return

        ranges = np.array(msg.ranges)

        # 過濾掉無效值 (inf 或超過最大範圍的點)
        valid_indices = np.where((ranges > msg.range_min) & (ranges < msg.range_max))[0]

        if len(valid_indices) == 0:
            # 四周沒有有效回波，視為淨空
            self.front_dist = float('inf')
            self.min_threat = float('inf')
            self.min_threat_angle = 0.0
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

        # 存起來，交給 run_dwa_navigation 的節流輸出一起印 (避免 30Hz 洗版)
        self.front_dist = float(front_dist)
        self.min_threat = float(min_dist)
        self.min_threat_angle = min_angle_deg

    def timer_callback(self):
        """主控制迴圈 (20Hz)"""
        # 1. 永遠保持發送 Offboard 心跳 (PX4 安全機制)
        # NAVIGATE 階段用速度控制 x/y、位置控制 z，其餘階段全程用位置控制
        self.publish_offboard_control_heartbeat(use_velocity=(self.nav_state == "NAVIGATE"))

        # 2. 狀態機控制
        if self.nav_state == "INIT":
            # 先發送 20 次心跳，確保通訊穩定後再解鎖
            if self.heartbeat_counter == 20:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # 切換 Offboard
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0) # 馬達解鎖
                self.nav_state = "TAKEOFF"
                self.get_logger().info("Arming")

        elif self.nav_state == "TAKEOFF":
            # Climb in place to 2 m, already yawing to face the goal so the
            # drone starts navigation pointed at it (lidar front coverage too)
            goal_n, goal_e = self.get_goal_ned()
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_alt,
                                           yaw=self.yaw_to_goal(goal_n, goal_e))

            # 判斷是否抵達高度 (誤差 0.2m 內)
            if self.current_alt < (self.takeoff_alt + 0.2):
                self.nav_state = "NAVIGATE"
                self.get_logger().info("🚁 到達目標高度！開始 DWA 導航...")

        elif self.nav_state == "NAVIGATE":
            self.run_dwa_navigation()

        elif self.nav_state == "GOAL_REACHED":
            # Hold the position/heading captured when the goal was reached,
            # instead of chasing the live odometry (which would slowly drift)
            self.publish_position_setpoint(self.hold_x, self.hold_y, self.takeoff_alt,
                                           yaw=self.hold_yaw)

        self.heartbeat_counter += 1

    def run_dwa_navigation(self):
        """在 NAVIGATE 狀態下，用 LiDAR 障礙點雲跑一次 DWA 搜尋並發布速度指令"""
        goal_n, goal_e = self.get_goal_ned()

        dist_to_goal = math.hypot(goal_n - self.pos_x, goal_e - self.pos_y)
        if dist_to_goal < self.dwa_config.goal_threshold:
            self.get_logger().info(">>> GOAL REACHED! <<<")
            self.hold_x, self.hold_y, self.hold_yaw = self.pos_x, self.pos_y, self.yaw
            self.nav_state = "GOAL_REACHED"
            return

        goal_yaw = self.yaw_to_goal(goal_n, goal_e)

        if self.latest_scan is None:
            # 還沒收到任何 LiDAR 資料前，原地懸停等待，不盲飛
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_alt, yaw=goal_yaw)
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec - self.last_lidar_warning_time > 2.0:
                self.get_logger().warn("尚未收到 LiDAR 資料，請確認 Gazebo scan topic 與 ros_gz_bridge 是否正在發布。")
                self.last_lidar_warning_time = now_sec
            return

        obstacle_points = dwa_core.scan_to_world_points(
            self.latest_scan.ranges,
            self.latest_scan.angle_min,
            self.latest_scan.angle_increment,
            self.latest_scan.range_min,
            self.latest_scan.range_max,
            self.pos_x, self.pos_y, self.yaw,
            stride=self.lidar_stride,
            # gz gpu_lidar scan is z-up (+angle = body LEFT); our frame is
            # NED/FRD (+y = right). Without the flip the point cloud is mirrored
            # across the body axis - see scan_to_world_points.
            flip_y=True,
        )

        state = {"x": self.pos_x, "y": self.pos_y, "vx": self.vel_x, "vy": self.vel_y}
        vx_cmd, vy_cmd, ok, dbg = dwa_core.dwa_control(
            state, (goal_n, goal_e), obstacle_points, self.dwa_config, return_debug=True
        )

        if not ok:
            vx_cmd, vy_cmd = 0.0, 0.0

        # x/y velocity from DWA, z position-held, nose kept on the goal
        self.publish_velocity_setpoint(vx_cmd, vy_cmd, self.takeoff_alt, yaw=goal_yaw)

        # 節流輸出一整套狀態 (~1Hz)：位置、DWA 最佳速度、雷達距離、三項分數
        # pos 是 PX4 NED local frame (x=North, y=East)，與 DWA 用的座標一致
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_status_print_sec >= self.status_print_period:
            self.last_status_print_sec = now_sec
            if ok and dbg is not None:
                self.get_logger().info(
                    f"[DWA] pos(N,E)=({self.pos_x:+.2f}, {self.pos_y:+.2f}) "
                    f"| cmd v=({vx_cmd:+.2f}, {vy_cmd:+.2f}) "
                    f"| 前方={self.front_dist:.2f}m 最近威脅={self.min_threat:.2f}m@{self.min_threat_angle:+.0f}° "
                    f"| 分數 heading={dbg['heading_score']:.3f} clearance={dbg['clearance_score']:.3f} "
                    f"velocity={dbg['velocity_score']:.3f} "
                    f"| 加權 H={dbg['heading_term']:.3f} C={dbg['clearance_term']:.3f} V={dbg['velocity_term']:.3f} "
                    f"總分={dbg['total']:.3f} | 距目標={dist_to_goal:.2f}m"
                )
            else:
                self.get_logger().warn(
                    f"[DWA] pos(N,E)=({self.pos_x:+.2f}, {self.pos_y:+.2f}) 找不到可行速度，煞車懸停 "
                    f"| 前方={self.front_dist:.2f}m 最近威脅={self.min_threat:.2f}m@{self.min_threat_angle:+.0f}° "
                    f"| 距目標={dist_to_goal:.2f}m"
                )

    # --- PX4 底層發布函式 ---
    def publish_offboard_control_heartbeat(self, use_velocity=False):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = use_velocity
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_velocity_setpoint(self, vx, vy, z, yaw=0.0):
        """z 用位置控制維持高度，x/y 用 DWA 算出的速度指令 (PX4 逐軸 NaN passthrough)"""
        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.position = [nan, nan, z]
        msg.velocity = [vx, vy, nan]
        msg.yaw = float(yaw)
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
