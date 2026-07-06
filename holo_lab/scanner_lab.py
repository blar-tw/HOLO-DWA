#!/usr/bin/env python3
# HOLO-DWA lab node: an instrumented + automated copy of ../scanner.py.
#
# What it adds on top of the plain scanner:
#   1. INSTRUMENTATION - every control tick is written to a session CSV, and a
#      one-line summary per navigation run is appended to summary.jsonl. All of
#      it lands in holo_lab/logs/ (next to this file), nothing outside.
#   2. AUTOMATION - a /holo_lab/reset service (and an n_runs batch param) flies
#      the drone home via DWA and starts a fresh run WITHOUT restarting
#      PX4 / gz / bridge. That is the whole point: repeat trials cheaply.
#
# This file is deliberately utilitarian, not tidy - it only needs to run and
# produce data. The flight/DWA behavior is byte-for-byte the same logic as
# scanner.py; the extra code is logging + a RETURN_HOME state.
import os
import csv
import json
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from rcl_interfaces.msg import ParameterDescriptor

import dwa_core

# Accept an int OR a float from the CLI for numeric params. Without this, a
# param declared with a float default (e.g. 0.0) rejects `run_timeout:=0`
# (parsed as INTEGER) with InvalidParameterTypeException. We cast on read.
DYN = ParameterDescriptor(dynamic_typing=True)

LAB_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(LAB_DIR, "logs")

# One row per control tick. Kept flat so pandas/np.genfromtxt can read it raw.
CSV_FIELDS = [
    "t",              # seconds since node start
    "run_id",         # 0 = pre-nav (init/takeoff); >=1 = k-th navigate run
    "state",          # INIT / TAKEOFF / NAVIGATE / RETURN_HOME / HOLD
    "pos_n", "pos_e", "alt",
    "vel_n", "vel_e", "yaw",
    "cmd_vx", "cmd_vy", "speed",
    "target_n", "target_e", "dist_to_target",
    "front_dist", "min_threat", "min_threat_angle", "collision",
    "ok", "n_obs", "compute_ms",
    "heading_score", "clearance_score", "velocity_score",
    "heading_term", "clearance_term", "velocity_term", "total", "safe_dist",
]


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
        self.declare_parameter('goal_x', 12.0, DYN)
        self.declare_parameter('goal_y', 0.0, DYN)

        # --- lab automation params ---
        # n_runs : how many navigate runs to fly unattended (>0). After the last
        #          one the drone holds at the goal. <=0 means "loop forever".
        # home_threshold : arrival radius (m) for the RETURN_HOME leg.
        # run_timeout    : abort a stuck run after this many seconds (0 = never).
        # collision_dist : nearest LiDAR return below this (m) counts as a
        #                  collision. It is a proxy - the 2D LiDAR sits at the
        #                  body, so a hit within ~body-radius means contact.
        #                  Not a Gazebo contact sensor (see README for that).
        self.declare_parameter('n_runs', 1, DYN)
        self.declare_parameter('home_threshold', 0.6, DYN)
        self.declare_parameter('run_timeout', 0.0, DYN)
        self.declare_parameter('collision_dist', 0.3, DYN)
        self.n_runs = int(self.get_parameter('n_runs').value)
        self.home_threshold = float(self.get_parameter('home_threshold').value)
        self.run_timeout = float(self.get_parameter('run_timeout').value)
        self.collision_dist = float(self.get_parameter('collision_dist').value)

        # Hold target captured at the moment the goal is reached
        self.hold_x = 0.0
        self.hold_y = 0.0
        self.hold_yaw = 0.0

        # xy captured when TAKEOFF begins - the drone climbs in place here.
        # On a warm restart (scanner relaunched while the drone is still
        # airborne out in the arena) this is wherever it currently is, so we
        # never drag it sideways through obstacles at low altitude.
        self.takeoff_x = 0.0
        self.takeoff_y = 0.0

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
        # Planner keep-out must be >= the 0.3 m collision proxy, otherwise the
        # planner is allowed to fly through what the lab records as a hit
        # (x500 body is ~0.25 m anyway; 0.2 was optimistic).
        self.dwa_config.robot_radius = 0.30
        self.dwa_config.goal_threshold = 0.5
        # Scoring (iter2): component velocity with a scalar floor stops the
        # box-corner diagonal drift in open space while keeping wall-slide;
        # clearance saturating at 0.5 m removes the doorway penalty; weights
        # rebalanced so heading/clearance are not drowned by velocity.
        # clearance_lookahead makes the clearance term direction-based
        # (speed-independent) - the time-based version rewards creeping and
        # let the drone inch straight into a cylinder (iter1, run 1).
        self.dwa_config.velocity_mode = "blend"
        self.dwa_config.blend_alpha = 0.5
        self.dwa_config.clearance_norm = 0.5
        self.dwa_config.clearance_lookahead = 1.5
        self.dwa_config.heading_weight = 0.3
        self.dwa_config.clearance_weight = 0.3
        self.dwa_config.velocity_weight = 0.4

        # --- instrumentation state ---
        self.t0 = time.time()               # node start, for the "t" column
        self.run_id = 0                     # bumped to 1 when the first run starts
        self.runs_completed = 0            # reached/timeout runs (not resets)
        os.makedirs(LOG_DIR, exist_ok=True)
        # microsecond suffix so a quick scanner restart (./run_lab.sh reset)
        # never overwrites the previous session's CSV - two nodes can otherwise
        # start within the same wall-clock second and collide on the filename.
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(self.t0)) + f"_{int((self.t0 % 1) * 1e6):06d}"
        self.session_csv_path = os.path.join(LOG_DIR, f"session_{stamp}.csv")
        self.summary_path = os.path.join(LOG_DIR, "summary.jsonl")
        self._csv_file = open(self.session_csv_path, "w", newline="")
        self._csv = csv.DictWriter(self._csv_file, fieldnames=CSV_FIELDS)
        self._csv.writeheader()
        self._csv_file.flush()
        self._run_reset_accumulators()

        # --- reset service: fly home + start a fresh run, no stack restart ---
        self.reset_srv = self.create_service(Trigger, '/holo_lab/reset', self.reset_cb)

        self.get_logger().info(
            f"[LAB] logging to {self.session_csv_path} | n_runs={self.n_runs} "
            f"run_timeout={self.run_timeout}s | reset via: "
            f"ros2 service call /holo_lab/reset std_srvs/srv/Trigger")

    # ------------------------------------------------------------------ utils
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
        gz_x = float(self.get_parameter('goal_x').value)
        gz_y = float(self.get_parameter('goal_y').value)
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

        # 導航與返航階段都要更新雷達回報 (以免返航時 log 是舊值)
        if self.nav_state not in ("NAVIGATE", "RETURN_HOME"):
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

        # 計算該障礙物的實際角度 (度數)。取負號把 gz 掃描的 z-up 慣例
        # (正角 = 機體左) 轉成規劃器使用的 FRD 慣例 (正角 = 機體右),
        # 與 scan_to_world_points(flip_y=True) 一致。
        min_angle_rad = -(msg.angle_min + min_idx * msg.angle_increment)
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

    # ------------------------------------------------------- run bookkeeping
    def _run_reset_accumulators(self):
        """Zero the per-run metric accumulators (called at each run start)."""
        self.run_t_start = time.time()
        self.run_start_pos = (self.pos_x, self.pos_y)
        self.run_prev_pos = (self.pos_x, self.pos_y)
        self.run_path_len = 0.0
        self.run_min_clear = float('inf')
        self.run_max_speed = 0.0
        self.run_ticks = 0
        self.run_infeasible = 0
        self.run_compute_ms_sum = 0.0
        self.run_compute_ms_max = 0.0
        # collision tracking
        self.run_collisions = 0            # distinct collision episodes
        self.run_collided = False
        self.run_min_threat = float('inf')  # closest the drone ever got
        self.run_first_collision = None
        self._in_collision = False         # debounce: one count per episode

    def _start_run(self):
        """Open a new navigate run: bump id, reset metrics."""
        self.run_id += 1
        self._run_reset_accumulators()
        goal_n, goal_e = self.get_goal_ned()
        self.get_logger().info(
            f"[LAB] === run {self.run_id} start === goal_ned=({goal_n:.1f}, {goal_e:.1f}) "
            f"from pos=({self.pos_x:+.2f}, {self.pos_y:+.2f})")

    def _finalize_run(self, outcome):
        """Write the summary line for the run that just ended."""
        goal_n, goal_e = self.get_goal_ned()
        dur = time.time() - self.run_t_start
        straight = math.hypot(goal_n - self.run_start_pos[0], goal_e - self.run_start_pos[1])
        end_dist = math.hypot(goal_n - self.pos_x, goal_e - self.pos_y)
        mean_ms = self.run_compute_ms_sum / self.run_ticks if self.run_ticks else 0.0
        summary = {
            "run_id": self.run_id,
            "outcome": outcome,                    # reached / timeout / reset
            "goal_ned": [round(goal_n, 3), round(goal_e, 3)],
            "start_wall": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(self.run_t_start)),
            "duration_s": round(dur, 3),
            "ticks": self.run_ticks,
            "infeasible_ticks": self.run_infeasible,
            "path_length_m": round(self.run_path_len, 3),
            "straight_line_m": round(straight, 3),
            "path_efficiency": round(straight / self.run_path_len, 3) if self.run_path_len > 1e-6 else None,
            "min_clearance_m": round(self.run_min_clear, 3) if math.isfinite(self.run_min_clear) else None,
            "max_speed_mps": round(self.run_max_speed, 3),
            "compute_ms_mean": round(mean_ms, 3),
            "compute_ms_max": round(self.run_compute_ms_max, 3),
            "end_pos_ned": [round(self.pos_x, 3), round(self.pos_y, 3)],
            "end_dist_to_goal_m": round(end_dist, 3),
            # collision record (LiDAR-proxy; see _update_collision / README)
            "collided": self.run_collided,
            "collisions": self.run_collisions,
            "min_dist_m": round(self.run_min_threat, 3) if math.isfinite(self.run_min_threat) else None,
            "first_collision": self.run_first_collision,
        }
        with open(self.summary_path, "a") as f:
            f.write(json.dumps(summary) + "\n")
        self.get_logger().info(
            f"[LAB] === run {self.run_id} {outcome} === "
            f"t={dur:.1f}s path={self.run_path_len:.1f}m (eff {summary['path_efficiency']}) "
            f"collisions={self.run_collisions} (min_dist={summary['min_dist_m']}m) "
            f"min_clear={summary['min_clearance_m']}m infeasible={self.run_infeasible}/{self.run_ticks} "
            f"compute mean/max={mean_ms:.2f}/{self.run_compute_ms_max:.2f}ms")

    def _enter_hold(self):
        """Capture the current pose and settle into the HOLD state."""
        self.hold_x, self.hold_y, self.hold_yaw = self.pos_x, self.pos_y, self.yaw
        self.nav_state = "HOLD"

    def _begin_return(self):
        """Kick off the RETURN_HOME leg (DWA back to the origin)."""
        self.nav_state = "RETURN_HOME"
        self.return_t_start = time.time()
        self.get_logger().info("[LAB] returning to origin (0,0) via DWA...")

    def reset_cb(self, request, response):
        """/holo_lab/reset: fly home and start a fresh run, no stack restart."""
        if self.nav_state in ("INIT", "TAKEOFF"):
            response.success = False
            response.message = "not flying yet - wait for NAVIGATE"
            return response
        if self.nav_state == "NAVIGATE":
            self._finalize_run("reset")   # abort the in-flight run
        self._begin_return()
        response.success = True
        response.message = "returning to origin, a new run will start on arrival"
        return response

    # ------------------------------------------------------------- main loop
    def timer_callback(self):
        """主控制迴圈 (20Hz)"""
        # 1. 永遠保持發送 Offboard 心跳 (PX4 安全機制)
        # NAVIGATE / RETURN_HOME 用速度控制 x/y、位置控制 z，其餘階段全程用位置控制
        moving = self.nav_state in ("NAVIGATE", "RETURN_HOME")
        self.publish_offboard_control_heartbeat(use_velocity=moving)

        # 2. 狀態機控制
        # NOTE: log the tick BEFORE any state transition so the CSV "state"
        # column matches the branch that actually ran this tick.
        if self.nav_state == "INIT":
            self._log_tick(0.0, 0.0, None, None, None)
            if self.heartbeat_counter == 20:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # Offboard
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)  # arm
                # Capture where we are now so TAKEOFF climbs in place (works for
                # both a cold start at the origin and a warm restart mid-arena).
                self.takeoff_x, self.takeoff_y = self.pos_x, self.pos_y
                self.nav_state = "TAKEOFF"
                self.get_logger().info("Arming")

        elif self.nav_state == "TAKEOFF":
            goal_n, goal_e = self.get_goal_ned()
            self.publish_position_setpoint(self.takeoff_x, self.takeoff_y, self.takeoff_alt,
                                           yaw=self.yaw_to_goal(goal_n, goal_e))
            self._log_tick(0.0, 0.0, None, None, None)
            if self.current_alt < (self.takeoff_alt + 0.2):
                # At altitude. If we are not at the origin (a warm restart left
                # the drone out in the arena), fly home via DWA first so runs
                # always start from (0,0); otherwise begin navigating right away.
                if math.hypot(self.pos_x, self.pos_y) > self.home_threshold:
                    self.get_logger().info(
                        f"🚁 到達高度，但不在原點 (dist={math.hypot(self.pos_x, self.pos_y):.1f}m) "
                        f"→ 先用 DWA 飛回原點")
                    self._begin_return()
                else:
                    self._start_run()
                    self.nav_state = "NAVIGATE"
                    self.get_logger().info("🚁 到達目標高度！開始 DWA 導航...")

        elif self.nav_state == "NAVIGATE":
            goal_n, goal_e = self.get_goal_ned()
            self._navigate(goal_n, goal_e, is_return=False)

        elif self.nav_state == "RETURN_HOME":
            self._navigate(0.0, 0.0, is_return=True)

        elif self.nav_state == "HOLD":
            self.publish_position_setpoint(self.hold_x, self.hold_y, self.takeoff_alt,
                                           yaw=self.hold_yaw)
            self._log_tick(0.0, 0.0, (self.hold_x, self.hold_y), None, None)

        self.heartbeat_counter += 1

    def _navigate(self, target_n, target_e, is_return):
        """DWA toward (target_n, target_e). Shared by NAVIGATE and RETURN_HOME."""
        dist_to_target = math.hypot(target_n - self.pos_x, target_e - self.pos_y)
        arrive_thresh = self.home_threshold if is_return else self.dwa_config.goal_threshold

        # --- arrival handling -------------------------------------------------
        if dist_to_target < arrive_thresh:
            if is_return:
                self.get_logger().info(">>> back at origin, starting a new run <<<")
                self._start_run()
                self.nav_state = "NAVIGATE"
                return
            # reached the real goal
            self.get_logger().info(">>> GOAL REACHED! <<<")
            self._finalize_run("reached")
            self.runs_completed += 1
            self._next_after_run()
            return

        # --- run timeout (only on the outbound leg) --------------------------
        if (not is_return) and self.run_timeout > 0 and (time.time() - self.run_t_start) > self.run_timeout:
            self.get_logger().warn(f">>> run {self.run_id} TIMEOUT ({self.run_timeout}s) <<<")
            self._finalize_run("timeout")
            self.runs_completed += 1
            self._next_after_run()
            return

        target_yaw = self.yaw_to_goal(target_n, target_e)

        if self.latest_scan is None:
            # 還沒收到任何 LiDAR 資料前，原地懸停等待，不盲飛
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.takeoff_alt, yaw=target_yaw)
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec - self.last_lidar_warning_time > 2.0:
                self.get_logger().warn("尚未收到 LiDAR 資料，請確認 Gazebo scan topic 與 ros_gz_bridge 是否正在發布。")
                self.last_lidar_warning_time = now_sec
            self._log_tick(0.0, 0.0, (target_n, target_e), False, None, n_obs=0, compute_ms=0.0)
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
            # NED/FRD (+y = right). Without the flip the point cloud is
            # mirrored across the body axis - see scan_to_world_points.
            flip_y=True,
        )

        state = {"x": self.pos_x, "y": self.pos_y, "vx": self.vel_x, "vy": self.vel_y}
        t_compute = time.perf_counter()
        vx_cmd, vy_cmd, ok, dbg = dwa_core.dwa_control(
            state, (target_n, target_e), obstacle_points, self.dwa_config, return_debug=True
        )
        compute_ms = (time.perf_counter() - t_compute) * 1e3

        if not ok:
            vx_cmd, vy_cmd = 0.0, 0.0

        # x/y velocity from DWA, z position-held, nose kept on the target
        self.publish_velocity_setpoint(vx_cmd, vy_cmd, self.takeoff_alt, yaw=target_yaw)

        collision = self._update_collision(is_return)
        self._log_tick(vx_cmd, vy_cmd, (target_n, target_e), ok, dbg,
                       n_obs=obstacle_points.shape[0], compute_ms=compute_ms,
                       collision=collision)

        # metrics accumulate only on the real outbound run (not RETURN_HOME)
        if not is_return:
            self._accumulate_metrics(ok, dbg, compute_ms)

        # 節流輸出一整套狀態 (~1Hz)
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_status_print_sec >= self.status_print_period:
            self.last_status_print_sec = now_sec
            tag = "RTH" if is_return else "DWA"
            if ok and dbg is not None:
                self.get_logger().info(
                    f"[{tag}] run{self.run_id} pos(N,E)=({self.pos_x:+.2f}, {self.pos_y:+.2f}) "
                    f"| cmd v=({vx_cmd:+.2f}, {vy_cmd:+.2f}) "
                    f"| 前方={self.front_dist:.2f}m 最近威脅={self.min_threat:.2f}m@{self.min_threat_angle:+.0f}° "
                    f"| H={dbg['heading_term']:.3f} C={dbg['clearance_term']:.3f} V={dbg['velocity_term']:.3f} "
                    f"總分={dbg['total']:.3f} | 距目標={dist_to_target:.2f}m | {compute_ms:.1f}ms"
                )
            else:
                self.get_logger().warn(
                    f"[{tag}] run{self.run_id} pos(N,E)=({self.pos_x:+.2f}, {self.pos_y:+.2f}) 找不到可行速度，煞車懸停 "
                    f"| 前方={self.front_dist:.2f}m 最近威脅={self.min_threat:.2f}m@{self.min_threat_angle:+.0f}° "
                    f"| 距目標={dist_to_target:.2f}m"
                )

    def _next_after_run(self):
        """Decide what to do once an outbound run ends (reached/timeout)."""
        if self.n_runs > 0 and self.runs_completed >= self.n_runs:
            self.get_logger().info(f"[LAB] all {self.runs_completed} run(s) done - holding at goal.")
            self._enter_hold()
        else:
            self._begin_return()

    def _accumulate_metrics(self, ok, dbg, compute_ms):
        """Update per-run metrics from the current tick."""
        self.run_ticks += 1
        dp = math.hypot(self.pos_x - self.run_prev_pos[0], self.pos_y - self.run_prev_pos[1])
        self.run_path_len += dp
        self.run_prev_pos = (self.pos_x, self.pos_y)
        spd = math.hypot(self.vel_x, self.vel_y)
        self.run_max_speed = max(self.run_max_speed, spd)
        self.run_compute_ms_sum += compute_ms
        self.run_compute_ms_max = max(self.run_compute_ms_max, compute_ms)
        if not ok:
            self.run_infeasible += 1
        elif dbg is not None:
            self.run_min_clear = min(self.run_min_clear, dbg["safe_dist"])

    def _update_collision(self, is_return):
        """Flag/record a collision from the nearest LiDAR return. Returns 0/1
        for this tick. Counts one episode per contact (rising edge)."""
        mt = self.min_threat
        hit = math.isfinite(mt) and mt < self.collision_dist
        if (not is_return) and math.isfinite(mt):
            self.run_min_threat = min(self.run_min_threat, mt)
        if hit and not self._in_collision:
            self._in_collision = True     # new episode
            self.get_logger().warn(
                f"[COLLISION] run{self.run_id} min_dist={mt:.2f}m < {self.collision_dist:.2f}m "
                f"@ pos(N,E)=({self.pos_x:+.2f}, {self.pos_y:+.2f}) angle={self.min_threat_angle:+.0f}°")
            if not is_return:
                self.run_collisions += 1
                self.run_collided = True
                if self.run_first_collision is None:
                    self.run_first_collision = {
                        "t": round(time.time() - self.run_t_start, 2),
                        "pos_ned": [round(self.pos_x, 2), round(self.pos_y, 2)],
                        "min_dist_m": round(mt, 3),
                    }
        elif not hit:
            self._in_collision = False
        return int(hit)

    def _log_tick(self, cmd_vx, cmd_vy, target, ok, dbg, n_obs=0, compute_ms=0.0, collision=0):
        """Append one row to the session CSV. Cheap; runs every tick."""
        tn = target[0] if target is not None else ""
        te = target[1] if target is not None else ""
        dist = math.hypot(target[0] - self.pos_x, target[1] - self.pos_y) if target is not None else ""
        row = {
            "t": round(time.time() - self.t0, 3),
            "run_id": self.run_id,
            "state": self.nav_state,
            "pos_n": round(self.pos_x, 4), "pos_e": round(self.pos_y, 4),
            "alt": round(self.current_alt, 4),
            "vel_n": round(self.vel_x, 4), "vel_e": round(self.vel_y, 4),
            "yaw": round(self.yaw, 4),
            "cmd_vx": round(cmd_vx, 4), "cmd_vy": round(cmd_vy, 4),
            "speed": round(math.hypot(cmd_vx, cmd_vy), 4),
            "target_n": tn, "target_e": te,
            "dist_to_target": round(dist, 4) if dist != "" else "",
            "front_dist": self.front_dist if math.isfinite(self.front_dist) else "",
            "min_threat": self.min_threat if math.isfinite(self.min_threat) else "",
            "min_threat_angle": round(self.min_threat_angle, 2),
            "collision": collision,
            "ok": int(ok) if ok is not None else "",
            "n_obs": n_obs,
            "compute_ms": round(compute_ms, 3),
        }
        if dbg is not None:
            row.update({
                "heading_score": round(dbg["heading_score"], 4),
                "clearance_score": round(dbg["clearance_score"], 4),
                "velocity_score": round(dbg["velocity_score"], 4),
                "heading_term": round(dbg["heading_term"], 4),
                "clearance_term": round(dbg["clearance_term"], 4),
                "velocity_term": round(dbg["velocity_term"], 4),
                "total": round(dbg["total"], 4),
                "safe_dist": round(dbg["safe_dist"], 4),
            })
        self._csv.writerow(row)
        self._csv_file.flush()

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

    def destroy_node(self):
        try:
            self._csv_file.flush()
            self._csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DroneLidarScanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # normal teardown: Ctrl-C, tmux kill, ./run_lab.sh kill, timeout, ...
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
