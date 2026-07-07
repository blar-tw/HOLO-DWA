#!/usr/bin/env python3
# One-shot LiDAR frame-handedness verifier.
#
# Grabs one LaserScan + the latest odometry from the running stack, converts
# the scan to world points BOTH ways (flip_y True/False), and scores each
# against the known dwa_test.sdf arena geometry (mean distance of scan points
# to the nearest obstacle surface, in the gz frame). The correct handedness
# should sit at ~cm level (sensor noise); the mirrored one will be large.
#
#   source /opt/ros/humble/setup.bash && source ~/ws/install/setup.bash
#   python3 verify_frame.py
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleOdometry
from sensor_msgs.msg import LaserScan

import dwa_core
import sim_offline


def surface_dist_many(pts_gz):
    return np.array([sim_offline.surface_distance(x, y) for (x, y) in pts_gz])


class Verifier(Node):
    def __init__(self):
        super().__init__('frame_verifier')
        self.odom = None
        self.done = False
        lidar_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                               history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry',
                                 self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/lidar', self.on_scan, lidar_qos)

    def on_odom(self, msg):
        if np.all(np.isfinite(msg.position[:2])) and np.all(np.isfinite(msg.q)):
            w, x, y, z = msg.q
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            self.odom = (float(msg.position[0]), float(msg.position[1]), yaw)

    def on_scan(self, msg):
        if self.odom is None or self.done:
            return
        n, e, yaw = self.odom
        for flip in (False, True):
            pts_ned = dwa_core.scan_to_world_points(
                msg.ranges, msg.angle_min, msg.angle_increment,
                msg.range_min, msg.range_max, n, e, yaw, stride=6, flip_y=flip)
            if pts_ned.shape[0] == 0:
                print(f"flip_y={flip}: no points")
                continue
            pts_gz = np.stack([pts_ned[:, 1], pts_ned[:, 0]], axis=1)  # x=E, y=N
            d = surface_dist_many(pts_gz)
            print(f"flip_y={flip!s:5}: {len(d)} pts | surface dist "
                  f"mean={d.mean():.3f}m median={np.median(d):.3f}m "
                  f"p90={np.percentile(d, 90):.3f}m")
        print(f"(drone NED N={n:+.2f} E={e:+.2f} yaw={math.degrees(yaw):+.1f}deg)")
        self.done = True


def main():
    rclpy.init()
    node = Verifier()
    import time
    t0 = time.time()
    while rclpy.ok() and not node.done and time.time() - t0 < 15.0:
        rclpy.spin_once(node, timeout_sec=0.5)
    if not node.done:
        print("TIMEOUT: no scan+odom received in 15 s (is the stack up?)")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
