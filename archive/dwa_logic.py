import math
import time
import numpy as np
import pybullet as p
import pybullet_data


# =========================
# 1. Config
# =========================
class Config:
    """DWA parameters and simplified drone physical limits."""

    def __init__(self):
        # Velocity-space limits for a holonomic drone in the world XY plane.
        self.v_max = 10.0
        self.vx_min = -10.0
        self.vx_max = 10.0
        self.vy_min = -10.0
        self.vy_max = 10.0

        # Dynamic window acceleration limit.
        self.a_max = 5.0

        # Admissible velocity braking limit.
        self.brake_a_max = 5.0

        # DWA sampling and prediction parameters.
        self.vx_resolution = 0.1
        self.vy_resolution = 0.1
        self.control_dt = 0.2
        self.predict_time = 2.0
        self.predict_dt = 0.1

        # Scoring weights.
        self.heading_weight = 0.6
        self.clearance_weight = 0.3
        self.velocity_weight = 0.1

        # Safety parameter.
        self.robot_radius = 0.2


# =========================
# 2. Drone
# =========================
class X550Drone:
    """Simplified drone model for holonomic DWA."""

    def __init__(self, config, start_pos=(0, 0, 1), dt=1 / 30):
        self.config = config
        self.dt = dt

        self.pos = np.array(start_pos, dtype=float)
        self.vel = np.zeros(3)
        self.yaw = 0.0

        self.current_vx = 0.0
        self.current_vy = 0.0

        col = p.createCollisionShape(p.GEOM_SPHERE, radius=self.config.robot_radius)
        vis = p.createVisualShape(
            p.GEOM_SPHERE,
            radius=self.config.robot_radius,
            rgbaColor=[0.3, 0.3, 0.3, 1],
        )
        self.body = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=start_pos,
        )

    def get_state(self):
        """Pack the drone state for DWA."""
        return {
            "x": self.pos[0],
            "y": self.pos[1],
            "z": self.pos[2],
            "yaw": self.yaw,
            "vx": self.current_vx,
            "vy": self.current_vy,
        }

    def step(self, vx_cmd, vy_cmd):
        """Apply a holonomic velocity command in world coordinates."""
        speed = math.hypot(vx_cmd, vy_cmd)
        if speed > self.config.v_max:
            scale = self.config.v_max / speed
            vx_cmd *= scale
            vy_cmd *= scale

        self.current_vx = vx_cmd
        self.current_vy = vy_cmd
        self.vel = np.array([self.current_vx, self.current_vy, 0.0])

        self.pos += self.vel * self.dt

        # Yaw is visual only; point the sphere/body toward the velocity vector.
        if speed > 1e-6:
            self.yaw = math.atan2(self.current_vy, self.current_vx)

        quat = p.getQuaternionFromEuler([0, 0, self.yaw])
        p.resetBasePositionAndOrientation(self.body, self.pos.tolist(), quat)


# =========================
# 3. Goal, obstacles, environment
# =========================
class Goal:
    def __init__(self, position, threshold=0.3):
        self.position = np.array(position)
        self.threshold = threshold
        vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.2, rgbaColor=[0, 1, 0, 1])
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis, basePosition=position)

    def reached(self, pos):
        return np.linalg.norm(pos - self.position) < self.threshold


class Barrier:
    def __init__(self, shape, position, size):
        self.shape = shape
        self.position = np.array(position)
        self.size = size

    def build(self):
        if self.shape == "cylinder":
            r, h = self.size
            col = p.createCollisionShape(p.GEOM_CYLINDER, radius=r, height=h)
            vis = p.createVisualShape(
                p.GEOM_CYLINDER, radius=r, length=h, rgbaColor=[1, 0, 0, 1]
            )
        elif self.shape == "box":
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=self.size)
            vis = p.createVisualShape(
                p.GEOM_BOX, halfExtents=self.size, rgbaColor=[0, 0, 1, 1]
            )
        else:
            raise ValueError(f"Unsupported barrier shape: {self.shape}")

        p.createMultiBody(0, col, vis, self.position)

    def distance(self, point):
        pnt = np.array(point)
        if self.shape == "cylinder":
            r, _ = self.size
            return np.linalg.norm(pnt[:2] - self.position[:2]) - r

        if self.shape == "box":
            d = np.abs(pnt - self.position) - self.size
            return np.linalg.norm(np.maximum(d, 0))

        raise ValueError(f"Unsupported barrier shape: {self.shape}")


class Environment:
    def __init__(self):
        self.barriers = []

    def add(self, barrier):
        barrier.build()
        self.barriers.append(barrier)

    def min_distance(self, point):
        if not self.barriers:
            return float("inf")
        return min(barrier.distance(point) for barrier in self.barriers)


# =========================
# 4. Holonomic DWA
# =========================
def predict_trajectory(state, vx, vy, config):
    """Predict future positions under constant world-frame (vx, vy)."""
    traj = []
    x, y = state["x"], state["y"]

    elapsed = 0.0
    while elapsed <= config.predict_time:
        x += vx * config.predict_dt
        y += vy * config.predict_dt
        traj.append((x, y))
        elapsed += config.predict_dt

    return traj


def calculate_traj_distance(traj, env, z_height):
    """Return the closest obstacle distance along the predicted trajectory."""
    min_dist = float("inf")
    for point in traj:
        p3d = [point[0], point[1], z_height]
        min_dist = min(min_dist, env.min_distance(p3d))
    return min_dist


def compute_score(state, traj, goal, dist_to_obs, vx, vy, config):
    """
    Score a holonomic trajectory.

    The former yaw-heading score is replaced by a progress-to-goal score,
    because heading/yaw is not a planning variable for a holonomic drone.
    """
    goal_x, goal_y = goal.position[0], goal.position[1]
    min_dist_to_goal = min(math.hypot(goal_x - p[0], goal_y - p[1]) for p in traj)
    speed = math.hypot(vx, vy)

    if min_dist_to_goal < goal.threshold:
        return 10000.0 + speed

    start_dist = math.hypot(goal_x - state["x"], goal_y - state["y"])
    final_x, final_y = traj[-1]
    final_dist = math.hypot(goal_x - final_x, goal_y - final_y)
    progress_score = 0.0
    if start_dist > 1e-6:
        progress_score = (start_dist - final_dist) / start_dist
        progress_score = max(-1.0, min(1.0, progress_score))

    clearance_score = min(dist_to_obs, 1.0) / 1.0
    velocity_score = min(speed, config.v_max) / config.v_max

    return (
        config.heading_weight * progress_score
        + config.clearance_weight * clearance_score
        + config.velocity_weight * velocity_score
    )


def dwa_control(state, goal, env, config):
    """Compute the best holonomic velocity command (vx, vy)."""
    vx_curr = state["vx"]
    vy_curr = state["vy"]
    z_height = state["z"]

    vx_min_d = max(config.vx_min, vx_curr - config.a_max * config.control_dt)
    vx_max_d = min(config.vx_max, vx_curr + config.a_max * config.control_dt)
    vy_min_d = max(config.vy_min, vy_curr - config.a_max * config.control_dt)
    vy_max_d = min(config.vy_max, vy_curr + config.a_max * config.control_dt)

    best_vx, best_vy = 0.0, 0.0
    max_score = -float("inf")

    for vx in np.arange(vx_min_d, vx_max_d + config.vx_resolution, config.vx_resolution):
        for vy in np.arange(vy_min_d, vy_max_d + config.vy_resolution, config.vy_resolution):
            speed = math.hypot(vx, vy)
            if speed < 1e-6 or speed > config.v_max:
                continue

            traj = predict_trajectory(state, vx, vy, config)
            dist = calculate_traj_distance(traj, env, z_height)
            safe_dist = dist - config.robot_radius

            if safe_dist <= 0:
                continue

            safe_speed = math.sqrt(2 * safe_dist * config.brake_a_max)
            if speed > safe_speed:
                continue

            score = compute_score(state, traj, goal, safe_dist, vx, vy, config)
            if score > max_score:
                max_score = score
                best_vx = vx
                best_vy = vy

    return best_vx, best_vy


# =========================
# 5. Main
# =========================
def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    p.resetDebugVisualizerCamera(
        cameraDistance=8,
        cameraYaw=0,
        cameraPitch=-89,
        cameraTargetPosition=[2, 2, 0],
    )

    config = Config()
    drone = X550Drone(config, start_pos=(0, 0, 1.0))

    env = Environment()
    env.add(Barrier("cylinder", [5, 0, 1], [0.5, 2]))
    env.add(Barrier("cylinder", [7, 2, 1], [0.5, 2]))
    env.add(Barrier("cylinder", [9, 4, 1], [0.5, 2]))
    env.add(Barrier("cylinder", [11, 6, 1], [0.5, 2]))

    env.add(Barrier("box", [1, -2, 1], [0.5, 0.5, 1]))

    goal = Goal([12, 1, 1])

    while True:
        state = drone.get_state()
        pos = np.array([state["x"], state["y"], state["z"]])

        if goal.reached(pos):
            print(">>> GOAL REACHED! <<<")
            drone.step(0.0, 0.0)
            break

        vx_cmd, vy_cmd = dwa_control(state, goal, env, config)
        print(
            f"pos: ({pos[0]:.2f}, {pos[1]:.2f}) | "
            f"cmd -> vx: {vx_cmd:.2f}, vy: {vy_cmd:.2f}"
        )

        drone.step(vx_cmd, vy_cmd)

        # Update the camera position so it follows the drone from a top-down angle
        p.resetDebugVisualizerCamera(
            cameraDistance=10.0,           # camera height/distance
            cameraYaw=0,                  # yaw angle
            cameraPitch=-89.9,            # pitch angle (near -90 = straight down)
            cameraTargetPosition=[pos[0], pos[1], pos[2]] # follow the drone's position
        )

        p.stepSimulation()
        time.sleep(1 / 30)


if __name__ == "__main__":
    main()