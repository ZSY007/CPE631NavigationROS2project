#!/usr/bin/env python3
"""
social_nav_node.py  –  Enhanced Anisotropic Gaussian Social Costmap

Algorithm Core (v2 - Comprehensive Enhancement):
  1. Subscribe to pedestrian poses, stable tracking with nearest-neighbor data association
  2. Estimate pedestrian velocity (exponential smoothing + jump filtering + speed limit)
  3. Adaptive prediction horizon (linked to pedestrian speed)
  4. Non-linear uncertainty growth (far-field diffuses faster) + heading inertia constraint
  5. Asymmetric anisotropic Gaussian (front stronger, rear weaker)
  6. Group / Crowd cost (intensified cost in dense pedestrian zones)
  7. Robot direction-aware cost decay (rear pedestrians downweighted)
  8. Interactive prediction: simple pedestrian reaction offset to multiple robot candidate trajectories

Topic In:  /pedestrian_poses  (geometry_msgs/PoseArray)
           /amcl_pose         (geometry_msgs/PoseWithCovarianceStamped)
           /odom              (nav_msgs/Odometry, velocity only)
Topic Out: /social_costmap    (nav_msgs/OccupancyGrid)
"""

from collections import deque
from itertools import combinations, permutations
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


# ──────────────────────────────────────────────────────────────
#  PedestrianTracker (Enhanced Version)
# ──────────────────────────────────────────────────────────────
class PedestrianTracker:
    """Position and velocity estimator for a single pedestrian (Enhanced: jump filtering + speed limit + wandering detection)."""

    def __init__(
        self,
        init_pos: np.ndarray,
        init_time: float,
        window: int = 25,
        alpha: float = 0.35,
        velocity_tau: float = 0.35,
        max_ped_speed: float = 1.8,
        max_jump_dist: float = 1.0,
        init_heading: np.ndarray | None = None,
    ):
        self.history: deque[tuple[float, np.ndarray]] = deque(maxlen=window)
        self.history.append((init_time, init_pos.copy()))
        self.velocity_history: deque[np.ndarray] = deque(maxlen=window)
        self.velocity_history.append(np.zeros(2))
        self.velocity = np.zeros(2)
        self.heading = (
            init_heading.copy()
            if init_heading is not None
            else np.array([1.0, 0.0])
        )
        self.alpha = alpha
        self.velocity_tau = max(velocity_tau, 1e-3)
        self.max_ped_speed = max_ped_speed
        self.max_jump_dist = max_jump_dist
        self._initialized = False

    # ── Convenience properties ──────────────────────────────────────────────
    @property
    def position(self) -> np.ndarray:
        """Return the latest known position."""
        return self.history[-1][1]

    # ── Wandering mode detection ─────────────────────────────────────────
    def is_wandering(self, ratio_thresh: float = 0.3) -> bool:
        """Detect if pedestrian is wandering (meandering movement, net displacement much less than path length).

        Principle: compute history start-end net displacement / total path length.
        If ratio < ratio_thresh, pedestrian is spinning in place or random walking.
        """
        if len(self.history) < 3:
            return False
        positions = [pos for _, pos in self.history]
        displacement = float(np.linalg.norm(positions[-1] - positions[0]))
        total_path = sum(
            float(np.linalg.norm(positions[i + 1] - positions[i]))
            for i in range(len(positions) - 1)
        )
        if total_path < 0.1:
            return False  # Movement too small to judge
        return (displacement / total_path) < ratio_thresh

    # ── Velocity update ──────────────────────────────────────────────
    def update(
        self,
        pos: np.ndarray,
        stamp: float,
        observed_heading: np.ndarray | None = None,
    ) -> None:
        """Update velocity with new observations, including jump filtering and speed limit protection."""
        last_stamp, last_pos = self.history[-1]
        dt = stamp - last_stamp

        if dt > 1e-6:
            displacement = pos - last_pos
            dist = float(np.linalg.norm(displacement))

            # ── Anomalous jump filtering ──
            if dist > self.max_jump_dist:
                # Position jump is unreasonable, only advance timestamp, do not update velocity/position
                self.history.append((stamp, last_pos.copy()))
                return

            instant_velocity = displacement / dt

            if not self._initialized:
                # Direct assignment on first frame to avoid underestimation from smoothing from zero
                self.velocity = instant_velocity
                self._initialized = True
            else:
                alpha_t = 1.0 - math.exp(-dt / self.velocity_tau)
                self.velocity = (
                    alpha_t * instant_velocity
                    + (1.0 - alpha_t) * self.velocity
                )

            # ── Speed limit protection ──
            speed = float(np.linalg.norm(self.velocity))
            if speed > self.max_ped_speed:
                self.velocity = self.velocity / speed * self.max_ped_speed
                speed = self.max_ped_speed
            self.velocity_history.append(self.velocity.copy())

            if speed > 1e-3:
                velocity_heading = self.velocity / speed
                if observed_heading is None:
                    self.heading = velocity_heading
                else:
                    # Gazebo yaw gives useful direction during slow/stop frames;
                    # velocity remains dominant when displacement is reliable.
                    mixed = 0.8 * velocity_heading + 0.2 * observed_heading
                    norm = float(np.linalg.norm(mixed))
                    self.heading = mixed / norm if norm > 1e-6 else velocity_heading
            elif observed_heading is not None:
                self.heading = observed_heading

        self.history.append((stamp, pos.copy()))

    def velocity_uncertainty(self) -> float:
        """Estimate speed-estimation uncertainty from recent velocity samples."""
        if len(self.velocity_history) < 3:
            return 0.0
        velocities = np.array(self.velocity_history, dtype=float)
        return float(np.linalg.norm(np.std(velocities, axis=0)))

    # ── Trajectory prediction ─────────────────────────────────────────────
    def predict(
        self,
        dt: float,
        q_scale: float,
        lateral_q_scale: float,
        longitudinal_sigma: float,
        lateral_sigma: float,
        speed_sigma_scale: float,
        stationary_sigma: float,
        min_motion_speed: float,
        # Adaptive horizon
        base_lookahead: float = 2.0,
        speed_lookahead_gain: float = 1.0,
        min_lookahead: float = 1.5,
        max_lookahead: float = 5.0,
        # Non-linear uncertainty
        uncertainty_exponent: float = 1.3,
        velocity_uncertainty_gain: float = 0.35,
        max_uncertainty_scale: float = 1.6,
        # Heading inertia
        max_heading_change: float = 0.524,  # ~30° in rad
        # Interactive prediction
        robot_trajs: list[list[np.ndarray]] | None = None,
        ped_react_gain: float = 0.15,
        # Three-tier hybrid mode
        slow_speed_thresh: float = 0.3,
        wandering_ratio: float = 0.3,
    ) -> list[tuple[np.ndarray, np.ndarray, float, float]]:
        """
        Return (mean, heading, sigma_long, sigma_short) for each future step.

        Enhancements:
          – Three-tier hybrid mode (stationary/wandering→circular, slow→weak anisotropic, fast→narrow-long ellipse)
          – Adaptive horizon (fast pedestrians predict further)
          – Non-linear uncertainty growth (k^exponent)
          – Heading inertia constraint (limit heading change per step)
          – Interactive prediction (pedestrian shifts away from nearest robot candidate trajectory)
        """
        base_pos = self.position
        speed = float(np.linalg.norm(self.velocity))

        # ── Adaptive prediction horizon ──
        lookahead_time = float(np.clip(
            base_lookahead + speed_lookahead_gain * speed,
            min_lookahead,
            max_lookahead,
        ))
        effective_horizon = max(1, int(lookahead_time / dt))

        # ── Three-tier hybrid mode sigma selection ──
        wandering = self.is_wandering(ratio_thresh=wandering_ratio)

        if speed < min_motion_speed or wandering:
            # ▸ Mode A: stationary / wandering → circular symmetric Gaussian
            heading = self.heading
            base_long = stationary_sigma
            base_short = stationary_sigma
        elif speed < slow_speed_thresh:
            # ▸ Mode B: slow → weak anisotropic (ellipse more circular)
            heading = self.velocity / speed
            base_long = longitudinal_sigma + 0.5 * speed_sigma_scale * speed
            base_short = lateral_sigma * 1.3  # Lateral slightly wider, reflecting direction uncertainty
        else:
            # ▸ Mode C: fast → narrow-long ellipse (high directionality)
            heading = self.velocity / speed
            base_long = longitudinal_sigma + speed_sigma_scale * speed
            base_short = lateral_sigma * 0.8  # Lateral narrower, high direction certainty

        velocity_uncertainty = self.velocity_uncertainty()
        sigma_scale = float(np.clip(
            1.0 + velocity_uncertainty_gain * velocity_uncertainty,
            1.0,
            max_uncertainty_scale,
        ))
        base_long *= sigma_scale
        base_short *= sigma_scale

        predictions: list[tuple[np.ndarray, np.ndarray, float, float]] = []
        current_heading = heading.copy()
        prev_heading = heading.copy()

        for k in range(1, effective_horizon + 1):
            mean = base_pos + self.velocity * (k * dt)

            # ── Interactive prediction: pedestrian's response to robot ──
            if robot_trajs is not None:
                robot_pos_k = self._nearest_robot_prediction(robot_trajs, k - 1, mean)
                ped_to_robot = robot_pos_k - mean if robot_pos_k is not None else None
            else:
                ped_to_robot = None

            if ped_to_robot is not None:
                dist_to_robot = float(np.linalg.norm(ped_to_robot))
                if dist_to_robot > 0.3:
                    avoidance_dir = -ped_to_robot / dist_to_robot
                    offset = ped_react_gain / (dist_to_robot + 0.1) * avoidance_dir
                    mean = mean + offset

            # ── Heading inertia constraint: each step from prev_heading to target heading rotate at most max_heading_change ──
            # prev_heading is the constrained direction from previous step; heading is current velocity direction (target).
            # Rotate toward target each step but not exceed max_heading_change, avoid abrupt direction change in far-field.
            if speed >= min_motion_speed:
                cross = (prev_heading[0] * heading[1]
                         - prev_heading[1] * heading[0])
                dot = float(
                    prev_heading[0] * heading[0] + prev_heading[1] * heading[1]
                )
                angle_diff = math.atan2(cross, dot)
                if abs(angle_diff) > max_heading_change:
                    step = max_heading_change * (1.0 if angle_diff > 0 else -1.0)
                    cos_a = math.cos(step)
                    sin_a = math.sin(step)
                    current_heading = np.array([
                        prev_heading[0] * cos_a - prev_heading[1] * sin_a,
                        prev_heading[0] * sin_a + prev_heading[1] * cos_a,
                    ])
                else:
                    current_heading = heading.copy()
            else:
                current_heading = heading.copy()
            prev_heading = current_heading.copy()

            # ── Non-linear uncertainty growth ──
            q_growth_long = q_scale * (k ** uncertainty_exponent)
            q_growth_short = lateral_q_scale * (k ** uncertainty_exponent)
            sigma_long = math.sqrt(base_long ** 2 + q_growth_long)
            sigma_short = math.sqrt(base_short ** 2 + q_growth_short)

            predictions.append((
                mean.copy(),
                current_heading.copy(),
                sigma_long,
                sigma_short,
            ))

        return predictions

    @staticmethod
    def _nearest_robot_prediction(
        robot_trajs: list[list[np.ndarray]],
        step_index: int,
        ped_mean: np.ndarray,
    ) -> np.ndarray | None:
        nearest_pos = None
        nearest_dist = math.inf
        for traj in robot_trajs:
            if step_index >= len(traj):
                continue
            pos = traj[step_index]
            dist = float(np.linalg.norm(pos - ped_mean))
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_pos = pos
        return nearest_pos


# ──────────────────────────────────────────────────────────────
#  SocialNavNode (Enhanced Version)
# ──────────────────────────────────────────────────────────────
class SocialNavNode(Node):

    def __init__(self):
        super().__init__("social_nav_node")

        # ── Original parameters ─────────────────────────────────────────
        self.declare_parameter("prediction_dt", 0.5)
        self.declare_parameter("q_scale", 0.08)
        self.declare_parameter("lateral_q_scale", 0.02)
        self.declare_parameter("gamma", 0.72)
        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("input_topic", "/pedestrian_poses")
        self.declare_parameter("output_topic", "/social_costmap")
        self.declare_parameter("velocity_alpha", 0.35)
        self.declare_parameter("velocity_tau", 0.35)
        self.declare_parameter("track_timeout", 1.0)

        self.declare_parameter("grid_resolution", 0.05)
        self.declare_parameter("grid_width", 9.45)
        self.declare_parameter("grid_height", 22.35)
        self.declare_parameter("grid_origin_x", -5.999)
        self.declare_parameter("grid_origin_y", -11.076)

        self.declare_parameter("peak_cost", 55)
        self.declare_parameter("min_publish_cost", 1)
        self.declare_parameter("cutoff_sigma", 2.5)
        self.declare_parameter("longitudinal_sigma", 0.80)
        self.declare_parameter("lateral_sigma", 0.35)
        self.declare_parameter("stationary_sigma", 0.45)
        self.declare_parameter("speed_sigma_scale", 0.35)
        self.declare_parameter("min_motion_speed", 0.05)

        # ── New parameters ─────────────────────────────────────────
        # 1. Nearest-neighbor association
        self.declare_parameter("association_threshold", 2.0)
        # 2. Speed protection
        self.declare_parameter("max_ped_speed", 1.8)
        self.declare_parameter("max_jump_dist", 1.0)
        # 3. Asymmetric Gaussian
        self.declare_parameter("behind_sigma_ratio", 0.5)
        # 4. Group cost
        self.declare_parameter("group_threshold", 1.5)
        self.declare_parameter("group_sigma", 0.6)
        self.declare_parameter("group_cost_ratio", 0.45)
        # 5. Adaptive horizon
        self.declare_parameter("base_lookahead", 1.5)
        self.declare_parameter("speed_lookahead_gain", 0.7)
        self.declare_parameter("min_lookahead", 1.5)
        self.declare_parameter("max_lookahead", 3.5)
        # 6. Robot direction-aware
        self.declare_parameter("behind_ped_weight", 0.15)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("robot_pose_topic", "/amcl_pose")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        # 8. Advanced prediction
        self.declare_parameter("uncertainty_exponent", 1.3)
        self.declare_parameter("velocity_uncertainty_gain", 0.35)
        self.declare_parameter("max_uncertainty_scale", 1.6)
        self.declare_parameter("max_heading_change_deg", 30.0)
        self.declare_parameter("ped_react_gain", 0.15)
        # 9. Three-tier hybrid mode + wandering detection
        self.declare_parameter("slow_speed_thresh", 0.3)
        self.declare_parameter("wandering_ratio", 0.3)
        # 10. Robot relevance: keep near conflicts strong, soften distant forecasts
        self.declare_parameter("social_relevance_distance", 3.5)
        self.declare_parameter("social_relevance_soft_margin", 2.0)
        self.declare_parameter("min_relevance_weight", 0.10)
        self.declare_parameter("instant_cost_ratio", 0.85)
        self.declare_parameter("instant_sigma", 0.75)
        self.declare_parameter("approach_cost_gain", 0.3)
        self.declare_parameter("lethal_core_enabled", True)
        self.declare_parameter("lethal_core_radius", 0.25)
        self.declare_parameter("lethal_core_value", 90.0)
        # 11. Lightweight game-style prediction with robot trajectory hypotheses
        self.declare_parameter("use_ped_orientation", True)
        self.declare_parameter("ped_orientation_offset_deg", -90.0)
        self.declare_parameter("robot_traj_hypotheses", 3)
        self.declare_parameter("robot_traj_yaw_rate_span", 0.8)
        self.declare_parameter("robot_prediction_min_speed", 0.12)

        # ── Read parameters ─────────────────────────────────────────
        self.pred_dt = float(self.get_parameter("prediction_dt").value)
        self.q_scale = float(self.get_parameter("q_scale").value)
        self.lateral_q_scale = float(self.get_parameter("lateral_q_scale").value)
        self.gamma = float(self.get_parameter("gamma").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.velocity_alpha = float(self.get_parameter("velocity_alpha").value)
        self.velocity_tau = float(self.get_parameter("velocity_tau").value)
        self.track_timeout = float(self.get_parameter("track_timeout").value)

        self.resolution = float(self.get_parameter("grid_resolution").value)
        self.grid_width_m = float(self.get_parameter("grid_width").value)
        self.grid_height_m = float(self.get_parameter("grid_height").value)
        self.origin_x = float(self.get_parameter("grid_origin_x").value)
        self.origin_y = float(self.get_parameter("grid_origin_y").value)
        self.width_cells = max(1, int(math.ceil(self.grid_width_m / self.resolution)))
        self.height_cells = max(1, int(math.ceil(self.grid_height_m / self.resolution)))

        self.peak_cost = float(self.get_parameter("peak_cost").value)
        self.min_publish_cost = float(self.get_parameter("min_publish_cost").value)
        self.cutoff_sigma = float(self.get_parameter("cutoff_sigma").value)
        self.longitudinal_sigma = float(self.get_parameter("longitudinal_sigma").value)
        self.lateral_sigma = float(self.get_parameter("lateral_sigma").value)
        self.stationary_sigma = float(self.get_parameter("stationary_sigma").value)
        self.speed_sigma_scale = float(self.get_parameter("speed_sigma_scale").value)
        self.min_motion_speed = float(self.get_parameter("min_motion_speed").value)

        self.association_threshold = float(self.get_parameter("association_threshold").value)
        self.max_ped_speed = float(self.get_parameter("max_ped_speed").value)
        self.max_jump_dist = float(self.get_parameter("max_jump_dist").value)
        self.behind_sigma_ratio = float(self.get_parameter("behind_sigma_ratio").value)
        self.group_threshold = float(self.get_parameter("group_threshold").value)
        self.group_sigma = float(self.get_parameter("group_sigma").value)
        self.group_cost_ratio = float(self.get_parameter("group_cost_ratio").value)
        self.base_lookahead = float(self.get_parameter("base_lookahead").value)
        self.speed_lookahead_gain = float(self.get_parameter("speed_lookahead_gain").value)
        self.min_lookahead = float(self.get_parameter("min_lookahead").value)
        self.max_lookahead = float(self.get_parameter("max_lookahead").value)
        self.behind_ped_weight = float(self.get_parameter("behind_ped_weight").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.robot_pose_topic = str(self.get_parameter("robot_pose_topic").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_base_frame = str(self.get_parameter("robot_base_frame").value)
        self.uncertainty_exponent = float(self.get_parameter("uncertainty_exponent").value)
        self.velocity_uncertainty_gain = float(
            self.get_parameter("velocity_uncertainty_gain").value
        )
        self.max_uncertainty_scale = float(
            self.get_parameter("max_uncertainty_scale").value
        )
        self.max_heading_change = math.radians(
            float(self.get_parameter("max_heading_change_deg").value)
        )
        self.ped_react_gain = float(self.get_parameter("ped_react_gain").value)
        self.slow_speed_thresh = float(self.get_parameter("slow_speed_thresh").value)
        self.wandering_ratio = float(self.get_parameter("wandering_ratio").value)
        self.social_relevance_distance = float(
            self.get_parameter("social_relevance_distance").value
        )
        self.social_relevance_soft_margin = float(
            self.get_parameter("social_relevance_soft_margin").value
        )
        self.min_relevance_weight = float(
            self.get_parameter("min_relevance_weight").value
        )
        self.instant_cost_ratio = float(
            self.get_parameter("instant_cost_ratio").value
        )
        self.instant_sigma = float(
            self.get_parameter("instant_sigma").value
        )
        self.approach_cost_gain = float(
            self.get_parameter("approach_cost_gain").value
        )
        self.lethal_core_enabled = bool(
            self.get_parameter("lethal_core_enabled").value
        )
        self.lethal_core_radius = float(
            self.get_parameter("lethal_core_radius").value
        )
        self.lethal_core_value = float(
            self.get_parameter("lethal_core_value").value
        )
        self.use_ped_orientation = bool(self.get_parameter("use_ped_orientation").value)
        self.ped_orientation_offset = math.radians(
            float(self.get_parameter("ped_orientation_offset_deg").value)
        )
        self.robot_traj_hypotheses = int(
            self.get_parameter("robot_traj_hypotheses").value
        )
        self.robot_traj_yaw_rate_span = float(
            self.get_parameter("robot_traj_yaw_rate_span").value
        )
        self.robot_prediction_min_speed = float(
            self.get_parameter("robot_prediction_min_speed").value
        )

        # ── Grid coordinate precomputation ───────────────────────────────────
        xs = self.origin_x + (np.arange(self.width_cells) + 0.5) * self.resolution
        ys = self.origin_y + (np.arange(self.height_cells) + 0.5) * self.resolution
        self.cell_x, self.cell_y = np.meshgrid(xs, ys)

        # ── Tracker management ─────────────────────────────────────
        self.trackers: dict[int, PedestrianTracker] = {}
        self.last_stamp: dict[int, float] = {}
        self._next_id: int = 0  # Global auto-incrementing ID

        # ── Robot state (direction-aware + interactive prediction) ────────
        self.robot_pos: np.ndarray | None = None
        self.robot_heading: np.ndarray = np.array([1.0, 0.0])
        self.robot_velocity: np.ndarray = np.zeros(2)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Subscriptions / Publications ─────────────────────────────────────
        self.sub = self.create_subscription(
            PoseArray, self.input_topic, self._ped_callback, 10,
        )
        self.robot_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.robot_pose_topic, self._robot_pose_callback, 10,
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, 10,
        )
        self.pub = self.create_publisher(
            OccupancyGrid, self.output_topic, 1,
        )

        rate = float(self.get_parameter("publish_rate").value)
        self.timer = self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            "Enhanced anisotropic social costmap ready: "
            f"{self.input_topic} -> {self.output_topic}, "
            f"{self.width_cells}x{self.height_cells}@{self.resolution:.3f}m"
        )

    # ─────────────────────────────────────────────────────────
    #  Robot pose / velocity callbacks
    # ─────────────────────────────────────────────────────────
    def _robot_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """Get robot pose in map frame, consistent with map frame pedestrians / social_costmap."""
        self.robot_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        ], dtype=float)

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.robot_heading = np.array([math.cos(yaw), math.sin(yaw)])

    def _refresh_robot_pose_from_tf(self) -> bool:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                Time(),
            )
        except TransformException:
            return False

        translation = transform.transform.translation
        self.robot_pos = np.array([translation.x, translation.y], dtype=float)

        q = transform.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.robot_heading = np.array([math.cos(yaw), math.sin(yaw)])
        return True

    def _odom_callback(self, msg: Odometry) -> None:
        """Get robot velocity; pose/heading provided by /amcl_pose to avoid mixing odom/map.
        
        Only use /odom velocity. Pose must come from map frame (/amcl_pose or TF),
        otherwise robot_pos will misalign between odom/map frames.
        """
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        cos_y, sin_y = self.robot_heading
        self.robot_velocity = np.array([
            vx * cos_y - vy * sin_y,
            vx * sin_y + vy * cos_y,
        ], dtype=float)

    # ─────────────────────────────────────────────────────────
    #  Nearest-neighbor data association
    # ─────────────────────────────────────────────────────────
    def _associate(
        self, observations: list[np.ndarray], now: float
    ) -> list[tuple[int, np.ndarray, int]]:
        """
        Match current frame observations with existing trackers using global optimal nearest-neighbor.
        Observations with distance > association_threshold are treated as new pedestrians.
        """
        matched: list[tuple[int, np.ndarray, int]] = []

        if not self.trackers or not observations:
            for obs_idx, obs in enumerate(observations):
                matched.append((self._next_id, obs, obs_idx))
                self._next_id += 1
            return matched

        tracker_ids = list(self.trackers.keys())
        tracker_positions = np.array([
            self.trackers[tid].position for tid in tracker_ids
        ])
        obs_positions = np.array(observations)

        # Distance matrix (n_trackers, n_observations)
        diff = tracker_positions[:, None, :] - obs_positions[None, :, :]
        dist_matrix = np.linalg.norm(diff, axis=2)

        used_obs: set[int] = set()
        for t_idx, o_idx in self._global_assignment(dist_matrix):
            matched.append((tracker_ids[t_idx], observations[o_idx], o_idx))
            used_obs.add(o_idx)

        # Unmatched observations → new tracker
        for o_idx, obs in enumerate(observations):
            if o_idx not in used_obs:
                matched.append((self._next_id, obs, o_idx))
                self._next_id += 1

        return matched

    def _global_assignment(self, dist_matrix: np.ndarray) -> list[tuple[int, int]]:
        """Small-scale Hungarian-style global matching, avoid greedy ID swaps during pedestrian crossing."""
        n_trackers, n_obs = dist_matrix.shape
        max_pairs = min(n_trackers, n_obs)
        best_pairs: list[tuple[int, int]] = []
        best_cost = math.inf

        for pair_count in range(1, max_pairs + 1):
            for tracker_subset in combinations(range(n_trackers), pair_count):
                for obs_subset in combinations(range(n_obs), pair_count):
                    for obs_perm in permutations(obs_subset):
                        pairs = list(zip(tracker_subset, obs_perm))
                        dists = [float(dist_matrix[t, o]) for t, o in pairs]
                        if any(d > self.association_threshold for d in dists):
                            continue
                        cost = sum(dists)
                        if pair_count > len(best_pairs) or (
                            pair_count == len(best_pairs) and cost < best_cost
                        ):
                            best_pairs = pairs
                            best_cost = cost

        return best_pairs

    # ─────────────────────────────────────────────────────────
    #  Pedestrian callback (nearest-neighbor version)
    # ─────────────────────────────────────────────────────────
    def _ped_callback(self, msg: PoseArray) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9

        observations = [
            np.array([pose.position.x, pose.position.y], dtype=float)
            for pose in msg.poses
        ]
        observed_headings = [
            self._pose_heading(pose) if self.use_ped_orientation else None
            for pose in msg.poses
        ]

        associations = self._associate(observations, now)

        for ped_id, pos, obs_idx in associations:
            observed_heading = observed_headings[obs_idx]
            if ped_id not in self.trackers:
                self.trackers[ped_id] = PedestrianTracker(
                    pos, now,
                    alpha=self.velocity_alpha,
                    velocity_tau=self.velocity_tau,
                    max_ped_speed=self.max_ped_speed,
                    max_jump_dist=self.max_jump_dist,
                    init_heading=observed_heading,
                )
            else:
                self.trackers[ped_id].update(pos, now, observed_heading)

            self.last_stamp[ped_id] = now

    def _pose_heading(self, pose: Pose) -> np.ndarray | None:
        """Convert a pedestrian pose orientation into a map-frame heading."""
        q = pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-6:
            return None

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp) + self.ped_orientation_offset
        return np.array([math.cos(yaw), math.sin(yaw)], dtype=float)

    # ─────────────────────────────────────────────────────────
    #  Publish costmap
    # ─────────────────────────────────────────────────────────
    def _publish(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        self._drop_stale_tracks(now)

        costs = np.zeros((self.height_cells, self.width_cells), dtype=np.float32)

        # ── Multi-hypothesis robot trajectory extrapolation (lightweight interaction / game-theoretic prediction)────────
        robot_steps = max(1, int(math.ceil(self.max_lookahead / self.pred_dt)))
        robot_trajs = self._build_robot_trajectory_hypotheses(max_steps=robot_steps)

        for tracker in self.trackers.values():
            self._add_instant_pedestrian_cost(costs, tracker, robot_trajs)
            if self.lethal_core_enabled:
                self._mark_lethal_circle(
                    costs,
                    tracker.position,
                    self.lethal_core_radius,
                    self.lethal_core_value,
                )

            predictions = tracker.predict(
                dt=self.pred_dt,
                q_scale=self.q_scale,
                lateral_q_scale=self.lateral_q_scale,
                longitudinal_sigma=self.longitudinal_sigma,
                lateral_sigma=self.lateral_sigma,
                speed_sigma_scale=self.speed_sigma_scale,
                stationary_sigma=self.stationary_sigma,
                min_motion_speed=self.min_motion_speed,
                base_lookahead=self.base_lookahead,
                speed_lookahead_gain=self.speed_lookahead_gain,
                min_lookahead=self.min_lookahead,
                max_lookahead=self.max_lookahead,
                uncertainty_exponent=self.uncertainty_exponent,
                velocity_uncertainty_gain=self.velocity_uncertainty_gain,
                max_uncertainty_scale=self.max_uncertainty_scale,
                max_heading_change=self.max_heading_change,
                robot_trajs=robot_trajs,
                ped_react_gain=self.ped_react_gain,
                slow_speed_thresh=self.slow_speed_thresh,
                wandering_ratio=self.wandering_ratio,
            )

            # ── Robot direction-aware: rear pedestrians downweighted ──
            ped_behind_robot = False
            if self.robot_pos is not None:
                robot_to_ped = tracker.position - self.robot_pos
                dot = float(np.dot(robot_to_ped, self.robot_heading))
                if dot < 0:
                    ped_behind_robot = True

            for k, (mean, heading, sigma_long, sigma_short) in enumerate(predictions):
                amplitude = self.peak_cost * (self.gamma ** (k + 1))
                if ped_behind_robot:
                    amplitude *= self.behind_ped_weight
                amplitude *= self._approach_cost_scale(tracker)
                amplitude *= self._prediction_relevance_weight(mean, k, robot_trajs)
                if amplitude < self.min_publish_cost:
                    continue
                self._accumulate_gaussian(
                    costs, mean, heading, sigma_long, sigma_short, amplitude,
                )

        # ── Group / Crowd cost ───────────────────────────────
        self._add_group_costs(costs)

        self.pub.publish(self._build_grid(costs))

    def _add_instant_pedestrian_cost(
        self,
        costs: np.ndarray,
        tracker: PedestrianTracker,
        robot_trajs: list[list[np.ndarray]] | None,
    ) -> None:
        """Add k=0 pedestrian cost so fast crossings are visible immediately."""
        amplitude = self.peak_cost * self.instant_cost_ratio
        amplitude *= self._approach_cost_scale(tracker)
        amplitude *= self._prediction_relevance_weight(
            tracker.position, 0, robot_trajs,
        )
        if amplitude < self.min_publish_cost:
            return

        self._accumulate_gaussian(
            costs,
            tracker.position,
            tracker.heading,
            self.instant_sigma,
            self.instant_sigma,
            amplitude,
        )

    def _mark_lethal_circle(
        self,
        costs: np.ndarray,
        center: np.ndarray,
        radius: float,
        value: float,
    ) -> None:
        """Mark the current intimate zone as a hard obstacle in the social grid."""
        radius = max(radius, self.resolution)
        min_x = max(0, int(math.floor(
            (center[0] - radius - self.origin_x) / self.resolution)))
        max_x = min(self.width_cells, int(math.ceil(
            (center[0] + radius - self.origin_x) / self.resolution)))
        min_y = max(0, int(math.floor(
            (center[1] - radius - self.origin_y) / self.resolution)))
        max_y = min(self.height_cells, int(math.ceil(
            (center[1] + radius - self.origin_y) / self.resolution)))

        if min_x >= max_x or min_y >= max_y:
            return

        x_slice = self.cell_x[min_y:max_y, min_x:max_x]
        y_slice = self.cell_y[min_y:max_y, min_x:max_x]
        dist = np.sqrt((x_slice - center[0]) ** 2 + (y_slice - center[1]) ** 2)
        mask = dist <= radius
        window = costs[min_y:max_y, min_x:max_x]
        window[mask] = np.maximum(window[mask], value)

    def _build_robot_trajectory_hypotheses(
        self,
        max_steps: int,
    ) -> list[list[np.ndarray]] | None:
        """Generate short-horizon robot candidate trajectories for interaction prediction."""
        if self.robot_pos is None:
            return None

        speed = float(np.linalg.norm(self.robot_velocity))
        if speed < self.robot_prediction_min_speed:
            speed = self.robot_prediction_min_speed

        base_yaw = math.atan2(self.robot_heading[1], self.robot_heading[0])
        count = max(1, self.robot_traj_hypotheses)
        if count == 1:
            yaw_rates = [0.0]
        else:
            yaw_rates = np.linspace(
                -self.robot_traj_yaw_rate_span,
                self.robot_traj_yaw_rate_span,
                count,
            )

        trajectories: list[list[np.ndarray]] = []
        for yaw_rate in yaw_rates:
            pos = self.robot_pos.copy()
            yaw = base_yaw
            traj: list[np.ndarray] = []
            for _ in range(max_steps):
                yaw += float(yaw_rate) * self.pred_dt
                direction = np.array([math.cos(yaw), math.sin(yaw)], dtype=float)
                pos = pos + direction * speed * self.pred_dt
                traj.append(pos.copy())
            trajectories.append(traj)

        return trajectories

    # ─────────────────────────────────────────────────────────
    #  Group cost
    # ─────────────────────────────────────────────────────────
    def _add_group_costs(self, costs: np.ndarray) -> None:
        """When pedestrians at distance < threshold, add circular Gaussian at midpoint."""
        items = list(self.trackers.items())
        n = len(items)
        if n < 2:
            return

        for i in range(n):
            for j in range(i + 1, n):
                pos_i = items[i][1].position
                pos_j = items[j][1].position
                dist = float(np.linalg.norm(pos_i - pos_j))
                if dist < self.group_threshold:
                    midpoint = (pos_i + pos_j) / 2.0
                    heading = np.array([1.0, 0.0])  # Circular symmetric
                    amplitude = (
                        self.peak_cost
                        * self.group_cost_ratio
                        * self._prediction_relevance_weight(midpoint, 0, None)
                    )
                    self._accumulate_gaussian(
                        costs, midpoint, heading,
                        self.group_sigma, self.group_sigma, amplitude,
                    )

    # ─────────────────────────────────────────────────────────
    #  Robot relevance weighting
    # ─────────────────────────────────────────────────────────
    def _prediction_relevance_weight(
        self,
        mean: np.ndarray,
        step_index: int,
        robot_trajs: list[list[np.ndarray]] | None,
    ) -> float:
        """Reduce far-field cost away from robot current/predicted trajectory, minimize irrelevant pedestrian blocking."""
        if self.robot_pos is None:
            return 1.0

        current_dist = float(np.linalg.norm(mean - self.robot_pos))
        relevant_dist = current_dist

        if robot_trajs is not None:
            for traj in robot_trajs:
                if step_index >= len(traj):
                    continue
                predicted_dist = float(np.linalg.norm(mean - traj[step_index]))
                relevant_dist = min(relevant_dist, predicted_dist)

        hard = self.social_relevance_distance
        soft = max(self.social_relevance_soft_margin, 1e-3)
        floor = float(np.clip(self.min_relevance_weight, 0.0, 1.0))

        if relevant_dist <= hard:
            return 1.0
        if relevant_dist >= hard + soft:
            return floor

        ratio = (relevant_dist - hard) / soft
        return 1.0 - ratio * (1.0 - floor)

    def _approach_cost_scale(self, tracker: PedestrianTracker) -> float:
        """Boost costs when a pedestrian is moving toward the robot."""
        if self.robot_pos is None:
            return 1.0

        ped_to_robot = self.robot_pos - tracker.position
        dist = float(np.linalg.norm(ped_to_robot))
        if dist <= 0.1:
            return 1.0

        direction = ped_to_robot / dist
        approach = float(np.dot(tracker.velocity, direction))
        if approach <= 0.0:
            return 1.0

        speed_scale = max(self.max_ped_speed, 1e-3)
        approach_factor = min(approach / speed_scale, 1.0)
        return 1.0 + self.approach_cost_gain * approach_factor

    # ─────────────────────────────────────────────────────────
    #  Stale track cleanup
    # ─────────────────────────────────────────────────────────
    def _drop_stale_tracks(self, now: float) -> None:
        stale_ids = [
            ped_id
            for ped_id, stamp in self.last_stamp.items()
            if now - stamp > self.track_timeout
        ]
        for ped_id in stale_ids:
            self.trackers.pop(ped_id, None)
            self.last_stamp.pop(ped_id, None)

    # ─────────────────────────────────────────────────────────
    #  Asymmetric anisotropic Gaussian
    # ─────────────────────────────────────────────────────────
    def _accumulate_gaussian(
        self,
        costs: np.ndarray,
        mean: np.ndarray,
        heading: np.ndarray,
        sigma_long: float,
        sigma_short: float,
        amplitude: float,
    ) -> None:
        """Front sigma kept as-is, rear uses behind_sigma_ratio reduction."""
        radius = self.cutoff_sigma * max(sigma_long, sigma_short)
        min_x = max(0, int(math.floor(
            (mean[0] - radius - self.origin_x) / self.resolution)))
        max_x = min(self.width_cells, int(math.ceil(
            (mean[0] + radius - self.origin_x) / self.resolution)))
        min_y = max(0, int(math.floor(
            (mean[1] - radius - self.origin_y) / self.resolution)))
        max_y = min(self.height_cells, int(math.ceil(
            (mean[1] + radius - self.origin_y) / self.resolution)))

        if min_x >= max_x or min_y >= max_y:
            return

        x_slice = self.cell_x[min_y:max_y, min_x:max_x]
        y_slice = self.cell_y[min_y:max_y, min_x:max_x]
        dx = x_slice - mean[0]
        dy = y_slice - mean[1]

        long_axis = dx * heading[0] + dy * heading[1]
        short_axis = -dx * heading[1] + dy * heading[0]

        # ── Asymmetric: front sigma_long, rear behind_ratio * sigma_long ──
        sigma_long_front = sigma_long
        sigma_long_behind = self.behind_sigma_ratio * sigma_long
        sigma_long_used = np.where(long_axis >= 0, sigma_long_front, sigma_long_behind)

        exponent = -0.5 * (
            (long_axis / sigma_long_used) ** 2
            + (short_axis / sigma_short) ** 2
        )
        gaussian_cost = amplitude * np.exp(exponent)

        window = costs[min_y:max_y, min_x:max_x]
        np.maximum(window, gaussian_cost, out=window)

    # ─────────────────────────────────────────────────────────
    #  Build OccupancyGrid
    # ─────────────────────────────────────────────────────────
    def _build_grid(self, costs: np.ndarray) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = self.resolution
        msg.info.width = self.width_cells
        msg.info.height = self.height_cells

        origin = Pose()
        origin.position.x = self.origin_x
        origin.position.y = self.origin_y
        origin.position.z = 0.0
        origin.orientation.w = 1.0
        msg.info.origin = origin

        grid = np.rint(np.clip(costs, 0, 100)).astype(np.int8)
        grid[grid < self.min_publish_cost] = 0
        msg.data = grid.flatten().tolist()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = SocialNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
