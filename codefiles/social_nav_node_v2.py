#!/usr/bin/env python3
"""
social_nav_node_v2.py  –  Enhanced Social Costmap (dstar_plus_v2 mode)

Adds two targeted improvements on top of SocialNavNode:

  1. Proxemics Direction Modulation  (~20 lines)
     Hall's proxemics: frontal personal space is asymmetrically larger than rear.
     When the robot is directly in front of a pedestrian (head-on encounter),
     the social cost is boosted by up to proxemics_front_gain (default 40 %).
     When the robot is behind the pedestrian the cost is reduced by up to
     proxemics_back_gain (default 30 %).  The scale is proportional to the
     cosine of the angle between the pedestrian's heading and the
     pedestrian→robot vector, so it blends smoothly from side-approach (×1.0)
     to full head-on (×1.4) or full rear approach (×0.7).

  2. Uncertainty Staleness Decay  (~5 lines)
     Stale tracks (not refreshed near track_timeout) kept full amplitude in
     the original node, making the cost-map "too conservative" and blocking
     paths unnecessarily.  Here each track's total cost is multiplied by
       exp(−staleness_decay_rate × age_seconds)
     so a 1.0 s-old track (at default rate 0.5) retains ≈ 61 % amplitude
     and fades smoothly before being dropped by _drop_stale_tracks().

Topic In / Out: identical to social_nav_node.py.
New ROS2 parameters (all optional, sensible defaults):
  proxemics_direction_enabled  bool  true
  proxemics_front_gain         float 0.4
  proxemics_back_gain          float 0.3
  staleness_decay_enabled      bool  true
  staleness_decay_rate         float 0.5   (1/s)
"""

import math

import numpy as np
import rclpy

from social_nav_node import PedestrianTracker, SocialNavNode


class SocialNavNodeV2(SocialNavNode):
    """SocialNavNode + proxemics direction modulation + staleness decay."""

    def __init__(self):
        super().__init__()

        # ── Feature 1: Proxemics direction modulation ─────────────────────
        self.declare_parameter("proxemics_direction_enabled", True)
        self.declare_parameter("proxemics_front_gain", 0.4)
        self.declare_parameter("proxemics_back_gain", 0.3)

        self.proxemics_direction_enabled = bool(
            self.get_parameter("proxemics_direction_enabled").value
        )
        self.proxemics_front_gain = float(
            self.get_parameter("proxemics_front_gain").value
        )
        self.proxemics_back_gain = float(
            self.get_parameter("proxemics_back_gain").value
        )

        # ── Feature 2: Staleness decay ────────────────────────────────────
        self.declare_parameter("staleness_decay_enabled", True)
        self.declare_parameter("staleness_decay_rate", 0.5)

        self.staleness_decay_enabled = bool(
            self.get_parameter("staleness_decay_enabled").value
        )
        self.staleness_decay_rate = float(
            self.get_parameter("staleness_decay_rate").value
        )

        self.get_logger().info(
            f"SocialNavNodeV2 active — "
            f"proxemics_direction={'ON' if self.proxemics_direction_enabled else 'OFF'} "
            f"(front_gain={self.proxemics_front_gain}, back_gain={self.proxemics_back_gain}), "
            f"staleness_decay={'ON' if self.staleness_decay_enabled else 'OFF'} "
            f"(rate={self.staleness_decay_rate:.2f} 1/s)"
        )

    # ─────────────────────────────────────────────────────────────────────
    #  Feature 1 – Proxemics direction scale
    # ─────────────────────────────────────────────────────────────────────
    def _proxemics_direction_scale(self, tracker: PedestrianTracker) -> float:
        """Return a multiplier in [~0.05, 1+front_gain] based on approach angle.

        cos_angle = dot(ped_heading, ped→robot_unit_vec)
          +1  → robot squarely in front of pedestrian  → boost cost
           0  → robot to the side                      → neutral
          -1  → robot squarely behind pedestrian       → reduce cost
        """
        if not self.proxemics_direction_enabled or self.robot_pos is None:
            return 1.0

        ped_to_robot = self.robot_pos - tracker.position
        dist = float(np.linalg.norm(ped_to_robot))
        if dist <= 0.1:
            return 1.0

        cos_angle = float(np.dot(tracker.heading, ped_to_robot / dist))

        if cos_angle >= 0.0:
            return 1.0 + self.proxemics_front_gain * cos_angle
        return max(0.05, 1.0 + self.proxemics_back_gain * cos_angle)

    # ─────────────────────────────────────────────────────────────────────
    #  Override instant cost to apply both new modifiers
    # ─────────────────────────────────────────────────────────────────────
    def _add_instant_pedestrian_cost_v2(
        self,
        costs: np.ndarray,
        tracker: PedestrianTracker,
        robot_trajs,
        staleness_factor: float,
    ) -> None:
        amplitude = self.peak_cost * self.instant_cost_ratio
        amplitude *= staleness_factor
        amplitude *= self._proxemics_direction_scale(tracker)
        amplitude *= self._approach_cost_scale(tracker)
        amplitude *= self._prediction_relevance_weight(tracker.position, 0, robot_trajs)
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

    # ─────────────────────────────────────────────────────────────────────
    #  Override main publish loop to wire in both features
    # ─────────────────────────────────────────────────────────────────────
    def _publish(self) -> None:  # noqa: C901
        now = self.get_clock().now().nanoseconds * 1e-9
        self._drop_stale_tracks(now)

        costs = np.zeros((self.height_cells, self.width_cells), dtype=np.float32)

        robot_steps = max(1, int(math.ceil(self.max_lookahead / self.pred_dt)))
        robot_trajs = self._build_robot_trajectory_hypotheses(max_steps=robot_steps)

        for ped_id, tracker in self.trackers.items():

            # ── Feature 2: staleness decay factor ──────────────────────
            if self.staleness_decay_enabled:
                age = now - self.last_stamp.get(ped_id, now)
                staleness_factor = math.exp(-self.staleness_decay_rate * age)
            else:
                staleness_factor = 1.0

            # k=0 instant cost (uses both new features)
            self._add_instant_pedestrian_cost_v2(costs, tracker, robot_trajs, staleness_factor)

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

            ped_behind_robot = False
            if self.robot_pos is not None:
                robot_to_ped = tracker.position - self.robot_pos
                if float(np.dot(robot_to_ped, self.robot_heading)) < 0:
                    ped_behind_robot = True

            # ── Feature 1: compute proxemics scale once per tracker ─────
            prox_scale = self._proxemics_direction_scale(tracker)

            for k, (mean, heading, sigma_long, sigma_short) in enumerate(predictions):
                amplitude = self.peak_cost * (self.gamma ** (k + 1))
                amplitude *= staleness_factor          # Feature 2
                amplitude *= prox_scale                # Feature 1
                if ped_behind_robot:
                    amplitude *= self.behind_ped_weight
                amplitude *= self._approach_cost_scale(tracker)
                amplitude *= self._prediction_relevance_weight(mean, k, robot_trajs)
                if amplitude < self.min_publish_cost:
                    continue
                self._accumulate_gaussian(
                    costs, mean, heading, sigma_long, sigma_short, amplitude,
                )

        self._add_group_costs(costs)
        self.pub.publish(self._build_grid(costs))


def main(args=None):
    rclpy.init(args=args)
    node = SocialNavNodeV2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
