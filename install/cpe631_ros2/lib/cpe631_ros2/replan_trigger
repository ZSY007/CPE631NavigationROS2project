#!/usr/bin/env python3
"""
replan_trigger.py — dstar_plus feature 2

When any pedestrian comes within TRIGGER_DIST metres of the robot, clear the
global costmap so that D* replans immediately with fresh social-cost data
rather than waiting for the next BT RateController tick (up to 0.5 s).

Topics subscribed:
  /pedestrian_poses  (geometry_msgs/PoseArray)
  /amcl_pose         (geometry_msgs/PoseWithCovarianceStamped)

Services called (on trigger):
  /global_costmap/clear_entirely_global_costmap  (nav2_msgs/ClearEntireCostmap)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from nav2_msgs.srv import ClearEntireCostmap
from std_msgs.msg import Bool


TRIGGER_DIST = 1.5   # metres — proximity threshold for immediate replan
COOLDOWN_S   = 2.0   # seconds — minimum gap between consecutive clears


class ReplanTrigger(Node):
    def __init__(self):
        super().__init__('replan_trigger')

        self.robot_x = None
        self.robot_y = None
        self._last_trigger = 0.0

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_cb,
            best_effort,
        )
        self.create_subscription(
            PoseArray,
            '/pedestrian_poses',
            self._ped_cb,
            10,
        )

        self._near_pub = self.create_publisher(Bool, '/near_pedestrian', 10)

        self._clear_cli = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap',
        )

        self.get_logger().info(
            f'ReplanTrigger ready — trigger dist={TRIGGER_DIST}m, cooldown={COOLDOWN_S}s'
        )

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def _ped_cb(self, msg: PoseArray):
        if self.robot_x is None:
            return

        min_dist = float('inf')
        for pose in msg.poses:
            dx = pose.position.x - self.robot_x
            dy = pose.position.y - self.robot_y
            dist = math.hypot(dx, dy)
            if dist < min_dist:
                min_dist = dist

        near = min_dist < TRIGGER_DIST
        self._near_pub.publish(Bool(data=near))

        if not near:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_trigger < COOLDOWN_S:
            return

        if not self._clear_cli.service_is_ready():
            return

        self._last_trigger = now
        self.get_logger().info(
            f'Pedestrian at {min_dist:.2f}m — clearing global costmap for immediate replan'
        )
        req = ClearEntireCostmap.Request()
        self._clear_cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = ReplanTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
