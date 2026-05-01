#!/usr/bin/env python3
"""
goal_sender.py — Nav2 action-client based route executor.

Sends waypoints sequentially via the NavigateToPose action interface.
This ensures goal_sender and data_collector agree on navigation outcomes
(succeeded / aborted / canceled) since both observe the same Nav2 action
status, eliminating the old AMCL-distance heuristic that could disagree
with Nav2's internal state.

Exit codes:
    0 — Route succeeded (all goals reached)
    2 — Route aborted  (Nav2 rejected / gave up on a goal)
    3 — Route canceled
    4 — Route timeout   (experiment_timeout exceeded)
    5 — Unexpected exception
"""

import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node


DEFAULT_GOALS = [
    ( 0.50, -6.50, -1.06),   # G1 south-center      (ped_1 / ped_3 interaction zone)
    ( 2.00, -2.50,  0.00),   # G2 east-mid           (ped_1 nearby; prev (2.80,-3.00) was in obstacle)
    ( 0.50,  6.50,  3.14),   # G3 north-center       (ped_4 interaction zone)
    (-3.50,  3.50,  1.57),   # G4 northwest          (ped_2 corridor)
    (-1.50,  1.50, -1.57),   # G5 center-west        (ped_5 zone; prev (-1.00,0.50) was in obstacle)
]

_STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
}


class GoalSender(Node):
    def __init__(self):
        super().__init__("goal_sender")

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("dwell_time", 1.5)
        self.declare_parameter("start_delay", 3.0)
        self.declare_parameter("loop", False)
        self.declare_parameter("route", "default")
        # experiment_timeout is measured in ROS/simulation time so it matches
        # data_collector's CSV time_s. A separate wall-clock watchdog can be
        # enabled to prevent infinite waits if Gazebo or /clock stalls.
        self.declare_parameter("experiment_timeout", 720)
        self.declare_parameter(
            "experiment_wall_timeout",
            0.0,
            ParameterDescriptor(dynamic_typing=True),
        )

        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.dwell_time = float(self.get_parameter("dwell_time").value)
        self.start_delay = float(self.get_parameter("start_delay").value)
        self.loop = bool(self.get_parameter("loop").value)
        self.experiment_timeout = float(
            self.get_parameter("experiment_timeout").value
        )
        self.experiment_wall_timeout = float(
            self.get_parameter("experiment_wall_timeout").value
        )
        if self.experiment_wall_timeout <= 0.0 and self.experiment_timeout > 0.0:
            self.experiment_wall_timeout = self.experiment_timeout

        self.goals = self._load_route(str(self.get_parameter("route").value))
        self._goal_index = 0
        self._goal_handle = None
        self._route_finished = False
        self._dwell_timer = None
        self._exit_code = 5  # default: exception

        # Action client for Nav2 NavigateToPose
        self._action_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )

        # Still publish to /goal_pose so data_collector auto_start triggers
        self._goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)

        self.get_logger().info(
            f"GoalSender ready: {len(self.goals)} goals, "
            f"action=navigate_to_pose, topic={self.goal_topic}"
        )

        # Startup: wait for start_delay then connect to action server
        self._start_wall = time.monotonic()
        self._started = False
        self._route_start_sim = None
        self._route_start_wall = None
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0, self._startup_tick, clock=self._steady_clock)

        # Check timeouts from a steady timer, but compare experiment_timeout
        # against ROS/sim time captured when the route actually starts. This
        # avoids the old mixed-clock issue where CSV time_s and timeout budget
        # used different clocks.
        self.create_timer(0.5, self._timeout_tick, clock=self._steady_clock)

    # ── Route parsing ────────────────────────────────────────

    def _load_route(self, route: str):
        if route == "default":
            return list(DEFAULT_GOALS)
        goals = []
        for item in route.split(";"):
            item = item.strip()
            if not item:
                continue
            parts = [float(x.strip()) for x in item.split(",")]
            if len(parts) != 3:
                raise ValueError("route must be 'x,y,yaw;x,y,yaw;...'")
            goals.append(tuple(parts))
        if not goals:
            raise ValueError("route is empty")
        return goals

    # ── Startup ──────────────────────────────────────────────

    def _startup_tick(self):
        if self._started or self._route_finished:
            return
        if time.monotonic() - self._start_wall < self.start_delay:
            return
        if not self._action_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().info(
                "Waiting for navigate_to_pose action server...",
                throttle_duration_sec=5.0,
            )
            return
        self._started = True
        self._route_start_sim = self.get_clock().now().nanoseconds * 1e-9
        self._route_start_wall = time.monotonic()
        self.get_logger().info("Nav2 action server ready. Starting route.")
        self._send_goal()

    def _timeout_tick(self):
        """Abort the route when sim-time budget or wall watchdog expires."""
        if self._route_finished or not self._started:
            return

        if self.experiment_timeout > 0 and self._route_start_sim is not None:
            now_sim = self.get_clock().now().nanoseconds * 1e-9
            elapsed_sim = now_sim - self._route_start_sim
            if elapsed_sim >= self.experiment_timeout:
                self.get_logger().error(
                    f"Experiment sim-time timeout "
                    f"({self.experiment_timeout}s) reached!"
                )
                self._cancel_and_timeout()
                return

        if (
            self.experiment_wall_timeout > 0
            and self._route_start_wall is not None
        ):
            elapsed_wall = time.monotonic() - self._route_start_wall
            if elapsed_wall >= self.experiment_wall_timeout:
                self.get_logger().error(
                    f"Experiment wall-clock watchdog "
                    f"({self.experiment_wall_timeout}s) reached!"
                )
                self._cancel_and_timeout()

    def _cancel_and_timeout(self):
        """Cancel any active goal and finish as timeout."""
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._finish_route(success=False, reason="timeout")

    # ── Goal lifecycle ───────────────────────────────────────

    def _make_pose(self, goal_tuple):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = goal_tuple[0]
        msg.pose.position.y = goal_tuple[1]
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(goal_tuple[2] / 2.0)
        msg.pose.orientation.w = math.cos(goal_tuple[2] / 2.0)
        return msg

    def _send_goal(self):
        """Send the goal at self._goal_index via the action client."""
        if self._goal_index >= len(self.goals):
            self._finish_route(success=True)
            return

        goal_tuple = self.goals[self._goal_index]
        pose_msg = self._make_pose(goal_tuple)

        # Publish to /goal_pose FIRST so data_collector auto_start fires
        # before the action status arrives
        self._goal_pub.publish(pose_msg)

        # Send via Nav2 action
        action_goal = NavigateToPose.Goal()
        action_goal.pose = pose_msg

        self.get_logger().info(
            f"Sent route goal {self._goal_index + 1}/{len(self.goals)}: "
            f"x={goal_tuple[0]:.2f}, y={goal_tuple[1]:.2f}, "
            f"yaw={goal_tuple[2]:.2f}"
        )

        future = self._action_client.send_goal_async(
            action_goal, feedback_callback=self._feedback_cb
        )
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        """Called when the action server accepts or rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                f"Goal {self._goal_index + 1}/{len(self.goals)} "
                f"REJECTED by Nav2"
            )
            self._finish_route(success=False, reason="rejected")
            return

        self.get_logger().info(
            f"Goal {self._goal_index + 1}/{len(self.goals)} accepted"
        )
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        """Called when the NavigateToPose action finishes."""
        result = future.result()
        status = result.status
        self._goal_handle = None
        name = _STATUS_NAMES.get(status, f"UNKNOWN({status})")

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"Reached route goal "
                f"{self._goal_index + 1}/{len(self.goals)} [{name}]"
            )
            self._goal_index += 1
            self._schedule_next()
        else:
            self.get_logger().warning(
                f"Goal {self._goal_index + 1}/{len(self.goals)} "
                f"failed [{name}]"
            )
            self._finish_route(success=False, reason=name.lower())

    def _feedback_cb(self, feedback_msg):
        """Optional: could log ETA / distance remaining."""
        pass

    # ── Dwell between goals ──────────────────────────────────

    def _schedule_next(self):
        """Wait dwell_time seconds then send the next goal."""
        if self._route_finished:
            return
        self._dwell_timer = self.create_timer(
            self.dwell_time, self._dwell_done
        )

    def _dwell_done(self):
        """One-shot: cancel the timer then send the next goal."""
        if self._dwell_timer is not None:
            self._dwell_timer.cancel()
            self.destroy_timer(self._dwell_timer)
            self._dwell_timer = None
        self._send_goal()

    # ── Route completion ─────────────────────────────────────

    def _finish_route(self, success: bool, reason: str = ""):
        if self._route_finished:
            return
        self._route_finished = True

        if success:
            self._exit_code = 0
            self.get_logger().info("Route succeeded.")
        else:
            done = self._goal_index
            total = len(self.goals)
            # Map reason → exit code
            reason_codes = {
                "aborted": 2, "rejected": 2,
                "canceled": 3,
                "timeout": 4,
            }
            self._exit_code = reason_codes.get(reason, 2)
            level = (
                self.get_logger().error
                if reason in ("aborted", "rejected", "timeout")
                else self.get_logger().warning
            )
            level(
                f"Route {reason} at goal {done + 1}/{total}. "
                f"Completed {done}/{total} goals."
            )

        # Trigger clean shutdown so the process exits with _exit_code
        try:
            rclpy.shutdown()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
    exit_code = 5  # default: exception
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._exit_code = 3  # treat Ctrl-C as canceled
    except Exception as exc:
        node.get_logger().error(f"Unexpected exception: {exc}")
        node._exit_code = 5
    finally:
        exit_code = node._exit_code
        # Cancel active Nav2 goal before shutting down
        if node._goal_handle is not None:
            node.get_logger().info("Canceling active Nav2 goal...")
            node._goal_handle.cancel_goal_async()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
