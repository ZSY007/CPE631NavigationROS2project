#!/usr/bin/env python3
"""
data_collector.py
─────────────────
Collect navigation performance metrics for baseline / social comparison experiments.

Metrics:
  1. path_length_m - total path distance traveled
  2. time_s - navigation time elapsed
  3. encounters - number of times within encounter_threshold (default 1.0 m)
  4. personal_violations - times in personal space (< 1.2 m, Proxemics)
  5. intimate_violations - times in intimate space (< 0.45 m, Proxemics)
  6. min_ped_dist_m - global minimum distance to any pedestrian
  7. mean_angular_acceleration - mean |Δω/Δt| (rad/s²), lower = smoother path
"""

import csv
import math
import os
import signal
import threading
from datetime import datetime

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import (
    PoseArray,
    PoseStamped,
    PoseWithCovarianceStamped,
    TwistStamped,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class DataCollector(Node):
    def __init__(self):
        super().__init__("data_collector")

        # ── Parameters ─────────────────────────────────────────────────
        self.declare_parameter("mode", "social")
        self.declare_parameter("csv_file", "~/ros2_ws/experiment_results.csv")
        self.declare_parameter("encounter_threshold", 1.0)
        self.declare_parameter("personal_zone", 1.2)
        self.declare_parameter("intimate_zone", 0.45)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("robot_pose_topic", "/amcl_pose")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("pedestrian_topic", "/pedestrian_poses")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("status_topic", "/navigate_to_pose/_action/status")
        self.declare_parameter("distance_check_period", 0.1)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("auto_stop", False)  # default off; shell controls stop
        self.declare_parameter("timestamp_filename", True)
        self.declare_parameter("experiment_id", 1)
        self.declare_parameter("attempt", 0)
        self.declare_parameter("exit_after_result", False)
        self.declare_parameter("result_file", "")  # shell writes result here for SIGUSR1

        self.mode = str(self.get_parameter("mode").value)
        csv_base = os.path.expanduser(str(self.get_parameter("csv_file").value))
        self.encounter_threshold = float(self.get_parameter("encounter_threshold").value)
        self.personal_zone = float(self.get_parameter("personal_zone").value)
        self.intimate_zone = float(self.get_parameter("intimate_zone").value)
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.auto_stop = bool(self.get_parameter("auto_stop").value)
        self.experiment_id = int(self.get_parameter("experiment_id").value)
        self.attempt = int(self.get_parameter("attempt").value)
        self.exit_after_result = bool(
            self.get_parameter("exit_after_result").value
        )
        self.result_file = str(
            self.get_parameter("result_file").value
        )
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_base_frame = str(self.get_parameter("robot_base_frame").value)

        if bool(self.get_parameter("timestamp_filename").value):
            base, ext = os.path.splitext(csv_base)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.csv_file = f"{base}_{ts}{ext}"
        else:
            self.csv_file = csv_base

        # ── State variables ─────────────────────────────────────────────
        self.robot_pos = None
        self.robot_pos_map = None
        self.ped_poses = []
        self.path_length = 0.0
        self.prev_pos = None
        self.start_time = None
        self.encounter_count = 0
        self.personal_violations = 0
        self.intimate_violations = 0
        self.min_ped_dist = float("inf")
        self.close_ped_indices = set()
        self.personal_ped_indices = set()
        self.intimate_ped_indices = set()
        self.prev_angular_z = 0.0
        self.prev_cmd_time = None
        self.total_angular_accel = 0.0
        self.angular_accel_samples = 0
        self.is_navigating = False
        self.result_saved = False
        self.last_terminal_status = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Subscribe ─────────────────────────────────────────────────
        self.create_subscription(Odometry,
            str(self.get_parameter("odom_topic").value), self.odom_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped,
            str(self.get_parameter("robot_pose_topic").value), self.amcl_cb, 10)
        self.create_subscription(PoseArray,
            str(self.get_parameter("pedestrian_topic").value), self.ped_cb, 10)
        self.create_subscription(PoseStamped,
            str(self.get_parameter("goal_topic").value), self.goal_cb, 10)
        self.create_subscription(GoalStatusArray,
            str(self.get_parameter("status_topic").value), self.status_cb, 10)
        self.create_subscription(TwistStamped,
            str(self.get_parameter("cmd_vel_topic").value), self.cmd_vel_cb, 10)

        check_period = float(self.get_parameter("distance_check_period").value)
        self.create_timer(max(check_period, 0.02), self.record_loop)
        self.init_csv()

        self.get_logger().info(
            f"DataCollector ready  mode={self.mode}  "
            f"encounter={self.encounter_threshold:.2f}m  "
            f"personal={self.personal_zone:.2f}m  "
            f"intimate={self.intimate_zone:.2f}m"
        )
        self.get_logger().info(f"Results CSV: {self.csv_file}")

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def init_csv(self):
        csv_dir = os.path.dirname(self.csv_file)
        if csv_dir:
            os.makedirs(csv_dir, exist_ok=True)
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, "w", newline="") as f:
                csv.writer(f).writerow([
                    "experiment", "mode", "result", "attempt",
                    "path_length_m", "time_s",
                    "encounters", "personal_violations", "intimate_violations",
                    "min_ped_dist_m", "mean_angular_acceleration",
                ])

    # ── Callbacks ─────────────────────────────────────────────

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.robot_pos = (x, y)
        if self.is_navigating and self.prev_pos is not None:
            d = math.hypot(x - self.prev_pos[0], y - self.prev_pos[1])
            if d < 0.5:
                self.path_length += d
        self.prev_pos = (x, y)

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.robot_pos_map = (x, y)

    def robot_pose_in_map(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                Time(),
            )
        except TransformException:
            return self.robot_pos_map

        translation = transform.transform.translation
        return (translation.x, translation.y)

    def ped_cb(self, msg: PoseArray):
        self.ped_poses = [(p.position.x, p.position.y) for p in msg.poses]

    def goal_cb(self, _msg: PoseStamped):
        if self.auto_start and not self.is_navigating:
            self.start_experiment(self.mode)

    def status_cb(self, msg: GoalStatusArray):
        if not self.auto_stop or not self.is_navigating or not msg.status_list:
            return
        status = msg.status_list[-1].status
        if status in (GoalStatus.STATUS_SUCCEEDED,
                      GoalStatus.STATUS_ABORTED,
                      GoalStatus.STATUS_CANCELED):
            if status != self.last_terminal_status:
                result = {
                    GoalStatus.STATUS_SUCCEEDED: "succeeded",
                    GoalStatus.STATUS_ABORTED: "aborted",
                    GoalStatus.STATUS_CANCELED: "canceled",
                }[status]
                self.stop_experiment(result=result)
                self.last_terminal_status = status
        else:
            self.last_terminal_status = None

    def cmd_vel_cb(self, msg: TwistStamped):
        """Track angular velocity change rate (angular acceleration |Δω/Δt|), quantify path smoothness."""
        if not self.is_navigating:
            return
        stamp = msg.header.stamp
        now = stamp.sec + stamp.nanosec * 1e-9
        if now <= 0.0:
            now = self.now_sec()
        omega = msg.twist.angular.z
        if self.prev_cmd_time is not None:
            dt = now - self.prev_cmd_time
            if 1e-6 < dt < 1.0:
                self.total_angular_accel += abs(omega - self.prev_angular_z) / dt
                self.angular_accel_samples += 1
        self.prev_angular_z = omega
        self.prev_cmd_time = now

    # ── Periodic pedestrian distance check ──────────────────────────────────

    def record_loop(self):
        robot_pos_map = self.robot_pose_in_map()
        if not self.is_navigating or robot_pos_map is None:
            return

        cur_close, cur_personal, cur_intimate = set(), set(), set()
        robot_x, robot_y = robot_pos_map
        for i, (px, py) in enumerate(self.ped_poses):
            dist = math.hypot(robot_x - px, robot_y - py)
            self.min_ped_dist = min(self.min_ped_dist, dist)
            if dist < self.encounter_threshold:
                cur_close.add(i)
            if dist < self.personal_zone:
                cur_personal.add(i)
            if dist < self.intimate_zone:
                cur_intimate.add(i)

        new_enc = cur_close - self.close_ped_indices
        new_per = cur_personal - self.personal_ped_indices
        new_int = cur_intimate - self.intimate_ped_indices

        if new_enc:
            self.encounter_count += len(new_enc)
            self.get_logger().info(
                f"Encounter (<{self.encounter_threshold:.1f}m) +{len(new_enc)}, total={self.encounter_count}")
        if new_per:
            self.personal_violations += len(new_per)
            self.get_logger().warning(
                f"Personal space violation (<{self.personal_zone:.2f}m) +{len(new_per)}, total={self.personal_violations}")
        if new_int:
            self.intimate_violations += len(new_int)
            self.get_logger().warning(
                f"Intimate space violation (<{self.intimate_zone:.2f}m) +{len(new_int)}, total={self.intimate_violations}")

        self.close_ped_indices = cur_close
        self.personal_ped_indices = cur_personal
        self.intimate_ped_indices = cur_intimate

    # ── Experiment control ─────────────────────────────────────────────

    def start_experiment(self, mode=None):
        # Reset every per-run metric here. The node may live across retries or
        # manual runs, so safety metrics must never carry over from a prior run.
        self.path_length = 0.0
        self.encounter_count = 0
        self.personal_violations = 0
        self.intimate_violations = 0
        self.min_ped_dist = float("inf")
        self.close_ped_indices = set()
        self.personal_ped_indices = set()
        self.intimate_ped_indices = set()
        self.prev_angular_z = 0.0
        self.prev_cmd_time = None
        self.total_angular_accel = 0.0
        self.angular_accel_samples = 0
        self.start_time = self.now_sec()
        self.is_navigating = True
        self.result_saved = False
        self.mode = mode or self.mode
        self.prev_pos = self.robot_pos
        self.last_terminal_status = None
        self.get_logger().info(f"Started experiment [{self.mode}]")

    def stop_experiment(self, result="manual"):
        if not self.is_navigating:
            return
        if self.result_saved:
            return
        self.result_saved = True
        elapsed = self.now_sec() - self.start_time
        self.is_navigating = False
        min_dist = None if math.isinf(self.min_ped_dist) else self.min_ped_dist
        mean_accel = (
            round(self.total_angular_accel / self.angular_accel_samples, 4)
            if self.angular_accel_samples > 0
            else None
        )

        self.get_logger().info("=" * 55)
        self.get_logger().info(f"Experiment #{self.experiment_id} result: {result}")
        self.get_logger().info(f"  Mode:                    {self.mode}")
        self.get_logger().info(f"  Attempt:                 {self.attempt}")
        self.get_logger().info(f"  Path length:             {self.path_length:.2f} m")
        self.get_logger().info(f"  Navigation time:         {elapsed:.2f} s")
        self.get_logger().info(f"  Encounters (<{self.encounter_threshold:.1f}m):      {self.encounter_count}")
        self.get_logger().info(f"  Personal violations (<{self.personal_zone:.2f}m): {self.personal_violations}")
        self.get_logger().info(f"  Intimate violations (<{self.intimate_zone:.2f}m): {self.intimate_violations}")
        self.get_logger().info(f"  Min pedestrian dist:     {f'{min_dist:.3f} m' if min_dist is not None else 'N/A'}")
        self.get_logger().info(f"  Mean angular accel:      {f'{mean_accel:.4f} rad/s²' if mean_accel is not None else 'N/A (no cmd_vel)'}")
        self.get_logger().info("=" * 55)

        with open(self.csv_file, "a", newline="") as f:
            csv.writer(f).writerow([
                self.experiment_id, self.mode, result, self.attempt,
                round(self.path_length, 3), round(elapsed, 2),
                self.encounter_count, self.personal_violations, self.intimate_violations,
                "" if min_dist is None else round(min_dist, 3),
                "" if mean_accel is None else mean_accel,
            ])
        self.get_logger().info(f"Saved to {self.csv_file}")

        if self.exit_after_result:
            self.get_logger().info("exit_after_result=True, shutting down.")
            try:
                rclpy.shutdown()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()

    # SIGUSR1: script can send this signal to make data_collector save data immediately
    # Shell writes result to result_file, SIGUSR1 handler reads it
    def _sigusr1_handler(_signum, _frame):
        result = "script_stop"
        if node.result_file:
            try:
                with open(node.result_file, "r") as f:
                    result = f.read().strip() or result
                node.get_logger().info(f"SIGUSR1: read result_file → '{result}'")
            except Exception as e:
                node.get_logger().warning(f"SIGUSR1: failed to read result_file ({e}), use '{result}'")
        else:
            node.get_logger().info("SIGUSR1: no result_file, use 'script_stop'")
        node.stop_experiment(result=result)
        if not node.is_navigating and node.exit_after_result:
            node.get_logger().info(
                "SIGUSR1: experiment was not active, shutting down."
            )
            try:
                rclpy.shutdown()
            except Exception:
                pass

    signal.signal(signal.SIGUSR1, _sigusr1_handler)

    def input_loop():
        while rclpy.ok():
            print("\nCommands: [s] start  [e] end  [1] social  [2] baseline  [3] dynamic  [q] quit")
            try:
                cmd = input("cmd> ").strip().lower()
            except EOFError:
                return
            if cmd == "s":
                node.start_experiment()
            elif cmd == "1":
                node.start_experiment("social")
            elif cmd == "2":
                node.start_experiment("baseline")
            elif cmd == "3":
                node.start_experiment("dynamic")
            elif cmd == "e":
                node.stop_experiment()
            elif cmd == "q":
                rclpy.shutdown()
                return

    threading.Thread(target=input_loop, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # When process exits, if experiment is still running, force-save collected data
        if node.is_navigating and not node.result_saved:
            node.get_logger().info("Process exiting, force-saving experiment data...")
            node.stop_experiment(result="interrupted")
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
