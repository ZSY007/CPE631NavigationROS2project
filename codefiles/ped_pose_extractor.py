#!/usr/bin/env python3
"""
ped_pose_extractor.py
─────────────────────
Extract pedestrian Actor poses from Gazebo /world/cafe/dynamic_pose/info,
filter out robot and static objects, publish as PoseArray for social_nav_node consumption.

Topic In:  /gz/model_poses  (Pose_V → PoseArray converted by gz-ros bridge)
Topic Out: /pedestrian_poses (geometry_msgs/PoseArray)

Note: Pose_V loses name information after bridge conversion, using two filtering strategies:
  1. Explicit index list (ped_indices parameter): most stable, fixed after debugging.
  2. Range slice (start_index + pedestrian_count): backward compatible fallback.
Priority: use explicit index list; fallback to range slice if ped_indices is empty.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray


class PedPoseExtractor(Node):

    def __init__(self):
        super().__init__("ped_pose_extractor")

        # ── Parameter declaration ──────────────────────────────
        self.declare_parameter("input_topic", "/gz/model_poses")
        self.declare_parameter("output_topic", "/pedestrian_poses")
        # Explicit index list (priority): comma-separated, e.g. "1,2,3,4"
        # Leave empty "" to fallback to start_index + pedestrian_count range slice
        self.declare_parameter("ped_indices", "1,2,3,4,5")
        # Fallback range parameters
        self.declare_parameter("start_index", 1)
        self.declare_parameter("pedestrian_count", 5)

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        ped_indices_str = str(self.get_parameter("ped_indices").value).strip()
        self.start_index = int(self.get_parameter("start_index").value)
        self.pedestrian_count = int(self.get_parameter("pedestrian_count").value)

        # Parse explicit index list
        if ped_indices_str:
            try:
                self.ped_indices = [int(x) for x in ped_indices_str.split(",") if x.strip()]
            except ValueError:
                self.get_logger().error(
                    f"ped_indices format error: '{ped_indices_str}', fallback to range slice"
                )
                self.ped_indices = []
        else:
            self.ped_indices = []

        self.use_explicit_indices = bool(self.ped_indices)

        # ── Subscriber / Publisher ────────────────────────────────
        self.sub = self.create_subscription(
            PoseArray,
            in_topic,
            self.gz_pose_callback,
            10,
        )
        self.pub = self.create_publisher(PoseArray, out_topic, 10)

        self._first_frame_checked = False

        if self.use_explicit_indices:
            self.get_logger().info(
                f"PedPoseExtractor: {in_topic} → {out_topic} "
                f"[explicit indices: {self.ped_indices}]"
            )
        else:
            end = self.start_index + self.pedestrian_count - 1
            self.get_logger().info(
                f"PedPoseExtractor: {in_topic} → {out_topic} "
                f"[range slice: [{self.start_index}, {end}]]"
            )

    def gz_pose_callback(self, msg: PoseArray):
        total = len(msg.poses)

        # First frame checks index bounds
        if not self._first_frame_checked:
            self._first_frame_checked = True
            self.get_logger().info(
                f"First frame received {total} model poses (index 0 ~ {total - 1})"
            )
            if self.use_explicit_indices:
                bad = [i for i in self.ped_indices if i >= total or i < 0]
                if bad:
                    self.get_logger().error(
                        f"ped_indices have out-of-bounds indices {bad} (total {total} models). "
                        "Check ped_indices parameter or adjust start_index/pedestrian_count."
                    )
            else:
                end_index = self.start_index + self.pedestrian_count
                if end_index > total:
                    self.get_logger().error(
                        f"Slice range [{self.start_index}:{end_index}] exceeds PoseArray length {total}. "
                        "Adjust start_index or pedestrian_count parameters."
                    )

        # ── Extract pedestrian poses ─────────────────────────────────────────
        if self.use_explicit_indices:
            ped_poses = [
                msg.poses[i]
                for i in self.ped_indices
                if 0 <= i < total
            ]
        else:
            end_index = self.start_index + self.pedestrian_count
            ped_poses = msg.poses[self.start_index:end_index]

        if not ped_poses:
            return

        out = PoseArray()
        out.header = msg.header
        out.header.frame_id = "map"
        out.poses = ped_poses
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PedPoseExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
