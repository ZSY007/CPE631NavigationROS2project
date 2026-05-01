#!/usr/bin/env python3
"""
ped_pose_extractor.py
─────────────────────
从 Gazebo 的 /world/cafe/dynamic_pose/info 中提取行人 Actor 位姿，
过滤掉机器人和静态物体，发布为 PoseArray 供 social_nav_node 消费。

Topic In:  /gz/model_poses  (由 gz-ros bridge 桥接 Pose_V → PoseArray)
Topic Out: /pedestrian_poses (geometry_msgs/PoseArray)

注：Pose_V 经 bridge 转换后名字信息丢失，采用两种过滤策略：
  1. 显式索引列表 (ped_indices 参数): 最稳定，调试后固定。
  2. 范围切片 (start_index + pedestrian_count): 向后兼容备用。
优先使用显式索引列表；如果 ped_indices 为空则回退到范围切片。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray


class PedPoseExtractor(Node):

    def __init__(self):
        super().__init__("ped_pose_extractor")

        # ── 参数声明 ──────────────────────────────────────────────
        self.declare_parameter("input_topic", "/gz/model_poses")
        self.declare_parameter("output_topic", "/pedestrian_poses")
        # 显式索引列表（优先）: 逗号分隔，例如 "1,2,3,4"
        # 留空 "" 则回退到 start_index + pedestrian_count 范围切片
        self.declare_parameter("ped_indices", "1,2,3,4,5")
        # 备用范围参数
        self.declare_parameter("start_index", 1)
        self.declare_parameter("pedestrian_count", 5)

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        ped_indices_str = str(self.get_parameter("ped_indices").value).strip()
        self.start_index = int(self.get_parameter("start_index").value)
        self.pedestrian_count = int(self.get_parameter("pedestrian_count").value)

        # 解析显式索引列表
        if ped_indices_str:
            try:
                self.ped_indices = [int(x) for x in ped_indices_str.split(",") if x.strip()]
            except ValueError:
                self.get_logger().error(
                    f"ped_indices 格式错误: '{ped_indices_str}'，回退到范围切片"
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
                f"[显式索引: {self.ped_indices}]"
            )
        else:
            end = self.start_index + self.pedestrian_count - 1
            self.get_logger().info(
                f"PedPoseExtractor: {in_topic} → {out_topic} "
                f"[范围切片: [{self.start_index}, {end}]]"
            )

    def gz_pose_callback(self, msg: PoseArray):
        total = len(msg.poses)

        # 首帧做索引越界检查
        if not self._first_frame_checked:
            self._first_frame_checked = True
            self.get_logger().info(
                f"首帧接收到 {total} 个模型位姿 (index 0 ~ {total - 1})"
            )
            if self.use_explicit_indices:
                bad = [i for i in self.ped_indices if i >= total or i < 0]
                if bad:
                    self.get_logger().error(
                        f"ped_indices 中有越界索引 {bad}（总共 {total} 个模型）。"
                        "请检查参数 ped_indices 或调整 start_index/pedestrian_count。"
                    )
            else:
                end_index = self.start_index + self.pedestrian_count
                if end_index > total:
                    self.get_logger().error(
                        f"切片范围 [{self.start_index}:{end_index}] 超出 PoseArray 长度 {total}。"
                        "请调整 start_index 或 pedestrian_count 参数。"
                    )

        # ── 提取行人位姿 ─────────────────────────────────────────
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
