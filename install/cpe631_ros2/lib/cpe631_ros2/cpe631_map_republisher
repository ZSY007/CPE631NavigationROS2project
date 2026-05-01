#!/usr/bin/env python3

import copy

import rclpy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


class MapRepublisher(Node):
    def __init__(self):
        super().__init__('map_republisher')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        self.set_parameters(
            [Parameter('use_sim_time', Parameter.Type.BOOL, self.get_parameter('use_sim_time').value)]
        )

        self.declare_parameter('input_map_topic', '/map')
        self.declare_parameter('output_map_topic', '/map_viz')
        self.declare_parameter('input_metadata_topic', '/map_metadata')
        self.declare_parameter('output_metadata_topic', '/map_metadata_viz')
        self.declare_parameter('republish_hz', 1.0)

        input_map_topic = str(self.get_parameter('input_map_topic').value)
        output_map_topic = str(self.get_parameter('output_map_topic').value)
        input_metadata_topic = str(self.get_parameter('input_metadata_topic').value)
        output_metadata_topic = str(self.get_parameter('output_metadata_topic').value)

        transient_local_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        volatile_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._last_map: OccupancyGrid | None = None
        self._last_metadata: MapMetaData | None = None

        self.map_pub = self.create_publisher(OccupancyGrid, output_map_topic, volatile_qos)
        self.metadata_pub = self.create_publisher(MapMetaData, output_metadata_topic, volatile_qos)

        self.create_subscription(OccupancyGrid, input_map_topic, self._on_map, transient_local_qos)
        self.create_subscription(MapMetaData, input_metadata_topic, self._on_metadata, transient_local_qos)

        republish_hz = float(self.get_parameter('republish_hz').value)
        republish_hz = republish_hz if republish_hz > 0.0 else 1.0
        self.timer = self.create_timer(1.0 / republish_hz, self._republish)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._last_map = msg

    def _on_metadata(self, msg: MapMetaData) -> None:
        self._last_metadata = msg

    def _republish(self) -> None:
        now = self.get_clock().now().to_msg()

        if self._last_map is not None:
            out = copy.deepcopy(self._last_map)
            out.header.stamp = now
            self.map_pub.publish(out)

        if self._last_metadata is not None:
            out = copy.deepcopy(self._last_metadata)
            out.map_load_time = now
            self.metadata_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = MapRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

