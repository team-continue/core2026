#!/usr/bin/env python3
"""起動した地図を /map（OccupancyGrid）と /map/name（String）に出す.

どちらも最後の値を残して出す（transient_local）ので、後から繋いだノードや操縦の UI（ros2_ws_bridge 経由）にも届く。
地図は map_yaml（map_server の yaml）か、image_path・resolution・origin_x・origin_y で指定する。
"""

from array import array
from pathlib import Path
import sys

from core_nav_launch.occupancy_map import (
    load_map_yaml,
    load_occupancy_map,
    MapSource,
    OccupancyMapError,
)
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

LATCHED = QoSProfile(
    depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class MapPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__('map_publisher')
        map_name = self.declare_parameter('map_name', '').value
        map_yaml = self.declare_parameter('map_yaml', '').value
        image_path = self.declare_parameter('image_path', '').value
        resolution = self.declare_parameter('resolution', 0.0).value
        origin_x = self.declare_parameter('origin_x', 0.0).value
        origin_y = self.declare_parameter('origin_y', 0.0).value
        frame_id = self.declare_parameter('frame_id', 'map').value

        if map_yaml:
            source = load_map_yaml(Path(map_yaml))
        elif image_path:
            source = MapSource(Path(image_path), float(resolution), float(origin_x), float(origin_y))
        else:
            raise OccupancyMapError('map_yaml か image_path を指定してください')
        occupancy = load_occupancy_map(source)
        name = map_name or source.image.stem

        grid = OccupancyGrid()
        grid.header.frame_id = frame_id
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = float(occupancy.resolution)
        height, width = occupancy.cells.shape
        grid.info.width = int(width)
        grid.info.height = int(height)
        grid.info.origin.position.x = float(occupancy.origin_x)
        grid.info.origin.position.y = float(occupancy.origin_y)
        grid.info.origin.orientation.w = 1.0
        grid.data = array('b', occupancy.cells.reshape(-1).tobytes())

        self._map_pub = self.create_publisher(OccupancyGrid, '/map', LATCHED)
        self._name_pub = self.create_publisher(String, '/map/name', LATCHED)
        self._map_pub.publish(grid)
        self._name_pub.publish(String(data=name))
        self.get_logger().info(
            f'地図 {name} を出しました（{width} x {height}、{occupancy.resolution} m、'
            f'原点 {occupancy.origin_x:.3f}, {occupancy.origin_y:.3f}、画像 {source.image}）')


def main() -> int:
    rclpy.init()
    try:
        node = MapPublisherNode()
    except OccupancyMapError as error:
        print(f'[map_publisher] {error}', file=sys.stderr, flush=True)
        rclpy.try_shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
