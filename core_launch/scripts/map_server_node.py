#!/usr/bin/env python3
"""Publish global_map.png as OccupancyGrid on /map and /costmap/global."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from PIL import Image


class MapServerNode(Node):
    def __init__(self):
        super().__init__('map_server_node')

        self.declare_parameter('image_path', 'global_map.png')
        self.declare_parameter('resolution', 0.05)  # 5cm/px
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('occupied_thresh', 0.65)
        self.declare_parameter('free_thresh', 0.25)

        image_path = self.get_parameter('image_path').value
        resolution = self.get_parameter('resolution').value
        origin_x = self.get_parameter('origin_x').value
        origin_y = self.get_parameter('origin_y').value
        map_frame = self.get_parameter('map_frame').value
        odom_frame = self.get_parameter('odom_frame').value
        occupied_thresh = self.get_parameter('occupied_thresh').value
        free_thresh = self.get_parameter('free_thresh').value

        # Load image
        self.get_logger().info(f'Loading map from: {image_path}')
        img = Image.open(image_path).convert('L')
        img_array = np.array(img)

        # Flip vertically (image top-left → OccupancyGrid bottom-left)
        img_array = np.flipud(img_array)

        # Convert grayscale to occupancy using thresholds
        # Normalize pixel values to [0, 1] where 0=black, 1=white
        pmin, pmax = int(img_array.min()), int(img_array.max())
        self.get_logger().info(f'Pixel value range: {pmin}-{pmax}')
        normalized = img_array.astype(np.float32) / 255.0
        # bright (high value) = free, dark (low value) = occupied
        # Invert: 1.0=white→free, 0.0=black→occupied
        occupancy = np.full_like(img_array, -1, dtype=np.int8)  # unknown
        occupancy[normalized > (1.0 - free_thresh)] = 0          # free
        occupancy[normalized < (1.0 - occupied_thresh)] = 100    # occupied
        num_free = int(np.sum(occupancy == 0))
        num_occ = int(np.sum(occupancy == 100))
        num_unk = int(np.sum(occupancy == -1))
        self.get_logger().info(
            f'Occupancy: free={num_free}, occupied={num_occ}, unknown={num_unk}')

        height, width = occupancy.shape

        # Build OccupancyGrid for /map (frame: map)
        self.map_msg = self._build_grid(
            occupancy, width, height, resolution, origin_x, origin_y, map_frame)

        # Build OccupancyGrid for /costmap/global (frame: odom)
        # Uses same data since map→odom is identity
        self.global_costmap_msg = self._build_grid(
            occupancy, width, height, resolution, origin_x, origin_y, odom_frame)

        # Transient local QoS for /map (path_planner expects this)
        transient_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', transient_qos)

        # /costmap/global for MPPI
        self.global_costmap_pub = self.create_publisher(
            OccupancyGrid, '/costmap/global', transient_qos)

        # Publish once (transient_local QoS ensures late subscribers get it)
        self._publish()

        self.get_logger().info(
            f'Map published: {width}x{height}, resolution={resolution:.5f} m/px')

    def _build_grid(self, occupancy, width, height, resolution, ox, oy, frame_id):
        msg = OccupancyGrid()
        msg.header.frame_id = frame_id
        msg.info.resolution = float(resolution)
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = ox
        msg.info.origin.position.y = oy
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = occupancy.flatten().tolist()
        return msg

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        self.map_msg.header.stamp = stamp
        self.global_costmap_msg.header.stamp = stamp
        self.map_pub.publish(self.map_msg)
        self.global_costmap_pub.publish(self.global_costmap_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
