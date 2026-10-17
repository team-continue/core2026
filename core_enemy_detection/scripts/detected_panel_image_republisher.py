#!/usr/bin/env python3
"""Republish the image embedded in DetectedPanelInfoArray without conversion."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from core_msgs.msg import DetectedPanelInfoArray
from sensor_msgs.msg import Image


class DetectedPanelImageRepublisher(Node):
    def __init__(self):
        super().__init__('detected_panel_image_republisher')
        # Reliable output matches target_detector's default subscription QoS.
        self.publisher = self.create_publisher(Image, 'raw_image', 10)
        # Best-effort input accepts reliable and best-effort bag publishers.
        self.subscription = self.create_subscription(
            DetectedPanelInfoArray, '/oakd/detected_panel_info',
            self.on_panel, qos_profile_sensor_data)

    def on_panel(self, message):
        # A frame is useful even when the original detector found zero panels.
        frame = message.image
        if not frame.data or frame.width == 0 or frame.height == 0:
            self.get_logger().warning('Skipping an empty embedded image.')
            return
        # Preserve image timestamp, frame_id, encoding, step and pixels.
        self.publisher.publish(frame)


def main():
    rclpy.init()
    node = DetectedPanelImageRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
