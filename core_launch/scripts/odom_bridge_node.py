#!/usr/bin/env python3
"""Bridge /sim_odom to /odom, /start_pose, and TF odom->base_link."""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


def _quat_yaw(q):
    """Extract yaw from quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _yaw_to_quat(yaw):
    """Create a quaternion from yaw (z-axis rotation only)."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class OdomBridgeNode(Node):
    def __init__(self):
        super().__init__('odom_bridge_node')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.init_x = self.get_parameter('init_x').value
        self.init_y = self.get_parameter('init_y').value

        # Offset computed from first sim_odom message
        self._offset_x = None
        self._offset_y = None

        self.sub = self.create_subscription(
            Odometry, '/sim_odom', self.on_sim_odom, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.start_pub = self.create_publisher(PoseStamped, '/start_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f'Odom bridge started (init_pose=({self.init_x}, {self.init_y}) in odom frame)')

    def on_sim_odom(self, msg: Odometry):
        stamp = self.get_clock().now().to_msg()

        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y
        rz = msg.pose.pose.position.z
        rq = msg.pose.pose.orientation

        # Unity ground plane → ROS: X=forward, Y=left
        # Unity: sim_x=right, sim_y=forward → ROS: ros_x=forward, ros_y=left
        raw_x = -ry
        raw_y = rx
        yaw = _quat_yaw(rq) + math.pi / 2.0
        oq = _yaw_to_quat(yaw)

        # Calibrate offset on first message so that initial ROS pose = (init_x, init_y)
        if self._offset_x is None:
            self._offset_x = self.init_x - raw_x
            self._offset_y = self.init_y - raw_y
            self.get_logger().info(
                f'First sim_odom: ({rx:.2f}, {ry:.2f}, {rz:.2f}), '
                f'yaw: {math.degrees(_quat_yaw(rq)):.1f}deg → '
                f'offset=({self._offset_x:.2f}, {self._offset_y:.2f})')

        ox = raw_x + self._offset_x
        oy = raw_y + self._offset_y

        # Re-publish as /odom
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = ox
        odom.pose.pose.position.y = oy
        odom.pose.pose.position.z = rz
        odom.pose.pose.orientation = oq
        # Rotate velocity the same way as position
        odom.twist.twist.linear.x = -msg.twist.twist.linear.y
        odom.twist.twist.linear.y = msg.twist.twist.linear.x
        odom.twist.twist.linear.z = msg.twist.twist.linear.z
        odom.twist.twist.angular = msg.twist.twist.angular
        self.odom_pub.publish(odom)

        # Publish /start_pose for path_planner (2D, z=0)
        start = PoseStamped()
        start.header.stamp = stamp
        start.header.frame_id = self.odom_frame
        start.pose.position.x = ox
        start.pose.position.y = oy
        start.pose.position.z = 0.0
        start.pose.orientation = oq
        self.start_pub.publish(start)

        # Broadcast TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = ox
        t.transform.translation.y = oy
        t.transform.translation.z = rz
        t.transform.rotation = oq
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
