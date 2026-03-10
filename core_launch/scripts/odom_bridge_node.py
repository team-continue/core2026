#!/usr/bin/env python3
"""Bridge /sim_odom or /Odometry to /odom, /start_pose, and TF odom->base_link."""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PoseStamped, TransformStamped, Quaternion, Twist,
)
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


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
        self.declare_parameter('init_yaw', 0.0)
        self.declare_parameter('odom_source', 'sim')

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.init_x = self.get_parameter('init_x').value
        self.init_y = self.get_parameter('init_y').value
        self.init_yaw = self.get_parameter('init_yaw').value
        self.odom_source = self.get_parameter('odom_source').value

        if self.odom_source not in ('sim', 'fastlio'):
            self.get_logger().error(
                f'Invalid odom_source: "{self.odom_source}". '
                f'Must be "sim" or "fastlio". Defaulting to "sim".')
            self.odom_source = 'sim'

        # Offset computed from first odom message
        self._offset_x = None
        self._offset_y = None

        # FAST-LIO specific: first raw position and yaw offset
        self._first_raw_x = None
        self._first_raw_y = None
        self._offset_yaw = None

        # Publishers (common)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.start_pub = self.create_publisher(PoseStamped, '/start_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create subscription based on odom source
        if self.odom_source == 'fastlio':
            self.static_tf_broadcaster = StaticTransformBroadcaster(self)
            self.sub = self.create_subscription(
                Odometry, '/Odometry', self.on_fastlio_odom, 10)
            self.get_logger().info(
                f'Odom bridge started in FAST-LIO mode '
                f'(init_pose=({self.init_x}, {self.init_y}), '
                f'init_yaw={math.degrees(self.init_yaw):.1f}deg in odom frame)')
        else:
            self.sub = self.create_subscription(
                Odometry, '/sim_odom', self.on_sim_odom, 10)
            self.get_logger().info(
                f'Odom bridge started in sim mode '
                f'(init_pose=({self.init_x}, {self.init_y}) in odom frame)')

    # ---- Sim mode callback (unchanged logic) ----
    def on_sim_odom(self, msg: Odometry):
        stamp = self.get_clock().now().to_msg()

        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y
        rz = msg.pose.pose.position.z
        rq = msg.pose.pose.orientation

        # Unity ground plane -> ROS: X=forward, Y=left
        raw_x = -ry
        raw_y = rx
        yaw = _quat_yaw(rq) + math.pi / 2.0
        oq = _yaw_to_quat(yaw)

        # Calibrate offset on first message
        if self._offset_x is None:
            self._offset_x = self.init_x - raw_x
            self._offset_y = self.init_y - raw_y
            self.get_logger().info(
                f'First sim_odom: ({rx:.2f}, {ry:.2f}, {rz:.2f}), '
                f'yaw: {math.degrees(_quat_yaw(rq)):.1f}deg -> '
                f'offset=({self._offset_x:.2f}, {self._offset_y:.2f})')

        ox = raw_x + self._offset_x
        oy = raw_y + self._offset_y

        # Rotate velocity from Unity to ROS frame
        rotated_twist = Twist()
        rotated_twist.linear.x = -msg.twist.twist.linear.y
        rotated_twist.linear.y = msg.twist.twist.linear.x
        rotated_twist.linear.z = msg.twist.twist.linear.z
        rotated_twist.angular = msg.twist.twist.angular

        self._publish_all(stamp, ox, oy, rz, oq, rotated_twist)

    # ---- FAST-LIO mode callback ----
    def on_fastlio_odom(self, msg: Odometry):
        stamp = self.get_clock().now().to_msg()

        # FAST-LIO publishes in camera_init frame (X=fwd, Y=right, Z=down)
        ci_x = msg.pose.pose.position.x
        ci_y = msg.pose.pose.position.y
        ci_q = msg.pose.pose.orientation

        # camera_init -> odom 2D transform:
        #   x_odom =  x_ci    (X=forward in both)
        #   y_odom = -y_ci    (Y flipped: right -> left)
        #   yaw_odom = -yaw_ci (Z flipped: down -> up reverses rotation)
        raw_x = ci_x
        raw_y = -ci_y
        raw_yaw = -_quat_yaw(ci_q)

        # Calibrate on first message
        if self._first_raw_x is None:
            self._first_raw_x = raw_x
            self._first_raw_y = raw_y
            self._offset_yaw = self.init_yaw - raw_yaw
            self.get_logger().info(
                f'First FAST-LIO odom: ci=({ci_x:.2f}, {ci_y:.2f}), '
                f'ci_yaw={math.degrees(_quat_yaw(ci_q)):.1f}deg -> '
                f'odom offset=({self.init_x:.2f}, {self.init_y:.2f}), '
                f'yaw_offset={math.degrees(self._offset_yaw):.1f}deg')
            self._publish_static_tf_camera_init(stamp)

        # Displacement from first position in axis-flipped frame
        dx = raw_x - self._first_raw_x
        dy = raw_y - self._first_raw_y

        # Rotate displacement by yaw offset (handles init_yaw != 0)
        cos_a = math.cos(self._offset_yaw)
        sin_a = math.sin(self._offset_yaw)
        ox = self.init_x + cos_a * dx - sin_a * dy
        oy = self.init_y + sin_a * dx + cos_a * dy
        yaw = raw_yaw + self._offset_yaw
        oq = _yaw_to_quat(yaw)

        # FAST-LIO does not publish twist
        self._publish_all(stamp, ox, oy, 0.0, oq, Twist())

    # ---- Static TF: odom -> camera_init (for RViz point cloud display) ----
    def _publish_static_tf_camera_init(self, stamp):
        """Publish odom->camera_init so FAST-LIO point clouds are visible in RViz."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = 'camera_init'
        t.transform.translation.x = self.init_x
        t.transform.translation.y = self.init_y
        t.transform.translation.z = 0.6  # livox height above base_link
        # Rotation: rot_z(init_yaw) * rot_x(pi)
        # = quat(w=0, x=cos(yaw/2), y=sin(yaw/2), z=0)
        half_yaw = self.init_yaw / 2.0
        t.transform.rotation.w = 0.0
        t.transform.rotation.x = math.cos(half_yaw)
        t.transform.rotation.y = math.sin(half_yaw)
        t.transform.rotation.z = 0.0
        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info('Published static TF: odom -> camera_init')

    # ---- Common publish logic ----
    def _publish_all(self, stamp, ox, oy, oz, oq, twist):
        # Publish /odom
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = ox
        odom.pose.pose.position.y = oy
        odom.pose.pose.position.z = oz
        odom.pose.pose.orientation = oq
        odom.twist.twist = twist
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
        t.transform.translation.z = oz
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
