#!/usr/bin/env python3
"""Publish /localization/dead_reckoning from Unity /sim_odom (sim only).

Unity moves the robot body directly from cmd_vel and leaves the wheel
velocities in joint_states at zero, so dead_reckoning_node always reports
zero in sim. This node stands in for it with the same message layout:
pose relative to the first message and twist in the base_link frame.

The velocity is obtained by differentiating the pose, because the frame of
the twist in /sim_odom is not documented.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry


def _quat_yaw(q):
    """Extract yaw from quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def unity_to_ros(x, y, yaw):
    """Unity ground plane -> ROS (X=forward, Y=left). Same as odom_bridge_node."""
    return -y, x, yaw + math.pi / 2.0


def relative_pose(origin, pose):
    """Express pose (x, y, yaw) in the frame of origin (x, y, yaw)."""
    dx = pose[0] - origin[0]
    dy = pose[1] - origin[1]
    c = math.cos(-origin[2])
    s = math.sin(-origin[2])
    return (c * dx - s * dy, s * dx + c * dy, _normalize_angle(pose[2] - origin[2]))


def body_velocity(prev, curr, dt):
    """Velocity (vx, vy, yaw_rate) in the body frame between two world poses."""
    dyaw = _normalize_angle(curr[2] - prev[2])
    heading = prev[2] + dyaw / 2.0
    wx = (curr[0] - prev[0]) / dt
    wy = (curr[1] - prev[1]) / dt
    c = math.cos(heading)
    s = math.sin(heading)
    return (c * wx + s * wy, -s * wx + c * wy, dyaw / dt)


class SimDeadReckoningNode(Node):
    def __init__(self):
        super().__init__('sim_dead_reckoning_node')

        self.declare_parameter('frame_id', 'dead_reckoning')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('max_dt_sec', 0.2)

        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame = self.get_parameter('base_frame').value
        self.max_dt_sec = float(self.get_parameter('max_dt_sec').value)
        if not math.isfinite(self.max_dt_sec) or self.max_dt_sec <= 0.0:
            raise ValueError('max_dt_sec must be finite and positive')

        self._origin = None
        self._prev_pose = None
        self._prev_time = None
        self._velocity = (0.0, 0.0, 0.0)

        self.pub = self.create_publisher(Odometry, 'dead_reckoning', 10)
        self.sub = self.create_subscription(Odometry, '/sim_odom', self.on_sim_odom, 10)
        self.get_logger().info('Publishing dead_reckoning from /sim_odom')

    def _message_time(self, msg):
        """Seconds of the header stamp, or of arrival when the stamp is unusable."""
        stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if stamp > 0.0 and (self._prev_time is None or stamp > self._prev_time):
            return stamp
        return self.get_clock().now().nanoseconds * 1e-9

    def on_sim_odom(self, msg: Odometry):
        p = msg.pose.pose
        world = unity_to_ros(p.position.x, p.position.y, _quat_yaw(p.orientation))
        if not all(math.isfinite(v) for v in world):
            self.get_logger().warning(
                'Ignoring /sim_odom with a non-finite pose', throttle_duration_sec=1.0)
            return
        now = self._message_time(msg)

        if self._origin is None:
            self._origin = world
        elif self._prev_time is not None:
            dt = now - self._prev_time
            if 0.0 < dt <= self.max_dt_sec:
                self._velocity = body_velocity(self._prev_pose, world, dt)
            else:
                self.get_logger().warning(
                    f'Skipping velocity: /sim_odom interval {dt:.3f} s is outside '
                    f'(0, {self.max_dt_sec:.3f}]', throttle_duration_sec=1.0)
        self._prev_pose = world
        self._prev_time = now

        x, y, yaw = relative_pose(self._origin, world)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        odom.twist.twist.linear.x = self._velocity[0]
        odom.twist.twist.linear.y = self._velocity[1]
        odom.twist.twist.angular.z = self._velocity[2]
        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = SimDeadReckoningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
