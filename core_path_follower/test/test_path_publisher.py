#!/usr/bin/env python3
# Copyright 2026 team-continue
# SPDX-License-Identifier: Apache-2.0

"""Test helper node that publishes preset paths for core_path_follower."""

import math

from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

try:
    from core_msgs.msg import CANArray
    _HAS_CAN_MSG = True
except ImportError:
    _HAS_CAN_MSG = False


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------

def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q


def make_pose_stamped(x, y, yaw=0.0, frame_id='chassis_link', stamp=None):
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    if stamp is not None:
        ps.header.stamp = stamp
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.orientation = yaw_to_quaternion(yaw)
    return ps


# ---------------------------------------------------------------------------
# Preset path generators
# ---------------------------------------------------------------------------

def _gen_square(side=2.0, n=20):
    pts = []
    for i in range(n):
        pts.append((side * i / n, 0.0))
    for i in range(n):
        pts.append((side, side * i / n))
    for i in range(n):
        pts.append((side - side * i / n, side))
    for i in range(n):
        pts.append((0.0, side - side * i / n))
    pts.append((0.0, 0.0))
    return pts


def _gen_figure8(r=1.0, n=120):
    return [
        (r * math.sin(2 * math.pi * i / n),
         r * math.sin(2 * math.pi * i / n) * math.cos(2 * math.pi * i / n))
        for i in range(n + 1)
    ]


def _gen_circle(r=1.5, n=80):
    return [
        (r * math.cos(2 * math.pi * i / n) - r,
         r * math.sin(2 * math.pi * i / n))
        for i in range(n + 1)
    ]


def _gen_straight(length=3.0, n=30):
    return [(length * i / n, 0.0) for i in range(n + 1)]


def _gen_slalom(amp=0.8, wl=2.0, length=6.0, n=100):
    return [
        (length * i / n, amp * math.sin(2 * math.pi * (length * i / n) / wl))
        for i in range(n + 1)
    ]


def _gen_diamond(size=1.5, n=15):
    pts = []
    for i in range(n):
        t = i / n
        pts.append((size * t, size * t))
    for i in range(n):
        t = i / n
        pts.append((size + size * t, size - size * t))
    for i in range(n):
        t = i / n
        pts.append((2 * size - size * t, -size * t))
    for i in range(n):
        t = i / n
        pts.append((size - size * t, -size + size * t))
    pts.append((0.0, 0.0))
    return pts


def _gen_lateral(length=2.0, n=20):
    """Pure +y translation for mecanum lateral drive test."""
    return [(0.0, length * i / n) for i in range(n + 1)]


PATH_GENERATORS = {
    'square': _gen_square,
    'figure8': _gen_figure8,
    'circle': _gen_circle,
    'straight': _gen_straight,
    'slalom': _gen_slalom,
    'diamond': _gen_diamond,
    'lateral': _gen_lateral,
}

CYCLE_ORDER = ['straight', 'square', 'slalom', 'circle', 'figure8', 'diamond', 'lateral']


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class TestPathPublisher(Node):

    def __init__(self):
        super().__init__('test_path_publisher')

        # Parameters
        self.declare_parameter('mode', 'path')
        self.declare_parameter('path_type', 'straight')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('use_local_frame', True)
        self.declare_parameter('frame_id', 'chassis_link')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('odom_rate', 50.0)
        self.declare_parameter('path_scale', 1.0)
        self.declare_parameter('one_shot', False)
        self.declare_parameter('cycle', False)
        self.declare_parameter('monitor_can', True)
        self.declare_parameter('publish_joint_states', False)

        self.mode = self.get_parameter('mode').value
        path_type = self.get_parameter('path_type').value
        path_topic = self.get_parameter('path_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.use_local_frame = self.get_parameter('use_local_frame').value
        self.frame_id = self.get_parameter('frame_id').value
        publish_rate = self.get_parameter('publish_rate').value
        odom_rate = self.get_parameter('odom_rate').value
        self.path_scale = self.get_parameter('path_scale').value
        self.one_shot = self.get_parameter('one_shot').value
        self.cycle = self.get_parameter('cycle').value
        monitor_can = self.get_parameter('monitor_can').value
        pub_js = self.get_parameter('publish_joint_states').value

        self.publish_odom = not self.use_local_frame

        # Validate path type
        if path_type not in PATH_GENERATORS:
            self.get_logger().error(
                "Unknown path_type '%s'. "
                'Available: %s' % (path_type, list(PATH_GENERATORS.keys()))
            )
            raise SystemExit(1)

        self.current_path_name = path_type
        self.cycle_index = (
            CYCLE_ORDER.index(path_type) if path_type in CYCLE_ORDER else 0
        )
        self.path_msg = self._build_path(path_type)

        # Publishers
        latching = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, path_topic, latching)

        if self.mode == 'planner':
            map_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
            self.start_pub = self.create_publisher(PoseStamped, '/start', 5)
            self.goal_pub = self.create_publisher(PoseStamped, '/goal', 5)

        # Subscribers
        self.cmd_sub = self.create_subscription(
            Twist, cmd_vel_topic, self._on_cmd_vel, 10)
        self.goal_sub = self.create_subscription(
            Bool, '/goal_reached', self._on_goal_reached, 10)

        self._can_count = 0
        if monitor_can and _HAS_CAN_MSG:
            self.can_sub = self.create_subscription(
                CANArray, 'can/tx', self._on_can, 10)

        # Simulated odometry (world-frame mode only)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.latest_cmd = Twist()

        if self.publish_odom:
            self.odom_pub = self.create_publisher(Odometry, odom_topic, 50)
            self.odom_timer = self.create_timer(1.0 / odom_rate, self._tick_odom)

        # Simulated joint_states for body_controller
        if pub_js:
            self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
            self.js_timer = self.create_timer(0.05, self._tick_js)

        # Path publish timer
        self.path_timer = self.create_timer(1.0 / publish_rate, self._tick_path)
        self.pub_count = 0
        self.cmd_count = 0

        self.get_logger().info(
            f'TestPathPublisher: mode={self.mode} path={path_type} '
            f'({len(self.path_msg.poses)} pts) frame={self.frame_id} '
            f'scale={self.path_scale} cycle={self.cycle}'
        )

    # -- Path building -------------------------------------------------------

    def _build_path(self, name: str) -> Path:
        raw = PATH_GENERATORS[name]()
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        s = self.path_scale
        for i, (x, y) in enumerate(raw):
            dx = (raw[i + 1][0] - x) if i + 1 < len(raw) else (x - raw[i - 1][0])
            dy = (raw[i + 1][1] - y) if i + 1 < len(raw) else (y - raw[i - 1][1])
            yaw = math.atan2(dy, dx)
            path.poses.append(
                make_pose_stamped(x * s, y * s, yaw, self.frame_id, path.header.stamp)
            )
        return path

    # -- Timer callbacks ------------------------------------------------------

    def _tick_path(self):
        if self.one_shot and self.pub_count >= 1:
            return
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)
        self.pub_count += 1
        if self.pub_count <= 3:
            self.get_logger().info(
                "Published '%s' (%d poses) [%d]"
                % (self.current_path_name, len(self.path_msg.poses), self.pub_count)
            )
        if self.mode == 'planner' and self.pub_count <= 2:
            self._publish_planner_topics()

    def _tick_odom(self):
        dt = 1.0 / self.get_parameter('odom_rate').value
        v = self.latest_cmd
        self.odom_yaw += v.angular.z * dt
        c, s = math.cos(self.odom_yaw), math.sin(self.odom_yaw)
        self.odom_x += (v.linear.x * c - v.linear.y * s) * dt
        self.odom_y += (v.linear.x * s + v.linear.y * c) * dt

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation = yaw_to_quaternion(self.odom_yaw)
        odom.twist.twist = self.latest_cmd
        self.odom_pub.publish(odom)

    def _tick_js(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['wheel_fl', 'wheel_fr', 'wheel_br', 'wheel_bl', 'base_to_chassis']
        js.position = [0.0] * 5
        js.velocity = [0.0] * 5
        js.effort = [0.0] * 5
        self.js_pub.publish(js)

    # -- Subscription callbacks -----------------------------------------------

    def _on_cmd_vel(self, msg: Twist):
        self.latest_cmd = msg
        self.cmd_count += 1
        if self.cmd_count % 20 == 1:
            self.get_logger().info(
                f'[cmd_vel #{self.cmd_count:>5d}] '
                f'vx={msg.linear.x:+.3f} vy={msg.linear.y:+.3f} '
                f'w={msg.angular.z:+.3f}'
            )

    def _on_goal_reached(self, msg: Bool):
        if not msg.data:
            return
        self.get_logger().info(
            "Goal reached: '%s' (cmds=%d)" % (self.current_path_name, self.cmd_count))
        if self.cycle:
            self.cycle_index = (self.cycle_index + 1) % len(CYCLE_ORDER)
            name = CYCLE_ORDER[self.cycle_index]
            self.current_path_name = name
            self.path_msg = self._build_path(name)
            self.pub_count = 0
            self.cmd_count = 0
            self.get_logger().info(
                "Cycle -> '%s' (%d poses)" % (name, len(self.path_msg.poses)))

    def _on_can(self, msg):
        self._can_count += 1
        if self._can_count % 100 == 1:
            wheels = [
                f'W{c.id}={c.data[1]:+.1f}'
                for c in msg.array if c.id <= 3 and len(c.data) >= 2
            ]
            if wheels:
                self.get_logger().info('[CAN #%d] %s' % (self._can_count, ', '.join(wheels)))

    # -- Planner-compatible mode ----------------------------------------------

    def _publish_planner_topics(self):
        stamp = self.get_clock().now().to_msg()

        grid = OccupancyGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = 'map'
        grid.info = MapMetaData()
        grid.info.resolution = 0.1
        grid.info.width = 100
        grid.info.height = 100
        grid.info.origin.position.x = -5.0
        grid.info.origin.position.y = -5.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (100 * 100)
        self.map_pub.publish(grid)

        self.start_pub.publish(make_pose_stamped(0, 0, 0, 'map', stamp))
        last = self.path_msg.poses[-1] if self.path_msg.poses else None
        gx = last.pose.position.x if last else 2.0
        gy = last.pose.position.y if last else 0.0
        self.goal_pub.publish(make_pose_stamped(gx, gy, 0, 'map', stamp))


def main(args=None):
    rclpy.init(args=args)
    node = TestPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
