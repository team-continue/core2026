"""Bounded single-axis excitation through the commissioning controllers."""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64


LIMITS = {'waist': 0.35, 'vx': 0.20, 'vy': 0.20, 'wz': 0.35}
HAZARD = '/system/emergency/hazard_status'
CAN = '/hardware/can/tx'
PROFILES = {
    'step': [(0, 1), (1, 1), (0, 1), (-1, 1), (0, 2)],
    'multistep': [(0, 1), (0.5, 0.8), (1, 0.8), (0, 1), (-0.75, 0.8),
                  (-0.25, 0.8), (0.5, 0.8), (0, 2)],
}


def yaw(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(x) for x in values) or abs(math.hypot(*values) - 1) > 0.05:
        raise ValueError('invalid orientation quaternion')
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class Probe(Node):
    def __init__(self, args):
        super().__init__('base_sysid_probe')
        self.args = args
        self.samples = {}
        self.tripped = False
        self.origin = None
        self.waist_origin = None
        self.started = False
        self.subs = []
        self.subs.append(self.create_subscription(Bool, HAZARD, self.on_hazard, 1))
        self.subs.append(self.create_subscription(
            JointState, '/hardware/joint_states', self.on_joints, qos_profile_sensor_data))
        self.subs.append(self.create_subscription(
            Imu, '/sensing/imu', self.on_imu, qos_profile_sensor_data))
        if args.axis != 'waist':
            self.subs.append(self.create_subscription(
                Odometry, args.odom_topic, self.on_odom, qos_profile_sensor_data))
        self.is_waist = args.axis == 'waist'
        self.topic = '/sysid/waist_velocity' if self.is_waist else '/sysid/drive'
        self.tx = self.create_publisher(Float64 if self.is_waist else Twist, self.topic, 1)

    def on_hazard(self, msg):
        self.tripped |= msg.data
        self.samples['hazard'] = time.monotonic()

    def on_joints(self, msg):
        if len(msg.position) < 5 or not all(map(math.isfinite, msg.position[:5])):
            self.tripped = True
            return
        angle = msg.position[4]
        if self.started and abs(angle - self.waist_origin) > 0.65 and self.is_waist:
            self.tripped = True
        self.latest_waist = angle
        self.samples['joints'] = time.monotonic()

    def on_imu(self, msg):
        try:
            yaw(msg.orientation)
        except ValueError:
            self.tripped = True
        self.samples['imu'] = time.monotonic()

    def on_odom(self, msg):
        p = msg.pose.pose
        try:
            heading = yaw(p.orientation)
        except ValueError:
            self.tripped = True
            return
        if not all(map(math.isfinite, (p.position.x, p.position.y))) or (
                msg.child_frame_id != self.args.base_frame):
            self.tripped = True
            return
        self.latest_pose = (p.position.x, p.position.y, heading, msg.header.frame_id)
        if self.origin is not None:
            x, y, a, frame = self.origin
            turn = math.remainder(heading - a, 2 * math.pi)
            if (math.hypot(p.position.x-x, p.position.y-y) > 0.45 or
                    abs(turn) > 0.7 or frame != msg.header.frame_id):
                self.tripped = True
        self.samples['odom'] = time.monotonic()

    def ready(self):
        if self.tripped:
            raise RuntimeError('stopped: hazard, invalid feedback or excursion limit')
        needed = ['hazard', 'joints', 'imu']
        if not self.is_waist:
            needed.append('odom')
        now = time.monotonic()
        if any(now - self.samples.get(key, -math.inf) > 0.35 for key in needed):
            return False
        # This launch deliberately excludes shooter, operator and navigation writers.
        writers = sorted((n.node_namespace, n.node_name)
                         for n in self.get_publishers_info_by_topic(CAN))
        expected = 'sysid_yaw' if self.is_waist else 'sysid_body'
        listeners = self.get_subscriptions_info_by_topic(self.topic)
        controller_present = any((n.node_namespace, n.node_name) == ('/control', expected)
                                 for n in listeners)
        return (writers == [('/control', 'sysid_body'), ('/control', 'sysid_yaw')]
                and self.count_publishers(self.topic) == 1 and controller_present)

    def send(self, value):
        if self.is_waist:
            self.tx.publish(Float64(data=float(value)))
        else:
            msg = Twist()
            if self.args.axis == 'wz':
                msg.angular.z = value
            elif self.args.axis == 'vx':
                msg.linear.x = value
            else:
                msg.linear.y = value
            self.tx.publish(msg)

    def run(self, plan):
        deadline = time.monotonic() + 5
        while not self.ready():
            if time.monotonic() >= deadline:
                raise RuntimeError('no motion: check hazard, feedback, test controllers and publishers')
            rclpy.spin_once(self, timeout_sec=0.03)
        self.waist_origin = self.latest_waist
        if not self.is_waist:
            self.origin = self.latest_pose
        self.started = True
        try:
            for value, duration in plan:
                self.get_logger().info(f'{self.args.axis}: {value:+.3f} for {duration:.1f} s')
                end = time.monotonic() + duration
                while time.monotonic() < end:
                    tick = time.monotonic()
                    # Drain all ready feedback before sending the next sample.
                    for _ in range(12):
                        rclpy.spin_once(self, timeout_sec=0)
                    if not self.ready():
                        raise RuntimeError('aborted: stale feedback or changed command route')
                    self.send(value)
                    time.sleep(max(0, 0.02 - (time.monotonic() - tick)))
        finally:
            if rclpy.ok():
                self.send(0.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('axis', choices=LIMITS)
    parser.add_argument('--profile', choices=PROFILES, default='step')
    parser.add_argument('--amplitude', type=float, default=0.12)
    parser.add_argument('--odom-topic', default='/localization/odom')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--execute', action='store_true',
                        help='Send commands (default only prints the plan).')
    args = parser.parse_args()
    if not 0 < args.amplitude <= LIMITS[args.axis]:
        parser.error(f'amplitude must be in (0, {LIMITS[args.axis]}]')
    plan = [(level * args.amplitude, duration) for level, duration in PROFILES[args.profile]]
    print(f'{args.axis}: ' + ', '.join(f'{v:+.3f} x {d:g}s' for v, d in plan), flush=True)
    if not args.execute:
        return
    rclpy.init()
    probe = Probe(args)
    try:
        probe.run(plan)
    except (RuntimeError, KeyboardInterrupt) as exc:
        parser.exit(1, f'{exc}\n')
    finally:
        probe.destroy_node()
        rclpy.try_shutdown()
