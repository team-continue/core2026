#!/usr/bin/env python3
"""
実機なしで同定・MPC 試走の手順をリハーサルするための模擬サーボ.

core_hardware の代わりに /hardware/can/tx の位置指令を受け、むだ時間 + 2次遅れ
（+ 速度制限・ノイズ）で動く仮想モータの角度を /joint_states に 100Hz で出す。
turret_sysid_logger / turret_mpc_trial / turret_sysid_identify の動作確認用で、
実機の応答を表すものではない。

実機の core_hardware と同時に起動しないこと（/joint_states が二重になる）。
別の ROS_DOMAIN_ID か ROS_LOCALHOST_ONLY=1 で、ロボットのネットワークから切り離して使う。
"""

import collections
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from core_msgs.msg import CANArray

JOINT_NUM = 15  # core_hardware の kJointNum と同じ


class FakeServo(Node):
    """位置指令に対して むだ時間 + 2次遅れ で応答する仮想モータ群."""

    def __init__(self):
        super().__init__('turret_fake_servo')
        p = self._declare
        self.motor_ids = list(p('motor_ids', [5, 6, 7, 11]))
        self.gain = p('gain', 1.0)
        self.wn = p('natural_frequency_rad_per_sec', 25.0)
        self.zeta = p('damping_ratio', 0.8)
        self.delay = p('delay_sec', 0.04)
        self.rate_limit = p('rate_limit_rad_per_sec', 6.0)  # 0 以下で無効
        self.noise_std = p('noise_std_rad', 0.0005)
        self.publish_rate = p('publish_rate_hz', 100.0)
        can_topic = p('can_topic', '/hardware/can/tx')
        joint_topic = p('joint_states_topic', '/joint_states')

        self.sub_dt = 0.0005
        self.pos = {i: 0.0 for i in self.motor_ids}
        self.vel = {i: 0.0 for i in self.motor_ids}
        self.target = {i: 0.0 for i in self.motor_ids}       # むだ時間後に効いている指令
        self.queue = {i: collections.deque() for i in self.motor_ids}
        self.last_time = self._now()
        self.rng = random.Random(0)

        self.pub = self.create_publisher(JointState, joint_topic, 10)
        self.create_subscription(CANArray, can_topic, self._on_can, 10)
        self.create_timer(1.0 / self.publish_rate, self._on_timer)
        self.get_logger().info(
            f'fake servo: motors={self.motor_ids} K={self.gain} wn={self.wn} zeta={self.zeta} '
            f'delay={self.delay}s rate_limit={self.rate_limit}rad/s on {can_topic}')

    def _declare(self, name, default):
        return self.declare_parameter(name, default).value

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_can(self, msg):
        t = self._now()
        for can in msg.array:
            if can.id in self.queue and len(can.data) > 0:
                self.queue[can.id].append((t + self.delay, float(can.data[0])))

    def _advance(self, t_end):
        t = self.last_time
        while t < t_end:
            h = min(self.sub_dt, t_end - t)
            t += h
            for i in self.motor_ids:
                q = self.queue[i]
                while q and q[0][0] <= t:
                    self.target[i] = q.popleft()[1]
                # x'' = wn^2 (K u - x) - 2 zeta wn x'（半陰的オイラー）
                acc = self.wn ** 2 * (self.gain * self.target[i] - self.pos[i]) \
                    - 2.0 * self.zeta * self.wn * self.vel[i]
                self.vel[i] += acc * h
                if self.rate_limit > 0.0:
                    self.vel[i] = max(-self.rate_limit, min(self.rate_limit, self.vel[i]))
                self.pos[i] += self.vel[i] * h
        self.last_time = t_end

    def _on_timer(self):
        now = self._now()
        self._advance(now)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [0.0] * JOINT_NUM
        msg.velocity = [0.0] * JOINT_NUM
        msg.effort = [0.0] * JOINT_NUM
        for i in self.motor_ids:
            if 0 <= i < JOINT_NUM:
                noise = self.rng.gauss(0.0, self.noise_std) if self.noise_std > 0 else 0.0
                msg.position[i] = self.pos[i] + noise
                msg.velocity[i] = self.vel[i]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeServo()
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
