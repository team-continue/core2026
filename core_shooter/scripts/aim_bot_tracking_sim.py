#!/usr/bin/env python3
"""
aim_bot の画像追尾を実機なしで試すための仮想砲塔・仮想カメラ.

aim_bot の位置指令（/hardware/can/tx）を受けて むだ時間 + 2次遅れ で動く仮想モータの角度を
/joint_states に出し、仮想の的と「撮影時刻の砲塔角」の差を画像座標[px]に直して
target_pose として遅延付きで出す。終了時に追従誤差を集計して表示し、CSV に保存する。

実機の core_hardware や敵検出と同時に起動しないこと（/joint_states や target_pose が二重になる）。
ROS_LOCALHOST_ONLY=1 か別の ROS_DOMAIN_ID で、ロボットのネットワークから切り離して使う。
"""

import collections
import csv
import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from core_msgs.msg import CANArray

JOINT_NUM = 15  # core_hardware の kJointNum と同じ
AXES = ('yaw', 'pitch')


class Axis:
    """位置指令に対して むだ時間 + 2次遅れ（+ 速度制限）で応答する仮想モータ1軸."""

    def __init__(self, wn, zeta, delay, rate_limit):
        self.wn = wn
        self.zeta = zeta
        self.delay = delay
        self.rate_limit = rate_limit
        self.pos = 0.0
        self.vel = 0.0
        self.effective_command = 0.0
        self.last_command = 0.0
        self.queue = collections.deque()

    def command(self, t, value):
        self.last_command = value
        self.queue.append((t + self.delay, value))

    def step(self, t, h):
        while self.queue and self.queue[0][0] <= t:
            self.effective_command = self.queue.popleft()[1]
        acc = self.wn ** 2 * (self.effective_command - self.pos) \
            - 2.0 * self.zeta * self.wn * self.vel
        self.vel += acc * h
        if self.rate_limit > 0.0:
            self.vel = max(-self.rate_limit, min(self.rate_limit, self.vel))
        self.pos += self.vel * h


def triangle(phase):
    """周期1・振幅1の三角波（phase=0 で 0 から上昇）."""
    p = phase % 1.0
    if p < 0.25:
        return 4.0 * p
    if p < 0.75:
        return 2.0 - 4.0 * p
    return 4.0 * p - 4.0


class TrackingSim(Node):
    """仮想砲塔 + 仮想カメラ。aim_bot と閉ループを組み、追従誤差を記録する."""

    def __init__(self):
        super().__init__('aim_bot_tracking_sim')
        p = self._declare

        # aim_bot 側の設定（launch が shooter.params.yaml から同じ値を渡す）
        self.motor_id = {'yaw': p('yaw_motor_id', 5), 'pitch': p('pitch_motor_id', 7)}
        self.image_gain = {'yaw': p('yaw_image_gain', 0.0005),
                           'pitch': p('pitch_image_gain', 0.008)}
        self.direction = {'yaw': p('yaw_direction', 1.0), 'pitch': p('pitch_direction', 1.0)}
        self.image_size = {'yaw': p('image_width', 320.0), 'pitch': p('image_height', 320.0)}
        self.image_center = {'yaw': p('image_center_x', 0.5), 'pitch': p('image_center_y', 0.5)}
        self.shoot_tolerance_px = {'yaw': p('image_shoot_tolerance_x_px', 50.0),
                                   'pitch': p('image_shoot_tolerance_y_px', 50.0)}

        # 仮想モータ
        self.axes = {
            axis: Axis(
                p(f'plant.{axis}.natural_frequency_rad_per_sec', 20.0),
                p(f'plant.{axis}.damping_ratio', 0.8),
                p(f'plant.{axis}.delay_sec', 0.03),
                p(f'plant.{axis}.rate_limit_rad_per_sec', 5.0))
            for axis in AXES}
        self.joint_rate = p('plant.joint_states_rate_hz', 100.0)
        self.joint_noise = p('plant.joint_noise_std_rad', 0.0)

        # 仮想カメラ
        self.fps = p('camera.fps', 30.0)
        self.latency = p('camera.latency_sec', 0.06)
        self.latency_jitter = p('camera.latency_jitter_sec', 0.01)
        self.noise_px = p('camera.noise_std_px', 1.0)
        # 実際の換算が aim_bot の image_gain とずれている場合を試す倍率（1.0 で一致）
        self.scale_error = p('camera.gain_scale_error', 1.0)
        # "capture": 撮影時刻を stamp に入れる / "none": stamp を 0 にする（image_latency_sec の確認用）
        self.stamp_mode = p('camera.stamp_mode', 'capture')

        # 仮想の的（モータ角空間, 初期角 0 の砲塔から見た角度）
        self.scenario = p('target.scenario', 'triangle')
        self.amplitude = {'yaw': p('target.yaw_amplitude_rad', 0.3),
                          'pitch': p('target.pitch_amplitude_rad', 0.1)}
        self.period = p('target.period_sec', 4.0)
        self.offset = {'yaw': p('target.yaw_offset_rad', 0.05),
                       'pitch': p('target.pitch_offset_rad', 0.02)}

        self.duration = p('duration_sec', 15.0)
        self.metrics_start = p('metrics_start_sec', 3.0)
        self.csv_path = p('csv_path', '')

        if self.scenario not in ('static', 'step', 'triangle', 'sine'):
            raise ValueError(
                f"target.scenario='{self.scenario}' must be static/step/triangle/sine")
        if self.stamp_mode not in ('capture', 'none'):
            raise ValueError(f"camera.stamp_mode='{self.stamp_mode}' must be capture/none")

        self.rng = random.Random(0)
        self.sub_dt = 0.0005
        self.start_time = self._now()
        self.last_plant_time = self.start_time
        self.pending_images = collections.deque()
        self.rows = []
        self.captures = 0
        self.lost_captures = 0
        self.finished = False

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.target_pub = self.create_publisher(PointStamped, 'target_pose', 10)
        self.hazard_pub = self.create_publisher(Bool, '/system/emergency/hazard_status', 10)
        self.create_subscription(CANArray, '/hardware/can/tx', self._on_can, 10)

        self.create_timer(1.0 / self.joint_rate, self._on_joint_timer)
        self.create_timer(1.0 / self.fps, self._on_capture)
        self.create_timer(0.002, self._on_image_queue)
        self.create_timer(0.1, self._on_hazard_timer)

        self.get_logger().info(
            f'tracking sim: scenario={self.scenario} amplitude={self.amplitude} '
            f'period={self.period}s fps={self.fps} latency={self.latency}s '
            f'stamp_mode={self.stamp_mode} duration={self.duration}s')

    def _declare(self, name, default):
        return self.declare_parameter(name, default).value

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _elapsed(self, t):
        return t - self.start_time

    def _target(self, t):
        """時刻 t の的の角度 [rad]（モータ角）."""
        s = self._elapsed(t)
        result = {}
        for axis in AXES:
            a = self.amplitude[axis]
            if self.scenario == 'static':
                value = 0.0
            elif self.scenario == 'step':
                # 周期ごとに +a / -a を行き来する
                value = a if (s // (0.5 * self.period)) % 2 == 0 else -a
            elif self.scenario == 'triangle':
                value = a * triangle(s / self.period)
            else:
                value = a * math.sin(2.0 * math.pi * s / self.period)
            result[axis] = self.offset[axis] + value
        return result

    def _advance_plant(self, t_end):
        t = self.last_plant_time
        while t < t_end:
            h = min(self.sub_dt, t_end - t)
            t += h
            for axis in self.axes.values():
                axis.step(t, h)
        self.last_plant_time = max(self.last_plant_time, t_end)

    def _on_can(self, msg):
        t = self._now()
        for can in msg.array:
            for name, motor_id in self.motor_id.items():
                if can.id == motor_id and len(can.data) > 0:
                    self.axes[name].command(t, float(can.data[0]))

    def _on_hazard_timer(self):
        self.hazard_pub.publish(Bool(data=False))

    def _on_joint_timer(self):
        now = self._now()
        self._advance_plant(now)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [0.0] * JOINT_NUM
        msg.velocity = [0.0] * JOINT_NUM
        msg.effort = [0.0] * JOINT_NUM
        for name, axis in self.axes.items():
            noise = self.rng.gauss(0.0, self.joint_noise) if self.joint_noise > 0.0 else 0.0
            msg.position[self.motor_id[name]] = axis.pos + noise
            msg.velocity[self.motor_id[name]] = axis.vel
        self.joint_pub.publish(msg)

        target = self._target(now)
        self.rows.append([
            self._elapsed(now),
            target['yaw'], self.axes['yaw'].pos, self.axes['yaw'].last_command,
            target['pitch'], self.axes['pitch'].pos, self.axes['pitch'].last_command])

        if self._elapsed(now) >= self.duration and not self.finished:
            self.finished = True
            self._report()
            raise SystemExit

    def _on_capture(self):
        """撮影: この瞬間の的と砲塔の角度差を画像座標にし、遅延後に送る."""
        now = self._now()
        self._advance_plant(now)
        target = self._target(now)

        point = PointStamped()
        point.header.frame_id = 'sim_camera'
        if self.stamp_mode == 'capture':
            point.header.stamp = self.get_clock().now().to_msg()
        visible = True
        px = {}
        for axis in AXES:
            # aim_bot は offset = direction * image_gain * (px - center) で角度に戻す
            angle_error = (target[axis] - self.axes[axis].pos) * self.scale_error
            center_px = (self.image_center[axis] - 0.5) * self.image_size[axis]
            value = center_px + angle_error / (self.direction[axis] * self.image_gain[axis])
            value += self.rng.gauss(0.0, self.noise_px) if self.noise_px > 0.0 else 0.0
            if abs(value) > 0.5 * self.image_size[axis]:
                visible = False
            px[axis] = value
        if visible:
            point.point.x, point.point.y, point.point.z = px['yaw'], px['pitch'], 0.0
        else:
            point.point.x, point.point.y, point.point.z = 0.0, 0.0, 1.0
        if self._elapsed(now) >= self.metrics_start:
            self.captures += 1
            self.lost_captures += 0 if visible else 1

        jitter = self.rng.uniform(-self.latency_jitter, self.latency_jitter) \
            if self.latency_jitter > 0.0 else 0.0
        self.pending_images.append((now + max(0.0, self.latency + jitter), point))

    def _on_image_queue(self):
        now = self._now()
        while self.pending_images and self.pending_images[0][0] <= now:
            self.target_pub.publish(self.pending_images.popleft()[1])

    def _report(self):
        rows = [r for r in self.rows if r[0] >= self.metrics_start]
        if not rows:
            self.get_logger().error('no samples after metrics_start_sec: increase duration_sec')
            return
        columns = {'yaw': (1, 2), 'pitch': (4, 5)}
        lines = [f'tracking result ({len(rows)} samples, t >= {self.metrics_start:.1f}s):']
        for axis, (ti, pi) in columns.items():
            errors = [r[ti] - r[pi] for r in rows]
            rms = math.sqrt(sum(e * e for e in errors) / len(errors))
            peak = max(abs(e) for e in errors)
            tol_rad = self.shoot_tolerance_px[axis] * abs(self.image_gain[axis])
            within = 100.0 * sum(1 for e in errors if abs(e) <= tol_rad) / len(errors)
            px_per_rad = 1.0 / abs(self.image_gain[axis])
            lines.append(
                f'  {axis:5s}: RMS={rms:.4f}rad ({rms * px_per_rad:.1f}px)  '
                f'max={peak:.4f}rad ({peak * px_per_rad:.1f}px)  '
                f'within shoot tolerance={within:.1f}%')
        if self.captures > 0:
            lost = 100.0 * self.lost_captures / self.captures
            lines.append(
                f'  target out of view: {self.lost_captures}/{self.captures} frames ({lost:.1f}%)')
            if self.lost_captures > 0:
                lines.append(
                    '  (the target left the image: errors include lost-target behavior; '
                    'reduce target amplitude if this is not intended)')
        self.get_logger().info('\n'.join(lines))

        if self.csv_path:
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    't', 'target_yaw', 'yaw', 'command_yaw',
                    'target_pitch', 'pitch', 'command_pitch'])
                writer.writerows(self.rows)
            self.get_logger().info(f'saved {len(self.rows)} rows to {self.csv_path}')


def main():
    rclpy.init()
    node = TrackingSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
