#!/usr/bin/env python3
"""
砲塔1軸のシステム同定用データ取得ノード.

指定した軸（yaw / pitch）に、安全な振幅のステップ・多段ステップ・チャープ・PRBS の
位置指令を /can/tx（実機では /hardware/can/tx）へ送り、送った指令と /joint_states の実測角を
時刻付きで CSV に記録する。記録したデータは turret_sysid_identify で同定する。

安全機構:
  - angle_min / angle_max（絶対角 [rad]）を必須にし、計画した指令列が 1 点でもはみ出す設定は
    何も送らずに拒否する。振幅は amplitude_hard_limit_rad でも頭打ちにする。
  - 実行中、実測角の範囲逸脱・joint_states 途絶・追従誤差の持続・非常停止を検出したら
    「現在角を保持する指令」を送って停止する。Ctrl+C でも同じ。
  - もう一方の軸（hold_motor_id）は開始時の角度で保持し続ける。
  - dry_run:=true なら指令列の検証と保存だけを行い、何も送らない。

aim_bot と同時に動かすと /can/tx の指令が競合するので、必ず aim_bot を止めてから使うこと。
手順は docs/packages/core_shooter/turret_mpc_sysid.md を参照。
"""

import csv
import datetime
import math
import os
import random
import signal
import socket
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import yaml

from core_msgs.msg import CAN, CANArray

try:
    from rclpy.signals import SignalHandlerOptions
except ImportError:  # 古い rclpy 向け
    SignalHandlerOptions = None

# shooter.params.yaml の aim_bot と同じ割り当て
DEFAULT_MOTOR_IDS = {
    ('left', 'yaw'): 5,
    ('left', 'pitch'): 7,
    ('right', 'yaw'): 6,
    ('right', 'pitch'): 11,
}

SIGNAL_KINDS = ('step', 'multistep', 'chirp', 'prbs')

_stop_requested = False


def _on_signal(signum, frame):
    del signum, frame
    global _stop_requested
    _stop_requested = True


# ---------------------------------------------------------------------------
# 励起信号の生成（純粋関数）
# ---------------------------------------------------------------------------

def _hold(value, seconds, dt):
    return [value] * max(int(round(seconds / dt)), 0)


def generate_offsets(kind, amplitude, dt, cfg):
    """
    中心角からの相対指令列 [rad] を周期 dt ごとに返す（前後の静止区間を含む）.

    cfg は step_hold_sec, step_repeats, chirp_*, prbs_*, seed, pre_hold_sec, post_hold_sec を持つ dict。
    """
    a = amplitude
    body = []
    if kind == 'step':
        # +a → 0 → -a → 0 を繰り返す。立ち上がり・立ち下がりの両方向を取る
        for _ in range(cfg['step_repeats']):
            for level in (a, 0.0, -a, 0.0):
                body += _hold(level, cfg['step_hold_sec'], dt)
    elif kind == 'multistep':
        # 振幅の異なるステップ。小振幅と大振幅で応答が変わる（速度飽和・摩擦）かを見る
        levels = (0.2, 0.5, 1.0, 0.3, -0.3, -1.0, -0.5, 0.0, 0.7, -0.7, 0.1, 0.0)
        for _ in range(cfg['step_repeats']):
            for level in levels:
                body += _hold(a * level, cfg['step_hold_sec'], dt)
    elif kind == 'chirp':
        # 対数掃引の正弦波。高周波で角速度が大きくなりすぎないよう、各瞬間の振幅を
        # chirp_max_rate_rad_per_sec / (2 pi f) で頭打ちにする
        f0 = cfg['chirp_f0_hz']
        f1 = cfg['chirp_f1_hz']
        duration = cfg['chirp_duration_sec']
        n = int(round(duration / dt))
        k = math.log(f1 / f0) / duration
        max_rate = cfg['chirp_max_rate_rad_per_sec']
        for i in range(n):
            t = i * dt
            f = f0 * math.exp(k * t)
            phase = 2.0 * math.pi * f0 * (math.exp(k * t) - 1.0) / k
            amp = a if max_rate <= 0.0 else min(a, max_rate / (2.0 * math.pi * f))
            body.append(amp * math.sin(phase))
    elif kind == 'prbs':
        # ±a をランダムな長さ（prbs_dwell_sec の 1..prbs_max_dwell_multiple 倍）で切り替える
        rng = random.Random(cfg['seed'])
        n_total = int(round(cfg['prbs_duration_sec'] / dt))
        dwell_steps = max(int(round(cfg['prbs_dwell_sec'] / dt)), 1)
        level = a
        while len(body) < n_total:
            body += [level] * (dwell_steps * rng.randint(1, cfg['prbs_max_dwell_multiple']))
            level = -level
        body = body[:n_total]
    else:
        raise ValueError(f'unknown signal kind: {kind}')
    return _hold(0.0, cfg['pre_hold_sec'], dt) + body + _hold(0.0, cfg['post_hold_sec'], dt)


def move_toward(current, target, max_step):
    if max_step <= 0.0 or abs(target - current) <= max_step:
        return target
    return current + math.copysign(max_step, target - current)


# ---------------------------------------------------------------------------
# ノード
# ---------------------------------------------------------------------------

class TurretSysidLogger(Node):
    """1軸に励起指令を送り、指令と実測角を CSV に記録する."""

    def __init__(self):
        super().__init__('turret_sysid_logger')
        nan = float('nan')
        p = self._declare
        self.side = p('side', 'left')
        self.axis = p('axis', 'yaw')
        self.motor_id = p('motor_id', -1)            # -1: side/axis から自動
        self.hold_motor_id = p('hold_motor_id', -2)  # -2: 同じ側のもう一方の軸 / -1: 保持しない
        self.signal_kind = p('signal', 'step')
        self.amplitude = p('amplitude_rad', 0.1)
        self.amplitude_hard_limit = p('amplitude_hard_limit_rad', 0.8)
        self.center_param = p('center_rad', nan)      # NaN: 開始時の現在角
        self.angle_min = p('angle_min', nan)
        self.angle_max = p('angle_max', nan)
        self.range_margin = p('range_margin_rad', 0.05)
        self.command_rate = p('command_rate_hz', 100.0)
        self.approach_rate = p('approach_rate_rad_per_sec', 0.3)
        self.joint_timeout = p('joint_timeout_sec', 0.2)
        self.max_tracking_error = p('max_tracking_error_rad', 0.3)
        self.tracking_error_time = p('tracking_error_time_sec', 1.0)
        self.abort_on_hazard = p('abort_on_hazard', True)
        hazard_topic = p('hazard_topic', '/system/emergency/hazard_status')
        can_topic = p('can_topic', '/hardware/can/tx')
        joint_topic = p('joint_states_topic', '/joint_states')
        self.output_dir = p('output_dir', 'sysid_data')
        self.run_label = p('run_label', '')
        self.dry_run = p('dry_run', False)
        self.signal_cfg = {
            'pre_hold_sec': p('pre_hold_sec', 1.0),
            'post_hold_sec': p('post_hold_sec', 1.5),
            'step_hold_sec': p('step_hold_sec', 1.0),
            'step_repeats': p('step_repeats', 3),
            'chirp_f0_hz': p('chirp_f0_hz', 0.2),
            'chirp_f1_hz': p('chirp_f1_hz', 5.0),
            'chirp_duration_sec': p('chirp_duration_sec', 20.0),
            'chirp_max_rate_rad_per_sec': p('chirp_max_rate_rad_per_sec', 2.0),
            'prbs_dwell_sec': p('prbs_dwell_sec', 0.1),
            'prbs_max_dwell_multiple': p('prbs_max_dwell_multiple', 8),
            'prbs_duration_sec': p('prbs_duration_sec', 20.0),
            'seed': p('seed', 0),
        }

        self._validate_static()
        self.dt = 1.0 / self.command_rate
        self.offsets = generate_offsets(
            self.signal_kind, self.amplitude, self.dt, self.signal_cfg)

        # 状態
        self.phase = 'wait_joint'
        self.finished = False
        self.status = 'running'
        self.has_joint = False
        self.measured = nan
        self.hold_measured = nan
        self.last_joint_time = None
        self.hazard = None
        self.center = nan
        self.hold_target = None
        self.last_command = nan
        self.plan_index = 0
        self.error_time_acc = 0.0
        self.abort_elapsed = 0.0
        self.start_time = self._now()

        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        name = f'{self.side}_{self.axis}_{self.signal_kind}_{stamp}'
        if self.run_label:
            name += f'_{self.run_label}'
        self.run_dir = os.path.join(self.output_dir, name)
        os.makedirs(self.run_dir, exist_ok=False)
        self.cmd_file = open(os.path.join(self.run_dir, 'command.csv'), 'w', newline='')
        self.joint_file = open(os.path.join(self.run_dir, 'joint.csv'), 'w', newline='')
        self.cmd_csv = csv.writer(self.cmd_file)
        self.joint_csv = csv.writer(self.joint_file)
        self.cmd_csv.writerow(['send_time', 'command', 'hold_command', 'phase'])
        self.joint_csv.writerow(
            ['recv_time', 'stamp', 'position', 'hold_position', 'velocity_raw', 'effort_raw'])

        self.can_pub = self.create_publisher(CANArray, can_topic, 10)
        self.create_subscription(JointState, joint_topic, self._on_joint, 10)
        hazard_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, hazard_topic, self._on_hazard, hazard_qos)
        self.timer = self.create_timer(self.dt, self._on_timer)

        self.get_logger().info(
            f'turret_sysid_logger: side={self.side} axis={self.axis} motor_id={self.motor_id} '
            f'hold_motor_id={self.hold_motor_id} signal={self.signal_kind} '
            f'amplitude={self.amplitude:.3f}rad '
            f'range=[{self.angle_min:.3f}, {self.angle_max:.3f}] '
            f'rate={self.command_rate:.0f}Hz duration={len(self.offsets) * self.dt:.1f}s '
            f'dry_run={self.dry_run} out={self.run_dir}')

    # ---- 補助 ----
    def _declare(self, name, default):
        return self.declare_parameter(name, default).value

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _fail(self, message):
        self.get_logger().fatal(message)
        raise ValueError(message)

    def _validate_static(self):
        if (self.side, self.axis) not in DEFAULT_MOTOR_IDS:
            self._fail("side must be 'left' or 'right' and axis must be 'yaw' or 'pitch'")
        if self.motor_id < 0:
            self.motor_id = DEFAULT_MOTOR_IDS[(self.side, self.axis)]
        if self.hold_motor_id == -2:
            other = 'pitch' if self.axis == 'yaw' else 'yaw'
            self.hold_motor_id = DEFAULT_MOTOR_IDS[(self.side, other)]
        if self.hold_motor_id == self.motor_id:
            self._fail('hold_motor_id must differ from motor_id')
        if self.signal_kind not in SIGNAL_KINDS:
            self._fail(f'signal must be one of {SIGNAL_KINDS}')
        if not (math.isfinite(self.angle_min) and math.isfinite(self.angle_max)) or \
                self.angle_min >= self.angle_max:
            self._fail('angle_min / angle_max (absolute motor angle [rad]) must be set explicitly')
        if not (0.0 < self.amplitude <= self.amplitude_hard_limit):
            self._fail(
                f'amplitude_rad={self.amplitude} must be in (0, amplitude_hard_limit_rad='
                f'{self.amplitude_hard_limit}]')
        if not (1.0 <= self.command_rate <= 500.0):
            self._fail('command_rate_hz must be within [1, 500]')
        if self.approach_rate <= 0.0:
            self._fail('approach_rate_rad_per_sec must be positive')
        cfg = self.signal_cfg
        if self.signal_kind == 'chirp' and not (0.0 < cfg['chirp_f0_hz'] < cfg['chirp_f1_hz']):
            self._fail('chirp requires 0 < chirp_f0_hz < chirp_f1_hz')
        if cfg['step_repeats'] < 1 or cfg['prbs_max_dwell_multiple'] < 1:
            self._fail('step_repeats and prbs_max_dwell_multiple must be >= 1')

    def _validate_plan(self, current):
        """開始時の現在角が決まった後で、計画した全指令が範囲内かを確かめる."""
        if not (self.angle_min <= current <= self.angle_max):
            return f'current angle {current:.3f} is outside [{self.angle_min}, {self.angle_max}]'
        lo = self.center + min(self.offsets)
        hi = self.center + max(self.offsets)
        if lo < self.angle_min or hi > self.angle_max:
            return (f'planned commands [{lo:.3f}, {hi:.3f}] exceed the allowed range '
                    f'[{self.angle_min:.3f}, {self.angle_max:.3f}]')
        return None

    def _publish(self, command):
        msg = CANArray()
        can = CAN()
        can.id = self.motor_id
        can.data = [float(command)]
        msg.array.append(can)
        if self.hold_target is not None:
            hold = CAN()
            hold.id = self.hold_motor_id
            hold.data = [float(self.hold_target)]
            msg.array.append(hold)
        if not self.dry_run:
            self.can_pub.publish(msg)
        self.cmd_csv.writerow([
            f'{self._now():.6f}', f'{command:.6f}',
            '' if self.hold_target is None else f'{self.hold_target:.6f}', self.phase])
        self.last_command = command

    def _abort(self, reason):
        if self.phase in ('abort', 'done'):
            return
        self.get_logger().error(f'ABORT: {reason} -> holding current angle')
        self.status = f'aborted: {reason}'
        self.phase = 'abort'
        self.abort_elapsed = 0.0
        self.abort_hold = self.measured if math.isfinite(self.measured) else self.last_command

    def _finish(self, status=None):
        if status is not None:
            self.status = status
        self.phase = 'done'
        self.finished = True

    # ---- コールバック ----
    def _on_joint(self, msg):
        n = len(msg.position)
        if self.motor_id >= n or (self.hold_motor_id >= 0 and self.hold_motor_id >= n):
            self.get_logger().warn(
                f'joint_states too short: size={n}', throttle_duration_sec=2.0)
            return
        self.measured = msg.position[self.motor_id]
        if self.hold_motor_id >= 0:
            self.hold_measured = msg.position[self.hold_motor_id]
        self.last_joint_time = self._now()
        self.has_joint = True
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        vel = msg.velocity[self.motor_id] if self.motor_id < len(msg.velocity) else float('nan')
        eff = msg.effort[self.motor_id] if self.motor_id < len(msg.effort) else float('nan')
        self.joint_csv.writerow([
            f'{self.last_joint_time:.6f}', f'{stamp:.6f}', f'{self.measured:.6f}',
            f'{self.hold_measured:.6f}', f'{vel:.6f}', f'{eff:.6f}'])

    def _on_hazard(self, msg):
        self.hazard = bool(msg.data)

    def _on_timer(self):
        if self.finished:
            return
        now = self._now()
        if _stop_requested and self.phase not in ('abort', 'done'):
            if self.phase == 'wait_joint':
                self._finish('interrupted before start')
                return
            self._abort('interrupted (Ctrl+C)')

        if self.phase == 'wait_joint':
            if not self.has_joint:
                if now - self.start_time > 3.0:
                    self.get_logger().error('no joint_states within 3 s; nothing was commanded')
                    self._finish('failed: no joint_states')
                return
            if self.abort_on_hazard and self.hazard:
                self.get_logger().error('emergency stop is active; nothing was commanded')
                self._finish('failed: hazard active at start')
                return
            if self.hazard is None:
                self.get_logger().warn(
                    'hazard_status has not been received; emergency stop is not monitored')
            self.center = self.center_param if math.isfinite(self.center_param) else self.measured
            problem = self._validate_plan(self.measured)
            if problem:
                self.get_logger().error(f'refusing to start: {problem}')
                self._finish(f'failed: {problem}')
                return
            if self.hold_motor_id >= 0:
                self.hold_target = self.hold_measured
            self.last_command = self.measured
            self.phase = 'approach'
            self.get_logger().info(
                f'start: current={self.measured:.4f} center={self.center:.4f} '
                f'hold={self.hold_target}')

        # ---- 監視（指令を出し始めた後） ----
        if self.phase not in ('abort', 'done') and not self.dry_run:
            if self.last_joint_time is None or now - self.last_joint_time > self.joint_timeout:
                self._abort('joint_states timeout')
            elif not (self.angle_min - self.range_margin <= self.measured
                      <= self.angle_max + self.range_margin):
                self._abort(f'measured angle {self.measured:.4f} is outside the allowed range')
            elif self.abort_on_hazard and self.hazard:
                self._abort('emergency stop (hazard_status=true)')
            elif abs(self.measured - self.last_command) > self.max_tracking_error:
                self.error_time_acc += self.dt
                if self.error_time_acc > self.tracking_error_time:
                    self._abort('tracking error stayed above max_tracking_error_rad '
                                '(stall or collision?)')
            else:
                self.error_time_acc = 0.0

        if self.phase == 'approach':
            command = move_toward(self.last_command, self.center, self.approach_rate * self.dt)
            self._publish(command)
            if command == self.center:
                self.phase = 'excite'
                self.get_logger().info('excitation started')
        elif self.phase == 'excite':
            command = self.center + self.offsets[self.plan_index]
            # 最終防衛線（計画は検証済みだが念のため）
            command = min(max(command, self.angle_min), self.angle_max)
            self._publish(command)
            self.plan_index += 1
            if self.plan_index >= len(self.offsets):
                self.get_logger().info('excitation finished')
                self._finish('completed')
        elif self.phase == 'abort':
            self._publish(self.abort_hold)
            self.abort_elapsed += self.dt
            if self.abort_elapsed > 0.3:
                self._finish()

    def close(self):
        """記録を閉じ、実行条件と結果を meta.yaml に残す."""
        self.cmd_file.close()
        self.joint_file.close()
        params = {name: self.get_parameter(name).value for name in sorted(self._parameters)}
        meta = {
            'turret_sysid_run': {
                'side': self.side,
                'axis': self.axis,
                'motor_id': self.motor_id,
                'hold_motor_id': self.hold_motor_id,
                'signal': self.signal_kind,
                'amplitude_rad': self.amplitude,
                'center_rad': None if not math.isfinite(self.center) else float(self.center),
                'hold_angle_rad': None if self.hold_target is None else float(self.hold_target),
                'command_rate_hz': self.command_rate,
                'status': self.status,
                'dry_run': self.dry_run,
                'host': socket.gethostname(),
                'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', ''),
                'started_at': datetime.datetime.fromtimestamp(self.start_time).isoformat(),
                'parameters': params,
            }
        }
        with open(os.path.join(self.run_dir, 'meta.yaml'), 'w') as f:
            yaml.safe_dump(meta, f, sort_keys=False, allow_unicode=True)
        self.get_logger().info(f'saved: {self.run_dir} (status: {self.status})')


def main():
    if SignalHandlerOptions is not None:
        # Ctrl+C で即終了すると保持指令を送れないので、rclpy のシグナル処理を使わず自前で受ける
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    else:
        rclpy.init()
    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)
    exit_code = 0
    node = None
    try:
        node = TurretSysidLogger()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.01)
        exit_code = 0 if node.status == 'completed' else 1
    except ValueError as e:
        print(f'turret_sysid_logger: {e}', file=sys.stderr)
        exit_code = 2
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
