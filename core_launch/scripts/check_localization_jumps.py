#!/usr/bin/env python3
"""自己位置の跳びの指標を記録して出す（シミュレーションでも、実機の rosbag の再生でも使う）.

自己位置のスタック（あれば検出も）が動いている状態で実行する。次のどれかで止まり、動きの区分ごとの指標を
表と JSON に出す。
- --seconds の時間（シミュレーションの時刻）が過ぎた
- /lio/odom が --idle-s 秒（実時間）届かない（rosbag の再生が終わった）
- SIGINT か SIGTERM

指標は map→odom の段差、地図照合の補正と捨てた回数、DEGRADED と FAILED の時間。/ground_truth/odom が
届くとき（シミュレーション）は、map→base_link の位置の誤差も出す。区分は core_nav_check/jumps.py の BANDS。
"""

import argparse
import csv
import json
import math
from pathlib import Path
import signal
import sys
import time
from typing import Final

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Quaternion
from core_nav_check.jumps import (
    Alignment,
    Crowd,
    format_table,
    JumpError,
    Level,
    Motion,
    parse_alignment,
    PlanarPose,
    Record,
    summarize,
    time_base_gap,
    TIME_BASE_GAP_S,
)
from core_msgs.msg import LocalizationHealth, TrackedObjectArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

MAP_FRAME: Final = 'map'
ODOM_FRAME: Final = 'odom'


def stamp_s(stamp: object) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9  # type: ignore[attr-defined]


def yaw_of(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class JumpRecorder(Node):
    """跳びの指標に使うトピックを記録する."""

    def __init__(self, near_radius_m: float) -> None:
        super().__init__('localization_jump_recorder',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.near_radius_m = near_radius_m
        self.map_odom: list[PlanarPose] = []
        self.odom_base: list[PlanarPose] = []
        self.motion: list[Motion] = []
        self.alignments: list[Alignment] = []
        self.levels: list[Level] = []
        self.crowd: list[Crowd] = []
        self.truth: list[PlanarPose] = []
        self.last_lio_wall = math.nan
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(TFMessage, '/tf', self._on_tf, reliable)
        self.create_subscription(DiagnosticArray, '/localization/alignment_status', self._on_alignment, reliable)
        self.create_subscription(LocalizationHealth, '/localization/health', self._on_health, reliable)
        self.create_subscription(Odometry, '/localization/lio/odom', self._on_lio, reliable)
        self.create_subscription(TrackedObjectArray, '/perception/tracks', self._on_tracks, reliable)
        self.create_subscription(Odometry, '/ground_truth/odom', self._on_truth, reliable)

    def _on_tf(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            if transform.header.frame_id == MAP_FRAME and transform.child_frame_id == ODOM_FRAME:
                t = transform.transform.translation
                self.map_odom.append(PlanarPose(
                    stamp_s(transform.header.stamp), t.x, t.y, yaw_of(transform.transform.rotation)))

    def _on_alignment(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            alignment = parse_alignment(stamp_s(msg.header.stamp), {kv.key: kv.value for kv in status.values})
            if alignment is not None:
                self.alignments.append(alignment)

    def _on_health(self, msg: LocalizationHealth) -> None:
        self.levels.append(Level(stamp_s(msg.header.stamp), int(msg.level), msg.reason))

    def _on_lio(self, msg: Odometry) -> None:
        t = stamp_s(msg.header.stamp)
        pose = msg.pose.pose
        self.odom_base.append(PlanarPose(t, pose.position.x, pose.position.y, yaw_of(pose.orientation)))
        twist = msg.twist.twist
        self.motion.append(Motion(t, math.hypot(twist.linear.x, twist.linear.y), twist.angular.z))
        self.last_lio_wall = time.monotonic()

    def _on_tracks(self, msg: TrackedObjectArray) -> None:
        robot = self._robot_in(msg.header.frame_id)
        if robot is None:
            return
        near = sum(
            1 for obj in msg.objects
            if math.hypot(obj.position.x - robot[0], obj.position.y - robot[1]) <= self.near_radius_m
        )
        self.crowd.append(Crowd(stamp_s(msg.header.stamp), near))

    def _robot_in(self, frame: str) -> tuple[float, float] | None:
        """最新の機体の位置を frame の座標で返す（odom か map）."""
        if not self.odom_base:
            return None
        base = self.odom_base[-1]
        if frame == ODOM_FRAME:
            return base.x, base.y
        if frame == MAP_FRAME and self.map_odom:
            correction = self.map_odom[-1]
            cos, sin = math.cos(correction.yaw), math.sin(correction.yaw)
            return (correction.x + cos * base.x - sin * base.y, correction.y + sin * base.x + cos * base.y)
        return None

    def _on_truth(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.truth.append(PlanarPose(
            stamp_s(msg.header.stamp), pose.position.x, pose.position.y, yaw_of(pose.orientation)))

    def record(self) -> Record:
        end = max((m.t for m in self.motion), default=0.0)
        return Record(list(self.map_odom), list(self.odom_base), list(self.motion), list(self.alignments),
                      list(self.levels), list(self.crowd), list(self.truth), end)


def run(args: argparse.Namespace) -> int:
    rclpy.init()
    node = JumpRecorder(args.near_radius)
    stopping = {'signal': False}

    def on_signal(_signum: int, _frame: object) -> None:
        stopping['signal'] = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    start_sim = math.nan
    try:
        while not stopping['signal']:
            rclpy.spin_once(node, timeout_sec=0.1)
            now_sim = node.get_clock().now().nanoseconds * 1e-9
            if math.isnan(start_sim) and node.motion:
                start_sim = now_sim
            if not math.isnan(start_sim) and now_sim - start_sim >= args.seconds:
                break
            if not math.isnan(node.last_lio_wall) and time.monotonic() - node.last_lio_wall > args.idle_s:
                print(f'/lio/odom が {args.idle_s} 秒届かないので止めます', flush=True)
                break
        record = node.record()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    if not record.motion:
        print('/lio/odom を 1 回も受け取れませんでした', file=sys.stderr)
        return 1
    summary = summarize(record)
    gap = time_base_gap(record.levels, record.motion)
    if abs(gap) > TIME_BASE_GAP_S:
        print(f'[警告] /localization/health の時刻が /lio/odom と {gap:.0f} 秒ずれています（時刻の基準が違う）。'
              '区分ごとの DEGRADED と FAILED の時間は正しくありません。シミュレーションの rosbag は '
              '--use-sim-time を付けて録ってください', file=sys.stderr, flush=True)
    print(f'記録: map→odom {len(record.map_odom)} 回、地図照合 {len(record.alignments)} 回、'
          f'LIO {len(record.motion)} 回、追跡 {len(record.crowd)} 回、真値 {len(record.truth)} 回', flush=True)
    print('\n'.join(format_table(summary)), flush=True)
    if args.json is not None:
        args.json.write_text(json.dumps(summary, ensure_ascii=False, indent=1), encoding='utf-8')
        print(f'JSON: {args.json}', flush=True)
    if args.raw is not None:
        write_raw(args.raw, record)
        print(f'生の値: {args.raw}_*.csv', flush=True)
    return 0


def write_raw(prefix: Path, record: Record) -> None:
    """記録した値をそのまま CSV に書く（跳びの原因を調べるため）."""
    prefix.parent.mkdir(parents=True, exist_ok=True)
    tables = {
        'map_odom': (('t', 'x', 'y', 'yaw'), [(p.t, p.x, p.y, p.yaw) for p in record.map_odom]),
        'odom_base': (('t', 'x', 'y', 'yaw'), [(p.t, p.x, p.y, p.yaw) for p in record.odom_base]),
        'alignment': (
            ('t', 'converged', 'fitness', 'correction_m', 'correction_deg', 'consecutive_rejected',
             'motion_gated'),
            [(a.t, int(a.converged), a.fitness, a.correction_m, a.correction_deg, a.consecutive_rejected,
              int(a.motion_gated)) for a in record.alignments]),
        'truth': (('t', 'x', 'y', 'yaw'), [(p.t, p.x, p.y, p.yaw) for p in record.truth]),
        'health': (('t', 'level', 'reason'), [(h.t, h.level, h.reason) for h in record.levels]),
        'motion': (('t', 'speed_mps', 'yaw_rate'), [(m.t, m.speed_mps, m.yaw_rate) for m in record.motion]),
    }
    for name, (header, rows) in tables.items():
        with Path(f'{prefix}_{name}.csv').open('w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow(header)
            writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--seconds', type=float, default=math.inf, help='記録する時間（シミュレーションの時刻）[s]')
    parser.add_argument('--idle-s', type=float, default=5.0, help='/lio/odom がこの時間届かないと止める [s]')
    parser.add_argument('--near-radius', type=float, default=2.0, help='混雑とみなす追跡の距離 [m]')
    parser.add_argument('--json', type=Path, default=None)
    parser.add_argument('--raw', type=Path, default=None, help='生の値を <この名前>_*.csv に書く')
    try:
        return run(parser.parse_args())
    except JumpError as error:
        print(f'集計できません: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
