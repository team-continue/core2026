#!/usr/bin/env python3
"""EKF の自己位置（/localization/kinematic_state）を、真値と今の自己位置（LIO + 地図照合）と比べる.

sim_localization.launch.py state_source:=ekf を起動した状態で実行する。シミュレーションでは map と world が一致する。
check_localization.py と同じ走り方（静止、直進、旋回、急な加減速、柵への衝突、周回）で、区間ごとに比べる。
- 位置と向きの誤差: EKF、LIO + 地図照合（map→odom と /lio/odom）、地図照合そのもの（/pcl_pose）
- 速度の誤差（地図の座標）: EKF、LIO
- EKF の出力: 周期、最大の間、遅れ（受けたときの真値の時刻 − ヘッダの時刻）、補正の段差（速度一定で進めた位置との差）

--dropout（既定）では、地図照合の姿勢を区間を決めて止めた 2 つ目の EKF（ekf_localizer_dropout）も動かし、
止めている間の誤差の伸び（車輪の速度とジャイロだけで進めた分）と、戻ってから誤差が 5cm 以内に戻るまでの時間を測る。

終了コードは、衝突の区間を除く EKF の位置誤差が目標以内で、出力の周期が足り、想定外の柵への接近がなければ 0。
"""

import argparse
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from typing import Final, Sequence

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from core_nav_check.common import (
    CheckError,
    COLLISION_PHASE,
    DriveNode,
    FloatArray,
    left_safe_area,
    Pose,
    rms,
    run_with_node,
    stamp_to_sec,
    standard_phases,
    to_pose,
    wrap_angle,
)
from nav_msgs.msg import Odometry
import numpy as np
from numpy.typing import NDArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
import tf2_ros

EKF_TOPIC: Final = '/localization/kinematic_state'
DROPOUT_EKF_TOPIC: Final = '/localization/kinematic_state_dropout'
NDT_TOPIC: Final = '/localization/pcl_pose'
GATED_NDT_TOPIC: Final = '/localization/pcl_pose_gated'
TARGET_POSITION_M: Final = 0.10
MIN_RATE_HZ: Final = 45.0
# /pcl_pose のうち、照合を捨てた回に LIO で進めた姿勢（標準偏差 5m）は地図照合として数えない
MAX_NDT_VARIANCE_XY: Final = 1.0
# 地図照合を止めた後、誤差がこれ以内に戻るまでの時間を測る
RECOVERED_POSITION_M: Final = 0.05
# 周回の区間の始まりから、地図照合を止める（始まり [s], 長さ [s]）
DROPOUT_WINDOWS: Final = ((20.0, 5.0), (60.0, 10.0))
DROPOUT_PHASE: Final = '周回'


@dataclass(frozen=True)
class StateSample:
    t: float
    x: float
    y: float
    yaw: float
    # 地図の座標の速度
    vx: float
    vy: float
    # 受けたときの真値の時刻
    received: float


@dataclass(frozen=True)
class PhaseRecord:
    name: str
    start: float
    end: float


@dataclass(frozen=True)
class PhaseResult:
    # 推定ごと（ekf、lio_map、ndt）の誤差の集計
    errors: dict[str, dict[str, float]]
    # EKF の出力の周期、間、遅れ、段差
    timing: dict[str, float]


def state_sample(msg: Odometry, received: float) -> StateSample:
    pose = to_pose(msg)
    body_vx = msg.twist.twist.linear.x
    body_vy = msg.twist.twist.linear.y
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return StateSample(
        t=pose.t, x=pose.x, y=pose.y, yaw=pose.yaw,
        vx=c * body_vx - s * body_vy, vy=s * body_vx + c * body_vy, received=received,
    )


class Recorder(DriveNode):
    def __init__(self) -> None:
        super().__init__('check_ekf')
        reliable = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
        self._lock = threading.Lock()
        self.ekf: list[StateSample] = []
        self.dropout_ekf: list[StateSample] = []
        self.lio_map: list[StateSample] = []
        self.ndt: list[Pose] = []
        # 地図照合を止める区間（真値の時刻）。周回の区間が始まったら決める
        self.dropouts: list[tuple[float, float]] = []
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.create_subscription(Odometry, EKF_TOPIC, self._on_ekf, reliable)
        self.create_subscription(Odometry, DROPOUT_EKF_TOPIC, self._on_dropout_ekf, reliable)
        self.create_subscription(Odometry, '/localization/lio/odom', self._on_lio, reliable)
        self.create_subscription(PoseWithCovarianceStamped, NDT_TOPIC, self._on_ndt, reliable)
        self._gated = self.create_publisher(PoseWithCovarianceStamped, GATED_NDT_TOPIC, reliable)

    def _received(self) -> float:
        return self.truth[-1].t if self.truth else float('nan')

    def _on_ekf(self, msg: Odometry) -> None:
        self.ekf.append(state_sample(msg, self._received()))

    def _on_dropout_ekf(self, msg: Odometry) -> None:
        self.dropout_ekf.append(state_sample(msg, self._received()))

    def _on_ndt(self, msg: PoseWithCovarianceStamped) -> None:
        stamp = stamp_to_sec(msg.header.stamp)
        with self._lock:
            dropped = any(start <= stamp < end for start, end in self.dropouts)
        if not dropped:
            self._gated.publish(msg)
        covariance = msg.pose.covariance
        if max(covariance[0], covariance[7]) > MAX_NDT_VARIANCE_XY:
            return
        q = msg.pose.pose.orientation
        self.ndt.append(Pose(
            t=stamp, x=msg.pose.pose.position.x, y=msg.pose.pose.position.y, z=msg.pose.pose.position.z,
            yaw=math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
        ))

    def _on_lio(self, msg: Odometry) -> None:
        pose = to_pose(msg)
        try:
            transform = self._tf_buffer.lookup_transform('map', 'odom', Time())
        except tf2_ros.TransformException:
            return
        t = transform.transform.translation
        q = transform.transform.rotation
        offset_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(offset_yaw), math.sin(offset_yaw)
        yaw = wrap_angle(offset_yaw + pose.yaw)
        body_vx = msg.twist.twist.linear.x
        body_vy = msg.twist.twist.linear.y
        cy, sy = math.cos(yaw), math.sin(yaw)
        self.lio_map.append(StateSample(
            t=pose.t, x=t.x + c * pose.x - s * pose.y, y=t.y + s * pose.x + c * pose.y, yaw=yaw,
            vx=cy * body_vx - sy * body_vy, vy=sy * body_vx + cy * body_vy, received=self._received(),
        ))

    def schedule_dropouts(self, start: float) -> None:
        with self._lock:
            self.dropouts = [(start + offset, start + offset + length) for offset, length in DROPOUT_WINDOWS]


class Truth:
    """真値を時刻で引けるようにしたもの。速度は位置の差分から求める."""

    def __init__(self, truth: Sequence[Pose]) -> None:
        if len(truth) < 2:
            raise CheckError('真値がほとんどありません')
        self.t = np.array([p.t for p in truth])
        self.x = np.array([p.x for p in truth])
        self.y = np.array([p.y for p in truth])
        self.yaw = np.unwrap(np.array([p.yaw for p in truth]))
        self.vx = np.gradient(self.x, self.t)
        self.vy = np.gradient(self.y, self.t)

    def covers(self, t: float) -> bool:
        return bool(self.t[0] <= t <= self.t[-1])

    def position_error(self, t: FloatArray, x: FloatArray, y: FloatArray) -> FloatArray:
        return np.hypot(x - np.interp(t, self.t, self.x), y - np.interp(t, self.t, self.y))

    def yaw_error(self, t: FloatArray, yaw: FloatArray) -> FloatArray:
        error = yaw - np.interp(t, self.t, self.yaw)
        return np.abs(np.arctan2(np.sin(error), np.cos(error)))

    def velocity_error(self, t: FloatArray, vx: FloatArray, vy: FloatArray) -> FloatArray:
        return np.hypot(vx - np.interp(t, self.t, self.vx), vy - np.interp(t, self.t, self.vy))


@dataclass(frozen=True)
class Errors:
    t: FloatArray
    position: FloatArray
    yaw: FloatArray
    velocity: FloatArray | None


def state_errors(truth: Truth, samples: Sequence[StateSample], label: str) -> Errors:
    selected = [s for s in samples if truth.covers(s.t)]
    if len(selected) < 2:
        raise CheckError(f'真値と時刻が重なる {label} の出力がありません')
    t = np.array([s.t for s in selected])
    return Errors(
        t=t,
        position=truth.position_error(t, np.array([s.x for s in selected]), np.array([s.y for s in selected])),
        yaw=truth.yaw_error(t, np.array([s.yaw for s in selected])),
        velocity=truth.velocity_error(t, np.array([s.vx for s in selected]), np.array([s.vy for s in selected])),
    )


def pose_errors(truth: Truth, poses: Sequence[Pose], label: str) -> Errors:
    selected = [p for p in poses if truth.covers(p.t)]
    if len(selected) < 2:
        raise CheckError(f'真値と時刻が重なる {label} の出力がありません')
    t = np.array([p.t for p in selected])
    return Errors(
        t=t,
        position=truth.position_error(t, np.array([p.x for p in selected]), np.array([p.y for p in selected])),
        yaw=truth.yaw_error(t, np.array([p.yaw for p in selected])),
        velocity=None,
    )


def correction_steps(samples: Sequence[StateSample]) -> FloatArray:
    """隣り合う出力の、前の出力を速度一定で進めた位置との差（地図照合で直した段差）."""
    steps = [
        math.hypot(b.x - (a.x + a.vx * (b.t - a.t)), b.y - (a.y + a.vy * (b.t - a.t)))
        for a, b in zip(samples, samples[1:]) if b.t > a.t
    ]
    return np.array(steps)


def summarize(errors: Errors, mask: NDArray[np.bool_]) -> dict[str, float]:
    if np.count_nonzero(mask) == 0:
        return {'position_rmse_cm': float('nan'), 'position_max_cm': float('nan'), 'yaw_max_deg': float('nan')}
    summary = {
        'position_rmse_cm': rms(errors.position[mask]) * 100.0,
        'position_max_cm': float(np.max(errors.position[mask])) * 100.0,
        'yaw_max_deg': math.degrees(float(np.max(errors.yaw[mask]))),
    }
    if errors.velocity is not None:
        summary['velocity_rmse_mps'] = rms(errors.velocity[mask])
    return summary


def output_timing(samples: Sequence[StateSample], start: float, end: float) -> dict[str, float]:
    selected = [s for s in samples if start <= s.t <= end]
    if len(selected) < 2:
        raise CheckError('EKF の出力がほとんどありません')
    stamps = np.array([s.t for s in selected])
    delays = np.array([s.received - s.t for s in selected if math.isfinite(s.received)])
    steps = correction_steps(selected)
    return {
        'rate_hz': float(len(stamps) - 1) / float(stamps[-1] - stamps[0]),
        'max_gap_s': float(np.max(np.diff(stamps))),
        'delay_mean_ms': float(np.mean(delays)) * 1000.0,
        'delay_p99_ms': float(np.percentile(delays, 99.0)) * 1000.0,
        'step_p99_cm': float(np.percentile(steps, 99.0)) * 100.0 if steps.size else float('nan'),
        'step_max_cm': float(np.max(steps)) * 100.0 if steps.size else float('nan'),
    }


def dropout_summary(
    truth: Truth, samples: Sequence[StateSample], windows: Sequence[tuple[float, float]],
) -> list[dict[str, float]]:
    errors = state_errors(truth, samples, '地図照合を止めた EKF')
    results = []
    for start, end in windows:
        inside = (errors.t >= start) & (errors.t < end)
        after = errors.t >= end
        recovered = [
            float(t - end) for t, value in zip(errors.t[after], errors.position[after])
            if value <= RECOVERED_POSITION_M
        ]
        before = (errors.t >= start - 1.0) & (errors.t < start)
        results.append({
            'start_s': start,
            'length_s': end - start,
            'position_before_cm': float(np.max(errors.position[before])) * 100.0 if np.any(before) else float('nan'),
            'position_max_cm': float(np.max(errors.position[inside])) * 100.0 if np.any(inside) else float('nan'),
            'yaw_max_deg': math.degrees(float(np.max(errors.yaw[inside]))) if np.any(inside) else float('nan'),
            'recovery_s': recovered[0] if recovered else float('nan'),
        })
    return results


class DropoutEkf:
    """地図照合を区間で止めた姿勢（/pcl_pose_gated）で動く、2 つ目の EKF のプロセス."""

    def __init__(self) -> None:
        prefix = Path(get_package_prefix('core_localization'))
        executable = prefix / 'lib' / 'core_localization' / 'ekf_localizer'
        config = Path(get_package_share_directory('core_launch')) / 'config' / 'ekf_localizer.yaml'
        if not executable.is_file():
            raise CheckError(f'ekf_localizer の実行ファイルがありません: {executable}')
        self._process = subprocess.Popen([
            str(executable), '--ros-args', '-r', '__node:=ekf_localizer_dropout',
            '--params-file', str(config), '-p', 'use_sim_time:=true',
            '-p', f'pose_topic:={GATED_NDT_TOPIC}', '-p', f'output_topic:={DROPOUT_EKF_TOPIC}',
        ], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, env=os.environ.copy())

    def close(self) -> None:
        if self._process.poll() is not None:
            return
        self._process.send_signal(signal.SIGINT)
        try:
            self._process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self._process.kill()
            self._process.wait(timeout=5.0)


def print_table(
    records: Sequence[PhaseRecord], estimates: dict[str, Errors], ekf: Sequence[StateSample],
) -> dict[str, PhaseResult]:
    print(
        '区間 | 時間[s] | EKF 位置RMSE/最大[cm] | EKF yaw最大[度] | EKF 速度RMSE[m/s] | '
        'LIO+照合 位置RMSE/最大[cm] | LIO 速度RMSE[m/s] | 照合 位置RMSE/最大[cm] | EKF 周期[Hz] | 遅れ平均[ms] | 段差99%[cm]'
    )
    phases: dict[str, PhaseResult] = {}
    for record in records:
        summaries = {
            label: summarize(errors, (errors.t >= record.start) & (errors.t <= record.end))
            for label, errors in estimates.items()
        }
        timing = output_timing(ekf, record.start, record.end)
        phases[record.name] = PhaseResult(summaries, timing)
        e, lio, ndt = summaries['ekf'], summaries['lio_map'], summaries['ndt']
        print(
            f'{record.name} | {record.end - record.start:.1f} | '
            f'{e["position_rmse_cm"]:.2f}/{e["position_max_cm"]:.2f} | {e["yaw_max_deg"]:.3f} | '
            f'{e["velocity_rmse_mps"]:.3f} | '
            f'{lio["position_rmse_cm"]:.2f}/{lio["position_max_cm"]:.2f} | {lio["velocity_rmse_mps"]:.3f} | '
            f'{ndt["position_rmse_cm"]:.2f}/{ndt["position_max_cm"]:.2f} | '
            f'{timing["rate_hz"]:.1f} | {timing["delay_mean_ms"]:.1f} | {timing["step_p99_cm"]:.2f}',
            flush=True,
        )
    return phases


def run(node: Recorder, lap_seconds: float, json_path: Path | None, dropout: DropoutEkf | None) -> bool:
    node.wait_for_truth()
    wall_start = time.monotonic()
    while not node.ekf or not node.lio_map:
        if time.monotonic() - wall_start > 120.0:
            raise CheckError(f'120秒待っても {EKF_TOPIC} か LIO + 地図照合の姿勢が得られません')
        time.sleep(0.1)
    node.stop(2.0)

    records: list[PhaseRecord] = []
    for name, drive in standard_phases(node, lap_seconds):
        start = node.now_sim()
        if name == DROPOUT_PHASE and dropout is not None:
            node.schedule_dropouts(start)
        drive()
        records.append(PhaseRecord(name, start, node.now_sim()))
        print(f'区間 {name} 完了', flush=True)
    node.command_world(0.0, 0.0, 0.0)

    # 受信は別スレッドで続くので、計算の前に固定する
    truth = Truth(list(node.truth))
    ekf = list(node.ekf)
    estimates = {
        'ekf': state_errors(truth, ekf, 'EKF'),
        'lio_map': state_errors(truth, list(node.lio_map), 'LIO + 地図照合'),
        'ndt': pose_errors(truth, list(node.ndt), '地図照合'),
    }
    phases = print_table(records, estimates, ekf)

    dropouts: list[dict[str, float]] = []
    if dropout is not None:
        dropouts = dropout_summary(truth, list(node.dropout_ekf), node.dropouts)
        for result in dropouts:
            print(
                f'地図照合を {result["length_s"]:.0f} 秒止めた EKF: 直前 {result["position_before_cm"]:.1f}cm → '
                f'止めている間の最大 {result["position_max_cm"]:.1f}cm、yaw 最大 {result["yaw_max_deg"]:.2f} 度、'
                f'{RECOVERED_POSITION_M * 100.0:.0f}cm 以内に戻るまで {result["recovery_s"]:.2f} 秒',
                flush=True,
            )

    unexpected = [
        r.name for r in records
        if r.name != COLLISION_PHASE and left_safe_area(list(node.truth), r.start, r.end)
    ]
    no_collision = [
        phases[r.name].errors['ekf']['position_max_cm'] for r in records if r.name != COLLISION_PHASE
    ]
    rates = [phases[r.name].timing['rate_hz'] for r in records]
    position_ok = max(no_collision) <= TARGET_POSITION_M * 100.0
    rate_ok = min(rates) >= MIN_RATE_HZ
    print(f'[{"NG" if unexpected else "OK"}] 想定外の柵への接近: {unexpected or "なし"}')
    print(f'[{"OK" if rate_ok else "NG"}] EKF の周期: 最小 {min(rates):.1f}Hz（目標 {MIN_RATE_HZ:.0f}Hz）')
    print(
        f'[{"OK" if position_ok else "NG"}] EKF の位置誤差（衝突を除く）: 最大 {max(no_collision):.1f}cm'
        f'（目標 {TARGET_POSITION_M * 100.0:.0f}cm）'
    )
    if json_path is not None:
        json_path.write_text(json.dumps({
            'phases': {name: asdict(result) for name, result in phases.items()}, 'dropouts': dropouts,
        }, ensure_ascii=False, indent=1))
    return position_ok and rate_ok and not unexpected


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--lap-seconds', type=float, default=120.0, help='周回区間の長さ [s]')
    parser.add_argument('--json', type=Path, default=None, help='結果を書き出すファイル')
    parser.add_argument(
        '--dropout', action=argparse.BooleanOptionalAction, default=True,
        help='地図照合を区間で止めた 2 つ目の EKF も動かす',
    )
    args = parser.parse_args()
    if args.dropout and args.lap_seconds < max(start + length for start, length in DROPOUT_WINDOWS) + 10.0:
        parser.error('--dropout には、地図照合を止める区間と戻りを含む長さの --lap-seconds が要ります')
    dropout = DropoutEkf() if args.dropout else None
    try:
        return run_with_node(Recorder, lambda node: run(node, args.lap_seconds, args.json, dropout))
    finally:
        if dropout is not None:
            dropout.close()


if __name__ == '__main__':
    sys.exit(main())
