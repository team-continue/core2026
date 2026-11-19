#!/usr/bin/env python3
"""地図照合の結果（map→base_link）を真値と比べる（手順3の完了条件）.

sim_localization.launch.py を起動した状態で実行する。シミュレーションでは map と world が一致する。
LIO 単体の誤差も同時に測る。

終了コードは、全区間で位置誤差が目標以内で、出力が途切れなければ 0。
"""

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import time
from typing import Final, Sequence

from core_nav_check.common import (
    CheckError,
    COLLISION_PHASE,
    cpu_percent,
    DriveNode,
    find_stack_processes,
    FloatArray,
    left_safe_area,
    missing_processes,
    Pose,
    ProcessUsage,
    read_usage,
    rms,
    run_with_node,
    standard_phases,
    to_pose,
    wrap_angle,
)
from diagnostic_msgs.msg import DiagnosticArray
from core_msgs.msg import ProcessStats
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
import tf2_ros

TARGET_POSITION_M: Final = 0.10
MIN_RATE_HZ: Final = 9.5
# 測るノード（別のプロセスで動かすときの実行ファイル）。composition:=true でコンテナに入るノードは、コンテナの
# プロセスで測る。composition の有無で合計を比べられるよう、コンテナに入るノードはすべて含める
PROCESSES: Final = (
    'points_to_livox', 'fastlio_mapping', 'lio_odom_adapter', 'scan_filter', 'lidar_localization_node',
    'localization_health', 'livox_filter',
)
# この 2 つ（またはそのコンテナ）がないと確認できない
REQUIRED_PROCESSES: Final = ('fastlio_mapping', 'lidar_localization_node')


@dataclass(frozen=True)
class MapSample:
    t: float
    x: float
    y: float
    z: float
    yaw: float
    map_to_odom: tuple[float, float, float]


@dataclass(frozen=True)
class PhaseRecord:
    name: str
    start: float
    end: float
    usage_start: dict[str, ProcessUsage]
    usage_end: dict[str, ProcessUsage]


class Recorder(DriveNode):
    def __init__(self) -> None:
        super().__init__('check_localization')
        self.samples: list[MapSample] = []
        self.lio: list[Pose] = []
        self.filter_ms: dict[str, list[float]] = {'scan_filter': [], 'livox_filter': []}
        self.filter_points: dict[str, list[int]] = {'scan_filter': [], 'livox_filter': []}
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.create_subscription(
            Odometry, '/localization/lio/odom', self._on_lio,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
        )
        for name in self.filter_ms:
            self.create_subscription(
                ProcessStats, f'/{name}/stats', self._stats_callback(name), 50,
            )
        self.alignment_ms: list[float] = []
        self.create_subscription(DiagnosticArray, '/localization/alignment_status', self._on_alignment, 50)

    def _stats_callback(self, name: str):  # noqa: ANN202
        def callback(msg: ProcessStats) -> None:
            self.filter_ms[name].append(msg.processing_ms)
            self.filter_points[name].append(msg.output_points)
        return callback

    def _on_alignment(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            for value in status.values:
                if value.key == 'alignment_time_sec':
                    self.alignment_ms.append(float(value.value) * 1000.0)

    def _on_lio(self, msg: Odometry) -> None:
        pose = to_pose(msg)
        self.lio.append(pose)
        try:
            transform = self._tf_buffer.lookup_transform('map', 'odom', Time())
        except tf2_ros.TransformException:
            return
        t = transform.transform.translation
        q = transform.transform.rotation
        offset_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(offset_yaw), math.sin(offset_yaw)
        self.samples.append(MapSample(
            t=pose.t,
            x=t.x + c * pose.x - s * pose.y,
            y=t.y + s * pose.x + c * pose.y,
            z=t.z + pose.z,
            yaw=wrap_angle(offset_yaw + pose.yaw),
            map_to_odom=(t.x, t.y, offset_yaw),
        ))


def truth_arrays(truth: Sequence[Pose]) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray, FloatArray]:
    return (
        np.array([p.t for p in truth]),
        np.array([p.x for p in truth]),
        np.array([p.y for p in truth]),
        np.array([p.z for p in truth]),
        np.unwrap(np.array([p.yaw for p in truth])),
    )


def map_errors(
    truth: Sequence[Pose], samples: Sequence[MapSample]
) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray]:
    """(時刻, 水平の位置誤差, 高さの誤差, yaw 誤差[rad]) を返す."""
    truth_t, truth_x, truth_y, truth_z, truth_yaw = truth_arrays(truth)
    selected = [s for s in samples if truth_t[0] <= s.t <= truth_t[-1]]
    if len(selected) < 2:
        raise CheckError('真値と時刻が重なる map→base_link のサンプルがありません')
    t = np.array([s.t for s in selected])
    yaw_error = np.array([s.yaw for s in selected]) - np.interp(t, truth_t, truth_yaw)
    return (
        t,
        np.hypot(
            np.array([s.x for s in selected]) - np.interp(t, truth_t, truth_x),
            np.array([s.y for s in selected]) - np.interp(t, truth_t, truth_y),
        ),
        np.abs(np.array([s.z for s in selected]) - np.interp(t, truth_t, truth_z)),
        np.arctan2(np.sin(yaw_error), np.cos(yaw_error)),
    )


def lio_position_errors(
    truth: Sequence[Pose], lio: Sequence[Pose]
) -> tuple[FloatArray, FloatArray]:
    """最初のサンプルで真値に合わせた LIO 単体の (時刻, 水平の位置誤差) を返す."""
    truth_t, truth_x, truth_y, _, truth_yaw = truth_arrays(truth)
    selected = [p for p in lio if truth_t[0] <= p.t <= truth_t[-1]]
    t = np.array([p.t for p in selected])
    x = np.array([p.x for p in selected])
    y = np.array([p.y for p in selected])
    expected_x = np.interp(t, truth_t, truth_x)
    expected_y = np.interp(t, truth_t, truth_y)
    offset = float(np.interp(t[0], truth_t, truth_yaw) - selected[0].yaw)
    c, s = math.cos(offset), math.sin(offset)
    aligned_x = c * (x - x[0]) - s * (y - y[0]) + expected_x[0]
    aligned_y = s * (x - x[0]) + c * (y - y[0]) + expected_y[0]
    return t, np.hypot(aligned_x - expected_x, aligned_y - expected_y)


def odom_step_cm(samples: Sequence[MapSample], start: float, end: float) -> float:
    """map→odom が変わったときの、機体の地図上の位置の跳び（周期ごと）の 99% 点を返す.

    map→odom の並進そのものは odom の原点を中心に回るので、原点から遠いと小さな向きの補正でも大きく変わる。
    そこで、各周期の機体の位置を 1 つ前の map→odom でも写し、今の map→odom で写した位置との差を跳びとする。
    """
    selected = [s for s in samples if start <= s.t <= end]
    if len(selected) < 2:
        return float('nan')
    steps = []
    for previous, current in zip(selected, selected[1:]):
        px, py, pyaw = previous.map_to_odom
        cx, cy, cyaw = current.map_to_odom
        # 今の地図上の位置を odom の座標に戻し、1 つ前の map→odom で写し直す
        dx, dy = current.x - cx, current.y - cy
        ox = math.cos(-cyaw) * dx - math.sin(-cyaw) * dy
        oy = math.sin(-cyaw) * dx + math.cos(-cyaw) * dy
        before_x = px + math.cos(pyaw) * ox - math.sin(pyaw) * oy
        before_y = py + math.sin(pyaw) * ox + math.cos(pyaw) * oy
        steps.append(math.hypot(current.x - before_x, current.y - before_y))
    return float(np.percentile(steps, 99.0)) * 100.0


def run(node: Recorder, lap_seconds: float, json_path: Path | None) -> bool:
    node.wait_for_truth()
    wall_start = time.monotonic()
    while not node.samples:
        if time.monotonic() - wall_start > 120.0:
            raise CheckError('120秒待っても map→odom が得られません')
        time.sleep(0.1)
    active = find_stack_processes(PROCESSES)
    missing = missing_processes(REQUIRED_PROCESSES, active)
    if missing:
        raise CheckError(f'必要なプロセス {", ".join(missing)} が動いていません（見つかったもの: {active}）')
    node.stop(2.0)

    records: list[PhaseRecord] = []
    for name, drive in standard_phases(node, lap_seconds):
        usage_start = {n: read_usage(pid) for n, pid in active.items()}
        start = node.now_sim()
        drive()
        records.append(PhaseRecord(
            name, start, node.now_sim(), usage_start,
            {n: read_usage(pid) for n, pid in active.items()},
        ))
        print(f'区間 {name} 完了', flush=True)
    node.command_world(0.0, 0.0, 0.0)

    # 受信は別スレッドで続くので、計算の前に固定する
    truth = list(node.truth)
    samples = list(node.samples)
    lio = list(node.lio)
    t, position, height, yaw = map_errors(truth, samples)
    lio_t, lio_position = lio_position_errors(truth, lio)

    print('区間 | 時間[s] | 位置RMSE[cm] | 位置最大[cm] | 高さ最大[cm] | yaw最大[度] | LIO位置最大[cm] | map→odom変化99%[cm] | 周期[Hz]')
    rates = []
    max_positions = []
    for record in records:
        mask = (t >= record.start) & (t <= record.end)
        lio_mask = (lio_t >= record.start) & (lio_t <= record.end)
        if np.count_nonzero(mask) < 2:
            raise CheckError(f'区間 {record.name} に map→base_link の出力がほとんどありません')
        stamps = t[mask]
        rate = float(len(stamps) - 1) / float(stamps[-1] - stamps[0])
        rates.append(rate)
        max_positions.append((record.name, float(np.max(position[mask]))))
        print(
            f'{record.name} | {record.end - record.start:.1f} | {rms(position[mask]) * 100.0:.2f} | '
            f'{np.max(position[mask]) * 100.0:.2f} | {np.max(height[mask]) * 100.0:.2f} | '
            f'{math.degrees(float(np.max(np.abs(yaw[mask])))):.3f} | '
            f'{np.max(lio_position[lio_mask]) * 100.0:.2f} | '
            f'{odom_step_cm(samples, record.start, record.end):.2f} | {rate:.2f}',
            flush=True,
        )

    first, last = records[0], records[-1]
    cpu = {
        name: cpu_percent(first.usage_start[name], last.usage_end[name]) for name in active
    }
    rss = {name: last.usage_end[name].rss_mib for name in active}
    print('CPU[%]: ' + ', '.join(f'{name} {value:.0f}' for name, value in cpu.items()))
    print('RSS[MiB]: ' + ', '.join(f'{name} {value:.0f}' for name, value in rss.items()))
    print(f'合計CPU[%]: {sum(cpu.values()):.0f}')
    filter_summary = {
        name: {
            'points': float(np.mean(node.filter_points[name])),
            'mean_ms': float(np.mean(node.filter_ms[name])),
            'p99_ms': float(np.percentile(node.filter_ms[name], 99.0)),
        }
        for name in node.filter_ms if node.filter_ms[name]
    }
    print(f'フィルタ: {filter_summary}')
    alignment_mean = float(np.mean(node.alignment_ms)) if node.alignment_ms else float('nan')
    alignment_p99 = float(np.percentile(node.alignment_ms, 99.0)) if node.alignment_ms else float('nan')
    print(f'地図照合の時間[ms]: 平均 {alignment_mean:.1f}、99%点 {alignment_p99:.1f}')

    no_collision = [(name, value) for name, value in max_positions if name != COLLISION_PHASE]
    unexpected = [
        r.name for r in records
        if r.name != COLLISION_PHASE and left_safe_area(truth, r.start, r.end)
    ]
    rate_ok = min(rates) >= MIN_RATE_HZ
    position_ok = all(value <= TARGET_POSITION_M for _, value in no_collision)
    print(f'[{"NG" if unexpected else "OK"}] 想定外の柵への接近: {unexpected or "なし"}')
    print(f'[{"OK" if rate_ok else "NG"}] 周期: 最小 {min(rates):.2f}Hz')
    print(
        f'[{"OK" if position_ok else "NG"}] 位置誤差（衝突を除く）: 最大 '
        f'{max(value for _, value in no_collision) * 100.0:.1f}cm（目標 {TARGET_POSITION_M * 100.0:.0f}cm）'
    )
    if json_path is not None:
        overall_mask = np.ones_like(t, dtype=bool)
        for record in records:
            if record.name == COLLISION_PHASE:
                overall_mask &= ~((t >= record.start) & (t <= record.end))
        json_path.write_text(json.dumps({
            'position_rmse_cm': rms(position[overall_mask]) * 100.0,
            'position_max_cm': float(np.max(position[overall_mask])) * 100.0,
            'height_max_cm': float(np.max(height[overall_mask])) * 100.0,
            'yaw_max_deg': math.degrees(float(np.max(np.abs(yaw[overall_mask])))),
            'lio_position_max_cm': float(np.max(lio_position)) * 100.0,
            'odom_step_p99_cm': odom_step_cm(samples, first.start, last.end),
            'min_rate_hz': min(rates),
            'cpu_percent': cpu,
            'cpu_total_percent': sum(cpu.values()),
            'rss_mib': rss,
            'filter': filter_summary,
            'alignment_mean_ms': alignment_mean,
            'alignment_p99_ms': alignment_p99,
        }, ensure_ascii=False, indent=1))
    return rate_ok and position_ok and not unexpected


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lap-seconds', type=float, default=300.0, help='周回区間の長さ [s]')
    parser.add_argument('--json', type=Path, default=None, help='結果を書き出すファイル')
    args = parser.parse_args()
    return run_with_node(Recorder, lambda node: run(node, args.lap_seconds, args.json))


if __name__ == '__main__':
    sys.exit(main())
