#!/usr/bin/env python3
"""FAST-LIO の自己位置を真値と比べる（手順2の完了条件）.

sim_lio.launch.py を起動した状態で実行する。機体を動かすのに真値を使い、
/lio/odom の誤差を区間ごとに表示する。位置は開始時の 1 回だけ真値に合わせる。

終了コードは、10Hz で出続けて発散しなければ 0、そうでなければ 1。
目安を超えただけでは失敗にせず、表に示す。
"""

import argparse
from dataclasses import dataclass
import math
import os
from pathlib import Path
import sys
import threading
import time
from typing import Callable, Final, Sequence

from geometry_msgs.msg import Twist
from core_nav_check.common import find_process as find_process_in_domain
from nav_msgs.msg import Odometry
import numpy as np
from numpy.typing import NDArray
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

FloatArray = NDArray[np.float64]

LIO_TOPIC: Final = '/localization/lio/odom'
TRUTH_TOPIC: Final = '/ground_truth/odom'
CMD_TOPIC: Final = '/control/cmd_vel'
LIO_PROCESS: Final = 'fastlio_mapping'

# 発散とみなす誤差と、出続けているとみなす周期
DIVERGED_POSITION_M: Final = 1.0
MIN_RATE_HZ: Final = 9.5
# 目安。超えても失敗にはしない
TARGET_POSITION_RMSE_M: Final = 0.10
TARGET_YAW_RMSE_DEG: Final = 1.0

# 機体の加速度の上限（5m/s2）より少し小さくして、目標の手前で確実に減速する
BRAKE_ACCEL_MPS2: Final = 4.0
ARRIVAL_RADIUS_M: Final = 0.03
COMMAND_PERIOD_S: Final = 0.02
WALL_TIMEOUT_S: Final = 120.0

COLLISION_PHASE: Final = '柵への衝突 3m/s'
# 機体の角までの距離（約 0.57m）を柵の位置から引いた範囲。ここを出たら柵に当たった可能性がある
SAFE_X_M: Final = 11.4
SAFE_Y_M: Final = 6.9


class CheckError(RuntimeError):
    """確認の続行に必要なデータが得られないときの例外."""


@dataclass(frozen=True)
class Pose:
    t: float
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class LioSample:
    pose: Pose
    # 受信した時点の真値の時刻。スタンプとの差が出力の遅れになる
    received_at: float


@dataclass(frozen=True)
class ProcessUsage:
    wall_s: float
    cpu_s: float
    rss_mib: float


@dataclass(frozen=True)
class PhaseRecord:
    name: str
    start: float
    end: float
    usage_start: ProcessUsage
    usage_end: ProcessUsage


@dataclass(frozen=True)
class PhaseResult:
    name: str
    duration_s: float
    position_rmse_m: float
    position_max_m: float
    z_max_m: float
    yaw_rmse_deg: float
    yaw_max_deg: float
    rate_hz: float
    max_gap_s: float
    latency_ms: float
    cpu_percent: float
    rss_mib: float


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def to_pose(msg: Odometry) -> Pose:
    q = msg.pose.pose.orientation
    p = msg.pose.pose.position
    return Pose(
        t=float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
        x=p.x,
        y=p.y,
        z=p.z,
        yaw=math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
    )


def find_process(name: str) -> int:
    """同じ ROS_DOMAIN_ID で動くプロセスの PID を返す."""
    pid = find_process_in_domain(name)
    if pid is None:
        raise CheckError(f'プロセス {name} が見つかりません')
    return pid


def read_usage(pid: int) -> ProcessUsage:
    """プロセスの累積 CPU 時間と使用メモリを読む."""
    try:
        # comm に空白が含まれても崩れないよう、閉じ括弧より後ろを使う
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        status = Path(f'/proc/{pid}/status').read_text()
    except OSError as error:
        raise CheckError(f'プロセス {pid} の情報を読めません: {error}') from error
    ticks = float(fields[11]) + float(fields[12])  # utime, stime
    rss_kib = next(float(line.split()[1]) for line in status.splitlines() if line.startswith('VmRSS'))
    return ProcessUsage(time.monotonic(), ticks / os.sysconf('SC_CLK_TCK'), rss_kib / 1024.0)


class Recorder(Node):
    """真値と LIO の出力を記録し、速度指令を出すノード."""

    def __init__(self) -> None:
        super().__init__('check_lio')
        self.truth: list[Pose] = []
        self.lio: list[LioSample] = []
        self.frames: set[tuple[str, str]] = set()
        self._cmd = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.create_subscription(
            Odometry, TRUTH_TOPIC, self._on_truth,
            QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_subscription(
            Odometry, LIO_TOPIC, self._on_lio,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
        )

    def _on_truth(self, msg: Odometry) -> None:
        self.truth.append(to_pose(msg))

    def _on_lio(self, msg: Odometry) -> None:
        self.frames.add((msg.header.frame_id, msg.child_frame_id))
        received_at = self.truth[-1].t if self.truth else float('nan')
        self.lio.append(LioSample(to_pose(msg), received_at))

    def now_sim(self) -> float:
        if not self.truth:
            raise CheckError(f'{TRUTH_TOPIC} を受信していません')
        return self.truth[-1].t

    def command_world(self, vx: float, vy: float, wz: float) -> None:
        """世界座標の速度を、現在の真値の向きで機体座標の指令に直して出す."""
        yaw = self.truth[-1].yaw
        msg = Twist()
        msg.linear.x = math.cos(yaw) * vx + math.sin(yaw) * vy
        msg.linear.y = -math.sin(yaw) * vx + math.cos(yaw) * vy
        msg.angular.z = wz
        self._cmd.publish(msg)

    def hold(self, vx: float, vy: float, wz: float, duration_s: float) -> None:
        start = self.now_sim()
        wall_start = time.monotonic()
        while self.now_sim() - start < duration_s:
            if time.monotonic() - wall_start > WALL_TIMEOUT_S:
                raise CheckError(f'{duration_s}秒待つ間にシミュレーション時間が進みませんでした')
            self.command_world(vx, vy, wz)
            time.sleep(COMMAND_PERIOD_S)

    def go_to(self, x: float, y: float, speed: float, wz: float = 0.0) -> None:
        """目標まで走り、手前で減速して止まる."""
        wall_start = time.monotonic()
        while True:
            now = self.truth[-1]
            dx, dy = x - now.x, y - now.y
            distance = math.hypot(dx, dy)
            if distance < ARRIVAL_RADIUS_M:
                return
            if time.monotonic() - wall_start > WALL_TIMEOUT_S:
                raise CheckError(f'目標 ({x}, {y}) に到達できません（現在 ({now.x:.2f}, {now.y:.2f})）')
            v = min(speed, math.sqrt(2.0 * BRAKE_ACCEL_MPS2 * distance))
            self.command_world(v * dx / distance, v * dy / distance, wz)
            time.sleep(COMMAND_PERIOD_S)

    def stop(self, duration_s: float = 1.0) -> None:
        self.hold(0.0, 0.0, 0.0, duration_s)


def wait_for_lio(recorder: Recorder) -> None:
    wall_start = time.monotonic()
    while not recorder.lio or not recorder.truth:
        if time.monotonic() - wall_start > WALL_TIMEOUT_S:
            raise CheckError(f'{WALL_TIMEOUT_S}秒待っても {LIO_TOPIC} を受信できません')
        time.sleep(0.1)


def phase_drives(recorder: Recorder, lap_seconds: float) -> list[tuple[str, Callable[[], None]]]:
    """区間の名前と、その区間の動きを返す."""

    def straight(speed: float) -> Callable[[], None]:
        def run() -> None:
            recorder.go_to(10.0, -2.5, speed)
            recorder.stop()
            recorder.go_to(-10.0, -2.5, speed)
            recorder.stop()
        return run

    def spin(rate: float) -> Callable[[], None]:
        # 2 回転して止まる
        return lambda: recorder.hold(0.0, 0.0, rate, 4.0 * math.pi / rate)

    def translate_and_spin() -> None:
        for x in (10.0, -10.0, 0.0):
            recorder.go_to(x, 0.0, 2.0, 1.0)
        recorder.stop()

    def stop_and_go() -> None:
        # 1 サイクルで約 3m 進むので、柵に当たらないよう端から始める
        recorder.go_to(-9.0, 0.0, 2.0)
        recorder.stop()
        for direction in (1.0, -1.0):
            for _ in range(6):
                recorder.hold(direction * 3.0, 0.0, 0.0, 1.0)
                recorder.stop(1.0)

    def fence_collision() -> None:
        # 実機でも起こりうる衝突の後に、自己位置が戻るかを見る。約 3.9 秒で柵に当たる
        recorder.go_to(0.0, 0.0, 2.0)
        recorder.stop()
        recorder.hold(3.0, 0.0, 0.0, 5.0)
        recorder.stop()
        recorder.go_to(0.0, 0.0, 2.0)
        recorder.stop()

    def laps() -> None:
        corners = ((10.0, -5.5), (10.0, 5.5), (-10.0, 5.5), (-10.0, -5.5))
        start = recorder.now_sim()
        while recorder.now_sim() - start < lap_seconds:
            for x, y in corners:
                recorder.go_to(x, y, 3.0, 0.3)
        recorder.stop()

    return [
        ('静止', lambda: recorder.stop(20.0)),
        ('直進 1m/s', straight(1.0)),
        ('直進 2m/s', straight(2.0)),
        ('直進 3m/s', straight(3.0)),
        ('その場旋回 1rad/s', spin(1.0)),
        ('その場旋回 2rad/s', spin(2.0)),
        ('並進と旋回の同時動作', translate_and_spin),
        ('急な加減速', stop_and_go),
        (COLLISION_PHASE, fence_collision),
        ('周回', laps),
        ('停止後', lambda: recorder.stop(5.0)),
    ]


def interpolate(truth_t: FloatArray, values: FloatArray, t: FloatArray) -> FloatArray:
    return np.interp(t, truth_t, values)


def compute_errors(
    truth: Sequence[Pose], lio: Sequence[LioSample]
) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray]:
    """最初の LIO のサンプルで真値に合わせ、その後の誤差を返す.

    戻り値は (時刻, 水平の位置誤差, 高さの誤差, yaw 誤差[rad])。
    """
    truth_t = np.array([p.t for p in truth])
    truth_x = np.array([p.x for p in truth])
    truth_y = np.array([p.y for p in truth])
    truth_z = np.array([p.z for p in truth])
    truth_yaw = np.unwrap(np.array([p.yaw for p in truth]))

    samples = [s.pose for s in lio if truth_t[0] <= s.pose.t <= truth_t[-1]]
    if len(samples) < 2:
        raise CheckError('真値と時刻が重なる LIO のサンプルがありません')
    t = np.array([p.t for p in samples])
    x = np.array([p.x for p in samples])
    y = np.array([p.y for p in samples])
    z = np.array([p.z for p in samples])
    yaw = np.unwrap(np.array([p.yaw for p in samples]))

    expected_x = interpolate(truth_t, truth_x, t)
    expected_y = interpolate(truth_t, truth_y, t)
    expected_z = interpolate(truth_t, truth_z, t)
    expected_yaw = interpolate(truth_t, truth_yaw, t)

    offset_yaw = float(expected_yaw[0] - yaw[0])
    c, s = math.cos(offset_yaw), math.sin(offset_yaw)
    aligned_x = c * (x - x[0]) - s * (y - y[0]) + expected_x[0]
    aligned_y = s * (x - x[0]) + c * (y - y[0]) + expected_y[0]
    aligned_z = z - z[0] + expected_z[0]
    yaw_error = np.arctan2(
        np.sin(yaw + offset_yaw - expected_yaw), np.cos(yaw + offset_yaw - expected_yaw)
    )
    return (
        t,
        np.hypot(aligned_x - expected_x, aligned_y - expected_y),
        np.abs(aligned_z - expected_z),
        yaw_error,
    )


def left_safe_area(truth: Sequence[Pose], record: PhaseRecord) -> bool:
    return any(
        abs(p.x) > SAFE_X_M or abs(p.y) > SAFE_Y_M
        for p in truth
        if record.start <= p.t <= record.end
    )


def rms(values: FloatArray) -> float:
    return float(np.sqrt(np.mean(np.square(values))))


def summarize(
    record: PhaseRecord,
    errors: tuple[FloatArray, FloatArray, FloatArray, FloatArray],
    lio: Sequence[LioSample],
) -> PhaseResult:
    t, position, height, yaw = errors
    mask = (t >= record.start) & (t <= record.end)
    if np.count_nonzero(mask) < 2:
        raise CheckError(f'区間 {record.name} に LIO の出力がほとんどありません')
    stamps = t[mask]
    latencies = [
        s.received_at - s.pose.t
        for s in lio
        if record.start <= s.pose.t <= record.end and not math.isnan(s.received_at)
    ]
    wall = record.usage_end.wall_s - record.usage_start.wall_s
    return PhaseResult(
        name=record.name,
        duration_s=record.end - record.start,
        position_rmse_m=rms(position[mask]),
        position_max_m=float(np.max(position[mask])),
        z_max_m=float(np.max(height[mask])),
        yaw_rmse_deg=math.degrees(rms(yaw[mask])),
        yaw_max_deg=math.degrees(float(np.max(np.abs(yaw[mask])))),
        rate_hz=float(len(stamps) - 1) / float(stamps[-1] - stamps[0]),
        max_gap_s=float(np.max(np.diff(stamps))),
        latency_ms=float(np.mean(latencies)) * 1000.0,
        cpu_percent=100.0 * (record.usage_end.cpu_s - record.usage_start.cpu_s) / wall,
        rss_mib=record.usage_end.rss_mib,
    )


def print_table(results: Sequence[PhaseResult]) -> None:
    print(
        '区間 | 時間[s] | 位置RMSE[cm] | 位置最大[cm] | 高さ最大[cm] | yawRMSE[度] | yaw最大[度] | '
        '周期[Hz] | 最大間隔[s] | 遅れ[ms] | CPU[%] | RSS[MiB]'
    )
    for r in results:
        print(
            f'{r.name} | {r.duration_s:.1f} | {r.position_rmse_m * 100.0:.2f} | '
            f'{r.position_max_m * 100.0:.2f} | {r.z_max_m * 100.0:.2f} | {r.yaw_rmse_deg:.3f} | '
            f'{r.yaw_max_deg:.3f} | {r.rate_hz:.2f} | {r.max_gap_s:.2f} | {r.latency_ms:.1f} | '
            f'{r.cpu_percent:.0f} | {r.rss_mib:.0f}',
            flush=True,
        )


def save_csv(path: Path, errors: tuple[FloatArray, FloatArray, FloatArray, FloatArray]) -> None:
    t, position, height, yaw = errors
    np.savetxt(
        path,
        np.column_stack((t - t[0], position, height, np.degrees(yaw))),
        delimiter=',',
        header='time_s,position_error_m,height_error_m,yaw_error_deg',
        comments='',
    )


def run(recorder: Recorder, lap_seconds: float, csv_path: Path | None) -> bool:
    wait_for_lio(recorder)
    pid = find_process(LIO_PROCESS)
    # 起動直後の IMU の初期化が終わるまで静止したまま待つ
    recorder.stop(2.0)

    records: list[PhaseRecord] = []
    for name, drive in phase_drives(recorder, lap_seconds):
        usage_start = read_usage(pid)
        start = recorder.now_sim()
        drive()
        records.append(PhaseRecord(name, start, recorder.now_sim(), usage_start, read_usage(pid)))
        print(f'区間 {name} 完了', flush=True)
    recorder.command_world(0.0, 0.0, 0.0)

    # 受信は別スレッドで続くので、計算の前に固定する
    truth, lio = list(recorder.truth), list(recorder.lio)
    errors = compute_errors(truth, lio)
    results = [summarize(record, errors, lio) for record in records]
    print_table(results)
    if csv_path is not None:
        save_csv(csv_path, errors)

    wall_elapsed = records[-1].usage_end.wall_s - records[0].usage_start.wall_s
    print(f'実時間比: {sum(r.duration_s for r in results) / wall_elapsed:.2f}')

    # 走らせ方を誤って柵に当てていると、誤差の原因を取り違える
    unexpected_contact = [
        r.name for r in records if r.name != COLLISION_PHASE and left_safe_area(truth, r)
    ]
    print(f'[{"NG" if unexpected_contact else "OK"}] 想定外の柵への接近: {unexpected_contact or "なし"}')

    frames = sorted(recorder.frames)
    final = results[-1]
    frames_ok = frames == [('odom', 'base_link')]
    rate_ok = all(r.rate_hz >= MIN_RATE_HZ for r in results)
    converged = all(r.position_max_m <= DIVERGED_POSITION_M for r in results)
    print(f'[{"OK" if frames_ok else "NG"}] フレーム: {frames}')
    print(f'[{"OK" if rate_ok else "NG"}] 周期: 最小 {min(r.rate_hz for r in results):.2f}Hz')
    print(
        f'[{"OK" if converged else "NG"}] 発散: 位置誤差 最大 '
        f'{max(r.position_max_m for r in results) * 100.0:.1f}cm'
    )
    print(
        f'停止後の誤差: 位置 {final.position_max_m * 100.0:.2f}cm、yaw {final.yaw_max_deg:.3f}度、'
        f'累積の走行時間 {sum(r.duration_s for r in results):.0f}秒'
    )
    over_target = [
        r.name
        for r in results
        if r.position_rmse_m > TARGET_POSITION_RMSE_M or r.yaw_rmse_deg > TARGET_YAW_RMSE_DEG
    ]
    print(f'目安（位置RMSE {TARGET_POSITION_RMSE_M * 100.0:.0f}cm、yawRMSE {TARGET_YAW_RMSE_DEG}度）を超えた区間: {over_target or "なし"}')
    return frames_ok and rate_ok and converged and not unexpected_contact


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lap-seconds', type=float, default=300.0, help='周回区間の長さ [s]')
    parser.add_argument('--csv', type=Path, default=None, help='時刻ごとの誤差を書き出すファイル')
    args = parser.parse_args()

    rclpy.init()
    recorder = Recorder()
    executor = SingleThreadedExecutor()
    executor.add_node(recorder)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        ok = run(recorder, args.lap_seconds, args.csv)
    except CheckError as error:
        print(f'[NG] 確認を中断しました: {error}', file=sys.stderr)
        ok = False
    finally:
        # 異常終了しても機体を走らせたままにしない。シグナルで止められたときは送れない
        if recorder.truth and rclpy.ok():
            recorder.command_world(0.0, 0.0, 0.0)
            time.sleep(0.1)
        executor.shutdown()
        spinner.join(timeout=5.0)
        recorder.destroy_node()
        rclpy.try_shutdown()
    print('結果: 完了条件を満たしました' if ok else '結果: 完了条件を満たしていません', flush=True)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
