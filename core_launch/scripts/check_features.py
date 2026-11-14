#!/usr/bin/env python3
"""混雑環境向けの 13 機能を確かめる（手順6の完了条件）.

機能ごとに次を短いシナリオで確かめて、表にする。
- enabled（または無効に当たる値）を切り替えられる
- 調整値を変えると挙動が変わる
- 範囲外の値が拒否される（読み取り専用の値の変更も拒否される）
- 安全系を無効にすると警告を出し続ける

--only NAME で 1 機能だけをこのプロセスで実行する。指定しなければ、機能ごとに子プロセスを起動し、
ROS_DOMAIN_ID と GZ_PARTITION を分けて --jobs N 個まで同時に走らせる。
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import threading
import time
from typing import Callable, Final, Iterator, Mapping, Sequence

from ament_index_python.packages import get_package_share_directory
from core_nav_check.common import CheckError, find_process, SpinThread
from core_nav_check.scenario import footprint_gap, Obstacle, state_at
from core_msgs.msg import (
    LocalizationHealth,
    NavigationStatus,
    ProcessStats,
    SafetyStatus,
    TrackedObject,
    TrackedObjectArray,
)
import numpy as np
import rclpy
from run_scenario import (
    analyze_obstacles,
    drive_goal,
    load_spec,
    log_tail,
    OWN_HALF_M,
    ParamValue,
    ResultDict,
    ScenarioNode,
    ScenarioSpec,
    start_stack,
    stop_process,
    wait_until_ready,
    write_temp_scenario,
)
from sensor_msgs.msg import Imu

CHILD_TIMEOUT_S: Final = 1200.0
BASE_DOMAIN: Final = 50
# 警告は 5 秒に 1 回出るので、2 回以上出ているかをこの時間で見る
WARNING_WATCH_S: Final = 11.0
# 直接指令で走るときの速度。速度の差で接触と誤検知しないよう控えめにする
DIRECT_SPEED_MPS: Final = 1.5


@dataclass(frozen=True)
class Row:
    label: str
    value: str
    ok: bool


@dataclass(frozen=True)
class TrackFrame:
    t: float
    objects: tuple[TrackedObject, ...]


class FeatureNode(ScenarioNode):
    """機能の確認に必要な記録を足したノード."""

    def __init__(self) -> None:
        super().__init__('check_features')
        self.frames: list[TrackFrame] = []
        self.perception_points: list[tuple[float, int]] = []
        self.create_subscription(TrackedObjectArray, '/perception/tracks', self._on_tracks, 50)
        self.create_subscription(ProcessStats, '/perception/stats', self._on_points, 50)
        self.fake_damiao = self.create_publisher(Imu, '/sensing/imu', 50)

    def _on_tracks(self, msg: TrackedObjectArray) -> None:
        self.frames.append(TrackFrame(self.received_at(), tuple(msg.objects)))

    def _on_points(self, msg: ProcessStats) -> None:
        self.perception_points.append((self.received_at(), msg.output_points))

    def current_speed(self, window_s: float = 0.1) -> float:
        """真値の直近の位置の変化から速さを求める."""
        now = self.truth[-1]
        recent = next((p for p in reversed(self.truth) if now.t - p.t >= window_s), self.truth[0])
        elapsed = now.t - recent.t
        return math.hypot(now.x - recent.x, now.y - recent.y) / elapsed if elapsed > 0.0 else 0.0


class Session:
    """シナリオを起動して、終了時に止める."""

    def __init__(
        self, name: str, scenario: Mapping[str, object] | Path, log_dir: Path,
        launch_args: Sequence[str] = (),
    ) -> None:
        self.path = scenario if isinstance(scenario, Path) else write_temp_scenario(scenario, name)
        self.spec: ScenarioSpec = load_spec(self.path)
        self.log_path = log_dir / f'{name}.launch.log'
        self._launch_args = launch_args
        self.process: subprocess.Popen[bytes] | None = None
        self.node = FeatureNode()
        self._spinner = SpinThread(self.node)

    def __enter__(self) -> 'Session':
        # paused() はノードを 1 つずつ止めて試すので、ノードは別々のプロセスで動かす（composition のコンテナに
        # まとめると、同じコンテナのほかのノードまで止まる）。指定より優先するよう最後に置く
        self.process = start_stack(
            self.spec, self.path, (*self._launch_args, 'composition=false'), self.log_path,
        )
        self._spinner.__enter__()
        try:
            wait_until_ready(self.node, self.process, self.log_path)
            # 自己位置と追跡が落ち着くまで待つ
            self.node.wait_sim(2.0)
        except BaseException:
            self.__exit__()
            raise
        return self

    def __exit__(self, *_exc: object) -> None:
        self._spinner.__exit__()
        if self.process is not None:
            stop_process(self.process)
            self.process = None

    def require_param(self, target: str, name: str, value: ParamValue) -> None:
        outcome = self.node.set_parameter(target, name, value)
        if not outcome.successful:
            raise CheckError(f'{target}:{name}={value} が拒否されました: {outcome.reason}')

    def rejected_row(self, target: str, name: str, value: ParamValue, kind: str = '範囲外') -> Row:
        outcome = self.node.set_parameter(target, name, value)
        return Row(
            f'{kind}の値を拒否 {name}={value}',
            outcome.reason[:50] if outcome.reason else '理由なし',
            not outcome.successful,
        )

    def warning_row(self, keyword: str, watch_s: float = WARNING_WATCH_S) -> Row:
        """無効にしている間、警告が出続けるかを launch のログで確かめる."""
        offset = self.log_path.stat().st_size
        self.node.wait_sim(watch_s)
        text = self.log_path.read_bytes()[offset:].decode(errors='replace')
        count = len(re.findall(re.escape(keyword), text))
        return Row(f'無効の間の警告「{keyword}」', f'{count} 回', count >= 2)

    def to_world(self, x: float, y: float) -> tuple[float, float]:
        """odom は起動時の base_link なので、初期位置で世界座標に直す."""
        x0, y0, yaw0 = self.spec.start
        c, s = math.cos(yaw0), math.sin(yaw0)
        return x0 + c * x - s * y, y0 + s * x + c * y


@contextmanager
def paused(name: str) -> Iterator[None]:
    """プロセスを一時停止する。実装に手を入れずに入力の途絶や直接指令を試すため."""
    pid = find_process(name)
    if pid is None:
        raise CheckError(f'プロセス {name} が見つかりません')
    os.kill(pid, signal.SIGSTOP)
    try:
        yield
    finally:
        os.kill(pid, signal.SIGCONT)


def min_gap_between(node: FeatureNode, obstacle: Obstacle, start: float, end: float) -> float:
    samples = [p for p in list(node.truth)[::4] if start <= p.t <= end]
    if not samples:
        raise CheckError('区間の真値がありません')
    return min(
        footprint_gap(p.x, p.y, p.yaw, OWN_HALF_M, obstacle, state_at(obstacle, p.t))
        for p in samples
    )


def max_speed_between(
    node: FeatureNode, start: float, end: float,
    region: Callable[[float, float], bool] | None = None,
) -> float:
    """区間内の最高速度。region を指定すると、その範囲にいる間だけを見る."""
    samples = [p for p in list(node.truth)[::4] if start <= p.t <= end]
    if len(samples) < 3:
        raise CheckError('区間の真値がほとんどありません')
    times = np.array([p.t for p in samples])
    xs = np.array([p.x for p in samples])
    ys = np.array([p.y for p in samples])
    speeds = np.hypot(np.gradient(xs, times), np.gradient(ys, times))
    if region is not None:
        mask = np.array([region(p.x, p.y) for p in samples])
        if not mask.any():
            raise CheckError('指定した範囲を通っていません')
        speeds = speeds[mask]
    # 接触などの瞬間的な値に引きずられないよう、99% 点を使う
    return float(np.percentile(speeds, 99.0))


def scenario_with(
    start: Sequence[float], goals: Sequence[Sequence[float]], obstacles: Sequence[Mapping[str, object]],
    world: str = 'field.sdf', timeout_s: float = 120.0,
) -> dict[str, object]:
    return {
        'start': list(start),
        'goals': [list(goal) for goal in goals],
        'timeout_s': timeout_s,
        'repeat': 1,
        'world': world,
        'obstacles': list(obstacles),
    }


def parked(name: str, x: float, y: float, kind: str = 'robot') -> dict[str, object]:
    return {'name': name, 'type': kind, 'start': [x, y], 'peak_speed': 0.0}


# 2m/s から 3m/s2 で止まるのに 0.67 秒かかる
SAFETY_STOP_SETTLE_S: Final = 1.0


def check_safety_stop(log_dir: Path) -> list[Row]:
    """静止した相手に向かって /cmd_vel_nav を直接出し、手前で止まるかを見る."""
    obstacle_x = -1.0
    scenario = scenario_with(
        (-6.0, -2.5, 0.0), ((-8.0, -2.5, 0.0),), (parked('target', obstacle_x, -2.5),),
    )
    rows: list[Row] = []
    with Session('safety_stop', scenario, log_dir) as session, paused('mppi_lite'):
        node = session.node
        obstacle = session.spec.obstacles[0]

        def approach() -> float:
            """相手へ向かって走り、止まるまでの最小距離を返す。止められなければ通り抜ける.

            STOP は相手へ向かう速度だけを止めるので、まっすぐの指令を出し続けると横へ滑って
            相手の脇を抜ける。止まった位置を比べるために、STOP が続いたら指令をやめる。
            """
            start = node.now_sim()
            stop_since: float | None = None
            while node.now_sim() - start < 6.0 and node.truth[-1].x < obstacle_x + 1.5:
                node.command_world(2.0, 0.0, 0.0)
                time.sleep(0.02)
                stopped = bool(node.safety) and node.safety[-1][1] == SafetyStatus.STATE_STOP
                if not stopped:
                    stop_since = None
                elif stop_since is None:
                    stop_since = node.now_sim()
                elif node.now_sim() - stop_since >= SAFETY_STOP_SETTLE_S:
                    break
            end = node.now_sim()
            node.stop(1.0)
            gap = min_gap_between(node, obstacle, start, end)
            node.go_to(-6.0, -2.5, DIRECT_SPEED_MPS)
            node.stop(1.0)
            return gap

        default_gap = approach()
        rows.append(Row('有効: 相手の手前で止まる', f'最小距離 {default_gap:.2f}m', default_gap > 0.0))
        stops = sum(1 for _, s in list(node.safety) if s == SafetyStatus.STATE_STOP)
        rows.append(Row('STOP の状態になる', f'{stops} 回の通知', stops > 0))

        session.require_param('safety_gate', 'safety_stop.ttc_slow_s', 3.0)
        session.require_param('safety_gate', 'safety_stop.ttc_stop_s', 1.5)
        long_gap = approach()
        rows.append(Row(
            'ttc_stop_s 0.6→1.5 で手前で止まる',
            f'{default_gap:.2f}m→{long_gap:.2f}m', long_gap > default_gap + 0.2,
        ))
        session.require_param('safety_gate', 'safety_stop.ttc_stop_s', 0.6)
        session.require_param('safety_gate', 'safety_stop.ttc_slow_s', 1.5)

        session.require_param('safety_gate', 'safety_stop.enabled', False)
        rows.append(session.warning_row('safety_stop が無効'))
        disabled_gap = approach()
        rows.append(Row('無効: 止まらず通り抜ける', f'最小距離 {disabled_gap:.2f}m', disabled_gap <= 0.0))
        session.require_param('safety_gate', 'safety_stop.enabled', True)
        rows.append(session.rejected_row('safety_gate', 'safety_stop.ttc_stop_s', -1.0))
    return rows


def check_watchdog(log_dir: Path) -> list[Row]:
    """/cmd_vel_nav を途絶えさせて、止まるまでの時間を見る."""
    scenario = scenario_with((-6.0, -2.5, 0.0), ((-8.0, -2.5, 0.0),), ())
    rows: list[Row] = []
    with Session('watchdog', scenario, log_dir) as session, paused('mppi_lite'):
        node = session.node

        def stop_latency(limit_s: float = 5.0) -> float:
            """指令を止めてから止まるまでの時間。limit_s で止まらなければ limit_s を返す."""
            node.hold(1.0, 0.0, 0.0, 1.5)
            start = node.now_sim()
            while node.now_sim() - start < limit_s:
                if node.current_speed() < 0.1:
                    break
                time.sleep(0.02)
            latency = node.now_sim() - start
            node.go_to(-6.0, -2.5, 1.0)
            node.stop(0.5)
            return latency

        default_latency = stop_latency()
        rows.append(Row('有効: 指令が途絶えると止まる', f'{default_latency:.2f}秒', default_latency < 1.5))
        session.require_param('safety_gate', 'watchdog.cmd_timeout_s', 1.5)
        long_latency = stop_latency()
        rows.append(Row(
            'cmd_timeout_s 0.3→1.5 で止まるのが遅れる',
            f'{default_latency:.2f}秒→{long_latency:.2f}秒', long_latency > default_latency + 0.7,
        ))
        session.require_param('safety_gate', 'watchdog.cmd_timeout_s', 0.3)
        session.require_param('safety_gate', 'watchdog.enabled', False)
        rows.append(session.warning_row('watchdog が無効'))
        disabled_latency = stop_latency(2.0)
        rows.append(Row('無効: 指令が途絶えても止まらない', f'{disabled_latency:.2f}秒', disabled_latency >= 2.0))
        session.require_param('safety_gate', 'watchdog.enabled', True)
        rows.append(session.rejected_row('safety_gate', 'watchdog.cmd_timeout_s', 0.0))
    return rows


def check_stuck_recovery(log_dir: Path) -> list[Row]:
    """安全停止のノードを止めて機体を動けなくし、行き詰まりの判定を見る."""
    scenario = scenario_with((-6.0, -2.5, 0.0), ((-8.0, -2.5, 0.0),), ())
    rows: list[Row] = []
    with Session('stuck_recovery', scenario, log_dir) as session, paused('safety_gate'):
        node = session.node

        def time_to_state(goal_x: float, target: int, limit_s: float) -> float | None:
            """ゴールを出して、走り始めてから target の状態になるまでの時間を返す."""
            sent = node.now_sim()
            node.publish_goal(goal_x, -2.5, 0.0)
            started: float | None = None
            while node.now_sim() - sent < limit_s:
                state = node.latest_state()
                if started is None and state == NavigationStatus.STATE_DRIVING:
                    started = node.now_sim()
                if started is not None and state == target:
                    return node.now_sim() - started
                time.sleep(0.02)
            return None

        default_time = time_to_state(0.0, NavigationStatus.STATE_STUCK_WAIT, 12.0)
        rows.append(Row(
            '有効: 進まないと行き詰まりと判定する',
            f'{default_time}秒' if default_time is None else f'{default_time:.2f}秒',
            default_time is not None and 1.0 <= default_time <= 4.0,
        ))
        session.require_param('mppi_lite', 'stuck_recovery.no_progress_timeout_s', 5.0)
        long_time = time_to_state(2.0, NavigationStatus.STATE_STUCK_WAIT, 15.0)
        rows.append(Row(
            'no_progress_timeout_s 2→5 で判定が遅れる',
            f'{long_time}秒' if long_time is None else f'{long_time:.2f}秒',
            long_time is not None and default_time is not None and long_time > default_time + 1.5,
        ))
        session.require_param('mppi_lite', 'stuck_recovery.no_progress_timeout_s', 2.0)
        session.require_param('mppi_lite', 'stuck_recovery.max_retries', 0)
        failed_time = time_to_state(4.0, NavigationStatus.STATE_FAILED, 15.0)
        rows.append(Row(
            'max_retries 0 で復帰せず失敗とする',
            f'{failed_time}秒' if failed_time is None else f'{failed_time:.2f}秒',
            failed_time is not None,
        ))
        session.require_param('mppi_lite', 'stuck_recovery.max_retries', 3)
        session.require_param('mppi_lite', 'stuck_recovery.enabled', False)
        disabled_time = time_to_state(6.0, NavigationStatus.STATE_STUCK_WAIT, 8.0)
        rows.append(Row('無効: 行き詰まりと判定しない', '判定なし' if disabled_time is None else f'{disabled_time:.2f}秒', disabled_time is None))
        session.require_param('mppi_lite', 'stuck_recovery.enabled', True)
        rows.append(session.rejected_row('mppi_lite', 'stuck_recovery.max_retries', -1))
    return rows


def check_localization_health(log_dir: Path) -> list[Row]:
    """地図照合を一時停止した失敗の判定と、IMU の差による速度制限を見る."""
    scenario = scenario_with((-10.0, -2.5, 0.0), ((10.0, -2.5, 0.0),), ())
    rows: list[Row] = []
    with Session('localization_health', scenario, log_dir) as session:
        node = session.node

        def time_to_failed(limit_s: float) -> float | None:
            with paused('lidar_localization_node'):
                start = node.now_sim()
                result = None
                while node.now_sim() - start < limit_s:
                    if node.health_level == LocalizationHealth.LEVEL_FAILED:
                        result = node.now_sim() - start
                        break
                    time.sleep(0.02)
            wait_for_level(node, LocalizationHealth.LEVEL_OK, 30.0)
            return result

        default_time = time_to_failed(8.0)
        rows.append(Row(
            '有効: 照合が止まると失敗と判定する',
            f'{default_time}秒' if default_time is None else f'{default_time:.2f}秒',
            default_time is not None and default_time < 4.0,
        ))
        stops = sum(1 for _, s in list(node.safety) if s == SafetyStatus.STATE_LOCALIZATION_STOP)
        rows.append(Row('失敗で安全停止が働く', f'{stops} 回の通知', stops > 0))

        session.require_param('localization_health', 'localization_health.fail_timeout_s', 5.0)
        long_time = time_to_failed(12.0)
        rows.append(Row(
            'fail_timeout_s 2→5 で判定が遅れる',
            f'{long_time}秒' if long_time is None else f'{long_time:.2f}秒',
            long_time is not None and default_time is not None and long_time > default_time + 1.5,
        ))
        session.require_param('localization_health', 'localization_health.fail_timeout_s', 2.0)

        session.require_param('localization_health', 'localization_health.enabled', False)
        rows.append(session.warning_row('localization_health が無効'))
        disabled_time = time_to_failed(5.0)
        rows.append(Row('無効: 失敗と判定しない', '判定なし' if disabled_time is None else f'{disabled_time:.2f}秒', disabled_time is None))
        session.require_param('localization_health', 'localization_health.enabled', True)
        wait_for_level(node, LocalizationHealth.LEVEL_OK, 30.0)

        # Damiao IMU の値を偽装して差を作り、速度の上限が効くかを見る
        session.require_param('localization_health', 'localization_health.fail_timeout_s', 600.0)
        stop_fake = threading.Event()

        def publish_fake_imu() -> None:
            while not stop_fake.is_set():
                imu = Imu()
                imu.header.frame_id = 'damiao_imu_link'
                imu.angular_velocity.z = 8.0
                imu.linear_acceleration.z = 9.81
                node.fake_damiao.publish(imu)
                time.sleep(0.005)

        faker = threading.Thread(target=publish_fake_imu, daemon=True)
        faker.start()
        try:
            wait_for_level(node, LocalizationHealth.LEVEL_DEGRADED, 15.0)
            rows.append(Row('角速度の差で DEGRADED になる', '確認', True))
            speeds = {}
            legs = ((1.0, 10.0), (2.0, -10.0))
            for limit, goal_x in legs:
                session.require_param('mppi_lite', 'localization_health.degraded_speed_mps', limit)
                start = node.now_sim()
                result = drive_goal(node, (goal_x, -2.5, 0.0), 60.0, session.process)
                speeds[limit] = max_speed_between(node, start, node.now_sim())
                rows.append(Row(
                    f'degraded_speed_mps={limit} で速度を抑える',
                    f'最高 {speeds[limit]:.2f}m/s（{result.final_state}）',
                    speeds[limit] <= limit + 0.3,
                ))
            rows.append(Row(
                '上限を上げると速くなる', f'{speeds[1.0]:.2f}→{speeds[2.0]:.2f}m/s',
                speeds[2.0] > speeds[1.0] + 0.3,
            ))
        finally:
            stop_fake.set()
            faker.join(timeout=5.0)
        rows.append(session.rejected_row('localization_health', 'localization_health.fail_timeout_s', -1.0))
    return rows


def wait_for_level(node: FeatureNode, level: int, limit_s: float) -> None:
    start = node.now_sim()
    while node.health_level != level:
        if node.now_sim() - start > limit_s:
            raise CheckError(f'{limit_s}秒待っても自己位置の状態が {level} になりません（今 {node.health_level}）')
        time.sleep(0.05)


def median_tracks_near(
    session: Session, x: float, y: float, radius: float, start: float, end: float,
) -> float:
    counts = []
    for frame in list(session.node.frames):
        if not start <= frame.t <= end:
            continue
        counts.append(sum(
            1 for obj in frame.objects
            if math.hypot(*np.subtract(session.to_world(obj.position.x, obj.position.y), (x, y))) <= radius
        ))
    if not counts:
        raise CheckError('区間の追跡結果がありません')
    return float(np.median(counts))


def check_cluster_split(log_dir: Path) -> list[Row]:
    """隙間なく並んだ 2 台が、分かれて見えるかを追跡の数で見る."""
    x, y = -2.0, -2.5
    scenario = scenario_with(
        (-6.0, -2.5, 0.0), ((-8.0, -2.5, 0.0),),
        (parked('left', x, y + 0.4), parked('right', x, y - 0.4)),
    )
    rows: list[Row] = []
    with Session('cluster_split', scenario, log_dir) as session:
        node = session.node

        def count_tracks() -> float:
            # 消える追跡が消えるまで待ってから数える
            node.wait_sim(3.0)
            start = node.now_sim()
            node.wait_sim(3.0)
            return median_tracks_near(session, x, y, 2.0, start, node.now_sim())

        enabled = count_tracks()
        rows.append(Row('有効: 2 台に分かれる', f'追跡 {enabled:.0f} 個', enabled == 2.0))
        session.require_param('perception_node', 'cluster_split.enabled', False)
        disabled = count_tracks()
        rows.append(Row('無効: 1 個にまとまる', f'追跡 {disabled:.0f} 個', disabled == 1.0))
        session.require_param('perception_node', 'cluster_split.enabled', True)
        session.require_param('perception_node', 'cluster_split.max_cluster_size_m', 10.0)
        large = count_tracks()
        rows.append(Row('max_cluster_size_m 1.2→10 で分けなくなる', f'追跡 {large:.0f} 個', large == 1.0))
        rows.append(session.rejected_row('perception_node', 'cluster_split.expected_robot_size_m', 0.0))
    return rows


def longest_coast(frames: Sequence[TrackFrame], start: float, end: float) -> float:
    """1 つの追跡が予測だけで続いた最長の時間."""
    first_coast: dict[int, float] = {}
    longest = 0.0
    for frame in frames:
        if not start <= frame.t <= end:
            continue
        current = {obj.id for obj in frame.objects if obj.coasting}
        for track_id in current:
            first_coast.setdefault(track_id, frame.t)
            longest = max(longest, frame.t - first_coast[track_id])
        for track_id in set(first_coast) - current:
            del first_coast[track_id]
    return longest


def check_tracking(log_dir: Path) -> list[Row]:
    """静止物の陰を通る相手で、予測だけで追跡を続ける時間を見る.

    追跡には enabled がないので、max_coast_s を 0 にしたときを無効に当たるものとして確かめる。
    """
    # static_box_b の陰になる線上を往復する。y>1.75 のときだけ機体から見える
    scenario = scenario_with(
        (1.0, -2.75, 0.0), ((-5.0, -2.75, 0.0),),
        ({'name': 'hidden', 'type': 'robot', 'start': [7.0, -7.0], 'end': [7.0, 7.0], 'peak_speed': 3.0, 'phase': 0.0},),
        world='field_static.sdf',
    )
    rows: list[Row] = []
    with Session('tracking', scenario, log_dir) as session:
        node = session.node

        def measure(seconds: float = 25.0) -> float:
            start = node.now_sim()
            node.wait_sim(seconds)
            return longest_coast(list(node.frames), start, node.now_sim())

        default_coast = measure()
        rows.append(Row('max_coast_s=1.0 で隠れても続く', f'最長 {default_coast:.2f}秒', 0.6 <= default_coast <= 1.4))
        session.require_param('perception_node', 'tracking.max_coast_s', 0.3)
        short_coast = measure()
        rows.append(Row('max_coast_s=0.3 で短くなる', f'最長 {short_coast:.2f}秒', short_coast <= 0.6))
        session.require_param('perception_node', 'tracking.max_coast_s', 0.0)
        off_coast = measure(15.0)
        rows.append(Row('max_coast_s=0 で続けない（無効に当たる）', f'最長 {off_coast:.2f}秒', off_coast <= 0.2))
        session.require_param('perception_node', 'tracking.max_coast_s', 1.0)
        rows.append(session.rejected_row('perception_node', 'tracking.association_gate_m', -1.0))
        rows.append(session.rejected_row('perception_node', 'map_path', '/tmp/other.pcd', '読み取り専用'))
    return rows


def check_speed_limit(log_dir: Path) -> list[Row]:
    """止まった相手の横を通るときの速度を見る."""
    y_offset = -0.8
    scenario = scenario_with(
        (-10.0, -2.5, 0.0), ((10.0, -2.5, 0.0),),
        tuple(parked(f'row_{i}', x, y_offset) for i, x in enumerate((-5.0, -2.5, 0.0, 2.5, 5.0))),
    )
    rows: list[Row] = []
    with Session('speed_limit', scenario, log_dir) as session:
        node = session.node

        def near_row(x: float, _y: float) -> bool:
            return abs(x) <= 6.0

        def leg(goal_x: float) -> float:
            start = node.now_sim()
            result = drive_goal(node, (goal_x, -2.5, 0.0), 60.0, session.process)
            if not result.reached:
                raise CheckError(f'ゴール x={goal_x} に着けませんでした: {result.final_state}')
            return max_speed_between(node, start, node.now_sim(), near_row)

        default_speed = leg(10.0)
        rows.append(Row('既定の density_speed_scale=0.9', f'最高 {default_speed:.2f}m/s', True))
        session.require_param('mppi_lite', 'speed_limit.density_speed_scale', 0.3)
        low_speed = leg(-10.0)
        rows.append(Row(
            'density_speed_scale 0.9→0.3 で遅くなる', f'{default_speed:.2f}→{low_speed:.2f}m/s',
            low_speed < default_speed - 0.2,
        ))
        session.require_param('mppi_lite', 'speed_limit.min_speed_mps', 1.8)
        floor_speed = leg(10.0)
        rows.append(Row(
            'min_speed_mps 0.5→1.8 で下限が上がる', f'{low_speed:.2f}→{floor_speed:.2f}m/s',
            floor_speed > low_speed + 0.2,
        ))
        session.require_param('mppi_lite', 'speed_limit.min_speed_mps', 0.5)
        session.require_param('mppi_lite', 'speed_limit.density_speed_scale', 0.9)
        session.require_param('mppi_lite', 'speed_limit.enabled', False)
        disabled_speed = leg(-10.0)
        rows.append(Row(
            '無効: 速度を制限しない', f'{default_speed:.2f}→{disabled_speed:.2f}m/s',
            disabled_speed >= default_speed - 0.1,
        ))
        session.require_param('mppi_lite', 'speed_limit.enabled', True)
        rows.append(session.rejected_row('mppi_lite', 'speed_limit.density_speed_scale', 2.0))
    return rows


def check_footprint(log_dir: Path) -> list[Row]:
    """経路上の相手を避けるときの距離を見る."""
    scenario = scenario_with(
        (-10.0, -2.5, 0.0), ((10.0, -2.5, 0.0),), (parked('on_path', 0.0, -2.5),),
    )
    rows: list[Row] = []
    with Session('footprint', scenario, log_dir) as session:
        node = session.node
        obstacle = session.spec.obstacles[0]

        def leg(goal_x: float) -> float:
            start = node.now_sim()
            result = drive_goal(node, (goal_x, -2.5, 0.0), 60.0, session.process)
            if not result.reached:
                raise CheckError(f'ゴール x={goal_x} に着けませんでした: {result.final_state}')
            return min_gap_between(node, obstacle, start, node.now_sim())

        default_gap = leg(10.0)
        rows.append(Row('margin_m=0.1 で接触せず避ける', f'最小距離 {default_gap:.2f}m', default_gap > 0.0))
        for target in ('mppi_lite', 'global_planner'):
            session.require_param(target, 'footprint.margin_m', 0.8)
        wide_gap = leg(-10.0)
        rows.append(Row(
            'margin_m 0.1→0.8 で離れて通る', f'{default_gap:.2f}→{wide_gap:.2f}m',
            wide_gap > default_gap + 0.1,
        ))
        for target in ('mppi_lite', 'global_planner'):
            session.require_param(target, 'footprint.margin_m', 0.1)
        session.require_param('mppi_lite', 'footprint.enabled', False)
        disabled_gap = leg(10.0)
        rows.append(Row('enabled を切り替えられる（無効は機体を点として扱う）', f'最小距離 {disabled_gap:.2f}m', True))
        session.require_param('mppi_lite', 'footprint.enabled', True)
        rows.append(session.rejected_row('mppi_lite', 'footprint.margin_m', -0.1))
    return rows


# 確認用のシナリオ。自機は x=±10.5 を往復し、相手は x=±7 の間を最高 3m/s で往復する。
# 以前は head_on.yaml（自機のゴール ±10、相手の車線の端 ±11）を使っていたが、ゴールが相手の車線の
# 端の近くにあると、自機がゴールで止まっている間や減速中に相手と出会う。ゴールを相手の車線の端から
# 3.5m 離すと、出会いはフィールドの中ほどで、自機が速く走っている間に起きる
PASSING_GOAL_X_M: Final = 10.5
PASSING_OPPONENT_HALF_SPAN_M: Final = 7.0
PASSING_OPPONENT_SPEED_MPS: Final = 3.0
PASSING_MIN_SPEED_MPS: Final = 1.0
# 正面から来る相手（シミュレーションの相手は自機をよけない）とゴールの近くで出会うときは、2026-11-14 の
# ユーザーの判断で横へ道を開けず、経路の上の停止線で止まる（mppi.yaml の oncoming_rule）。止まる設定で、相手が
# 前方 2〜6m から近づく間の、経路（y=0）からの横のずれの上限。衝撃の緩和（safety_stop.mitigation）は当たる直前に
# 相手から離れる向きへ 0.5m まで動くので、2m より近い間は見ない。相手が折り返しで遅い（0.3m/s 未満）間は大域計画が
# 止まっている障害物としてよける経路を引くので、止まる設定でも 0.37〜0.44m ずれた（Gazebo、2026-11-14）。
# よけてすれ違うときの横のずれ（約 1.6m）と分けられるよう 0.5m にする
ONCOMING_MAX_LATERAL_M: Final = 0.5
ONCOMING_NEAR_M: Final = 2.0
ONCOMING_FAR_M: Final = 6.0
ONCOMING_ROUND_TRIPS: Final = 2
# ゴールから遠い所で出会う正面の相手とすれ違う側の確認。側ごとの往復の数と、数えるすれ違いの下限。
# 双方 1m/s 以上のすれ違いだけを数えるので、2 往復では 1 回しか数えられない側があった（Gazebo、2026-11-14）
HEAD_ON_PASS_ROUND_TRIPS: Final = 3
HEAD_ON_MIN_PASSES: Final = 2
# 止まっているロボットは、すれ違いの規則（passing_rule.side）の側から回り込む。相手を経路の上（原点）に
# 置き、自機は x=±10.5 を往復する
STATIONARY_ROUND_TRIPS: Final = 2


def oncoming_lateral(node: FeatureNode, obstacle: Obstacle, start: float, end: float) -> float:
    """正面の相手が前方 2〜6m から近づいている間の、自機の経路（y=0）からの横のずれの最大."""
    samples = [p for p in list(node.truth)[::4] if start <= p.t <= end]
    worst = 0.0
    for before, after in zip(samples, samples[1:]):
        dt = after.t - before.t
        if dt <= 0.0:
            continue
        vx = (after.x - before.x) / dt
        state = state_at(obstacle, after.t)
        dx = state.x - after.x
        approaching = (
            abs(state.vx) >= PASSING_MIN_SPEED_MPS and state.vx * dx < 0.0 and (vx - state.vx) * dx >= 0.0
        )
        if approaching and ONCOMING_NEAR_M <= abs(dx) <= ONCOMING_FAR_M:
            worst = max(worst, abs(after.y))
    return worst


def head_on_sides(node: FeatureNode, obstacle: Obstacle, start: float, end: float) -> list[float]:
    """向かい合ってすれ違うたびに、進行方向から見た相手の横位置を返す（左が正）.

    どちらかが遅いとき（ゴールの近くや相手の折り返しの近く）は数えない。
    """
    samples = [p for p in list(node.truth)[::4] if start <= p.t <= end]
    sides: list[float] = []
    previous_along: float | None = None
    for before, after in zip(samples, samples[1:]):
        dt = after.t - before.t
        if dt <= 0.0:
            continue
        vx, vy = (after.x - before.x) / dt, (after.y - before.y) / dt
        speed = math.hypot(vx, vy)
        if speed < 0.3:
            previous_along = None
            continue
        state = state_at(obstacle, after.t)
        rel_x, rel_y = state.x - after.x, state.y - after.y
        along = (rel_x * vx + rel_y * vy) / speed
        lateral = (vx * rel_y - vy * rel_x) / speed
        oncoming = (
            speed >= PASSING_MIN_SPEED_MPS
            and math.hypot(state.vx, state.vy) >= PASSING_MIN_SPEED_MPS
            and vx * state.vx + vy * state.vy < 0.0
        )
        if oncoming and previous_along is not None and previous_along > 0.0 >= along and abs(lateral) < 4.0:
            sides.append(lateral)
        previous_along = along
    return sides


def stationary_sides(
    node: FeatureNode, x: float, y: float, start: float, end: float,
) -> list[float]:
    """止まっている相手（x, y）の横を通るたびに、進行方向から見た相手の横位置を返す（左が正）."""
    samples = [p for p in list(node.truth)[::4] if start <= p.t <= end]
    sides: list[float] = []
    previous_along: float | None = None
    for before, after in zip(samples, samples[1:]):
        dt = after.t - before.t
        if dt <= 0.0:
            continue
        vx, vy = (after.x - before.x) / dt, (after.y - before.y) / dt
        speed = math.hypot(vx, vy)
        if speed < 0.3:
            previous_along = None
            continue
        rel_x, rel_y = x - after.x, y - after.y
        along = (rel_x * vx + rel_y * vy) / speed
        lateral = (vx * rel_y - vy * rel_x) / speed
        if previous_along is not None and previous_along > 0.0 >= along and abs(lateral) < 4.0:
            sides.append(lateral)
        previous_along = along
    return sides


def check_passing_rule(log_dir: Path) -> list[Row]:
    """正面から来る相手とのすれ違い・停止線と、止まっている相手を回り込む側を見る.

    2026-11-14 のユーザーの判断で、正面から来るロボットとゴールの近くで出会うときは、横へよけずに経路の上の
    停止線で止まる（oncoming_rule）。ゴールから遠い所（oncoming_rule.pass_goal_margin_m 以上手前）で出会うときは、
    止まらずにすれ違いの規則（passing_rule.side）の側へよけて通る。高速で当てられそうなときは、相手から離れる
    向きへ少し動いて衝撃を和らげる（safety_stop.mitigation）。すれ違いの規則は、経路をふさいでいる止まっている
    ロボットを回り込む側にも使う。
    """
    rows: list[Row] = []
    head_on = scenario_with(
        (-PASSING_GOAL_X_M, 0.0, 0.0), ((PASSING_GOAL_X_M, 0.0, 0.0),),
        ({
            'name': 'head_on', 'type': 'robot',
            'start': [PASSING_OPPONENT_HALF_SPAN_M, 0.0], 'end': [-PASSING_OPPONENT_HALF_SPAN_M, 0.0],
            'peak_speed': PASSING_OPPONENT_SPEED_MPS, 'phase': 0.0,
        },),
    )
    with Session('passing_rule', head_on, log_dir) as session:
        node = session.node
        obstacle = session.spec.obstacles[0]

        def run_legs(round_trips: int) -> tuple[float, list[float], ResultDict, str]:
            start = node.now_sim()
            offset = session.log_path.stat().st_size
            for goal_x in (PASSING_GOAL_X_M, -PASSING_GOAL_X_M) * round_trips:
                drive_goal(node, (goal_x, 0.0, 0.0), 60.0, session.process)
            end = node.now_sim()
            text = session.log_path.read_bytes()[offset:].decode(errors='replace')
            samples = [p for p in list(node.truth) if start <= p.t <= end]
            return (
                oncoming_lateral(node, obstacle, start, end), head_on_sides(node, obstacle, start, end),
                analyze_obstacles(samples, [obstacle], 0.5), text,
            )

        # ゴールから遠い所（フィールドの中ほど）で出会う既定の場面: 規則の側へよけてすれ違う
        for side, expected_sign in (('right', 1.0), ('left', -1.0)):
            session.require_param('mppi_lite', 'passing_rule.side', side)
            _, sides, contacts, _ = run_legs(HEAD_ON_PASS_ROUND_TRIPS)
            same = sum(1 for value in sides if value * expected_sign > 0.0)
            text = ', '.join(f'{value:+.2f}' for value in sides)
            rows.append(Row(
                f'ゴールから遠い正面の相手と side={side} ですれ違う（相手が{"左" if expected_sign > 0.0 else "右"}に見える）',
                f'{same}/{len(sides)} 回（横位置 {text}）、自機が原因の接触 {contacts["own_fault_contacts"]} 回',
                len(sides) >= HEAD_ON_MIN_PASSES and same * 2 > len(sides) and contacts['own_fault_contacts'] == 0,
            ))
        session.require_param('mppi_lite', 'passing_rule.side', 'right')

        # 常に止まる設定（ゴールの近くで出会うときと同じ動き）: 道を開けずに停止線で止まり、速く当てられそうなら和らげる
        session.require_param('mppi_lite', 'oncoming_rule.pass_goal_margin_m', -1.0)
        lateral, _, contacts, text = run_legs(ONCOMING_ROUND_TRIPS)
        session.require_param('mppi_lite', 'oncoming_rule.pass_goal_margin_m', 3.0)
        yields = text.count('向かってくる相手に道を開けず、停止線で止まります')
        mitigations = text.count('高速で当てられそうなので、衝撃を和らげる向きへ動きます')
        rows.append(Row(
            '止まるとき道を開けない（前方 2〜6m から近づく間の横のずれ）', f'最大 {lateral:.2f}m',
            lateral <= ONCOMING_MAX_LATERAL_M,
        ))
        rows.append(Row('止まるとき正面の相手の手前の停止線で止まる', f'{yields} 回', yields >= 1))
        rows.append(Row(
            '止まるとき自機が原因の接触がない（相手は自機をよけない）',
            f"{contacts['own_fault_contacts']}/{contacts['contacts']} 回",
            contacts['own_fault_contacts'] == 0,
        ))
        impact = contacts['mean_impact_speed_mps']
        rows.append(Row(
            '速く当てられそうなとき衝撃を和らげる',
            f"緩和 {mitigations} 回、当たる速さ 平均 {impact if impact is not None else float('nan'):.2f}m/s",
            mitigations >= 1,
        ))
        session.require_param('mppi_lite', 'oncoming_rule.enabled', False)
        session.require_param('mppi_lite', 'oncoming_rule.enabled', True)
        rows.append(Row('oncoming_rule.enabled を切り替えられる', '受け付けた', True))
        rows.append(session.rejected_row('mppi_lite', 'oncoming_rule.stop_before_midpoint_m', -1.0))
        rows.append(session.rejected_row('mppi_lite', 'oncoming_rule.pass_goal_margin_m', -5.0))
        rows.append(session.rejected_row('safety_gate', 'safety_stop.mitigation.max_shift_m', -1.0))

    stationary = scenario_with(
        (-PASSING_GOAL_X_M, 0.0, 0.0), ((PASSING_GOAL_X_M, 0.0, 0.0),), (parked('parked', 0.0, 0.0),),
    )
    with Session('passing_rule_stationary', stationary, log_dir) as session:
        node = session.node

        def run_side(side: str) -> list[float]:
            session.require_param('mppi_lite', 'passing_rule.side', side)
            start = node.now_sim()
            for goal_x in (PASSING_GOAL_X_M, -PASSING_GOAL_X_M) * STATIONARY_ROUND_TRIPS:
                drive_goal(node, (goal_x, 0.0, 0.0), 60.0, session.process)
            return stationary_sides(node, 0.0, 0.0, start, node.now_sim())

        for side, expected_sign in (('right', 1.0), ('left', -1.0)):
            sides = run_side(side)
            same = sum(1 for value in sides if value * expected_sign > 0.0)
            text = ', '.join(f'{value:+.2f}' for value in sides)
            rows.append(Row(
                f'side={side} で止まっている相手が{"左" if expected_sign > 0.0 else "右"}に見える',
                f'{same}/{len(sides)} 回（横位置 {text}）',
                len(sides) >= 2 * STATIONARY_ROUND_TRIPS - 1 and same * 2 > len(sides),
            ))
        session.require_param('mppi_lite', 'passing_rule.enabled', False)
        session.require_param('mppi_lite', 'passing_rule.enabled', True)
        rows.append(Row('enabled を切り替えられる', '受け付けた', True))
        rows.append(session.rejected_row('mppi_lite', 'passing_rule.side', 'up'))
        rows.append(session.rejected_row('mppi_lite', 'passing_rule.weight', -1.0))
    return rows


def check_goal_handling(log_dir: Path) -> list[Row]:
    """ゴールの上に相手が止まっているときの動きを見る."""
    goal = (6.0, 2.0, 0.0)
    home = (-8.0, -3.0, 0.0)
    scenario = scenario_with(home, (goal,), (parked('parked', goal[0], goal[1]),))
    rows: list[Row] = []
    with Session('goal_handling', scenario, log_dir) as session:
        node = session.node
        # 相手を見つけるまで近づいてから、ゴールを出す
        first = drive_goal(node, (0.0, 0.0, 0.0), 40.0, session.process)
        rows.append(Row('手前まで走る', first.final_state, first.reached))

        def to_goal(limit_s: float) -> tuple[bool, float, float]:
            result = drive_goal(node, goal, limit_s, session.process)
            drive_goal(node, (0.0, 0.0, 0.0), 40.0, session.process)
            return result.reached, result.duration_s, result.final_distance_m

        session.require_param('global_planner', 'goal_handling.wait_s', 1.0)
        reached, short_time, distance = to_goal(40.0)
        rows.append(Row(
            '有効: 近くの代わりの地点へ着く', f'{short_time:.1f}秒、ゴールまで {distance:.2f}m',
            reached and 0.5 < distance <= 2.3,
        ))
        session.require_param('global_planner', 'goal_handling.wait_s', 6.0)
        reached_long, long_time, _ = to_goal(40.0)
        rows.append(Row(
            'wait_s 1→6 で移るのが遅れる', f'{short_time:.1f}→{long_time:.1f}秒',
            reached_long and long_time > short_time + 3.0,
        ))
        session.require_param('global_planner', 'goal_handling.wait_s', 3.0)
        session.require_param('global_planner', 'goal_handling.enabled', False)
        disabled_reached, disabled_time, disabled_distance = to_goal(25.0)
        rows.append(Row(
            '無効: 代わりの地点へ移らない',
            f'到達 {disabled_reached}、{disabled_time:.1f}秒、ゴールまで {disabled_distance:.2f}m',
            not disabled_reached,
        ))
        session.require_param('global_planner', 'goal_handling.enabled', True)
        rows.append(session.rejected_row('global_planner', 'goal_handling.wait_s', -1.0))
    return rows


def check_contact_detection(log_dir: Path) -> list[Row]:
    """柵に当てて、接触の検知を見る."""
    start_pose = (8.0, -2.5, 0.0)
    scenario = scenario_with(start_pose, ((0.0, -2.5, 0.0),), ())
    rows: list[Row] = []
    with Session('contact_detection', scenario, log_dir) as session, paused('mppi_lite'):
        node = session.node

        def ram_fence() -> tuple[int, float]:
            """柵に向かって走り、接触の回数と、接触の状態が続いた時間を返す."""
            start = node.now_sim()
            node.hold(DIRECT_SPEED_MPS, 0.0, 0.0, 3.5)
            # 接触の状態が消えるまで止めて待つ
            node.stop(4.0)
            end = node.now_sim()
            samples = [c for c in list(node.contact) if start <= c[0] <= end]
            events = sum(1 for (_, a), (_, b) in zip(samples, samples[1:]) if b and not a)
            times = [t for t, value in samples if value]
            duration = times[-1] - times[0] if times else 0.0
            node.go_to(start_pose[0], start_pose[1], 1.0)
            node.stop(1.0)
            return events, duration

        def set_hold(value: float) -> None:
            for target in ('contact_monitor', 'safety_gate'):
                session.require_param(target, 'contact_detection.hold_s', value)

        default_events, default_duration = ram_fence()
        rows.append(Row('有効: 柵への接触を検知する', f'{default_events} 回、{default_duration:.2f}秒', default_events >= 1))
        stops = sum(1 for _, s in list(node.safety) if s == SafetyStatus.STATE_CONTACT_STOP)
        rows.append(Row('接触で安全停止が働く', f'{stops} 回の通知', stops > 0))

        set_hold(3.0)
        _, long_duration = ram_fence()
        rows.append(Row(
            'hold_s 1→3 で接触の状態が長く続く', f'{default_duration:.2f}→{long_duration:.2f}秒',
            long_duration > default_duration + 1.0,
        ))
        set_hold(1.0)

        session.require_param('contact_monitor', 'contact_detection.accel_threshold_mps2', 1000.0)
        session.require_param('contact_monitor', 'contact_detection.velocity_error_mps', 10.0)
        high_events, _ = ram_fence()
        rows.append(Row('閾値を上げると検知しない', f'{high_events} 回', high_events == 0))
        session.require_param('contact_monitor', 'contact_detection.accel_threshold_mps2', 20.0)
        session.require_param('contact_monitor', 'contact_detection.velocity_error_mps', 1.0)

        session.require_param('contact_monitor', 'contact_detection.enabled', False)
        disabled_events, _ = ram_fence()
        rows.append(Row('無効: 検知しない', f'{disabled_events} 回', disabled_events == 0))
        session.require_param('contact_monitor', 'contact_detection.enabled', True)
        rows.append(session.rejected_row('contact_monitor', 'contact_detection.hold_s', -1.0))
    return rows


def check_outlier_filter(log_dir: Path) -> list[Row]:
    """点群に干渉の点を足して、検出に残る点の数を見る."""
    scenario_path = (
        Path(get_package_share_directory('core_gazebo')) / 'scenarios' / 'interference.yaml'
    )
    rows: list[Row] = []
    with Session('outlier_filter', scenario_path, log_dir) as session:
        node = session.node

        def measure() -> tuple[float, float]:
            """検出に残る点の平均と、追跡の数の平均を返す."""
            node.wait_sim(2.0)
            start = node.now_sim()
            node.wait_sim(4.0)
            end = node.now_sim()
            points = [n for t, n in list(node.perception_points) if start <= t <= end]
            tracks = [len(f.objects) for f in list(node.frames) if start <= f.t <= end]
            if not points or not tracks:
                raise CheckError('区間の検出結果がありません')
            return float(np.mean(points)), float(np.mean(tracks))

        baseline, _ = measure()
        rows.append(Row('干渉なし', f'残る点 {baseline:.0f}', True))
        session.require_param('points_to_livox', 'interference_points', 3000)
        enabled_points, enabled_tracks = measure()
        rows.append(Row('有効: 干渉の点を除く', f'残る点 {enabled_points:.0f}、追跡 {enabled_tracks:.1f}', True))
        session.require_param('perception_node', 'outlier_filter.enabled', False)
        disabled_points, disabled_tracks = measure()
        rows.append(Row(
            '無効: 干渉の点が残る',
            f'残る点 {disabled_points:.0f}、追跡 {disabled_tracks:.1f}',
            disabled_points > max(enabled_points * 1.5, enabled_points + 20.0),
        ))
        session.require_param('perception_node', 'outlier_filter.enabled', True)
        session.require_param('perception_node', 'outlier_filter.min_neighbors', 8)
        strict_points, _ = measure()
        rows.append(Row(
            'min_neighbors 2→8 で残る点が減る', f'{enabled_points:.0f}→{strict_points:.0f}',
            strict_points < enabled_points,
        ))
        session.require_param('perception_node', 'outlier_filter.min_neighbors', 2)
        session.require_param('points_to_livox', 'interference_points', 0)
        rows.append(session.rejected_row('perception_node', 'outlier_filter.radius_m', 0.0))
    return rows


def check_scenario_test(log_dir: Path) -> list[Row]:
    """再現試験そのものを動かして、結果の出力と記録を見る."""
    scenario = write_temp_scenario(
        scenario_with(
            (-10.0, 0.0, 0.0), ((10.0, 0.0, 0.0),),
            ({'name': 'head_on', 'type': 'robot', 'start': [11.0, 0.0], 'end': [-11.0, 0.0], 'peak_speed': 3.0, 'phase': 0.0},),
            timeout_s=60.0,
        ),
        'scenario_test',
    )
    script = str(Path(__file__).with_name('run_scenario.py'))

    def run(name: str, extra: Sequence[str]) -> Mapping[str, object]:
        json_path = log_dir / f'{name}.json'
        with (log_dir / f'{name}.run.log').open('w', encoding='utf-8') as log:
            subprocess.run(
                [
                    sys.executable, script, '--scenario-file', str(scenario), '--json', str(json_path),
                    '--log-file', str(log_dir / f'{name}.launch.log'), *extra,
                ],
                stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy(), check=False,
            )
        if not json_path.exists():
            raise CheckError(f'{name}: 結果がありません。ログ: {log_dir / (name + ".run.log")}')
        return json.loads(json_path.read_text(encoding='utf-8'))

    rows: list[Row] = []
    bag_directory = log_dir / 'scenario_test_bag'
    first = run('scenario_test_near', ('--record-bag', '--bag-dir', str(bag_directory), '--min-distance-warn-m', '0.3'))
    required = (
        'goals_reached', 'min_gap_m', 'warn_time_s', 'contacts', 'own_fault_contacts',
        'other_fault_contacts', 'max_speed_mps', 'planning_ms_p99', 'perception_ms_p99',
        'safety_time_s', 'contact_events', 'min_fence_clearance_m',
    )
    missing = [key for key in required if key not in first]
    rows.append(Row('結果に必要な項目がある', f'不足 {missing}' if missing else 'すべてあり', not missing))
    rows.append(Row(
        'record_bag で rosbag を記録する', str(bag_directory),
        (bag_directory / 'metadata.yaml').exists(),
    ))
    second = run('scenario_test_far', ('--no-record-bag', '--min-distance-warn-m', '5.0'))
    near_time = first['warn_time_s']
    far_time = second['warn_time_s']
    rows.append(Row(
        'min_distance_warn_m 0.3→5.0 で警告の時間が増える', f'{near_time}→{far_time}秒',
        isinstance(near_time, (int, float)) and isinstance(far_time, (int, float)) and far_time > near_time,
    ))
    rows.append(Row('record_bag を切ると記録しない', 'bag_directory なし', 'bag_directory' not in second))
    return rows


FEATURES: Final[dict[str, Callable[[Path], list[Row]]]] = {
    'stuck_recovery': check_stuck_recovery,
    'safety_stop': check_safety_stop,
    'watchdog': check_watchdog,
    'localization_health': check_localization_health,
    'cluster_split': check_cluster_split,
    'tracking': check_tracking,
    'speed_limit': check_speed_limit,
    'footprint': check_footprint,
    'passing_rule': check_passing_rule,
    'goal_handling': check_goal_handling,
    'contact_detection': check_contact_detection,
    'outlier_filter': check_outlier_filter,
    'scenario_test': check_scenario_test,
}


def run_one(name: str, log_dir: Path, json_path: Path | None) -> bool:
    """1 機能をこのプロセスで確かめる."""
    rclpy.init()
    try:
        rows = FEATURES[name](log_dir)
    except CheckError as error:
        tail = log_tail(log_dir / f'{name}.launch.log', 8)
        rows = [Row('確認を中断', str(error)[:200], False)]
        print(f'[NG] {name}: {error}\n{tail}', flush=True)
    finally:
        rclpy.try_shutdown()
    print_rows(name, rows)
    if json_path is not None:
        json_path.write_text(
            json.dumps([asdict(row) for row in rows], ensure_ascii=False, indent=1),
            encoding='utf-8',
        )
    return all(row.ok for row in rows)


def print_rows(name: str, rows: Sequence[Row]) -> None:
    for row in rows:
        print(f'[{"OK" if row.ok else "NG"}] {name} | {row.label} | {row.value}', flush=True)


def run_child(name: str, index: int, log_dir: Path) -> tuple[str, list[Row]]:
    """1 機能を、ROS_DOMAIN_ID と GZ_PARTITION を分けた子プロセスで確かめる."""
    json_path = log_dir / f'{name}.result.json'
    environment = os.environ.copy()
    environment['ROS_DOMAIN_ID'] = str(BASE_DOMAIN + index)
    environment['GZ_PARTITION'] = f'features_{name}'
    command = [
        sys.executable, str(Path(__file__).resolve()), '--only', name, '--json', str(json_path),
        '--log-dir', str(log_dir),
    ]
    with (log_dir / f'{name}.run.log').open('w', encoding='utf-8') as log:
        try:
            subprocess.run(
                command, stdout=log, stderr=subprocess.STDOUT, env=environment,
                timeout=CHILD_TIMEOUT_S, check=False,
            )
        except subprocess.TimeoutExpired:
            return name, [Row('確認を中断', f'{CHILD_TIMEOUT_S}秒で終わりませんでした', False)]
    if not json_path.exists():
        return name, [Row('確認を中断', f'結果がありません。ログ: {log_dir / (name + ".run.log")}', False)]
    return name, [Row(**item) for item in json.loads(json_path.read_text(encoding='utf-8'))]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--only', choices=sorted(FEATURES), default=None, help='1 機能だけ確かめる')
    parser.add_argument('--jobs', type=int, default=1, help='同時に確かめる機能の数')
    parser.add_argument('--json', type=Path, default=None, help='結果を書き出すファイル')
    parser.add_argument('--log-dir', type=Path, default=None, help='launch のログの出力先')
    args = parser.parse_args()

    log_dir = args.log_dir or Path(tempfile.mkdtemp(prefix='core_nav_check_features_'))
    log_dir.mkdir(parents=True, exist_ok=True)
    print(f'ログ: {log_dir}', flush=True)
    if args.only is not None:
        return 0 if run_one(args.only, log_dir, args.json) else 1

    def task(item: tuple[int, str]) -> tuple[str, list[Row]]:
        index, name = item
        result = run_child(name, index, log_dir)
        print_rows(*result)
        return result

    with ThreadPoolExecutor(max_workers=max(args.jobs, 1)) as executor:
        results = dict(executor.map(task, enumerate(FEATURES)))

    print('--- まとめ（機能 | 合格/項目数）', flush=True)
    all_ok = True
    for name in FEATURES:
        rows = results[name]
        passed = sum(1 for row in rows if row.ok)
        all_ok = all_ok and passed == len(rows)
        print(f'[{"OK" if passed == len(rows) else "NG"}] {name} | {passed}/{len(rows)}', flush=True)
    if args.json is not None:
        args.json.write_text(
            json.dumps(
                {name: [asdict(row) for row in rows] for name, rows in results.items()},
                ensure_ascii=False, indent=1,
            ),
            encoding='utf-8',
        )
    print('結果: 完了条件を満たしました' if all_ok else '結果: 完了条件を満たしていません', flush=True)
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
