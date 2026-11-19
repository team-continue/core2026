#!/usr/bin/env python3
"""長時間走らせて、悪化がないことを確かめる（手順7の完了条件）.

crowd_goals のゴールを繰り返し回り、プロセスごとの CPU と使用メモリを 30 秒ごとに、
自己位置誤差（TF の map→base_link と真値の差）を 1 秒ごとに記録する。
最初と最後の 5 分を比べて、誤差の悪化とメモリの増加を判定する。
"""

import argparse
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
from typing import Callable, Final, Sequence

from ament_index_python.packages import get_package_share_directory
from core_nav_check.common import (
    CheckError,
    cpu_percent,
    find_stack_processes,
    missing_processes,
    Pose,
    ProcessUsage,
    read_usage,
    SpinThread,
    wrap_angle,
)
import numpy as np
import rclpy
from rclpy.time import Time
from run_scenario import (
    analyze_run,
    drive_goal,
    GoalResult,
    load_spec,
    log_tail,
    print_summary,
    ScenarioNode,
    start_stack,
    stop_process,
    wait_until_ready,
)
import tf2_ros

# 測るノード（別のプロセスで動かすときの実行ファイル）。composition:=true でコンテナに入るノードは、コンテナの
# プロセスで測る。composition の有無で合計を比べられるよう、コンテナに入るノードはすべて含める
PROCESSES: Final = (
    'fastlio_mapping', 'points_to_livox', 'perception_node', 'lio_odom_adapter', 'scan_filter',
    'lidar_localization_node', 'localization_health', 'global_planner', 'mppi_lite', 'contact_monitor',
    'safety_gate',
)
USAGE_PERIOD_S: Final = 30.0
ERROR_PERIOD_S: Final = 1.0
WINDOW_S: Final = 300.0
# 1 つのゴールの制限時間。詰まっても長時間試験を止めない
GOAL_TIME_LIMIT_S: Final = 120.0
# 判定の暫定の余裕。誤差は 1.5 倍かつ 3cm まで、メモリは 10% かつ 20MiB まで許す
ERROR_RATIO: Final = 1.5
ERROR_MARGIN_M: Final = 0.03
MEMORY_RATIO: Final = 1.1
MEMORY_MARGIN_MIB: Final = 20.0
# 距離の計算は 20Hz で十分。長時間だと計算量が大きいので減らす
TRUTH_STRIDE: Final = 10


@dataclass(frozen=True)
class UsageSample:
    t: float
    process: str
    cpu_percent: float
    rss_mib: float


@dataclass(frozen=True)
class ErrorSample:
    t: float
    position_m: float
    height_m: float
    yaw_deg: float


class LongRunNode(ScenarioNode):
    def __init__(self) -> None:
        super().__init__('check_long_run')
        self.errors: list[ErrorSample] = []
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    def sample_error(self) -> None:
        """今の map→base_link と、同じ時刻の真値を比べる。シミュレーションでは map と world が一致する."""
        try:
            transform = self._tf_buffer.lookup_transform('map', 'base_link', Time())
        except tf2_ros.TransformException as error:
            self.get_logger().warning(f'map→base_link を取得できません: {error}')
            return
        stamp = float(transform.header.stamp.sec) + float(transform.header.stamp.nanosec) * 1e-9
        truth = self._truth_at(stamp)
        if truth is None:
            return
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.errors.append(ErrorSample(
            t=stamp,
            position_m=math.hypot(t.x - truth.x, t.y - truth.y),
            height_m=abs(t.z - truth.z),
            yaw_deg=math.degrees(abs(wrap_angle(yaw - truth.yaw))),
        ))

    def _truth_at(self, t: float) -> Pose | None:
        """真値の新しいほうから探す."""
        recent = self.truth[-400:]
        return min(recent, key=lambda p: abs(p.t - t), default=None)


def collect_usage(
    now: float, pids: dict[str, int], previous: dict[str, ProcessUsage],
    samples: list[UsageSample],
) -> None:
    """プロセスごとの CPU と使用メモリを 1 回記録する."""
    for name, pid in pids.items():
        usage = read_usage(pid)
        samples.append(UsageSample(now, name, cpu_percent(previous[name], usage), usage.rss_mib))
        previous[name] = usage


def write_csv(path: Path, usage: Sequence[UsageSample], errors: Sequence[ErrorSample]) -> Path:
    with path.open('w', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
        writer.writerow(['time_s', 'process', 'cpu_percent', 'rss_mib'])
        writer.writerows((u.t, u.process, u.cpu_percent, u.rss_mib) for u in usage)
    error_path = path.with_name(f'{path.stem}_error{path.suffix}')
    with error_path.open('w', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
        writer.writerow(['time_s', 'position_error_m', 'height_error_m', 'yaw_error_deg'])
        writer.writerows((e.t, e.position_m, e.height_m, e.yaw_deg) for e in errors)
    return error_path


def judge(
    usage: Sequence[UsageSample], errors: Sequence[ErrorSample], start: float, end: float,
    window_s: float,
) -> tuple[bool, dict[str, object]]:
    """最初と最後の区間を比べて判定する."""
    first_end = start + window_s
    last_start = end - window_s
    first_errors = np.array([e.position_m for e in errors if start <= e.t < first_end])
    last_errors = np.array([e.position_m for e in errors if last_start <= e.t <= end])
    if first_errors.size == 0 or last_errors.size == 0:
        raise CheckError('比べる区間に自己位置誤差の記録がありません')
    first_error = float(np.sqrt(np.mean(first_errors ** 2)))
    last_error = float(np.sqrt(np.mean(last_errors ** 2)))
    error_ok = last_error <= max(first_error * ERROR_RATIO, first_error + ERROR_MARGIN_M)
    print(
        f'[{"OK" if error_ok else "NG"}] 位置誤差RMSE: 最初 {first_error * 100.0:.2f}cm、'
        f'最後 {last_error * 100.0:.2f}cm、全体の最大 '
        f'{max(e.position_m for e in errors) * 100.0:.2f}cm'
    )

    memory_ok = True
    processes: dict[str, object] = {}
    total_cpu = 0.0
    total_rss = 0.0
    print('プロセス | CPU平均[%] | RSS 最初[MiB] | RSS 最後[MiB]')
    for name in sorted({u.process for u in usage}):
        own = [u for u in usage if u.process == name]
        first = [u.rss_mib for u in own if start <= u.t < first_end]
        last = [u.rss_mib for u in own if last_start <= u.t <= end]
        if not first or not last:
            raise CheckError(f'{name} の比べる区間に使用メモリの記録がありません')
        first_rss = float(np.mean(first))
        last_rss = float(np.mean(last))
        cpu = float(np.mean([u.cpu_percent for u in own]))
        passed = last_rss <= max(first_rss * MEMORY_RATIO, first_rss + MEMORY_MARGIN_MIB)
        memory_ok = memory_ok and passed
        processes[name] = {'cpu_percent': cpu, 'first_rss_mib': first_rss, 'last_rss_mib': last_rss}
        total_cpu += cpu
        total_rss += last_rss
        print(f'{name} | {cpu:.0f} | {first_rss:.0f} | {last_rss:.0f}{"" if passed else "  ← 増加"}')
    print(f'合計 | {total_cpu:.0f} | - | {total_rss:.0f}')
    print(f'[{"OK" if memory_ok else "NG"}] 使用メモリが増え続けていない', flush=True)
    return error_ok and memory_ok, {
        'first_position_rmse_m': first_error,
        'last_position_rmse_m': last_error,
        'max_position_error_m': max(e.position_m for e in errors),
        'processes': processes,
        'total_cpu_percent': total_cpu,
        'total_last_rss_mib': total_rss,
    }


def run(args: argparse.Namespace) -> bool:
    scenario = args.scenario or (
        Path(get_package_share_directory('core_gazebo')) / 'scenarios' / 'crowd_goals.yaml'
    )
    spec = load_spec(scenario)
    log_path = args.log_file or Path(tempfile.mkstemp(prefix='long_run_', suffix='.launch.log')[1])
    print(f'launch のログ: {log_path}', flush=True)
    duration_s = args.minutes * 60.0
    window_s = min(WINDOW_S, duration_s / 3.0)
    process = start_stack(spec, scenario, args.launch_arg, log_path)
    rclpy.init()
    node = LongRunNode()
    try:
        with SpinThread(node):
            wait_until_ready(node, process, log_path)
            pids = find_stack_processes(PROCESSES)
            missing = missing_processes(PROCESSES, pids)
            if missing:
                raise CheckError(f'プロセス {", ".join(missing)} が動いていません')
            print('測るプロセス: ' + ', '.join(pids), flush=True)
            if args.pin_cpus is not None:
                pin_processes(pids, args.pin_cpus)
            node.wait_sim(2.0)

            previous = {name: read_usage(pid) for name, pid in pids.items()}
            usage: list[UsageSample] = []
            goal_results: list[GoalResult] = []
            start = node.now_sim()
            next_usage = start + USAGE_PERIOD_S
            next_error = start
            index = 0

            def monitor() -> None:
                """走行中に定期的に記録する."""
                nonlocal next_usage, next_error
                now = node.now_sim()
                if now >= next_error:
                    node.sample_error()
                    next_error = now + ERROR_PERIOD_S
                if now >= next_usage:
                    collect_usage(now, pids, previous, usage)
                    next_usage = now + USAGE_PERIOD_S

            while node.now_sim() - start < duration_s:
                goal = spec.goals[index % len(spec.goals)]
                index += 1
                goal_results.append(drive_goal_monitored(node, goal, process, monitor))
                result = goal_results[-1]
                print(
                    f'[{(node.now_sim() - start) / 60.0:.1f}分] ゴール {index}: {result.final_state}、'
                    f'{result.duration_s:.1f}秒',
                    flush=True,
                )
            end = node.now_sim()
            summary = analyze_run(node, spec, goal_results, start, end, args.warn_m, TRUTH_STRIDE)
            print_summary(summary)
            ok, judged = judge(usage, list(node.errors), start, end, window_s)
    finally:
        tail = log_tail(log_path, 5) if process.poll() is not None else ''
        stop_process(process)
        rclpy.try_shutdown()
        if tail:
            print(f'launch が途中で終了していました:\n{tail}', flush=True)

    if args.csv is not None:
        error_csv = write_csv(args.csv, usage, list(node.errors))
        print(f'CSV: {args.csv}、{error_csv}', flush=True)
    summary['judge'] = judged
    if args.json is not None:
        args.json.write_text(json.dumps(summary, ensure_ascii=False, indent=1), encoding='utf-8')
    reached = sum(1 for g in goal_results if g.reached)
    print(f'ゴール到達: {reached}/{len(goal_results)}', flush=True)
    return ok


def pin_processes(pids: dict[str, int], cpus: str) -> None:
    """スタックのプロセスを指定の CPU に固定する。計算機の CPU の数に近づけて測るときに使う."""
    for name, pid in pids.items():
        result = subprocess.run(
            ['taskset', '-acp', cpus, str(pid)], capture_output=True, text=True, check=False
        )
        if result.returncode != 0:
            raise CheckError(f'{name}（PID {pid}）を CPU {cpus} に固定できません: {result.stderr.strip()}')
    print(f'{len(pids)} 個のプロセスを CPU {cpus} に固定しました', flush=True)


def drive_goal_monitored(
    node: ScenarioNode, goal: tuple[float, float, float], process: subprocess.Popen[bytes],
    monitor: Callable[[], None],
) -> GoalResult:
    """ゴールを 1 つ走らせる間、別スレッドで記録を続ける."""
    finished = threading.Event()
    errors: list[CheckError] = []

    def worker() -> None:
        while not finished.is_set():
            try:
                monitor()
            except CheckError as error:
                # 別スレッドの例外は失われるので、呼び出し側へ渡す
                errors.append(error)
                return
            time.sleep(0.2)

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    try:
        result = drive_goal(node, goal, GOAL_TIME_LIMIT_S, process)
    finally:
        finished.set()
        thread.join(timeout=5.0)
    if errors:
        raise errors[0]
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--minutes', type=float, default=30.0, help='走らせる時間（シミュレーション時間）')
    parser.add_argument('--scenario', type=Path, default=None, help='既定は crowd_goals.yaml')
    parser.add_argument('--json', type=Path, default=None)
    parser.add_argument('--csv', type=Path, default=None, help='CPU とメモリ。誤差は *_error.csv に書く')
    parser.add_argument('--log-file', type=Path, default=None)
    parser.add_argument('--warn-m', type=float, default=0.3)
    parser.add_argument(
        '--pin-cpus', default=None, metavar='LIST',
        help='スタックのプロセスを固定する CPU（taskset の形式、例: 0-3）。Gazebo などは対象外',
    )
    parser.add_argument('--launch-arg', action='append', default=[], metavar='NAME=VALUE')
    args = parser.parse_args()
    try:
        ok = run(args)
    except CheckError as error:
        print(f'[NG] 確認を中断しました: {error}', flush=True)
        ok = False
    print('結果: 完了条件を満たしました' if ok else '結果: 完了条件を満たしていません', flush=True)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
