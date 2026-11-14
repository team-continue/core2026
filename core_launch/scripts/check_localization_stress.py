#!/usr/bin/env python3
"""自己位置の試験（密集・速さ・旋回）: sim_localization を起動し、決まった走り方で走らせ、跳びの指標で判定する.

機体は y=0 を往復し（x=-9〜9）、速さを上げていく（既定 2〜5m/s）。その場旋回（既定 2・4・6.28rad/s）、
並進しながらの旋回、急な加減速も入れる。他ロボットはシナリオの YAML（core_gazebo/scenarios/loc_*.yaml）の
とおりに動く。走っている間 check_localization_jumps.py で指標を記録し、全体の位置の誤差の最大と、
map→odom の段差の最大で判定する。

ROS_DOMAIN_ID と GZ_PARTITION はそのまま引き継ぐ。Gazebo のロック（ほかの試験と重ならないようにする）は
呼ぶ側で取る。シミュレーションの LiDAR は回っても点群が歪まないので、速い旋回の歪みはこの試験では出ない。
"""

import argparse
from collections.abc import Callable, Sequence
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Final

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from core_nav_check.common import CheckError, DriveNode, run_with_node
from core_msgs.msg import LocalizationHealth

START: Final = (-9.0, 0.0, 0.0)
READY_TIMEOUT_S: Final = 180.0
STOP_TIMEOUT_S: Final = 30.0
RECORDER_TIMEOUT_S: Final = 30.0


class StressNode(DriveNode):
    """機体を動かし、自己位置の状態を見る."""

    def __init__(self) -> None:
        super().__init__('localization_stress')
        self.health_level: int | None = None
        self.create_subscription(
            LocalizationHealth, '/localization/health',
            lambda msg: setattr(self, 'health_level', int(msg.level)), 10)


def resolve_scenario(name: str) -> Path:
    """パスか、core_gazebo の scenarios の名前（拡張子なしでもよい）."""
    path = Path(name)
    if path.is_file():
        return path.resolve()
    shared = Path(get_package_share_directory('core_gazebo')) / 'scenarios'
    candidate = shared / (name if name.endswith('.yaml') else f'{name}.yaml')
    if not candidate.is_file():
        raise CheckError(f'シナリオが見つかりません: {name}（{shared} も探しました）')
    return candidate


def start_stack(scenario: Path, launch_args: Sequence[str], log_path: Path) -> subprocess.Popen[bytes]:
    arguments = {'x': START[0], 'y': START[1], 'yaw': START[2], 'headless': 'true', 'scenario_file': scenario}
    for item in launch_args:
        name, separator, value = item.partition('=')
        if not separator:
            raise CheckError(f'--launch-arg は NAME=VALUE の形で指定してください: {item}')
        arguments[name] = value
    command = ['ros2', 'launch', 'core_launch', 'sim_localization.launch.py',
               *(f'{name}:={value}' for name, value in arguments.items())]
    with log_path.open('wb') as log:
        return subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)


def stop_group(process: subprocess.Popen[bytes], timeout_s: float = STOP_TIMEOUT_S) -> None:
    """プロセスグループに SIGINT を送り、止まらなければ強く止める（Gazebo とノードを残さない）."""
    if process.poll() is not None:
        return
    group = os.getpgid(process.pid)
    for sig, wait_s in ((signal.SIGINT, timeout_s), (signal.SIGTERM, 10.0), (signal.SIGKILL, 10.0)):
        os.killpg(group, sig)
        try:
            process.wait(timeout=wait_s)
            return
        except subprocess.TimeoutExpired:
            print(f'[警告] {sig.name} で止まらないので、もっと強く止めます', flush=True)


def wait_until_ready(node: StressNode, stack: subprocess.Popen[bytes]) -> None:
    start = time.monotonic()
    while not (node.truth and node.health_level == LocalizationHealth.LEVEL_OK):
        if stack.poll() is not None:
            raise CheckError(f'launch が終了しました（終了コード {stack.returncode}）')
        if time.monotonic() - start > READY_TIMEOUT_S:
            raise CheckError(f'{READY_TIMEOUT_S}秒待っても自己位置が OK になりません（状態 {node.health_level}）')
        time.sleep(0.1)


def phases(node: DriveNode, speeds: Sequence[float], spin_rates: Sequence[float]
           ) -> list[tuple[str, Callable[[], None]]]:
    def straight(speed: float) -> Callable[[], None]:
        def run() -> None:
            node.go_to(9.0, 0.0, speed)
            node.stop()
            node.go_to(-9.0, 0.0, speed)
            node.stop()
        return run

    def spin(rate: float) -> Callable[[], None]:
        def run() -> None:
            node.go_to(0.0, 0.0, 2.0)
            node.stop()
            node.hold(0.0, 0.0, rate, 4.0 * math.pi / rate)  # 2 回転
            node.stop()
        return run

    def translate_and_spin() -> None:
        for x in (9.0, -9.0):
            node.go_to(x, 0.0, 3.0, 3.0)
        node.stop()

    def stop_and_go() -> None:
        node.go_to(-9.0, 0.0, 2.0)
        node.stop()
        for direction in (1.0, -1.0):
            for _ in range(5):
                node.hold(direction * 3.0, 0.0, 0.0, 1.0)
                node.stop(1.0)

    return [
        ('静止', lambda: node.stop(15.0)),
        *[(f'直進 {speed:g}m/s', straight(speed)) for speed in speeds],
        *[(f'その場旋回 {rate:g}rad/s', spin(rate)) for rate in spin_rates],
        ('並進 3m/s と旋回 3rad/s', translate_and_spin),
        ('急な加減速 3m/s', stop_and_go),
        ('停止後', lambda: node.stop(5.0)),
    ]


def run(node: StressNode, args: argparse.Namespace, stack: subprocess.Popen[bytes]) -> bool:
    wait_until_ready(node, stack)
    node.stop(2.0)
    recorder_json = args.json.with_suffix('.jumps.json')
    recorder = subprocess.Popen(
        [sys.executable, str(Path(get_package_prefix('core_launch')) / 'lib' / 'core_launch'
                             / 'check_localization_jumps.py'), '--json', str(recorder_json),
         '--raw', str(args.json.with_suffix(''))],
    )
    try:
        node.stop(2.0)
        for name, drive in phases(node, args.speeds, args.spin_rates):
            drive()
            print(f'区間 {name} 完了', flush=True)
        node.command_world(0.0, 0.0, 0.0)
    finally:
        recorder.send_signal(signal.SIGINT)
        try:
            recorder.wait(timeout=RECORDER_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            recorder.kill()
            raise CheckError('指標の記録が止まりません') from None
    if recorder.returncode != 0 or not recorder_json.is_file():
        raise CheckError(f'指標を記録できませんでした（終了コード {recorder.returncode}）')
    summary = json.loads(recorder_json.read_text(encoding='utf-8'))
    whole = summary['全体']
    error_cm, step_cm = whole['位置の誤差_最大_cm'], whole['段差_最大_cm']
    checks = (
        ('位置の誤差の最大', error_cm, args.max_error_cm),
        ('map→odom の段差の最大', step_cm, args.max_step_cm),
    )
    ok = True
    for label, value, limit in checks:
        passed = math.isfinite(value) and value <= limit
        ok &= passed
        print(f'[{"OK" if passed else "NG"}] {label}: {value:.2f}cm（目標 {limit:g}cm 以下）', flush=True)
    args.json.write_text(json.dumps({'ok': ok, 'summary': summary}, ensure_ascii=False, indent=1),
                         encoding='utf-8')
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--scenario', default='loc_wall', help='シナリオの YAML（パスか core_gazebo の名前）')
    parser.add_argument('--speeds', type=float, nargs='+', default=[2.0, 3.0, 4.0, 5.0], help='直進の速さ [m/s]')
    parser.add_argument('--spin-rates', type=float, nargs='+', default=[2.0, 4.0, 6.28], help='旋回の速さ [rad/s]')
    parser.add_argument('--max-error-cm', type=float, default=10.0)
    parser.add_argument('--max-step-cm', type=float, default=5.0)
    parser.add_argument('--launch-arg', action='append', default=[], metavar='NAME=VALUE')
    parser.add_argument('--json', type=Path, required=True, help='結果（指標の記録は *.jumps.json）')
    parser.add_argument('--log-file', type=Path, default=None, help='launch の出力（既定は --json の隣）')
    args = parser.parse_args()
    try:
        scenario = resolve_scenario(args.scenario)
    except CheckError as error:
        print(f'[NG] {error}', flush=True)
        return 1
    log_path = args.log_file or args.json.with_suffix('.launch.log')
    stack = start_stack(scenario, args.launch_arg, log_path)
    try:
        return run_with_node(StressNode, lambda node: run(node, args, stack))
    finally:
        stop_group(stack)


if __name__ == '__main__':
    sys.exit(main())
