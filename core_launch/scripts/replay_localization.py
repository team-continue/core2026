#!/usr/bin/env python3
"""rosbag（record_localization_bag.sh で録ったもの）を再生しながら自己位置のスタックを動かし、跳びの指標を出す.

実機のデータでも、シミュレーションで録ったデータでも使う。再生するのは入力（LiDAR、IMU、固定の TF）だけで、
FAST-LIO・地図照合・異常の判定は localization_stack.launch.py で動かし直す。--localization-params-file で
新しい働きを入れた設定を重ねれば、同じデータで設定を比べられる。指標は check_localization_jumps.py が出す
（区分ごとの表と、--json、--raw）。

ROS_DOMAIN_ID はそのまま引き継ぐ。ほかの試験と重ならないよう、Gazebo のロックは呼ぶ側で取る。
"""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from typing import Final

from ament_index_python.packages import get_package_prefix

INPUT_TOPICS: Final = ('/sensing/livox/lidar', '/sensing/livox/imu', '/sensing/imu', '/tf_static', '/control/cmd_vel')
# 再生する /tf_static を、あとから購読したノードにも届くようにする。録った /tf_static は出したノードごとに
# 別のメッセージなので、深さが 1 だと最後の 1 つしか残らず、機体の TF が抜ける
TF_STATIC_QOS: Final = """/tf_static:
  depth: 100
  durability: transient_local
  reliability: reliable
  history: keep_last
"""
STARTUP_WAIT_S: Final = 10.0
STOP_TIMEOUT_S: Final = 30.0


class ReplayError(RuntimeError):
    pass


def stop_group(process: subprocess.Popen[bytes], timeout_s: float = STOP_TIMEOUT_S) -> None:
    """プロセスグループに SIGINT を送り、止まらなければ強く止める（ノードを残さない）."""
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


def start(command: list[str], log_path: Path) -> subprocess.Popen[bytes]:
    with log_path.open('wb') as log:
        return subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)


def run(args: argparse.Namespace) -> int:
    if not args.bag.exists():
        raise ReplayError(f'rosbag が見つかりません: {args.bag}')
    if not args.map.is_file():
        raise ReplayError(f'地図が見つかりません: {args.map}')
    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    launch_args = {
        'use_sim_time': 'true',
        'map_path': str(args.map.resolve()),
        'initial_x': str(args.initial_x),
        'initial_y': str(args.initial_y),
        'initial_yaw': str(args.initial_yaw),
    }
    if args.localization_params_file is not None:
        launch_args['localization_params_file'] = str(args.localization_params_file.resolve())
    for item in args.launch_arg:
        name, separator, value = item.partition('=')
        if not separator:
            raise ReplayError(f'--launch-arg は NAME=VALUE の形で指定してください: {item}')
        launch_args[name] = value
    stack = start(['ros2', 'launch', 'core_launch', 'localization_stack.launch.py',
                   *(f'{k}:={v}' for k, v in launch_args.items())], out / 'launch.log')
    recorder: subprocess.Popen[bytes] | None = None
    player: subprocess.Popen[bytes] | None = None
    try:
        time.sleep(STARTUP_WAIT_S)
        if stack.poll() is not None:
            raise ReplayError(f'launch が終了しました（終了コード {stack.returncode}、{out / "launch.log"}）')
        recorder_script = Path(get_package_prefix('core_launch')) / 'lib' / 'core_launch' / (
            'check_localization_jumps.py')
        recorder = subprocess.Popen(
            [sys.executable, str(recorder_script), '--json', str(out / 'jumps.json'), '--raw', str(out / 'raw'),
             '--idle-s', str(args.idle_s)],
            start_new_session=True,
        )
        with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as qos:
            qos.write(TF_STATIC_QOS)
        player = start(['ros2', 'bag', 'play', str(args.bag), '--clock', '100', '--rate', str(args.rate),
                        '--qos-profile-overrides-path', qos.name, '--topics', *INPUT_TOPICS],
                       out / 'play.log')
        player.wait()
        print(f'再生が終わりました（終了コード {player.returncode}）。指標の記録が止まるのを待ちます', flush=True)
        try:
            recorder.wait(timeout=args.idle_s + 30.0)
        except subprocess.TimeoutExpired:
            # /lio/odom が 1 回も届かないと、記録は途絶えを待たずに動き続ける。止めて、ある分だけ書かせる
            recorder.send_signal(signal.SIGINT)
            recorder.wait(timeout=30.0)
            raise ReplayError(
                '再生が終わっても指標の記録が止まりませんでした。/lio/odom が届いていない可能性があります'
                f'（{out / "launch.log"} を見てください）') from None
        return recorder.returncode
    finally:
        for process in (player, recorder):
            if process is not None:
                stop_group(process, 10.0)
        stop_group(stack)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('bag', type=Path, help='record_localization_bag.sh で録った rosbag のディレクトリ')
    parser.add_argument('--map', type=Path, required=True, help='地図の PCD')
    parser.add_argument('--initial-x', type=float, required=True, help='録り始めの機体の地図上の位置 x [m]')
    parser.add_argument('--initial-y', type=float, required=True)
    parser.add_argument('--initial-yaw', type=float, required=True, help='録り始めの機体の向き [rad]')
    parser.add_argument('--rate', type=float, default=1.0, help='再生の速さ（1.0 で実時間）')
    parser.add_argument('--localization-params-file', type=Path, default=None,
                        help='地図照合と localization_health に重ねる設定（新しい働きを入れて比べるとき）')
    parser.add_argument('--launch-arg', action='append', default=[], metavar='NAME=VALUE')
    parser.add_argument('--idle-s', type=float, default=5.0, help='/lio/odom がこの時間届かなければ終わりとみなす')
    parser.add_argument('--out', type=Path, required=True, help='結果を書くディレクトリ（jumps.json、raw_*.csv、ログ）')
    try:
        return run(parser.parse_args())
    except (ReplayError, subprocess.TimeoutExpired) as error:
        print(f'[NG] 再生できません: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
