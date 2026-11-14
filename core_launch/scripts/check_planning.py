#!/usr/bin/env python3
"""グローバル計画と局所計画を確かめる（手順5の完了条件）.

静止物のあるワールドと空のワールドで、10 個以上のゴールを順に回る。次を確かめる。
- 全ゴールに到達し、柵と静止物に接触しない
- 直線で 2.7m/s 以上出る
- 計画の計算時間の 99% 点を記録する

run_scenario.py を別プロセスで呼ぶ。--jobs 2 で 2 つのワールドを同時に走らせる。
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Final, Mapping, Sequence

import yaml

TARGET_MAX_SPEED_MPS: Final = 2.7
# 引数なしで動くよう、ゴールはここに持つ
EMPTY_FIELD_GOALS: Final = (
    (10.0, -5.0, 0.0), (10.0, 5.0, 0.0), (-10.0, 5.0, 0.0), (-10.0, -5.0, 0.0), (0.0, 0.0, 0.0),
    (10.0, 0.0, 0.0), (-10.0, 0.0, 0.0), (0.0, 5.0, 0.0), (0.0, -5.0, 0.0), (5.0, 2.0, 0.0),
    (-5.0, -2.0, 0.0),
)
# field_static.sdf の静止物を避けて回る
STATIC_FIELD_GOALS: Final = (
    (6.0, -6.0, 0.0), (6.0, -1.0, 0.0), (10.5, 3.0, 0.0), (5.5, 5.5, 0.0), (-1.0, 5.5, 0.0),
    (-1.0, 1.5, 0.0), (1.5, -3.0, 0.0), (6.0, -3.0, 0.0), (-6.0, 5.5, 0.0), (-10.5, 3.0, 0.0),
    (-6.0, -2.5, 0.0),
)
# 長い直線を走れるように、順に回る時間は余裕を持たせる
TIMEOUT_S: Final = 300.0


@dataclass(frozen=True)
class Case:
    name: str
    world: str
    goals: tuple[tuple[float, float, float], ...]
    ros_domain: str
    partition: str


def write_scenario(case: Case, directory: Path) -> Path:
    path = directory / f'{case.name}.yaml'
    content = {
        'start': [-6.0, -2.5, 0.0],
        'goals': [list(goal) for goal in case.goals],
        'timeout_s': TIMEOUT_S,
        'repeat': 1,
        'world': case.world,
        'obstacles': [],
    }
    path.write_text(yaml.safe_dump(content), encoding='utf-8')
    return path


def run_case(case: Case, directory: Path) -> Mapping[str, object]:
    """1 つのワールドで run_scenario.py を実行し、JSON の結果を返す."""
    scenario = write_scenario(case, directory)
    json_path = directory / f'{case.name}.json'
    log_path = directory / f'{case.name}.run.log'
    environment = os.environ.copy()
    environment['ROS_DOMAIN_ID'] = case.ros_domain
    environment['GZ_PARTITION'] = case.partition
    command = [
        sys.executable, str(Path(__file__).with_name('run_scenario.py')),
        '--scenario-file', str(scenario),
        '--json', str(json_path),
        '--no-record-bag',
        '--log-file', str(directory / f'{case.name}.launch.log'),
    ]
    with log_path.open('w', encoding='utf-8') as log:
        process = subprocess.run(
            command, stdout=log, stderr=subprocess.STDOUT, env=environment, check=False,
        )
    if not json_path.exists():
        raise RuntimeError(
            f'{case.name}: 結果がありません（終了コード {process.returncode}）。ログ: {log_path}'
        )
    return json.loads(json_path.read_text(encoding='utf-8'))


def as_float(value: object) -> float:
    return float(value) if isinstance(value, (int, float)) else float('nan')


def evaluate(results: Mapping[str, Mapping[str, object]]) -> bool:
    print('ワールド | 到達 | 接触（相手/静止物） | 柵までの最小[m] | 静止物までの最小[m] | 最高速度[m/s] | 平均速度[m/s] | 計画99%点[ms] | 走行時間[s]')
    ok = True
    for name, result in results.items():
        reached = as_float(result['goals_reached'])
        total = as_float(result['goals_total'])
        contacts = as_float(result['contacts'])
        static_contacts = as_float(result['static_contacts'])
        fence = as_float(result['min_fence_clearance_m'])
        max_speed = as_float(result['max_speed_mps'])
        print(
            f"{name} | {reached:.0f}/{total:.0f} | {contacts:.0f}/{static_contacts:.0f} | {fence:.2f} | "
            f"{as_float(result['min_static_gap_m']):.2f} | {max_speed:.2f} | "
            f"{as_float(result['mean_speed_mps']):.2f} | {as_float(result['planning_ms_p99']):.2f} | "
            f"{as_float(result['duration_s']):.0f}",
            flush=True,
        )
        checks = (
            (f'{name}: 全ゴールに到達', reached == total and total >= 10.0),
            (f'{name}: 接触なし', contacts == 0.0 and static_contacts == 0.0 and fence > 0.0),
            (f'{name}: 直線で {TARGET_MAX_SPEED_MPS}m/s 以上', max_speed >= TARGET_MAX_SPEED_MPS),
        )
        for label, passed in checks:
            print(f'[{"OK" if passed else "NG"}] {label}')
            ok = ok and passed
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jobs', type=int, default=1, help='同時に走らせるワールドの数')
    parser.add_argument('--output-dir', type=Path, default=None, help='シナリオと結果の出力先')
    args = parser.parse_args()

    directory = args.output_dir or Path(tempfile.mkdtemp(prefix='core_nav_check_planning_'))
    directory.mkdir(parents=True, exist_ok=True)
    print(f'出力先: {directory}', flush=True)
    base_domain = int(os.environ.get('ROS_DOMAIN_ID', '0') or '0')
    cases: Sequence[Case] = (
        Case('empty', 'field.sdf', EMPTY_FIELD_GOALS, str(base_domain), f'planning{base_domain}'),
        Case(
            'static', 'field_static.sdf', STATIC_FIELD_GOALS,
            str(base_domain + 1), f'planning{base_domain + 1}',
        ),
    )
    try:
        with ThreadPoolExecutor(max_workers=max(args.jobs, 1)) as executor:
            outcomes = list(executor.map(lambda case: run_case(case, directory), cases))
    except RuntimeError as error:
        print(f'[NG] 確認を中断しました: {error}', flush=True)
        print('結果: 完了条件を満たしていません', flush=True)
        return 1
    ok = evaluate({case.name: outcome for case, outcome in zip(cases, outcomes)})
    print('結果: 完了条件を満たしました' if ok else '結果: 完了条件を満たしていません', flush=True)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
