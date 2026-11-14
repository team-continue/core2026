#!/usr/bin/env python3
"""検出と追跡の結果を、シナリオの真の位置と比べる（手順4の完了条件）.

sim_perception.launch.py を起動した状態で実行する。自機を止めた区間と走らせた区間で測る。
見通しのある相手だけを検出率の分母にする。
"""

import argparse
from collections import defaultdict
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import time
from typing import Final, Sequence

from ament_index_python.packages import get_package_share_directory
from core_nav_check.common import (
    CheckError,
    cpu_percent,
    DriveNode,
    find_stack_processes,
    read_usage,
    rms,
    run_with_node,
    stamp_to_sec,
)
from core_nav_check.scenario import load_scenario, Obstacle, state_at
from core_msgs.msg import ProcessStats, TrackedObject, TrackedObjectArray
import numpy as np

# 暫定の目標
TARGET_RECALL: Final = 0.9
TARGET_CLASS: Final = 0.9
TARGET_POSITION_M: Final = 0.2
TARGET_VELOCITY_MPS: Final = 0.5
TARGET_RANGE_M: Final = 7.0
MATCH_RADIUS_M: Final = 1.0
FALSE_RADIUS_M: Final = 1.5
# シナリオの相手は衝突せず重なって通り抜ける。実際には起きないので、重なる前後は ID の評価から除く
ID_ISOLATION_M: Final = 1.5
# 中心がこれより近い相手は実際には重なっていて、起こりえないので、位置と速度の誤差から除く
OVERLAP_DISTANCE_M: Final = 1.0
RANGE_BANDS: Final = ((0.0, 3.0), (3.0, 7.0), (7.0, 12.0), (12.0, 40.0))
OBSTACLE_RADIUS_M: Final = {'robot': 0.45, 'human': 0.3}
KIND_TO_CLASS: Final = {'robot': TrackedObject.CLASS_ROBOT, 'human': TrackedObject.CLASS_HUMAN}


@dataclass
class Frame:
    t: float
    objects: list[TrackedObject]


class Recorder(DriveNode):
    def __init__(self) -> None:
        super().__init__('check_perception')
        self.frames: list[Frame] = []
        self.processing_ms: list[float] = []
        self.create_subscription(TrackedObjectArray, '/perception/tracks', self._on_tracks, 50)
        self.create_subscription(ProcessStats, '/perception/stats', self._on_stats, 50)

    def _on_tracks(self, msg: TrackedObjectArray) -> None:
        self.frames.append(Frame(stamp_to_sec(msg.header.stamp), list(msg.objects)))

    def _on_stats(self, msg: ProcessStats) -> None:
        self.processing_ms.append(msg.processing_ms)


def occluded(
    sensor: tuple[float, float], target_index: int, positions: Sequence[tuple[float, float]],
    obstacles: Sequence[Obstacle],
) -> bool:
    """センサと相手を結ぶ線上に、別の相手がいれば true."""
    tx, ty = positions[target_index]
    dx, dy = tx - sensor[0], ty - sensor[1]
    length_squared = dx * dx + dy * dy
    if length_squared < 1e-6:
        return False
    for index, (px, py) in enumerate(positions):
        if index == target_index:
            continue
        ratio = ((px - sensor[0]) * dx + (py - sensor[1]) * dy) / length_squared
        if not 0.0 < ratio < 1.0:
            continue
        distance = math.hypot(sensor[0] + ratio * dx - px, sensor[1] + ratio * dy - py)
        if distance < OBSTACLE_RADIUS_M[obstacles[index].kind]:
            return True
    return False


@dataclass
class BandStats:
    visible: int = 0
    detected: int = 0
    occluded: int = 0
    occluded_detected: int = 0
    class_total: int = 0
    class_ok: int = 0
    class_unknown: int = 0
    position_errors: list[float] | None = None
    velocity_errors: list[float] | None = None

    def __post_init__(self) -> None:
        self.position_errors = []
        self.velocity_errors = []


def analyze(
    node: Recorder, obstacles: Sequence[Obstacle], start: float, end: float, json_path: Path | None,
    label: str,
) -> dict[str, float]:
    truth = list(node.truth)
    frames = [f for f in list(node.frames) if start <= f.t <= end]
    if not frames:
        raise CheckError(f'{label}: 追跡結果がありません')
    truth_t = np.array([p.t for p in truth])
    truth_x = np.array([p.x for p in truth])
    truth_y = np.array([p.y for p in truth])
    truth_yaw = np.unwrap(np.array([p.yaw for p in truth]))
    # odom は起動時の base_link に一致するので、最初の真値で世界座標に直す
    origin = truth[0]
    c0, s0 = math.cos(origin.yaw), math.sin(origin.yaw)

    bands = {band: BandStats() for band in RANGE_BANDS}
    false_counts = []
    id_segments: dict[int, list[list[int]]] = defaultdict(list)
    for frame in frames:
        own_x = float(np.interp(frame.t, truth_t, truth_x))
        own_y = float(np.interp(frame.t, truth_t, truth_y))
        states = [state_at(o, frame.t) for o in obstacles]
        positions = [(s.x, s.y) for s in states]
        world_objects = []
        for obj in frame.objects:
            world_objects.append((
                origin.x + c0 * obj.position.x - s0 * obj.position.y,
                origin.y + s0 * obj.position.x + c0 * obj.position.y,
                c0 * obj.velocity.x - s0 * obj.velocity.y,
                s0 * obj.velocity.x + c0 * obj.velocity.y,
                obj,
            ))
        used: set[int] = set()
        distances = [math.hypot(p[0] - own_x, p[1] - own_y) for p in positions]
        # 近い相手から順に最寄りの追跡を割り当てる
        for index in np.argsort(distances):
            obstacle = obstacles[int(index)]
            state = states[int(index)]
            distance = distances[int(index)]
            band = next((b for b in RANGE_BANDS if b[0] <= distance < b[1]), None)
            if band is None:
                continue
            stats = bands[band]
            hidden = occluded((own_x, own_y), int(index), positions, obstacles)
            best = None
            best_distance = MATCH_RADIUS_M
            for object_index, (x, y, _, _, _) in enumerate(world_objects):
                if object_index in used:
                    continue
                gap = math.hypot(x - state.x, y - state.y)
                if gap < best_distance:
                    best, best_distance = object_index, gap
            if hidden:
                stats.occluded += 1
                stats.occluded_detected += int(best is not None)
            else:
                stats.visible += 1
                stats.detected += int(best is not None)
            if best is None:
                continue
            used.add(best)
            x, y, vx, vy, obj = world_objects[best]
            nearest_other = min(
                (
                    math.hypot(positions[int(index)][0] - p[0], positions[int(index)][1] - p[1])
                    for other, p in enumerate(positions) if other != int(index)
                ),
                default=float('inf'),
            )
            if nearest_other < ID_ISOLATION_M:
                # 重なった前後の入れ替わりを数えないよう、履歴を区切る
                id_segments[int(index)].append([])
            else:
                if not id_segments[int(index)]:
                    id_segments[int(index)].append([])
                id_segments[int(index)][-1].append(obj.id)
            if not obj.coasting and nearest_other >= OVERLAP_DISTANCE_M:
                stats.position_errors.append(best_distance)
                stats.velocity_errors.append(math.hypot(vx - state.vx, vy - state.vy))
            stats.class_total += 1
            stats.class_ok += int(obj.classification == KIND_TO_CLASS[obstacle.kind])
            stats.class_unknown += int(obj.classification == TrackedObject.CLASS_UNKNOWN)
        false_counts.append(sum(
            1 for i, (x, y, _, _, _) in enumerate(world_objects)
            if i not in used and all(
                math.hypot(x - s.x, y - s.y) > FALSE_RADIUS_M for s in states
            )
        ))

    id_switches = sum(
        sum(1 for a, b in zip(ids, ids[1:]) if a != b)
        for segments in id_segments.values()
        for ids in segments
    )
    duration = frames[-1].t - frames[0].t
    print(f'--- {label}（{duration:.0f}秒、{len(frames)}フレーム）')
    print('距離[m] | 見通しあり検出率 | 隠れた相手の検出率 | 分類正解率 | 不明の割合 | 位置誤差[cm] | 速度誤差[m/s] | 標本数')
    for band, stats in bands.items():
        recall = stats.detected / stats.visible if stats.visible else float('nan')
        hidden_recall = stats.occluded_detected / stats.occluded if stats.occluded else float('nan')
        class_rate = stats.class_ok / stats.class_total if stats.class_total else float('nan')
        unknown = stats.class_unknown / stats.class_total if stats.class_total else float('nan')
        position = rms(np.array(stats.position_errors)) * 100.0 if stats.position_errors else float('nan')
        velocity = rms(np.array(stats.velocity_errors)) if stats.velocity_errors else float('nan')
        print(
            f'{band[0]:.0f}〜{band[1]:.0f} | {recall * 100.0:.1f}% | {hidden_recall * 100.0:.1f}% | '
            f'{class_rate * 100.0:.1f}% | {unknown * 100.0:.1f}% | {position:.1f} | {velocity:.2f} | '
            f'{stats.visible}'
        )
    print(f'誤検出（相手がいない場所の追跡）: 平均 {np.mean(false_counts):.2f} 個/フレーム')
    print(
        f'IDの入れ替わり（相手どうしが {ID_ISOLATION_M}m 以内の間を除く）: {id_switches} 回'
        f'（{id_switches / max(duration, 1e-6) * 60.0:.1f} 回/分）'
    )

    near = [s for band, s in bands.items() if band[1] <= TARGET_RANGE_M]
    visible = sum(s.visible for s in near)
    return {
        'recall': sum(s.detected for s in near) / visible if visible else float('nan'),
        'class_rate': (
            sum(s.class_ok for s in near) / max(sum(s.class_total for s in near), 1)
        ),
        'position_m': rms(np.array([e for s in near for e in s.position_errors]))
        if any(s.position_errors for s in near) else float('nan'),
        'velocity_mps': rms(np.array([e for s in near for e in s.velocity_errors]))
        if any(s.velocity_errors for s in near) else float('nan'),
        'false_per_frame': float(np.mean(false_counts)),
        'id_switches_per_min': id_switches / max(duration, 1e-6) * 60.0,
    }


def run(node: Recorder, scenario: Path, static_s: float, moving_s: float, json_path: Path | None) -> bool:
    obstacles = load_scenario(scenario)
    print(f'シナリオ: {scenario.name}（{len(obstacles)} 体）', flush=True)
    node.wait_for_truth()
    wall_start = time.monotonic()
    while not node.frames:
        if time.monotonic() - wall_start > 120.0:
            raise CheckError('120秒待っても /perception/tracks を受信できません')
        time.sleep(0.1)
    # composition:=true では、検出は点群の変換と同じコンテナ（lidar_container）で動くので、その合計を測る
    label, pid = next(iter(find_stack_processes(('perception_node',)).items()), ('perception_node', None))
    usage_start = read_usage(pid) if pid is not None else None

    node.stop(3.0)
    static_start = node.now_sim()
    node.stop(static_s)
    static_end = node.now_sim()

    moving_start = node.now_sim()
    corners = ((10.0, -5.5), (10.0, 5.5), (-10.0, 5.5), (-10.0, -5.5))
    while node.now_sim() - moving_start < moving_s:
        for x, y in corners:
            node.go_to(x, y, 3.0, 0.3)
    moving_end = node.now_sim()
    node.stop(1.0)

    results = {
        'static': analyze(node, obstacles, static_start, static_end, json_path, '自機が静止'),
        'moving': analyze(node, obstacles, moving_start, moving_end, json_path, '自機が3m/sで走行'),
    }
    processing = np.array(list(node.processing_ms))
    print(
        f'処理時間[ms]: 平均 {np.mean(processing):.2f}、99%点 {np.percentile(processing, 99.0):.2f}、'
        f'最大 {np.max(processing):.2f}'
    )
    summary = {'results': results, 'processing_mean_ms': float(np.mean(processing)),
               'processing_p99_ms': float(np.percentile(processing, 99.0))}
    if usage_start is not None and pid is not None:
        usage_end = read_usage(pid)
        print(f'{label}: CPU {cpu_percent(usage_start, usage_end):.0f}%、RSS {usage_end.rss_mib:.0f}MiB')
        summary['cpu_percent'] = cpu_percent(usage_start, usage_end)
        summary['cpu_process'] = label
    ok = True
    for name, result in results.items():
        checks = [
            (f'{name} 検出率（{TARGET_RANGE_M:.0f}m以内、見通しあり）', result['recall'], TARGET_RECALL, True),
            (f'{name} 分類正解率', result['class_rate'], TARGET_CLASS, True),
            (f'{name} 位置誤差[m]', result['position_m'], TARGET_POSITION_M, False),
            (f'{name} 速度誤差[m/s]', result['velocity_mps'], TARGET_VELOCITY_MPS, False),
        ]
        for label, value, target, larger_is_better in checks:
            passed = value >= target if larger_is_better else value <= target
            ok = ok and passed
            print(f'[{"OK" if passed else "NG"}] {label}: {value:.3f}（目標 {target}）')
    if json_path is not None:
        json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=1))
    return ok


def main() -> int:
    default_scenario = (
        Path(get_package_share_directory('core_gazebo')) / 'scenarios' / 'crowd_9r2h.yaml'
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', type=Path, default=default_scenario)
    parser.add_argument('--static-seconds', type=float, default=60.0)
    parser.add_argument('--moving-seconds', type=float, default=60.0)
    parser.add_argument('--json', type=Path, default=None)
    args = parser.parse_args()
    return run_with_node(
        Recorder,
        lambda node: run(node, args.scenario, args.static_seconds, args.moving_seconds, args.json),
    )


if __name__ == '__main__':
    sys.exit(main())
