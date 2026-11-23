"""シナリオファイルの読み込みと、他ロボットと人間の真の位置の計算.

sim（core_field_sim）の core_field_sim/src/moving_obstacles.cpp と同じ式を使う。変えるときは両方を直す。
z（足元の高さ）と spectator（観客。フィールドの外の人）は省略できる。
"""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Final, Sequence

import yaml

ROBOT_HALF_SIZE_M: Final = 0.4
HUMAN_RADIUS_M: Final = 0.25


@dataclass(frozen=True)
class Obstacle:
    name: str
    kind: str
    start: tuple[float, float]
    end: tuple[float, float]
    peak_speed: float
    phase: float
    # 足元の高さ。観客席の上の人に使う
    z: float = 0.0
    # 観客（フィールドの外の人）。機体とは当たらないので、接触と最小距離の計算から除く
    spectator: bool = False


@dataclass(frozen=True)
class ObstacleState:
    x: float
    y: float
    vx: float
    vy: float


def parse_obstacles(items: Sequence[dict[str, Any]]) -> list[Obstacle]:
    obstacles = []
    for item in items:
        start = (float(item['start'][0]), float(item['start'][1]))
        end_item = item.get('end', item['start'])
        obstacles.append(Obstacle(
            name=str(item['name']),
            kind=str(item['type']),
            start=start,
            end=(float(end_item[0]), float(end_item[1])),
            peak_speed=float(item.get('peak_speed', 0.0)),
            phase=float(item.get('phase', 0.0)),
            z=float(item.get('z', 0.0)),
            spectator=parse_flag(item.get('spectator', False), f"{item['name']} の spectator"),
        ))
    return obstacles


def parse_flag(value: object, where: str) -> bool:
    # 'false' などの文字列を真と取り違えないよう、YAML の真偽値だけを受ける
    if not isinstance(value, bool):
        raise ValueError(f'{where} は true か false にしてください: {value!r}')
    return value


def field_obstacles(obstacles: Sequence[Obstacle]) -> list[Obstacle]:
    """観客を除いた、フィールドの中の相手."""
    return [obstacle for obstacle in obstacles if not obstacle.spectator]


def load_scenario(path: Path) -> list[Obstacle]:
    with path.open(encoding='utf-8') as file:
        root = yaml.safe_load(file)
    return parse_obstacles(root.get('obstacles') or [])


def state_at(obstacle: Obstacle, t: float) -> ObstacleState:
    dx = obstacle.end[0] - obstacle.start[0]
    dy = obstacle.end[1] - obstacle.start[1]
    length = math.hypot(dx, dy)
    if length < 1e-6 or obstacle.peak_speed <= 0.0:
        return ObstacleState(obstacle.start[0], obstacle.start[1], 0.0, 0.0)
    angular_rate = 2.0 * obstacle.peak_speed / length
    angle = angular_rate * t + 2.0 * math.pi * obstacle.phase
    ratio = 0.5 * (1.0 - math.cos(angle))
    rate = 0.5 * angular_rate * math.sin(angle)
    return ObstacleState(
        obstacle.start[0] + dx * ratio,
        obstacle.start[1] + dy * ratio,
        dx * rate,
        dy * rate,
    )


def footprint_gap(
    own_x: float, own_y: float, own_yaw: float, own_half: float, obstacle: Obstacle,
    state: ObstacleState,
) -> float:
    """自機の箱と相手の間の距離を返す。重なっていれば負.

    相手のロボットは向きが軸に沿った箱、人は円。角の丸めの差は小さいので、
    相手のロボットを半径 half の円で近似せず、箱どうしの近似として軸方向の距離を使う。
    """
    c, s = math.cos(own_yaw), math.sin(own_yaw)
    rel_x = state.x - own_x
    rel_y = state.y - own_y
    local_x = c * rel_x + s * rel_y
    local_y = -s * rel_x + c * rel_y
    if obstacle.kind == 'human':
        gap_x = max(abs(local_x) - own_half, 0.0)
        gap_y = max(abs(local_y) - own_half, 0.0)
        inside = abs(local_x) <= own_half and abs(local_y) <= own_half
        distance = math.hypot(gap_x, gap_y) - HUMAN_RADIUS_M
        return -HUMAN_RADIUS_M if inside else distance
    # 分離軸の定理で、回転した 2 つの正方形の距離の下限を求める
    axes = ((c, s), (-s, c), (1.0, 0.0), (0.0, 1.0))
    separation = float('-inf')
    for ax, ay in axes:
        projection = abs(rel_x * ax + rel_y * ay)
        own_radius = own_half * (abs(c * ax + s * ay) + abs(-s * ax + c * ay))
        other_radius = ROBOT_HALF_SIZE_M * (abs(ax) + abs(ay))
        separation = max(separation, projection - own_radius - other_radius)
    return separation
