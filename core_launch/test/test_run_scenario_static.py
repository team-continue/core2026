"""run_scenario.py の静止物との接触の判定（analyze_static）の試験."""

import math
from pathlib import Path
import sys

from core_nav_check.common import Pose

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from run_scenario import (  # noqa: E402
    analyze_static,
    box_distance,
    chassis_samples,
    footprint_outline,
    OWN_HALF_M,
    StaticBox,
    static_gaps,
)

# field_obstacles.sdf の static_box_f と同じ箱（中心 (0.5, -2.75)、1m x 2m、高さ 1m）
WORLD = """<?xml version="1.0"?>
<sdf version="1.9"><world name="test">
  <model name="static_box_f"><static>true</static><pose>0.5 -2.75 0.5 0 0 0</pose>
    <link name="link"><collision name="collision"><geometry><box><size>1 2 1</size></box></geometry>
    </collision></link></model>
</world></sdf>
"""


def world_file(tmp_path: Path) -> Path:
    path = tmp_path / 'world.sdf'
    path.write_text(WORLD)
    return path


def approach(final_gap: float, speed: float) -> list[Pose]:
    """50Hz で -y へ speed で箱へ向かい、機体の下の面と箱の上の面（y = -1.75）の距離 final_gap で止まる."""
    stop_y = -1.75 + OWN_HALF_M + final_gap
    poses = []
    t = 0.0
    y = stop_y + 2.0
    while y > stop_y:
        poses.append(Pose(t, -0.03, y, 0.0, 0.0))
        t += 0.02
        y -= speed * 0.02
    # 物理がめり込みを止めたあと、押しつけたまま 1 秒
    poses.extend(Pose(t + 0.02 * i, -0.03, stop_y, 0.0, 0.0) for i in range(50))
    return poses


def test_counts_collision_stopped_by_physics(tmp_path: Path) -> None:
    # 記録（obs_A_3）と同じく、物理で止められて距離が 0 より下がらない接触も数える
    result = analyze_static(approach(0.0006, 1.2), world_file(tmp_path))
    assert result['static_contacts'] == 1
    assert result['min_static_gap_m'] < 0.01
    touch = result['static_touches'][0]
    assert touch['box'] == 'static_box_f'
    assert math.isclose(touch['speed_mps'], 1.2, rel_tol=0.05)


def test_does_not_count_close_pass(tmp_path: Path) -> None:
    result = analyze_static(approach(0.05, 1.2), world_file(tmp_path))
    assert result['static_contacts'] == 0
    assert result['static_touches'] == []
    assert math.isclose(result['min_static_gap_m'], 0.05, abs_tol=1e-6)


def test_counts_each_touch_once(tmp_path: Path) -> None:
    # 触れて離れ、また触れたら 2 回
    poses = approach(0.0, 1.0)
    t = poses[-1].t
    away = [Pose(t + 0.02 * (i + 1), -0.03, poses[-1].y + 0.01 * (i + 1), 0.0, 0.0) for i in range(20)]
    back = [Pose(away[-1].t + 0.02 * (i + 1), -0.03, away[-1].y - 0.02 * (i + 1), 0.0, 0.0) for i in range(10)]
    result = analyze_static(poses + away + back, world_file(tmp_path))
    assert result['static_contacts'] == 2


def test_vectorized_gap_matches_point_distance() -> None:
    boxes = [StaticBox('a', 0.5, -2.75, 0.0, 0.5, 1.0), StaticBox('b', -3.0, -3.0, math.pi / 4.0, 0.6, 0.6)]
    poses = [Pose(0.1 * i, -2.0 + 0.37 * i, -1.0 - 0.21 * i, 0.0, 0.3 * i) for i in range(12)]
    gaps, nearest = static_gaps(poses, boxes)
    outline = footprint_outline(OWN_HALF_M)
    for pose, gap, index in zip(poses, gaps, nearest):
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        distances = [
            min(box_distance(pose.x + c * ox - s * oy, pose.y + s * ox + c * oy, box) for ox, oy in outline)
            for box in boxes
        ]
        assert math.isclose(gap, min(distances), abs_tol=1e-9)
        assert distances[int(index)] == min(distances)


# --- CoRE-1 の試験のフィールド（押し出した構造物と進入禁止ゾーン）
from run_scenario import (  # noqa: E402
    analyze_structures,
    inside_polygon,
    polygon_distance,
)
import numpy as np  # noqa: E402

POLYLINE_WORLD = """<?xml version="1.0"?>
<sdf version="1.9"><world name="test">
  <model name="wall_1"><static>true</static>
    <link name="link"><collision name="collision"><geometry><polyline>
      <point>2 -1</point><point>2.2 -1</point><point>2.2 1</point><point>2 1</point><height>1.0</height>
    </polyline></geometry></collision></link></model>
</world></sdf>
"""


def test_polygon_distance_is_signed() -> None:
    square = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    px = np.array([0.5, 2.0, 0.5])
    py = np.array([0.5, 0.5, -0.25])
    distance = polygon_distance(px, py, square)
    assert distance[0] == -0.5
    assert distance[1] == 1.0
    assert distance[2] == 0.25
    assert inside_polygon(px, py, square).tolist() == [True, False, False]


def test_structures_count_gap_and_contacts(tmp_path: Path) -> None:
    world = tmp_path / 'world.sdf'
    world.write_text(POLYLINE_WORLD)
    # +x へ進み、機体の前の面（x + 0.4）が壁（x = 2.0）に触れて止まる
    poses = [Pose(t=0.02 * i, x=0.5 + 0.02 * i, y=0.0, z=0.0, yaw=0.0) for i in range(56)]
    result = analyze_structures(poses, world, 'field.sdf')
    assert result['min_structure_gap_m'] < 0.01
    assert result['structure_contacts'] == 1
    # field.sdf には入らない範囲がない
    assert result['keep_out_entries'] == 0


def test_chassis_samples_keep_yaw_without_waist() -> None:
    poses = [Pose(0.0, 1.0, 2.0, 0.0, 0.3), Pose(0.1, 1.0, 2.0, 0.0, 0.4)]
    assert chassis_samples(poses, []) == poses


def test_chassis_samples_add_latest_waist_angle() -> None:
    # 腰のある機体では、外周の向きは上半身の向き + 直前に受けた腰の角度。最初に受けるより前は最初の値
    poses = [Pose(0.0, 0.0, 0.0, 0.0, 0.5), Pose(1.0, 0.0, 0.0, 0.0, 0.5), Pose(2.5, 0.0, 0.0, 0.0, 3.0)]
    waist = [(0.5, -0.2), (2.0, 0.5)]
    yaws = [p.yaw for p in chassis_samples(poses, waist)]
    assert math.isclose(yaws[0], 0.3)
    assert math.isclose(yaws[1], 0.3)
    # 3.5 rad は -π〜π に折り返す
    assert math.isclose(yaws[2], 3.5 - 2.0 * math.pi)
    assert [(p.t, p.x, p.y) for p in chassis_samples(poses, waist)] == [(p.t, p.x, p.y) for p in poses]
