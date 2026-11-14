"""core_nav_launch.self_filter（自機の除去の箱と、URDF の取り付けの変換）の試験."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from core_nav_launch.self_filter import (
    compose,
    fast_lio_self_filter_parameters,
    fixed_transform,
    IDENTITY,
    LEGACY_BLIND_M,
    load_robot_description,
    load_self_box,
    parse_xacro_args,
    rpy_matrix,
    SelfFilterError,
    Transform,
)
import pytest
import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config'
ROBOT_XACRO = Path(get_package_share_directory('core_launch')) / 'urdf' / 'core2026.urdf.xacro'

# 機体の前の端、床から 0.25m、正立（lidar_nav の既定の取り付け。変換の計算を確かめる）
FRONT_MOUNT: dict[str, str] = {'lidar_x': '0.44', 'lidar_height': '0.25', 'lidar_roll': '0.0'}

def apply(transform: Transform, point: tuple[float, float, float]) -> tuple[float, ...]:
    r, t = transform.rotation, transform.translation
    return tuple(sum(r[3 * i + k] * point[k] for k in range(3)) + t[i] for i in range(3))


def assert_close(actual: tuple[float, ...], expected: tuple[float, ...]) -> None:
    assert len(actual) == len(expected)
    for a, e in zip(actual, expected):
        assert a == pytest.approx(e, abs=1e-9)


def urdf(joints: str, links: tuple[str, ...] = ('base_link', 'mount', 'livox_frame')) -> str:
    link_text = ''.join(f'<link name="{name}"/>' for name in links)
    return f'<robot name="r">{link_text}{joints}</robot>'


def test_rpy_matrix_follows_urdf_convention() -> None:
    # yaw 90 度で x 軸が y 軸へ、roll 90 度で y 軸が z 軸へ
    assert_close(apply(Transform(rpy_matrix(0.0, 0.0, math.pi / 2), (0, 0, 0)), (1, 0, 0)), (0, 1, 0))
    assert_close(apply(Transform(rpy_matrix(math.pi / 2, 0.0, 0.0), (0, 0, 0)), (0, 1, 0)), (0, 0, 1))
    # 固定軸の x, y, z の順: Rz(yaw) Ry(pitch) Rx(roll)
    roll, pitch, yaw = 0.3, -0.2, 1.1
    combined = compose(
        Transform(rpy_matrix(0, 0, yaw), (0, 0, 0)),
        compose(Transform(rpy_matrix(0, pitch, 0), (0, 0, 0)), Transform(rpy_matrix(roll, 0, 0), (0, 0, 0))),
    )
    assert_close(rpy_matrix(roll, pitch, yaw), combined.rotation)


def test_compose_applies_inner_first() -> None:
    outer = Transform(rpy_matrix(0, 0, math.pi / 2), (1.0, 0.0, 0.0))
    inner = Transform(IDENTITY.rotation, (0.5, 0.0, 0.2))
    # 内側で (0.5, 0, 0.2) 動かしてから、外側で 90 度回して x に 1 足す
    assert_close(apply(compose(outer, inner), (0, 0, 0)), (1.0, 0.5, 0.2))
    assert_close(compose(IDENTITY, inner).translation, inner.translation)


def test_robot_xacro_gives_lidar_mount() -> None:
    # core2026.urdf.xacro の base_to_livox は (lidar_x, 0, lidar_height)、前後の軸まわりに lidar_roll。
    # 既定は core2026 の実機（機体の中心、床から 0.5m、上下逆さ）
    default = fixed_transform(load_robot_description(ROBOT_XACRO, {}), 'base_link', 'livox_frame')
    assert_close(default.translation, (0.0, 0.0, 0.5))
    assert_close(default.rotation, rpy_matrix(math.pi, 0.0, 0.0))
    front = fixed_transform(load_robot_description(ROBOT_XACRO, FRONT_MOUNT), 'base_link', 'livox_frame')
    assert_close(front.translation, (0.44, 0.0, 0.25))
    assert_close(front.rotation, IDENTITY.rotation)
    raised = fixed_transform(
        load_robot_description(ROBOT_XACRO, {**FRONT_MOUNT, 'lidar_height': '0.95'}), 'base_link', 'livox_frame')
    assert_close(raised.translation, (0.44, 0.0, 0.95))
    # 機体の中心の上 0.85m
    centered = fixed_transform(
        load_robot_description(ROBOT_XACRO, {**FRONT_MOUNT, 'lidar_x': '0.0', 'lidar_height': '0.85'}),
        'base_link', 'livox_frame')
    assert_close(centered.translation, (0.0, 0.0, 0.85))
    # 途中の関節も合成する（livox_frame の下の IMU）
    imu = fixed_transform(load_robot_description(ROBOT_XACRO, FRONT_MOUNT), 'base_link', 'livox_imu_link')
    assert_close(imu.translation, (0.44 + 0.011, 0.02329, 0.25 - 0.04412))


def test_chain_of_rotated_joints() -> None:
    description = urdf(
        '<joint name="a" type="fixed"><parent link="base_link"/><child link="mount"/>'
        f'<origin xyz="0.1 0 0.5" rpy="0 0 {math.pi / 2}"/></joint>'
        '<joint name="b" type="fixed"><parent link="mount"/><child link="livox_frame"/>'
        f'<origin xyz="0.2 0 0.3" rpy="{math.pi} 0 0"/></joint>'
    )
    transform = fixed_transform(description, 'base_link', 'livox_frame')
    # LiDAR の原点は mount の x へ 0.2（base では y へ 0.2）、上へ 0.3
    assert_close(transform.translation, (0.1, 0.2, 0.8))
    # LiDAR の z（上下反転で下向き）は base の -z、LiDAR の x は base の y
    assert_close(apply(transform, (0, 0, 1)), (0.1, 0.2, -0.2))
    assert_close(apply(transform, (1, 0, 0)), (0.1, 1.2, 0.8))


@pytest.mark.parametrize(
    ('description', 'message'),
    [
        (urdf('', links=('base_link',)), 'livox_frame がありません'),
        (urdf('<joint name="a" type="fixed"><parent link="base_link"/><child link="mount"/></joint>'),
         'livox_frame の親の関節がありません'),
        (urdf('<joint name="a" type="fixed"><parent link="base_link"/><child link="mount"/></joint>'
              '<joint name="b" type="revolute"><parent link="mount"/><child link="livox_frame"/></joint>'),
         '固定でない関節 b'),
        (urdf('<joint name="b" type="fixed"><parent link="base_link"/><child link="livox_frame"/>'
              '<origin xyz="0 0"/></joint>'),
         '3 個の数値'),
        ('<robot', 'XML が正しくありません'),
    ],
)
def test_fixed_transform_errors(description: str, message: str) -> None:
    with pytest.raises(SelfFilterError, match=message):
        fixed_transform(description, 'base_link', 'livox_frame')


def test_detection_box_is_unchanged() -> None:
    # 移す前の検出の箱: footprint 0.8m の半分 + perception.yaml の self_filter_margin_m 0.05
    box = load_self_box(CONFIG / 'footprint.yaml')
    assert box.half_length_m == pytest.approx(0.45)
    assert box.half_width_m == pytest.approx(0.45)
    # 上面は機体の高さ 0.8m に余裕 0.05m（以前の LiDAR の高さ 0.85m と同じ）
    assert box.top_height_m == pytest.approx(0.85)
    # 余裕は footprint.yaml の 1 か所だけに置く
    perception = yaml.safe_load((CONFIG / 'perception.yaml').read_text())['/**']['ros__parameters']
    assert 'self_filter_margin_m' not in perception


@pytest.mark.parametrize(
    ('footprint', 'message'),
    [
        ({'length_m': 0.8, 'width_m': 0.8, 'height_m': 0.8}, 'self_filter_margin_m がありません'),
        ({'length_m': 0.8, 'width_m': 0.8, 'self_filter_margin_m': 0.05}, 'height_m がありません'),
        ({'length_m': 0.8, 'width_m': 0.8, 'height_m': 0.8, 'self_filter_margin_m': -0.1}, '0 以上'),
        ({'length_m': 0.0, 'width_m': 0.8, 'height_m': 0.8, 'self_filter_margin_m': 0.05}, '正の値'),
        ({'length_m': 0.8, 'width_m': 0.8, 'height_m': 0.0, 'self_filter_margin_m': 0.05}, '正の値'),
        ({'length_m': True, 'width_m': 0.8, 'height_m': 0.8, 'self_filter_margin_m': 0.05},
         '数値にしてください'),
    ],
)
def test_self_box_errors(tmp_path: Path, footprint: dict[str, object], message: str) -> None:
    path = tmp_path / 'footprint.yaml'
    path.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'footprint': footprint}}}))
    with pytest.raises(SelfFilterError, match=message):
        load_self_box(path)
    with pytest.raises(SelfFilterError, match='読めません'):
        load_self_box(tmp_path / 'missing.yaml')


def test_parse_xacro_args() -> None:
    assert parse_xacro_args('') == {}
    assert parse_xacro_args(' lidar_height:=0.95  damiao_imu_z:=0.4 ') == {
        'lidar_height': '0.95', 'damiao_imu_z': '0.4'}
    with pytest.raises(SelfFilterError, match='name:=value'):
        parse_xacro_args('lidar_height=0.95')


def test_robot_description_errors(tmp_path: Path) -> None:
    with pytest.raises(SelfFilterError, match='見つかりません'):
        load_robot_description(tmp_path / 'missing.urdf.xacro', {})
    plain = tmp_path / 'robot.urdf'
    plain.write_text('<robot name="r"/>')
    with pytest.raises(SelfFilterError, match='.xacro のときだけ'):
        load_robot_description(plain, {'lidar_height': '0.9'})
    broken = tmp_path / 'broken.urdf.xacro'
    broken.write_text('<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:undefined_macro/></robot>')
    with pytest.raises(SelfFilterError, match='xacro で処理できません'):
        load_robot_description(broken, {})


def test_fast_lio_parameters_for_each_mode() -> None:
    box = fast_lio_self_filter_parameters('box', CONFIG / 'footprint.yaml', ROBOT_XACRO, FRONT_MOUNT)
    assert box['preprocess.self_filter.enabled'] is True
    assert box['preprocess.self_filter.half_length'] == pytest.approx(0.45)
    assert box['preprocess.self_filter.half_width'] == pytest.approx(0.45)
    assert box['preprocess.self_filter.top_height'] == pytest.approx(0.85)
    assert_close(tuple(box['preprocess.self_filter.extrinsic_T']), (0.44, 0.0, 0.25))
    assert_close(tuple(box['preprocess.self_filter.extrinsic_R']), IDENTITY.rotation)
    assert 'preprocess.blind' not in box
    legacy = fast_lio_self_filter_parameters(
        'legacy_blind', CONFIG / 'footprint.yaml', ROBOT_XACRO, {})
    assert legacy == {'preprocess.self_filter.enabled': False, 'preprocess.blind': LEGACY_BLIND_M}
    with pytest.raises(SelfFilterError, match='lio_self_filter'):
        fast_lio_self_filter_parameters('sphere', CONFIG / 'footprint.yaml', ROBOT_XACRO, {})


def test_fast_lio_yaml_blind_is_sensor_minimum() -> None:
    # 自機は箱で除くので、blind は Mid-360 の最短距離だけにする
    document = yaml.safe_load((CONFIG / 'fast_lio.yaml').read_text())
    assert document['/**']['ros__parameters']['preprocess']['blind'] == pytest.approx(0.1)
