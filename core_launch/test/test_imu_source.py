"""core_nav_launch.imu_source（LIO に入れる IMU の選択と外部パラメータ）の試験."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from core_nav_launch.imu_source import (
    adapter_parameters,
    extrinsic_from_urdf,
    fast_lio_parameters,
    ImuSourceError,
    load_imu_selection,
)
from core_nav_launch.self_filter import (
    compose,
    IDENTITY,
    invert,
    load_robot_description,
    rpy_matrix,
    Transform,
)
import pytest
import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'imu_source.yaml'
ROBOT_XACRO = Path(get_package_share_directory('core_launch')) / 'urdf' / 'core2026.urdf.xacro'
# 今の取り付け（機体の前の端、床から 0.25m。xacro の既定）と、以前の取り付け（機体の中心の上 0.85m）
# 取り付けは明示する（core2026.urdf.xacro の既定は Unity の機体の、床から 0.484m・上下逆さ。Damiao も Unity の
# IMU の位置なので、ここでは以前の (0, 0, 0.4)m・回転なしにする）
UPRIGHT: dict[str, str] = {
    'lidar_y': '0.0', 'lidar_pitch': '0.0', 'lidar_yaw': '0.0',
    'damiao_imu_x': '0.0', 'damiao_imu_y': '0.0', 'damiao_imu_z': '0.4', 'damiao_imu_roll': '0.0'}
FRONT_MOUNT: dict[str, str] = {**UPRIGHT, 'lidar_x': '0.44', 'lidar_height': '0.25', 'lidar_roll': '0.0'}
CENTER_MOUNT = {**UPRIGHT, 'lidar_x': '0.0', 'lidar_height': '0.85', 'lidar_roll': '0.0'}
# FAST-LIO の mid360.yaml の値（IMU から見た LiDAR の位置）
LIVOX_EXTRINSIC_T = (-0.011, -0.02329, 0.04412)


def assert_close(actual: tuple[float, ...] | list[float], expected: tuple[float, ...]) -> None:
    assert len(actual) == len(expected)
    for a, e in zip(actual, expected):
        assert a == pytest.approx(e, abs=1e-9)


def robot(mount: dict[str, str]) -> str:
    return load_robot_description(ROBOT_XACRO, mount)


def write_config(tmp_path: Path, damiao_changes: dict[str, object], remove: tuple[str, ...] = ()) -> Path:
    """リポジトリの imu_source.yaml の damiao だけを書き換えた設定を作る."""
    document = yaml.safe_load(CONFIG.read_text())
    for key in remove:
        document['damiao'].pop(key, None)
    document['damiao'].update(damiao_changes)
    path = tmp_path / 'imu_source.yaml'
    path.write_text(yaml.safe_dump(document))
    return path


def test_invert_undoes_transform() -> None:
    transform = Transform(rpy_matrix(0.2, -0.4, 1.3), (0.3, -1.2, 0.8))
    for combined in (compose(invert(transform), transform), compose(transform, invert(transform))):
        assert_close(combined.rotation, IDENTITY.rotation)
        assert_close(combined.translation, (0.0, 0.0, 0.0))


@pytest.mark.parametrize(
    ('mount', 'expected'),
    [
        # IMU（base_link から (0, 0, 0.4)m）から見た LiDAR（(lidar_x, 0, lidar_height)m）
        (FRONT_MOUNT, (0.44, 0.0, -0.15)),
        (CENTER_MOUNT, (0.0, 0.0, 0.45)),
    ],
)
def test_damiao_extrinsic_follows_lidar_mount(mount: dict[str, str], expected: tuple[float, ...]) -> None:
    for name in ('damiao', 'fused'):
        selection = load_imu_selection(CONFIG, name, robot(mount))
        parameters = fast_lio_parameters(selection, 30.0)
        assert_close(parameters['mapping.extrinsic_T'], expected)
        assert_close(parameters['mapping.extrinsic_R'], IDENTITY.rotation)
        # FAST-LIO の姿勢は Damiao の座標系で出るので、変換のノードもその座標系を使う
        assert adapter_parameters(selection)['imu_frame'] == 'damiao_imu_link'
        assert adapter_parameters(selection)['lio_extrinsic_T'] == parameters['mapping.extrinsic_T']


@pytest.mark.parametrize('mount', [FRONT_MOUNT, CENTER_MOUNT])
def test_livox_extrinsic_does_not_depend_on_mount(mount: dict[str, str]) -> None:
    selection = load_imu_selection(CONFIG, 'livox', robot(mount))
    assert_close(fast_lio_parameters(selection, 30.0)['mapping.extrinsic_T'], LIVOX_EXTRINSIC_T)
    # 設定ファイルの値は、URDF の livox_frame→livox_imu_link と合う（lio_odom_adapter が起動時に比べるのと同じ）
    from_urdf, rotation = extrinsic_from_urdf(robot(mount), 'livox_imu_link')
    assert_close(from_urdf, LIVOX_EXTRINSIC_T)
    assert_close(rotation, IDENTITY.rotation)


def test_extrinsic_from_urdf_handles_rotated_imu() -> None:
    # IMU を z 軸まわりに 90 度回して付けたとき、IMU の x 軸は base の y 軸を向く
    description = (
        '<robot name="r"><link name="base_link"/><link name="livox_frame"/><link name="imu"/>'
        '<joint name="l" type="fixed"><parent link="base_link"/><child link="livox_frame"/>'
        '<origin xyz="1.0 0 0.5"/></joint>'
        '<joint name="i" type="fixed"><parent link="base_link"/><child link="imu"/>'
        f'<origin xyz="0 0 0.2" rpy="0 0 {math.pi / 2}"/></joint></robot>'
    )
    translation, rotation = extrinsic_from_urdf(description, 'imu')
    # LiDAR は base の前 1.0m（IMU の座標系では -y 方向）、IMU より 0.3m 上
    assert_close(translation, (0.0, -1.0, 0.3))
    # LiDAR の x 軸（base の x）は IMU の座標系では -y
    assert_close((rotation[0], rotation[3], rotation[6]), (0.0, -1.0, 0.0))


def test_urdf_extrinsic_needs_robot_description() -> None:
    with pytest.raises(ImuSourceError, match='ロボットの記述'):
        load_imu_selection(CONFIG, 'damiao')


def test_explicit_extrinsic_still_works_without_description(tmp_path: Path) -> None:
    path = write_config(
        tmp_path,
        {'extrinsic_T': [0.1, 0.2, 0.3], 'extrinsic_R': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]},
        remove=('extrinsic',),
    )
    selection = load_imu_selection(path, 'damiao')
    assert_close(fast_lio_parameters(selection, 30.0)['mapping.extrinsic_T'], (0.1, 0.2, 0.3))


@pytest.mark.parametrize(
    ('changes', 'message'),
    [
        ({'extrinsic': 'tf'}, 'extrinsic は urdf にするか'),
        ({'extrinsic_T': [0.0, 0.0, 0.0]}, '両方があります'),
    ],
)
def test_invalid_extrinsic_setting(tmp_path: Path, changes: dict[str, object], message: str) -> None:
    with pytest.raises(ImuSourceError, match=message):
        load_imu_selection(write_config(tmp_path, changes), 'damiao', robot(FRONT_MOUNT))


def test_missing_imu_frame_in_description_is_reported(tmp_path: Path) -> None:
    path = write_config(tmp_path, {'frame': 'no_such_imu_link'})
    with pytest.raises(ImuSourceError, match='no_such_imu_link'):
        load_imu_selection(path, 'damiao', robot(FRONT_MOUNT))
