"""core_nav_launch.state_source（自己位置の出し方）と、localization_stack.launch.py の EKF のノードの試験."""

import importlib.util
from pathlib import Path
from types import ModuleType

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from core_nav_launch.composition import CONTAINER_OF_EXECUTABLE, LOCALIZATION_CONTAINER
from core_nav_launch.state_source import ekf_node_specs, parse_state_source
import pytest
import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config'
LAUNCH = Path(__file__).resolve().parents[1] / 'launch'


def load_launch_module(name: str) -> ModuleType:
    spec = importlib.util.spec_from_file_location(name, LAUNCH / f'{name}.launch.py')
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def ros_parameters(name: str) -> dict[str, object]:
    return yaml.safe_load((CONFIG / name).read_text())['/**']['ros__parameters']


def stack_executables(tmp_path: Path, **arguments: str) -> list[str]:
    """localization_stack.launch.py の既定の引数に arguments を重ね、別のプロセスで起動するノードの実行ファイルを返す."""
    stack = load_launch_module('localization_stack')
    context = LaunchContext()
    for entity in stack.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.default_value is not None:
            context.launch_configurations[entity.name] = ''.join(
                substitution.perform(context) for substitution in entity.default_value
            )
    map_path = tmp_path / 'map.pcd'
    map_path.write_text('')
    context.launch_configurations.update({'map_path': str(map_path), 'composition': 'false', **arguments})
    return [
        entity.node_executable for entity in stack.build_nodes(context) if isinstance(entity, Node)
    ]


def test_parse_state_source() -> None:
    assert parse_state_source('lio') is False
    assert parse_state_source('ekf') is True
    for value in ('EKF', '', 'fused'):
        with pytest.raises(RuntimeError, match='state_source'):
            parse_state_source(value)


def test_ekf_nodes_use_config_and_fused_imu() -> None:
    specs = ekf_node_specs(CONFIG, '/sensing/imu/fused', True)
    assert [spec.executable for spec in specs] == ['gyro_odometer', 'ekf_localizer']
    for spec in specs:
        assert CONTAINER_OF_EXECUTABLE[spec.executable] == LOCALIZATION_CONTAINER
        assert Path(str(spec.parameters[0])).is_file()
        assert spec.parameters[1]['use_sim_time'] is True
    assert specs[0].parameters[1]['imu_topic'] == '/sensing/imu/fused'


def test_ekf_topics_connect() -> None:
    gyro = ros_parameters('gyro_odometer.yaml')
    ekf = ros_parameters('ekf_localizer.yaml')
    assert gyro['output_topic'] == ekf['twist_topic']
    # 地図照合の出力
    assert ekf['pose_topic'] == '/localization/pcl_pose'


def test_lio_starts_no_ekf_nodes(tmp_path: Path) -> None:
    executables = stack_executables(tmp_path, state_source='lio', imu_source='livox')
    assert 'gyro_odometer' not in executables
    assert 'ekf_localizer' not in executables
    assert 'imu_fusion' not in executables


def test_ekf_starts_fusion_for_its_gyro(tmp_path: Path) -> None:
    # FAST-LIO が Mid-360 の IMU を使っても、EKF のために融合を起動する
    executables = stack_executables(tmp_path, state_source='ekf', imu_source='livox')
    assert executables.count('imu_fusion') == 1
    assert 'gyro_odometer' in executables
    assert 'ekf_localizer' in executables


def test_ekf_shares_fusion_with_lio(tmp_path: Path) -> None:
    # FAST-LIO が融合を使うなら、同じ融合のノードを使い、2 つ起動しない
    executables = stack_executables(tmp_path, state_source='ekf', imu_source='fused')
    assert executables.count('imu_fusion') == 1
