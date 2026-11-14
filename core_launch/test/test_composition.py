"""core_nav_launch.composition（ノードを別のプロセスにするか、コンテナにまとめるか）と、確認スクリプトの
プロセスの探し方の試験."""

import importlib.util
import os
from pathlib import Path
import subprocess
import sys
from types import ModuleType

from launch import LaunchContext
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.actions.load_composable_nodes import get_composable_node_load_request
from core_nav_check import common
from core_nav_launch.composition import (
    composable_node,
    CONTAINER_OF_EXECUTABLE,
    launch_nodes,
    LIDAR_CONTAINER,
    LOCALIZATION_CONTAINER,
    NodeSpec,
    parse_composition,
    PLANNING_CONTAINER,
)
import pytest

LAUNCH = Path(__file__).resolve().parents[1] / 'launch'


def planner_spec(parameter_file: Path) -> NodeSpec:
    return NodeSpec(
        package='core_planning',
        executable='global_planner',
        plugin='core_planning::GlobalPlannerNode',
        name='global_planner',
        parameters=(str(parameter_file), {'use_sim_time': True}),
        remappings=(('input', '/localization/cloud'),),
    )


def load_launch_module(name: str) -> ModuleType:
    spec = importlib.util.spec_from_file_location(name, LAUNCH / f'{name}.launch.py')
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_parse_composition() -> None:
    assert parse_composition('true') is True
    assert parse_composition('false') is False
    for value in ('True', '1', ''):
        with pytest.raises(RuntimeError, match='composition'):
            parse_composition(value)


def test_standalone_runs_each_node_in_its_own_process(tmp_path: Path) -> None:
    specs = [planner_spec(tmp_path / 'p.yaml'), NodeSpec(
        package='core_planning', executable='mppi_lite', plugin='core_planning::MppiLiteNode',
        name='mppi_lite',
    )]
    nodes = launch_nodes(False, PLANNING_CONTAINER, specs)
    assert [type(node) for node in nodes] == [Node, Node]
    assert [node.node_executable for node in nodes] == ['global_planner', 'mppi_lite']


def test_composition_runs_nodes_in_one_container(tmp_path: Path) -> None:
    nodes = launch_nodes(True, PLANNING_CONTAINER, [planner_spec(tmp_path / 'p.yaml')])
    assert len(nodes) == 1
    assert isinstance(nodes[0], ComposableNodeContainer)


def test_load_request_applies_wildcard_parameter_file(tmp_path: Path) -> None:
    # 設定ファイルは /** で全ノードに当てている。コンテナに渡すときはノード名がないと当てはまらない
    parameter_file = tmp_path / 'planner.yaml'
    parameter_file.write_text(
        '/**:\n  ros__parameters:\n    planner:\n      period_s: 0.5\n      enabled: true\n',
        encoding='utf-8',
    )
    request = get_composable_node_load_request(
        composable_node(planner_spec(parameter_file)), LaunchContext(),
    )
    assert request.package_name == 'core_planning'
    assert request.plugin_name == 'core_planning::GlobalPlannerNode'
    assert request.node_name == 'global_planner'
    assert list(request.remap_rules) == ['input:=/localization/cloud']
    parameters = {p.name: p.value for p in request.parameters}
    assert parameters['planner.period_s'].double_value == pytest.approx(0.5)
    assert parameters['planner.enabled'].bool_value is True
    assert parameters['use_sim_time'].bool_value is True
    # 同じコンテナのノードの間はポインタで渡す
    extra = {p.name: p.value for p in request.extra_arguments}
    assert extra['use_intra_process_comms'].bool_value is True


def test_containers_split_by_stage() -> None:
    assert set(CONTAINER_OF_EXECUTABLE.values()) == {
        LIDAR_CONTAINER, LOCALIZATION_CONTAINER, PLANNING_CONTAINER,
    }
    # FAST-LIO と安全停止は、いつも別のプロセス
    assert 'fastlio_mapping' not in CONTAINER_OF_EXECUTABLE
    assert 'safety_gate' not in CONTAINER_OF_EXECUTABLE


def test_navigation_launch_matches_container_table() -> None:
    navigation = load_launch_module('planning')
    for executable, _plugin in navigation.PLANNING_NODES:
        assert CONTAINER_OF_EXECUTABLE[executable] == PLANNING_CONTAINER
    assert navigation.SAFETY_NODE[0] not in CONTAINER_OF_EXECUTABLE


def test_stack_processes_fall_back_to_container(monkeypatch: pytest.MonkeyPatch) -> None:
    running = {
        'fastlio_mapping': 10, 'safety_gate': 11, LOCALIZATION_CONTAINER: 20, PLANNING_CONTAINER: 30,
    }
    monkeypatch.setattr(common, 'find_process', running.get)
    names = (
        'fastlio_mapping', 'scan_filter', 'lidar_localization_node', 'global_planner', 'safety_gate',
        'perception_node',
    )
    found = common.find_stack_processes(names)
    # 同じコンテナのノードは 1 つにまとめて測る
    assert found == {
        'fastlio_mapping': 10, LOCALIZATION_CONTAINER: 20, PLANNING_CONTAINER: 30, 'safety_gate': 11,
    }
    # 検出も、そのコンテナもない
    assert common.missing_processes(names, found) == ['perception_node']


def test_find_process_by_node_name() -> None:
    # コンテナは実行ファイルが component_container_isolated なので、ノード名（__node:=）で探す
    name = f'test_container_{os.getpid()}'
    process = subprocess.Popen([
        sys.executable, '-c', 'import time; time.sleep(30)', '--ros-args', '-r', f'__node:={name}',
    ])
    try:
        assert common.find_process(name) == process.pid
        assert common.find_process(f'{name}_other') is None
    finally:
        process.kill()
        process.wait()
