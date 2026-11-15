"""グローバル計画、局所計画、安全停止、接触の検知を起動する.

composition:=true なら、安全停止（safety_gate）以外の 3 つを 1 つのプロセス（コンテナ planning_container）に
まとめる（core_nav_launch/composition.py）。安全停止は、計画が止まっても機体を止められるよう、いつも別のプロセスにする。
"""

from pathlib import Path
from typing import Final

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from core_nav_launch.composition import (
    COMPOSITION_MODES,
    DEFAULT_COMPOSITION,
    field_overlay,
    launch_nodes,
    NodeSpec,
    parse_composition,
    PLANNING_CONTAINER,
    PLANNING_NS,
    standalone_node,
)

# 機能ごとの設定。ノードは自分が宣言した名前だけを読む
CONFIG_NAMES: Final = (
    'field', 'footprint', 'mppi', 'speed_limit', 'passing_rule', 'stuck_recovery', 'planner',
    'goal_handling', 'safety_stop', 'watchdog', 'contact_detection', 'localization_health',
    'narrow_passage',
)
# 計画のコンテナに入れるノード（実行ファイル、component の型）。ノード名は実行ファイルと同じ
PLANNING_NODES: Final = (
    ('global_planner', 'core_planning::GlobalPlannerNode'),
    ('mppi_lite', 'core_planning::MppiLiteNode'),
    ('contact_monitor', 'core_planning::ContactMonitorNode'),
)
SAFETY_NODE: Final = ('safety_gate', 'core_planning::SafetyGateNode')


def build_nodes(context: LaunchContext) -> list[LaunchDescriptionEntity]:
    config = Path(get_package_share_directory('core_launch')) / 'config'
    parameters = (
        *(str(config / f'{name}.yaml') for name in CONFIG_NAMES),
        *field_overlay(LaunchConfiguration('field_file').perform(context)),
        {
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'map_path': LaunchConfiguration('map_path'),
        },
    )

    def spec(executable: str, plugin: str) -> NodeSpec:
        return NodeSpec(
            package='core_planning', executable=executable, plugin=plugin, name=executable,
            parameters=parameters, namespace=PLANNING_NS,
        )

    composition = parse_composition(LaunchConfiguration('composition').perform(context))
    return [
        *launch_nodes(composition, PLANNING_CONTAINER, [spec(*node) for node in PLANNING_NODES]),
        standalone_node(spec(*SAFETY_NODE)),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map_path', description='地図の PCD'),
        DeclareLaunchArgument(
            'field_file', default_value='',
            description='フィールドの範囲（field.x_min など）を config/field.yaml の後から重ねる設定ファイル'
                        '（空なら重ねない。navigation.launch.py は map_name のプリセットで選ぶ）',
        ),
        DeclareLaunchArgument(
            'composition', default_value=DEFAULT_COMPOSITION, choices=list(COMPOSITION_MODES),
            description='安全停止以外の計画のノードを 1 つのプロセス（コンテナ）にまとめる',
        ),
        OpaqueFunction(function=build_nodes),
    ])
