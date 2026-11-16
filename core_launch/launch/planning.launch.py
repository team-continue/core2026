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
# 安全停止（safety_gate）が自動の指令を出す話題
DEFAULT_AUTO_CMD_TOPIC: Final = '/control/auto/cmd_vel'


def params_overlay(params_file: str) -> list[str]:
    """既定の設定の後から重ねる設定ファイル（launch 引数 planning_params_file）。空なら何も重ねない."""
    if not params_file:
        return []
    if not Path(params_file).is_file():
        raise RuntimeError(f'計画の設定ファイル {params_file} がありません')
    return [params_file]


def build_nodes(context: LaunchContext) -> list[LaunchDescriptionEntity]:
    config = Path(get_package_share_directory('core_launch')) / 'config'
    parameters = (
        *(str(config / f'{name}.yaml') for name in CONFIG_NAMES),
        *field_overlay(LaunchConfiguration('field_file').perform(context)),
        *params_overlay(LaunchConfiguration('planning_params_file').perform(context)),
        {
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'map_path': LaunchConfiguration('map_path'),
        },
    )

    def spec(
        executable: str, plugin: str, remappings: tuple[tuple[str, str], ...] = ()
    ) -> NodeSpec:
        return NodeSpec(
            package='core_planning', executable=executable, plugin=plugin, name=executable,
            parameters=parameters, namespace=PLANNING_NS, remappings=remappings,
        )

    # 安全停止の出力先。sim_nav.launch.py は cmd_yaw_hold を挟むので /control/auto/cmd_vel_raw にする
    auto_cmd_topic = LaunchConfiguration('auto_cmd_topic').perform(context)
    safety_remappings = (
        (DEFAULT_AUTO_CMD_TOPIC, auto_cmd_topic),) if auto_cmd_topic != DEFAULT_AUTO_CMD_TOPIC else ()

    composition = parse_composition(LaunchConfiguration('composition').perform(context))
    return [
        *launch_nodes(composition, PLANNING_CONTAINER, [spec(*node) for node in PLANNING_NODES]),
        standalone_node(spec(*SAFETY_NODE, remappings=safety_remappings)),
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
            'auto_cmd_topic', default_value=DEFAULT_AUTO_CMD_TOPIC,
            description='安全停止（safety_gate）が自動の指令を出す話題（sim_nav.launch.py は cmd_yaw_hold を挟む'
                        'ので /control/auto/cmd_vel_raw）',
        ),
        DeclareLaunchArgument(
            'planning_params_file', default_value='',
            description='計画のノードに、既定の設定（とフィールドの設定）の後から重ねるパラメータファイル'
                        '（空なら重ねない。sim_nav.launch.py は config/sim_drive.yaml を渡す）',
        ),
        DeclareLaunchArgument(
            'composition', default_value=DEFAULT_COMPOSITION, choices=list(COMPOSITION_MODES),
            description='安全停止以外の計画のノードを 1 つのプロセス（コンテナ）にまとめる',
        ),
        OpaqueFunction(function=build_nodes),
    ])
