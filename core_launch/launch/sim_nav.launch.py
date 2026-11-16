"""シミュレーション、自己位置推定、検出と追跡、計画をすべて起動する.

composition:=true なら、ノードを処理の段ごとのコンテナ（1 つのプロセス）にまとめる（core_nav_launch/composition.py）。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import LaunchContext, LaunchDescriptionEntity
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from core_nav_launch.composition import COMPOSITION_MODES, DEFAULT_COMPOSITION
from core_nav_launch.imu_source import DEFAULT_IMU_SOURCE, IMU_SOURCES
from core_nav_launch.point_extraction import DEFAULT_POINT_EXTRACTION, POINT_EXTRACTION_MODES
from core_nav_launch.self_filter import DEFAULT_LIO_SELF_FILTER, LIO_SELF_FILTER_MODES
from core_nav_launch.state_source import DEFAULT_STATE_SOURCE, STATE_SOURCES


def map_publisher(context: LaunchContext) -> list[LaunchDescriptionEntity]:
    """地図（map_path と同じ名前の yaml）を /map と /map/name に出す。操縦の UI が ros2_ws_bridge 経由で読む."""
    occupancy = LaunchConfiguration('occupancy_map').perform(context)
    if not occupancy:
        candidate = Path(LaunchConfiguration('map_path').perform(context)).with_suffix('.yaml')
        if not candidate.is_file():
            # 占有地図のない地図（arena_obstacles など）は出さない。UI は地図を描かない
            return []
        occupancy = str(candidate)
    return [Node(
        package='core_launch',
        executable='map_publisher_node.py',
        name='map_publisher',
        output='screen',
        parameters=[{'map_yaml': occupancy, 'use_sim_time': True}],
    )]


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('core_launch'))
    sim_share = Path(get_package_share_directory('core_gazebo'))
    map_path = LaunchConfiguration('map_path')
    forwarded = (
        'x', 'y', 'yaw', 'lidar_x', 'lidar_height', 'headless', 'world', 'scenario_file', 'filter_layout',
        'every_n_frames', 'voxel_enabled', 'voxel_leaf_size_m', 'random_enabled',
        'random_target_points', 'imu_source', 'point_extraction', 'exclude_regions_file',
        'lio_self_filter', 'rear_lidar', 'composition', 'localization_params_file',
        'lidar_roll', 'body_height', 'lidar_scan_motion', 'lidar_no_return_points',
        'state_source', 'field_file',
    )
    # 実機と同じく、手動と自動の指令は control_mode_manager を通して /control/cmd_vel へ出す
    control_mode_manager = Node(
        package='core_body_controller',
        executable='control_mode_manager_node',
        name='control_mode_manager',
        namespace='control',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    # 緊急停止の判定（core_mode の emergency_handler。実機では mode.launch.py が起動する）。操縦の UI の緊急停止
    # （ros2_ws_bridge が /system/emergency/software_emergency に出す）を /system/emergency/hazard_status にし、
    # control_mode_manager が自動をやめて手動に戻す。マイコンと bridge の死活監視（diagnostic）は、
    # シミュレーションにはマイコンが無く常に通信断になるので起動しない。
    # 起動時の緊急停止のラッチは外す（UI を繋がずに RViz や run_scenario.py から目標を送れるように。
    # UI は繋いだときに緊急停止を入れるので、UI から使うときは実機と同じく解除してから走る）
    emergency_handler = Node(
        package='core_mode',
        executable='emergency_handler',
        name='emergency_handler',
        namespace='system/emergency',
        output='screen',
        parameters=[{'use_sim_time': True, 'start_latched': False}],
        remappings=[
            ('emergency_switch', '/hardware/hardware_emergency'),
            ('destroy', '/hardware/destroy'),
        ],
        # 実機と同じく、緊急停止の判定が止まったまま走り続けないよう、落ちたら全体を止める
        on_exit=Shutdown(reason='emergency_handler exited'),
    )
    # RViz の機体に重ねる自己位置の状態（床の色の円と評価値の文字）。RViz を別に開いても見えるよう、常に起動する
    localization_status = Node(
        package='core_launch',
        executable='localization_status_marker_node.py',
        name='localization_status_marker',
        namespace='localization',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    # 自律移動を見る RViz（地図・経路・検出・ゴール）。2D Goal Pose で /ui/auto/start に目標を送ると自動で走る
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(share / 'rviz' / 'nav.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='false', choices=['true', 'false'],
            description='自律移動を見る RViz（rviz/nav.rviz）を起動する',
        ),
        DeclareLaunchArgument('x', default_value='-6.0'),
        DeclareLaunchArgument('y', default_value='-2.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('lidar_x', default_value='0.1321', description='LiDARの前後の位置（base_link の x）[m]'),
        DeclareLaunchArgument('lidar_height', default_value='0.484173', description='LiDARの床からの高さ [m]'),
        DeclareLaunchArgument(
            'lidar_roll', default_value='3.14159',
            description='LiDAR の前後の軸まわりの回転 [rad]。3.14159 で上下逆さ（既定。Unity の機体と 2026-03 の'
                        '実機の bag の付け方）。左右の位置・ほかの軸の回転は URDF の既定値',
        ),
        DeclareLaunchArgument(
            'body_height', default_value='0.95',
            description='機体の上面の床からの高さ [m]（points_to_livox で車体に当たった点を見分ける）',
        ),
        DeclareLaunchArgument(
            'lidar_scan_motion', default_value='true', choices=['true', 'false'],
            description=(
                'シミュレーションの点に、実機と同じく 1 走査（100ms）の時刻と、その間の動きによるずれを付ける'
                '（2026-11-13 に既定にした。false で全点を撮った時刻にする前の動き）'
            ),
        ),
        DeclareLaunchArgument(
            'lidar_no_return_points', default_value='false', choices=['true', 'false'],
            description='返りのない向きも含めて間引き、その点を実機のドライバと同じく (0,0,0) で出す',
        ),
        DeclareLaunchArgument(
            'rear_lidar', default_value='false', choices=['true', 'false'],
            description='検出のための 2 台目の LiDAR を後ろの端に付け、検出の入力を 2 台の合成にする',
        ),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('world', default_value=str(sim_share / 'worlds' / 'field.sdf')),
        DeclareLaunchArgument(
            'scenario_file', default_value=str(sim_share / 'scenarios' / 'crowd_9r2h.yaml'),
        ),
        DeclareLaunchArgument('map_path', default_value=str(share / 'maps' / 'arena_sim.pcd')),
        DeclareLaunchArgument(
            'field_file', default_value='',
            description='フィールドの範囲・静止物の高さ・入らない範囲を config/field.yaml の後から重ねる設定ファイル'
                        '（空なら重ねない。CoRE-1 の試験のフィールドは config/field_core1_2027.yaml）',
        ),
        DeclareLaunchArgument(
            'occupancy_map', default_value='',
            description='UI に送る占有地図（map_server の yaml）。空なら map_path と同じ名前の .yaml',
        ),
        DeclareLaunchArgument('filter_layout', default_value='map_matching'),
        DeclareLaunchArgument('every_n_frames', default_value='1'),
        DeclareLaunchArgument('voxel_enabled', default_value='true'),
        DeclareLaunchArgument('voxel_leaf_size_m', default_value='0.5'),
        DeclareLaunchArgument('random_enabled', default_value='true'),
        DeclareLaunchArgument('random_target_points', default_value='3000'),
        DeclareLaunchArgument(
            'imu_source', default_value=DEFAULT_IMU_SOURCE, choices=list(IMU_SOURCES),
            description='FAST-LIO に入れる IMU（値は config/imu_source.yaml）',
        ),
        DeclareLaunchArgument(
            'point_extraction', default_value=DEFAULT_POINT_EXTRACTION,
            choices=list(POINT_EXTRACTION_MODES),
            description='地図照合の入力の点群抽出（off / ceiling / field_objects）',
        ),
        DeclareLaunchArgument(
            'exclude_regions_file', default_value=str(share / 'config' / 'exclude_regions.yaml'),
            description='点群抽出の ceiling で除く観客席の箱の設定ファイル',
        ),
        DeclareLaunchArgument(
            'lio_self_filter', default_value=DEFAULT_LIO_SELF_FILTER, choices=list(LIO_SELF_FILTER_MODES),
            description='FAST-LIO での自機の除き方（box / legacy_blind）',
        ),
        DeclareLaunchArgument(
            'composition', default_value=DEFAULT_COMPOSITION, choices=list(COMPOSITION_MODES),
            description='ノードを処理の段ごとに 1 つのプロセス（コンテナ）にまとめる',
        ),
        DeclareLaunchArgument(
            'state_source', default_value=DEFAULT_STATE_SOURCE, choices=list(STATE_SOURCES),
            description='自己位置の出し方（lio / ekf: 車輪の速度・IMU・地図照合の EKF も起動する）',
        ),
        DeclareLaunchArgument(
            'localization_params_file', default_value='',
            description='地図照合と localization_health に、既定の設定の後から重ねるパラメータファイル（試験用）',
        ),
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(share / 'launch' / 'sim_perception.launch.py')),
                launch_arguments={
                    'map_path': map_path,
                    **{name: LaunchConfiguration(name) for name in forwarded},
                }.items(),
            ),
        ]),
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(share / 'launch' / 'planning.launch.py')),
                launch_arguments={
                    'use_sim_time': 'true', 'map_path': map_path,
                    'composition': LaunchConfiguration('composition'),
                    'field_file': LaunchConfiguration('field_file'),
                }.items(),
            ),
        ]),
        control_mode_manager,
        emergency_handler,
        OpaqueFunction(function=map_publisher),
        localization_status,
        rviz,
    ])
