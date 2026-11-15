"""シミュレーション、自己位置推定、検出と追跡、計画をすべて起動する.

composition:=true なら、ノードを処理の段ごとのコンテナ（1 つのプロセス）にまとめる（core_nav_launch/composition.py）。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from core_nav_launch.composition import COMPOSITION_MODES, DEFAULT_COMPOSITION
from core_nav_launch.imu_source import DEFAULT_IMU_SOURCE, IMU_SOURCES
from core_nav_launch.point_extraction import DEFAULT_POINT_EXTRACTION, POINT_EXTRACTION_MODES
from core_nav_launch.self_filter import DEFAULT_LIO_SELF_FILTER, LIO_SELF_FILTER_MODES
from core_nav_launch.state_source import DEFAULT_STATE_SOURCE, STATE_SOURCES


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
        'state_source',
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
    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='-6.0'),
        DeclareLaunchArgument('y', default_value='-2.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('lidar_x', default_value='0.0', description='LiDARの前後の位置（base_link の x）[m]'),
        DeclareLaunchArgument('lidar_height', default_value='0.5', description='LiDARの床からの高さ [m]'),
        DeclareLaunchArgument(
            'lidar_roll', default_value='3.141592653589793',
            description='LiDAR の前後の軸まわりの回転 [rad]。3.14159 で上下逆さ（2026-03 の実機の bag の付け方）',
        ),
        DeclareLaunchArgument(
            'body_height', default_value='0.3',
            description='シミュレーションの機体の箱の高さ [m]（LiDAR を機体の中心の低い所に付けるときに下げる）',
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
                }.items(),
            ),
        ]),
        control_mode_manager,
    ])
