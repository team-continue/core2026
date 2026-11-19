"""シミュレーション、FAST-LIO、地図照合を起動する.

composition:=true なら、点群の変換（points_to_livox）を検出と同じコンテナ（lidar_container）で起動し、
FAST-LIO より後ろのノードもコンテナにまとめる（core_nav_launch/composition.py）。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from core_nav_launch.composition import (
    COMPOSITION_MODES,
    DEFAULT_COMPOSITION,
    launch_nodes,
    LIDAR_CONTAINER,
    NodeSpec,
    parse_composition,
    SENSING_NS,
)
from core_nav_launch.imu_source import DEFAULT_IMU_SOURCE, IMU_SOURCES
from core_nav_launch.point_extraction import DEFAULT_POINT_EXTRACTION, POINT_EXTRACTION_MODES
from core_nav_launch.self_filter import DEFAULT_LIO_SELF_FILTER, LIO_SELF_FILTER_MODES
from core_nav_launch.state_source import DEFAULT_STATE_SOURCE, EKF, STATE_SOURCES


def build_lidar_nodes(context: LaunchContext) -> list[LaunchDescriptionEntity]:
    """Gazebo の点群を実機のドライバと同じ形にする。composition:=true なら、検出と同じコンテナで起動する."""
    converter = {
        'use_sim_time': True,
        'scan_motion': LaunchConfiguration('lidar_scan_motion').perform(context) == 'true',
        'include_no_return': LaunchConfiguration('lidar_no_return_points').perform(context) == 'true',
        # 自機の車体の高さ（車体に当たった点は走査の間の動きでずらさない）
        'body.height_m': float(LaunchConfiguration('body_height').perform(context)),
    }
    specs = [NodeSpec(
        package='core_gazebo',
        executable='points_to_livox',
        plugin='core_gazebo::PointsToLivox',
        name='points_to_livox',
        namespace=SENSING_NS,
        parameters=(converter,),
    )]
    if LaunchConfiguration('rear_lidar').perform(context) == 'true':
        # 検出のための 2 台目の LiDAR の変換。1 台目と同じ点数に間引き、/livox/lidar2 に出す。
        # LIO と地図照合は 1 台目だけを使う
        specs.append(NodeSpec(
            package='core_gazebo',
            executable='points_to_livox',
            plugin='core_gazebo::PointsToLivox',
            name='points_to_livox_rear',
            namespace=SENSING_NS,
            parameters=(converter,),
            remappings=(('/sensing/sim/points', '/sensing/sim/points_rear'), ('/sensing/livox/lidar', '/sensing/livox/lidar2')),
        ))
    composition = parse_composition(LaunchConfiguration('composition').perform(context))
    return launch_nodes(composition, LIDAR_CONTAINER, specs)


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('core_launch'))
    sim_share = Path(get_package_share_directory('core_gazebo'))
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    forwarded = (
        'filter_layout', 'every_n_frames', 'voxel_enabled', 'voxel_leaf_size_m',
        'random_enabled', 'random_target_points', 'imu_source', 'point_extraction',
        'exclude_regions_file', 'lio_self_filter', 'composition', 'localization_params_file',
        'state_source', 'field_file',
    )
    # 渡した引数がこの launch の引数を上書きしないよう、スコープを分ける
    simulation = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim_share / 'launch' / 'sim.launch.py')),
            launch_arguments={
                'x': x,
                'y': y,
                'yaw': yaw,
                'lidar_x': LaunchConfiguration('lidar_x'),
                'lidar_height': LaunchConfiguration('lidar_height'),
                'lidar_roll': LaunchConfiguration('lidar_roll'),
                'body_height': LaunchConfiguration('body_height'),
                'rear_lidar': LaunchConfiguration('rear_lidar'),
                'waist': LaunchConfiguration('waist'),
                'headless': LaunchConfiguration('headless'),
                'world': LaunchConfiguration('world'),
                'scenario_file': LaunchConfiguration('scenario_file'),
                # base_link の親は odom にするので、真値の TF は出さない
                'publish_ground_truth_tf': 'false',
                # EKF の自己位置は、実機の足回りが返す車輪の速度の代わりを使う
                'wheel_odometry': PythonExpression([
                    "'true' if '", LaunchConfiguration('state_source'), f"' == '{EKF}' else 'false'",
                ]),
                'rviz': 'false',
            }.items(),
        ),
    ])
    localization = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'localization_stack.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'map_path': LaunchConfiguration('map_path'),
                'initial_x': LaunchConfiguration('initial_x'),
                'initial_y': LaunchConfiguration('initial_y'),
                'initial_yaw': LaunchConfiguration('initial_yaw'),
                # FAST-LIO の自機の除去は、シミュレーションの機体と同じ取り付けの URDF を使う
                'robot_description_args': [
                    'lidar_x:=', LaunchConfiguration('lidar_x'),
                    ' lidar_height:=', LaunchConfiguration('lidar_height'),
                    ' lidar_roll:=', LaunchConfiguration('lidar_roll'),
                    ' body_height:=', LaunchConfiguration('body_height'),
                ],
                **{name: LaunchConfiguration(name) for name in forwarded},
            }.items(),
        ),
    ])
    # シミュレーションでは地図を世界座標で作るので、map と world は一致する
    world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['--frame-id', 'world', '--child-frame-id', 'map'],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='-6.0', description='機体の初期位置 x [m]'),
        DeclareLaunchArgument('y', default_value='-2.5', description='機体の初期位置 y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='機体の初期方位 [rad]'),
        # 地図照合に渡す初期値。わざとずらして収束を確かめられるよう、機体の位置とは別に持つ
        DeclareLaunchArgument('initial_x', default_value=x),
        DeclareLaunchArgument('initial_y', default_value=y),
        DeclareLaunchArgument('initial_yaw', default_value=yaw),
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
            description='検出のための 2 台目の LiDAR を機体の後ろの端に付ける（LIO と地図照合は 1 台目だけ）',
        ),
        DeclareLaunchArgument(
            'waist', default_value='false', choices=['true', 'false'],
            description='腰（足回りと上半身の間）を回る関節にし、実機と同じ body_controller の指令で足回りと腰を動かす'
                        '（sim だけ。false なら機体全体を一つの塊として動かす）',
        ),
        DeclareLaunchArgument('headless', default_value='true', description='GazeboのGUIを出さない'),
        DeclareLaunchArgument(
            'world', default_value=str(sim_share / 'worlds' / 'field.sdf'), description='ワールドの SDF',
        ),
        DeclareLaunchArgument(
            'scenario_file', default_value='', description='他ロボットと人間の動きの YAML',
        ),
        DeclareLaunchArgument(
            'map_path', default_value=str(share / 'maps' / 'arena_sim.pcd'), description='地図の PCD',
        ),
        DeclareLaunchArgument(
            'field_file', default_value='',
            description='フィールドの範囲・静止物の高さ・入らない範囲を config/field.yaml の後から重ねる設定ファイル（空なら重ねない）',
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
            description='点群の変換と、FAST-LIO より後ろのノードを、それぞれ 1 つのプロセス（コンテナ）にまとめる',
        ),
        DeclareLaunchArgument(
            'state_source', default_value=DEFAULT_STATE_SOURCE, choices=list(STATE_SOURCES),
            description='自己位置の出し方（lio / ekf: 車輪の速度・IMU・地図照合の EKF も起動する）',
        ),
        DeclareLaunchArgument(
            'localization_params_file', default_value='',
            description='地図照合と localization_health に、既定の設定の後から重ねるパラメータファイル（試験用）',
        ),
        simulation,
        OpaqueFunction(function=build_lidar_nodes),
        localization,
        world_to_map,
    ])
