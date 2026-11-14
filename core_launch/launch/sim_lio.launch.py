"""シミュレーションと FAST-LIO を起動し、odom→base_link を出す.

imu_source で FAST-LIO に入れる IMU を選ぶ（livox / damiao / fused、値は config/imu_source.yaml）。
lio_self_filter で FAST-LIO での自機の除き方を選ぶ（box / legacy_blind、core_nav_launch/self_filter.py）。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from core_nav_launch.composition import FAST_LIO_REMAPPINGS, LOCALIZATION_NS, SENSING_NS
from core_nav_launch.imu_source import (
    adapter_parameters,
    DEFAULT_IMU_SOURCE,
    fast_lio_parameters,
    fusion_node,
    IMU_SOURCES,
    load_imu_selection,
    read_accel_clip_mps2,
)
from core_nav_launch.self_filter import (
    DEFAULT_LIO_SELF_FILTER,
    fast_lio_self_filter_parameters,
    LIO_SELF_FILTER_MODES,
    load_robot_description,
)


def build_lio_nodes(context: LaunchContext) -> list[LaunchDescriptionEntity]:
    """選んだ IMU に合わせて、FAST-LIO と変換のノード（fused なら融合も）を作る."""
    share = Path(get_package_share_directory('core_launch'))
    # シミュレーションの機体と同じ取り付けの URDF
    description_share = Path(get_package_share_directory('core_launch'))
    robot_xacro = description_share / 'urdf' / 'core2026.urdf.xacro'
    xacro_args = {
        'lidar_x': LaunchConfiguration('lidar_x').perform(context),
        'lidar_height': LaunchConfiguration('lidar_height').perform(context),
        'lidar_roll': LaunchConfiguration('lidar_roll').perform(context),
        'body_height': LaunchConfiguration('body_height').perform(context),
    }
    # extrinsic: urdf の IMU（Damiao）の外部パラメータは、この取り付けから求める
    imu = load_imu_selection(
        share / 'config' / 'imu_source.yaml', LaunchConfiguration('imu_source').perform(context),
        load_robot_description(robot_xacro, xacro_args),
    )
    fast_lio_file = share / 'config' / 'fast_lio.yaml'
    # 自機の除去
    self_filter = fast_lio_self_filter_parameters(
        LaunchConfiguration('lio_self_filter').perform(context),
        share / 'config' / 'footprint.yaml',
        robot_xacro,
        xacro_args,
    )
    nodes: list[LaunchDescriptionEntity] = [
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='laser_mapping',
            namespace=LOCALIZATION_NS,
            output='screen',
            remappings=FAST_LIO_REMAPPINGS,
            parameters=[
                str(fast_lio_file),
                {
                    'use_sim_time': True,
                    **fast_lio_parameters(imu, read_accel_clip_mps2(fast_lio_file)),
                    **self_filter,
                    # 可視化するときだけ、位置合わせ後の点群を出す
                    'publish.scan_publish_en': ParameterValue(
                        LaunchConfiguration('rviz'), value_type=bool,
                    ),
                },
            ],
        ),
        Node(
            package='core_localization',
            executable='lio_odom_adapter',
            name='lio_odom_adapter',
            namespace=LOCALIZATION_NS,
            output='screen',
            parameters=[{'use_sim_time': True, **adapter_parameters(imu)}],
        ),
    ]
    fusion = fusion_node(imu, share / 'config' / 'imu_fusion.yaml', use_sim_time=True)
    return nodes if fusion is None else [*nodes, fusion]


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('core_launch'))
    sim_share = Path(get_package_share_directory('core_gazebo'))

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    rviz = LaunchConfiguration('rviz')

    # 渡した引数がこの launch の rviz などを上書きしないよう、スコープを分ける
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
                'headless': LaunchConfiguration('headless'),
                # base_link の親は odom にするので、真値の TF は出さない
                'publish_ground_truth_tf': 'false',
                'rviz': 'false',
            }.items(),
        ),
    ])

    converter = {
        'use_sim_time': True,
        'scan_motion': ParameterValue(LaunchConfiguration('lidar_scan_motion'), value_type=bool),
        'include_no_return': ParameterValue(LaunchConfiguration('lidar_no_return_points'), value_type=bool),
        # 自機の車体の高さ（車体に当たった点は走査の間の動きでずらさない）
        'body.height_m': ParameterValue(LaunchConfiguration('body_height'), value_type=float),
    }
    points_to_livox = Node(
        package='core_gazebo',
        executable='points_to_livox',
        name='points_to_livox',
        namespace=SENSING_NS,
        output='screen',
        parameters=[converter],
    )

    # 検出のための 2 台目の LiDAR（rear_lidar:=true）の変換。1 台目と同じ点数に間引き、/livox/lidar2 に出す。
    # LIO と地図照合は 1 台目だけを使う
    rear_points_to_livox = Node(
        package='core_gazebo',
        executable='points_to_livox',
        name='points_to_livox_rear',
        namespace=SENSING_NS,
        output='screen',
        parameters=[converter],
        remappings=[('/sensing/sim/points', '/sensing/sim/points_rear'), ('/sensing/livox/lidar', '/sensing/livox/lidar2')],
        condition=IfCondition(LaunchConfiguration('rear_lidar')),
    )

    # odom は起動時の base_link なので、初期位置が world との差になる。真値と見比べるために出す
    world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '--x', x, '--y', y, '--yaw', yaw,
            '--frame-id', 'world', '--child-frame-id', 'odom',
        ],
        parameters=[{'use_sim_time': True}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', str(share / 'rviz' / 'sim_lio.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='-6.0', description='初期位置 x [m]'),
        DeclareLaunchArgument('y', default_value='-2.5', description='初期位置 y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='初期方位 [rad]'),
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
            description='検出のための 2 台目の LiDAR を機体の後ろの端に付ける（LIO と地図照合は 1 台目だけ）',
        ),
        DeclareLaunchArgument('headless', default_value='true', description='GazeboのGUIを出さない'),
        DeclareLaunchArgument('rviz', default_value='false', description='RVizを起動する'),
        DeclareLaunchArgument(
            'imu_source', default_value=DEFAULT_IMU_SOURCE, choices=list(IMU_SOURCES),
            description='FAST-LIO に入れる IMU（値は config/imu_source.yaml）',
        ),
        DeclareLaunchArgument(
            'lio_self_filter', default_value=DEFAULT_LIO_SELF_FILTER, choices=list(LIO_SELF_FILTER_MODES),
            description='FAST-LIO での自機の除き方（box / legacy_blind）',
        ),
        simulation,
        points_to_livox,
        rear_points_to_livox,
        OpaqueFunction(function=build_lio_nodes),
        world_to_odom,
        rviz_node,
    ])
