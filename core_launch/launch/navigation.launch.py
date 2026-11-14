"""
Navigation launch file.

Launches the full navigation pipeline with configurable environment,
control modes, and map presets.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# ── Map presets ──────────────────────────────────────────────────────
MAP_PRESETS = {
    'core1_field': {
        'image': 'core1_field.png',
        'resolution': 0.025,
        'origin_x': -13.675,
        'origin_y': -9.15,
        'init_x': -10.7,
        'init_y': 5.9,
    },
    'curious_house': {
        'image': 'curious_house.png',
        'resolution': 0.025,
        'origin_x': -4.5,
        'origin_y': -7.5,
        'init_x': 0.0,
        'init_y': 0.0,
    },
}


def _launch_nodes(context):
    """Resolve map preset and return all node actions."""
    # ── Resolve launch arguments ─────────────────────────────────────
    env = context.launch_configurations['environment']
    map_name = context.launch_configurations['map_name']
    use_rviz = context.launch_configurations['use_rviz']

    is_real = (env == 'real')
    use_damiao_imu = is_real and context.launch_configurations.get(
        'use_damiao_imu', 'true').lower() == 'true'

    # ── Map preset ───────────────────────────────────────────────────
    preset = MAP_PRESETS.get(map_name)
    if preset is None:
        available = ', '.join(MAP_PRESETS.keys())
        raise RuntimeError(
            f"Unknown map_name '{map_name}'. Available: {available}")

    core_launch_share = get_package_share_directory('core_launch')

    # ── Package directories ──────────────────────────────────────────
    body_ctrl_share = get_package_share_directory('core_body_controller')

    rviz_config = os.path.join(core_launch_share, 'config', 'navigation.rviz')

    livox_user_config = PathJoinSubstitution([
        FindPackageShare('livox_ros_driver2'), 'config', 'MID360_config.json',
    ])

    actions = []

    # ── 1. ROS-TCP-Endpoint (sim only) ───────────────────────────────
    if not is_real:
        actions.append(Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='ros_tcp_endpoint',
            output='screen',
            parameters=[{'ROS_IP': '127.0.0.1', 'ROS_TCP_PORT': 10000}],
        ))

    # ── 2. Livox driver (real only) ──────────────────────────────────
    # FAST-LIO と検出は CustomMsg（xfer_format 1）を受ける
    if is_real:
        actions.append(Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_driver',
            namespace='sensing',
            output='screen',
            remappings=[
                ('livox/lidar', '/sensing/livox/lidar'),
                ('livox/imu', '/sensing/livox/imu'),
            ],
            parameters=[{
                'xfer_format': 1,  # CustomMsg (MID-360)
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 10.0,
                'output_data_type': 0,
                'frame_id': 'livox_frame',
                'lvx_file_path': '/home/livox/livox_test.lvx',
                'user_config_path': livox_user_config,
                'cmdline_input_bd_code': 'livox0000000001',
            }],
        ))

    # ── 3. 自律移動（real only） ─────────────────────────────────────
    # 設計: 1 台構成・コンポーネント版。base_link→センサの固定 TF は URDF から出す
    if is_real:
        robot_xacro = os.path.join(core_launch_share, 'urdf', 'core2026.urdf.xacro')
        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', robot_xacro]), value_type=str),
            }],
            remappings=[('joint_states', '/hardware/joint_states')],
        ))

        map_path = context.launch_configurations.get('map_path', '')
        if not map_path:
            map_path = os.path.join(core_launch_share, 'maps', f'{map_name}.pcd')
        if not os.path.isfile(map_path):
            raise RuntimeError(
                f"PCD map not found: '{map_path}'. "
                f"Place core_launch/maps/{map_name}.pcd or pass map_path:=<pcd>.")
        composition = context.launch_configurations['composition']

        def include(name, arguments):
            return IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(core_launch_share, 'launch', name)),
                launch_arguments=arguments.items(),
            )

        actions.append(include('localization_stack.launch.py', {
            'map_path': map_path,
            'initial_x': str(preset['init_x']),
            'initial_y': str(preset['init_y']),
            'initial_yaw': context.launch_configurations['init_yaw'],
            'composition': composition,
        }))
        actions.append(include('perception.launch.py', {
            'map_path': map_path, 'composition': composition,
        }))
        actions.append(include('planning.launch.py', {
            'map_path': map_path, 'composition': composition,
        }))
    else:
        # Unity シミュレーションは自律移動を起動しない（Gazebo は sim_nav.launch.py を使う）
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_odom',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
        ))

    # ── 11. Body controller ─────────────────────────────────────────
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(body_ctrl_share, 'launch', 'body_controller.launch.py'),
        ),
        launch_arguments={
            'use_damiao_imu': 'true' if use_damiao_imu else 'false',
            'imu_port': context.launch_configurations['imu_port'],
            'imu_baudrate': context.launch_configurations['imu_baudrate'],
            'imu_frame_id': context.launch_configurations['imu_frame_id'],
            'imu_output_rate_hz': context.launch_configurations[
                'imu_output_rate_hz'],
        }.items(),
    ))

    # ── 12. RViz ────────────────────────────────────────────────────
    if use_rviz.lower() == 'true':
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'environment', default_value='sim',
            description="'sim' (Unity simulation) or 'real' (physical robot)",
        ),
        DeclareLaunchArgument(
            'map_name', default_value='core1_field',
            description=f"Map preset: {', '.join(MAP_PRESETS.keys())}",
        ),
        DeclareLaunchArgument(
            'init_yaw', default_value='0.0',
            description='Initial yaw on the map [rad]',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz2',
        ),
        DeclareLaunchArgument(
            'map_path', default_value='',
            description='PCD map for localization/perception/planning '
                        '(empty: core_launch/maps/<map_name>.pcd). Real only',
        ),
        DeclareLaunchArgument(
            'composition', default_value='true', choices=['true', 'false'],
            description='Run navigation nodes in per-stage component containers',
        ),
        DeclareLaunchArgument(
            'use_damiao_imu', default_value='true',
            description='Launch DM-IMU-L1 in real mode; always disabled in sim',
        ),
        DeclareLaunchArgument(
            'imu_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-DM-IMU_DM-IMU_USB_CDC_2025021200-if00'
            ),
            description='DM-IMU-L1 USB serial device (real mode only)',
        ),
        DeclareLaunchArgument(
            'imu_baudrate', default_value='921600',
            description='DM-IMU-L1 USB serial baud rate',
        ),
        DeclareLaunchArgument(
            'imu_frame_id', default_value='damiao_imu_link',
            description='Frame ID assigned to DM-IMU-L1 messages',
        ),
        DeclareLaunchArgument(
            'imu_output_rate_hz', default_value='200',
            description='DM-IMU-L1 output rate [Hz]',
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
