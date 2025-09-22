#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_core_gz = get_package_share_directory('core_gz')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='core2025_attacker',
        description='Name of the robot'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_core_gz, 'worlds', 'field_2025.sdf'),
        description='Path to the world file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            pkg_core_gz
        ]).strip(os.pathsep)
    )

    urdf_file = os.path.join(pkg_core_gz, 'urdf', 'core2025_attacker.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [LaunchConfiguration('world')]}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', LaunchConfiguration('robot_name'),
            '-x', '-11.0',
            '-y', '7.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
            'joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        robot_name_arg,
        world_arg,
        use_sim_time_arg,
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        bridge
    ])
