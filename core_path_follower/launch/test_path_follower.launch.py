# Copyright 2026 team-continue
# SPDX-License-Identifier: Apache-2.0

"""Launch path_follower with test_path_publisher for standalone testing."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('core_path_follower')
    param_file = os.path.join(pkg, 'param', 'default_params.yaml')

    args = [
        DeclareLaunchArgument(
            'path_type', default_value='straight',
            description='square|figure8|circle|straight|slalom|diamond|lateral'),
        DeclareLaunchArgument('use_local_frame', default_value='true'),
        DeclareLaunchArgument(
            'controller_type', default_value='cascade',
            description='cascade|pid|pure_pursuit'),
        DeclareLaunchArgument('path_scale', default_value='1.0'),
        DeclareLaunchArgument(
            'mode', default_value='path',
            description='path|planner'),
        DeclareLaunchArgument('cycle', default_value='false'),
        DeclareLaunchArgument('one_shot', default_value='false'),
        DeclareLaunchArgument('with_body_controller', default_value='false'),
        DeclareLaunchArgument('monitor_can', default_value='true'),
        DeclareLaunchArgument('publish_joint_states', default_value='false'),
    ]

    path_follower = Node(
        package='core_path_follower',
        executable='core_path_follower_node',
        name='core_path_follower',
        output='screen',
        parameters=[
            param_file,
            {
                'use_local_frame': LaunchConfiguration('use_local_frame'),
                'controller_type': LaunchConfiguration('controller_type'),
            },
        ],
    )

    test_publisher = Node(
        package='core_path_follower',
        executable='test_path_publisher',
        name='test_path_publisher',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('mode'),
            'path_type': LaunchConfiguration('path_type'),
            'use_local_frame': LaunchConfiguration('use_local_frame'),
            'frame_id': 'chassis_link',
            'publish_rate': 1.0,
            'path_scale': LaunchConfiguration('path_scale'),
            'one_shot': LaunchConfiguration('one_shot'),
            'cycle': LaunchConfiguration('cycle'),
            'monitor_can': LaunchConfiguration('monitor_can'),
            'publish_joint_states': LaunchConfiguration('publish_joint_states'),
        }],
    )

    body_controller = Node(
        package='core_body_controller',
        executable='body_control_node',
        name='body_control_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('with_body_controller')),
    )

    return LaunchDescription(args + [path_follower, test_publisher, body_controller])
