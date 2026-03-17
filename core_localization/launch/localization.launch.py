"""Standalone launch file for core_localization (testing / development)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('core_localization')
    default_config = os.path.join(pkg_dir, 'config', 'localization_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'pcd_map_path',
            default_value='',
            description='Path to pre-built PCD map file',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to localization parameter YAML',
        ),
        Node(
            package='core_localization',
            executable='localization_node',
            name='localization_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'global_map_path': LaunchConfiguration('pcd_map_path')},
            ],
        ),
    ])
