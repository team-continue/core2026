"""
Record the robot debug log (key topics and /rosout) with ros2 bag.

The topics and limits are in config/debug_log.yaml. The default output
directory is $CORE_DEBUG_LOG_DIR, or ~/rosbag/core_debug_log when it is unset.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'debug_log_config',
        default_value=PathJoinSubstitution(
            [FindPackageShare('core_launch'), 'config', 'debug_log.yaml']),
        description='記録するトピックと上限の設定')
    log_dir_arg = DeclareLaunchArgument(
        'debug_log_dir',
        default_value=EnvironmentVariable(
            'CORE_DEBUG_LOG_DIR',
            default_value=os.path.join(os.path.expanduser('~'), 'rosbag', 'core_debug_log')),
        description='記録を置くディレクトリ。起動ごとにサブディレクトリを作る')

    recorder = ExecuteProcess(
        name='debug_log_recorder',
        cmd=[
            PathJoinSubstitution(
                [FindPackagePrefix('core_launch'), 'lib', 'core_launch',
                 'debug_log_recorder.py']),
            '--config', LaunchConfiguration('debug_log_config'),
            '--log-dir', LaunchConfiguration('debug_log_dir'),
        ],
        output='screen',
        # metadata.yaml を書き終えるまで待つ
        sigterm_timeout='10',
    )

    return LaunchDescription([config_arg, log_dir_arg, recorder])
