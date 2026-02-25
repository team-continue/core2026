# Copyright 2026 team-continue
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('core_path_follower')
    param_file = os.path.join(pkg_share, 'param', 'default_params.yaml')

    path_follower_node = Node(
        package='core_path_follower',
        executable='core_path_follower_node',
        name='core_path_follower',
        output='screen',
        parameters=[param_file],
    )

    return LaunchDescription([path_follower_node])
