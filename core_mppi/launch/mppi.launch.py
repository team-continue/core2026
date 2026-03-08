# Copyright 2026 team-continue
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('core_mppi')
    param_file = os.path.join(pkg_share, 'param', 'default_params.yaml')

    mppi_node = Node(
        package='core_mppi',
        executable='core_mppi_node',
        name='core_mppi_node',
        output='screen',
        parameters=[param_file],
    )

    return LaunchDescription([mppi_node])
