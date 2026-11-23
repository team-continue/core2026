"""実機の Livox Mid-360 のドライバ（livox_ros_driver2）のノード（navigation.launch.py と mapping.launch.py で使う）.

FAST-LIO と検出は CustomMsg（xfer_format 1）を受ける。点群と IMU は /sensing/livox/lidar と /sensing/livox/imu に出す。
livox_ros_driver2 は依存に書かず、各自で入れる（実機はドライバ本体）。
"""

from __future__ import annotations

from typing import Final

from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

LIVOX_REMAPPINGS: Final = (
    ('livox/lidar', '/sensing/livox/lidar'),
    ('livox/imu', '/sensing/livox/imu'),
)
LIVOX_PARAMETERS: Final = {
    'xfer_format': 1,  # CustomMsg (MID-360)
    'multi_topic': 0,
    'data_src': 0,
    'publish_freq': 10.0,
    'output_data_type': 0,
    'frame_id': 'livox_frame',
    'lvx_file_path': '/home/livox/livox_test.lvx',
    'cmdline_input_bd_code': 'livox0000000001',
}


def livox_driver_node() -> Node:
    """ドライバの設定（IP など）は livox_ros_driver2 の config/MID360_config.json を使う."""
    user_config = PathJoinSubstitution([
        FindPackageShare('livox_ros_driver2'), 'config', 'MID360_config.json',
    ])
    return Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_driver',
        namespace='sensing',
        output='screen',
        remappings=list(LIVOX_REMAPPINGS),
        parameters=[{**LIVOX_PARAMETERS, 'user_config_path': user_config}],
    )
