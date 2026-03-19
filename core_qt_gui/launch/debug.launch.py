from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource
)
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # パッケージパス取得
    rosbridge_dir = get_package_share_directory('rosbridge_server')
    ros_player_controller_pkg_dir = get_package_share_directory('core_ros_player_controller')
    gui_qt_pkg_dir = get_package_share_directory('gui_qt')
    mode_pkg_dir = get_package_share_directory('core_mode')

    # rosbridge (XML)
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(rosbridge_dir, 'launch', 'rosbridge_websocket_launch.xml')
        ),
        launch_arguments={
            'port': '12345'
        }.items()
    )

    # wireless parser (Python)
    wireless_parser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_player_controller_pkg_dir, 'launch', 'wireless_parser_node.launch.py')
        )
    )

    # mode (Python)
    mode_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mode_pkg_dir, 'launch', 'mode.launch.py')
        )
    )

    return LaunchDescription([
        rosbridge_launch,
        #wireless_parser_launch,
        mode_launch
    ])