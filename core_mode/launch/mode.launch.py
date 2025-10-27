from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # パラメータファイルのパスを取得
    mode_params = os.path.join(
        get_package_share_directory("core_mode"),
        "config",
        "mode.params.yaml"
    )

    emergency_handler_node = Node(
        package="core_mode",
        executable="emergency_handler",
        name="emergency_handler",
        output="screen",
        parameters=[mode_params],
        remappings=[
            ("emergency_switch", "/emergency"),
            ("destroy", "/destoroy"),
            ("emergency_button_on", "/left/shoot_cmd"),
            ("emergency_button_off", "/right/shoot_cmd"),
        ]
    )

    diagnostic_node = Node(
        package="core_mode",
        executable="diagnostic",
        name="diagnostic",
        output="screen",
        parameters=[mode_params],
        remappings=[
            ("microcontroller_monitor", "/joint_state"),
            ("receive_module_monitor", "/wireless"),
        ]
    )

    return LaunchDescription([
        GroupAction([
            PushRosNamespace("system"),
            GroupAction([
                PushRosNamespace("emergency"),
                emergency_handler_node,
                diagnostic_node,
            ]),
        ]),
    ])
