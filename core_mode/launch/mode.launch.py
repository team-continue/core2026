from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction, Shutdown
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
            ("emergency_switch", "/hardware/hardware_emergency"),
            ("destroy", "/hardware/destroy"),
        ],
        # 緊急停止の判定が止まったまま動き続けないよう、落ちたら launch 全体を止める
        on_exit=Shutdown(reason="emergency_handler exited"),
    )

    diagnostic_node = Node(
        package="core_mode",
        executable="diagnostic",
        name="diagnostic",
        output="screen",
        parameters=[mode_params],
        remappings=[
            ("microcontroller_monitor", "/hardware/joint_states"),
            ("bridge_connected", "/bridge/connected"),
        ],
        on_exit=Shutdown(reason="diagnostic exited"),
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
