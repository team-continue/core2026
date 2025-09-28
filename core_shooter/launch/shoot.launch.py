from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # パラメータファイルのパスを取得
    shooter_params = os.path.join(
        get_package_share_directory("core_shooter"),
        "config",
        "shooter.params.yaml"
    )

    shooter_cmd_gate_node = Node(
        package="core_shooter",
        executable="shooter_cmd_gate",
        name="shooter_cmd_gate",
        output="screen",
        parameters=[shooter_params]
    )

    center_shoot_controller_node = Node(
        package="core_shooter",
        executable="shoot_controller",
        name="center_shoot_controller",
        output="screen",
        parameters=[
            shooter_params,
            {
                shoot_motor_id: 10,
                loading_motor_id: 7
            }
        ],
        remappings=[
            ("shoot_cmd", "center_shoot_cmd")
        ]
    )

    left_shoot_controller_node = Node(
        package="core_shooter",
        executable="shoot_controller",
        name="left_shoot_controller",
        output="screen",
        parameters=[
            shooter_params,
            {
                shoot_motor_id: 11,
                loading_motor_id: 8
            }
        ],
        remappings=[
            ("shoot_cmd", "left_shoot_cmd")
        ]
    )

    right_shoot_controller_node = Node(
        package="core_shooter",
        executable="shoot_controller",
        name="right_shoot_controller",
        output="screen",
        parameters=[
            shooter_params,
            {
                shoot_motor_id: 12,
                loading_motor_id: 9
            }
        ],
        remappings=[
            ("shoot_cmd", "right_shoot_cmd")
        ]
    )

    return LaunchDescription([
        shooter_cmd_gate_node,
        center_shoot_controller_node,
        left_shoot_controller_node,
        right_shoot_controller_node
    ])
