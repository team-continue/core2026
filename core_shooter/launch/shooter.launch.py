from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
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
        parameters=[shooter_params],
        remappings=[
            ("center_shoot_cmd", "/center/shoot_cmd"),
            ("left_shoot_cmd", "/left/shoot_cmd"),
            ("right_shoot_cmd", "/right/shoot_cmd"),
        ]
    )

    center_shooter_controller_node = Node(
        package="core_shooter",
        executable="shooter_controller",
        name="center_shooter_controller",
        output="screen",
        parameters=[
            shooter_params,
            {
                "shoot_motor_id": 10,
                "loading_motor_id": 7
            }
        ],
        # remappings=[
        #     ("shoot_cmd", "/center_shoot_cmd")
        # ]
    )

    left_shooter_controller_node = Node(
        package="core_shooter",
        executable="shooter_controller",
        name="left_shooter_controller",
        output="screen",
        parameters=[
            shooter_params,
            {
                "shoot_motor_id": 11,
                "loading_motor_id": 8
            }
        ],
        # remappings=[
        #     ("shoot_cmd", "/left_shoot_cmd")
        # ]
    )

    right_shooter_controller_node = Node(
        package="core_shooter",
        executable="shooter_controller",
        name="right_shooter_controller",
        output="screen",
        parameters=[
            shooter_params,
            {
                "shoot_motor_id": 12,
                "loading_motor_id": 9
            }
        ],
        # remappings=[
        #     ("shoot_cmd", "/right_shoot_cmd")
        # ]
    )

    center_magazine_manager_node = Node(
        package="core_shooter",
        executable="magazine_manager",
        name="center_magazine_manager",
        output="screen",
        parameters=[
            shooter_params,
        ],
        # remappings=[
        #     ("~/disk_distance_sensor", "distance")
        # ]
    )

    left_magazine_manager_node = Node(
        package="core_shooter",
        executable="magazine_manager",
        name="left_magazine_manager",
        output="screen",
        parameters=[
            shooter_params,
        ],
        # remappings=[
        #     ("~/disk_distance_sensor", "distance")
        # ]
    )

    right_magazine_manager_node = Node(
        package="core_shooter",
        executable="magazine_manager",
        name="right_magazine_manager",
        output="screen",
        parameters=[
            shooter_params,
        ],
        # remappings=[
        #     ("~/disk_distance_sensor", "distance")
        # ]
    )

    return LaunchDescription([
        shooter_cmd_gate_node,
        GroupAction([
            PushRosNamespace("center"),
            center_shooter_controller_node,
            center_magazine_manager_node,
        ]),

        GroupAction([
            PushRosNamespace("left"),
            left_shooter_controller_node,
            left_magazine_manager_node,
        ]),

        GroupAction([
            PushRosNamespace("right"),
            right_shooter_controller_node,
            right_magazine_manager_node,
        ]),
    ])
