from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
import os

from ament_index_python.packages import get_package_share_directory
'''
# モータ番号
0-3: 足回りDamiao×4
4: 無限回転Yaw Robostride 06
5-6: 砲台Yaw Robostride05×2
7-14: Feetech
15-16: ESC
17: 非常停止
'''


def generate_launch_description():
    # パラメータファイルのパスを取得
    shooter_params = os.path.join(
        get_package_share_directory("core_shooter"),
        "config",
        "shooter.params.yaml"
    )
    hazard_remaps = ("hazard_status", "/system/emergency/hazard_status")

    shooter_cmd_gate_node = Node(
        package="core_shooter",
        executable="shooter_cmd_gate",
        name="shooter_cmd_gate",
        output="screen",
        parameters=[shooter_params],
        remappings=[
            ("left_shoot_cmd", "/left/shoot_cmd"),
            ("right_shoot_cmd", "/right/shoot_cmd"),
        ]
    )

    left_shooter_controller_node = Node(
        package="core_shooter",
        executable="shooter_controller",
        name="left_shooter_controller",
        output="screen",
        parameters=[
            shooter_params,
            {
                "shoot_motor_id": 15,
                "loading_motor_id": 9
            }
        ],
        remappings=[
            hazard_remaps
        ]
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
                "loading_motor_id": 10
            }
        ],
        remappings=[
            hazard_remaps
        ]
    )

    left_magazine_manager_node = Node(
        package="core_shooter",
        executable="magazine_manager",
        name="left_magazine_manager",
        output="screen",
        parameters=[
            shooter_params,
            {
                "disk_hold_left_motor_id": 13,
                "disk_hold_right_motor_id": 14
            }
        ],
        remappings=[
            ("disk_distance_sensor", "distance2"),
            hazard_remaps,
        ]
    )

    right_magazine_manager_node = Node(
        package="core_shooter",
        executable="magazine_manager",
        name="right_magazine_manager",
        output="screen",
        parameters=[
            shooter_params,
            {
                "disk_hold_left_motor_id": 9,
                "disk_hold_right_motor_id": 10
            }
        ],
        remappings=[
            ("disk_distance_sensor", "distance3"),
            hazard_remaps,
        ]
    )

    left_aim_bot_node = Node(
        package="core_shooter",
        executable="aim_bot",
        name="left_aim_bot",
        output="screen",
        parameters=[
            shooter_params,
            {
                "pitch_motor_id": 11,
                "yaw_motor_id": 6,

            }
        ],
        remappings=[
            hazard_remaps
        ]
    )

    right_aim_bot_node = Node(
        package="core_shooter",
        executable="aim_bot",
        name="right_aim_bot",
        output="screen",
        parameters=[
            shooter_params,
            {
                "pitch_motor_id": 7,
                "yaw_motor_id": 5,
            }
        ],
        remappings=[
            hazard_remaps
        ]
    )

    return LaunchDescription([
        shooter_cmd_gate_node,

        GroupAction([
            PushRosNamespace("left"),
            left_shooter_controller_node,
            left_magazine_manager_node,
            left_aim_bot_node,
        ]),

        GroupAction([
            PushRosNamespace("right"),
            right_shooter_controller_node,
            right_magazine_manager_node,
            right_aim_bot_node,
        ]),
    ])
