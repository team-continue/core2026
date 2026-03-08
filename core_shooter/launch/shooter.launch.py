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
            ("manual_mode", "/manual_mode"),
            ("left_manual_mode", "/left/manual_mode"),
            ("right_manual_mode", "/right/manual_mode"),
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
                "loading_motor_id": 12,
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
                "shoot_motor_id": 16,
                "loading_motor_id": 8,
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
                "disk_hold_left_motor_id": 14,
                "disk_hold_right_motor_id": 13,
                "disk_hold_motor_left_angle": [0.0, -0.2],
                "disk_hold_motor_right_angle": [0.0, 0.2],
            }
        ],
        remappings=[
            ("disk_distance_sensor", "distance"),
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
                "disk_hold_left_motor_id": 10,
                "disk_hold_right_motor_id": 9,
                "disk_hold_motor_left_angle": [0.0, 0.2],
                "disk_hold_motor_right_angle": [0.0, -0.2],
            }
        ],
        remappings=[
            ("disk_distance_sensor", "distance"),
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
                "yaw_motor_id": 5,
                # "yaw_min_angle": -3.14159265359,
                # "yaw_max_angle": 3.14159265359,
                "pitch_min_angle": -3.13,
                "pitch_max_angle": 3.14,
                "zone.yaw_reversed": True,
                "zone.yaw_zone1_start": -0.5,
                "zone.yaw_boundary": -0.2,
                "zone.yaw_zone2_end": 0.2,
                "zone.yaw_zone3_end": 2.2,
                "zone.pitch_lower_limit": 0.0,
                "zone.pitch_zone2_upper": 3.14,
                "zone.pitch_zone2_lower": -3.14,
                "zone.pitch_zone2_upper_limit": 3.14,
                "zone.pitch_zone3_lower": -3.14,
                "zone.pitch_zone3_upper": 0.0,
                "zone.pitch_zone1_upper": 3.14,
                "control.hysteresis_rad": 0.017453292519943295,
                "control.pitch_correct_tolerance": 0.01,
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
                "yaw_motor_id": 6,
                # "yaw_min_angle": -3.14159265359,
                # "yaw_max_angle": 3.14159265359,
                "pitch_min_angle": -3.14,
                "pitch_max_angle": 3.14,
                "zone.yaw_reversed": False,
                "zone.yaw_zone1_start": -0.5,
                "zone.yaw_boundary": -0.2,
                "zone.yaw_zone2_end": 0.2,
                "zone.yaw_zone3_end": 2.2,
                "zone.pitch_lower_limit": 0.0,
                "zone.pitch_zone2_upper": 3.14,
                "zone.pitch_zone2_lower": -3.14,
                "zone.pitch_zone2_upper_limit": 3.14,
                "zone.pitch_zone3_lower": -3.14,
                "zone.pitch_zone3_upper": 0.0,
                "zone.pitch_zone1_upper": 3.14,
                "control.hysteresis_rad": 0.017453292519943295,
                "control.pitch_correct_tolerance": 0.01,
            }
        ],
        remappings=[
            hazard_remaps,
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
