import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events.process import ShutdownProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("gui_qt")
    params = [
        os.path.join(pkg_share, "config", "gui_qt.param.yaml"),
        os.path.join(pkg_share, "config", "global_battle.param.yaml"),
    ]

    gui_qt_node = Node(
        package="gui_qt",
        executable="gui_qt",
        name="gui_qt_node",
        output="screen",
        parameters=params,
        remappings=[
            ("~/input/log", "/gui_debug/log"),
            ("~/input/hp", "/hp"),
            ("~/input/ammo", "/mecha/shooter/remaining_disk"),
            ("~/input/compass", "/ui/yaw_degree"),
            ("~/input/speed", "/ui/speed_mps"),
            ("~/input/qe", "/ui/qe_degree"),
            ("~/input/camera", "/turret_camera_tps/color/image/compressed"),
            ("~/input/camera_sub", "/turret_camera_left/color/image/compressed"),
            ("~/input/camera_raw", "/turret_camera_tps/color/image"),
            ("~/input/camera_sub_raw", "/turret_camera_left/color/image"),
            ("~/input/ads", "/ads"),
            ("~/input/enemy_poses", "/enemy_poses"),
            ("~/input/hazard", "/system/emergency/hazard_status"),
            ("~/input/hazard_info", "")
            ("~/input/camera_change_1", "/pad/up"),
            ("~/input/camera_change_2", "/pad/down"),
            ("~/input/cursor_prev", "/pad/up"),
            ("~/input/cursor_next", "/pad/down"),
            ("~/input/value_down", "/pad/left"),
            ("~/input/value_up", "/pad/up"),
            ("~/input/cursor_ok", "/pad/cross"),
            ("~/input/cursor_back", "/pad/circle"),
            ("~/input/max_hp", "set/max_hp"),
            ("~/input/max_ammo", "/mecha/shooter/loading_disk"),
        ],
    )

    hardware_node = Node(
        package="gui_qt",
        executable="hardware_ui_converter_node",
        name="hardware_ui_converter_node",
        output="screen",
        remappings=[
            ("~/input/imu", "/imu"),
            ("~/output/yaw_degree", "/ui/yaw_degree"),
            ("~/output/qe_degree", "/ui/qe_degree"),
            ("~/output/speed_mps", "/ui/speed_mps"),
        ],
    )

    stop_hardware_on_gui_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=gui_qt_node,
            on_exit=[
                EmitEvent(
                    event=ShutdownProcess(
                        process_matcher=lambda action: action is hardware_node
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            gui_qt_node,
            hardware_node,
            stop_hardware_on_gui_exit,
        ]
    )
