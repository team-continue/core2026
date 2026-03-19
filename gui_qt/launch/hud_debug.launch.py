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
            ("~/input/hp", "gui_debug/hp"),
            ("~/input/ammo", "gui_debug/ammo"),
            ("~/input/compass", "gui_debug/compass"),
            ("~/input/speed", "gui_debug/speed"),
            ("~/input/qe", "/gui_debug/qe"),
            ("~/input/log", "/gui_debug/log"),
            ("~/input/camera", "/camera/color/image/compressed"),
            ("~/input/camera_sub", "/camera/color/image/compressed_sub"),
            ("~/input/enemy_poses", "/gui_debug/enemy_poses"),
            ("~/input/hazard", "/gui_debug/hazard"),
            ("~/input/camera_change_1", "/gui_debug/input/camera1"),
            ("~/input/camera_change_2", "/gui_debug/input/camera2"),
            ("~/input/cursor_prev", "/gui_debug/input/up"),
            ("~/input/cursor_next", "/gui_debug/input/down"),
            ("~/input/value_down", "/gui_debug/input/left"),
            ("~/input/value_up", "/gui_debug/input/right"),
            ("~/input/cursor_ok", "/gui_debug/input/ok"),
            ("~/input/cursor_back", "/gui_debug/input/back"),
            ("~/input/max_hp", "set/max_hp"),
            ("~/input/max_ammo", "set/max_ammo"),
        ],
    )

    hardware_node = Node(
        package="gui_qt",
        executable="hardware_ui_converter_node",
        name="hardware_ui_converter_node",
        output="screen",
        remappings=[
            ("~/input/imu", "/imu"),
            ("~/output/yaw_degree", "/gui_debug/compass"),
            ("~/output/qe_degree", "/gui_debug/qe"),
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
