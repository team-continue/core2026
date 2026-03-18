"""Aggregate launch file for the full core2026 system."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _launch_path(package_name: str, launch_file: str) -> str:
    return os.path.join(get_package_share_directory(package_name), "launch", launch_file)


def _include_launch(package_name: str, launch_file: str, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_path(package_name, launch_file)),
        launch_arguments=(launch_arguments or {}).items(),
    )


def _is_true(context, name: str) -> bool:
    return context.launch_configurations.get(name, "false").lower() == "true"


def _launch_all(context):
    environment = context.launch_configurations.get("environment", "sim").lower()

    actions = []

    if _is_true(context, "launch_state_publisher"):
        actions.append(
            _include_launch(
                "core_launch",
                "state_publisher.launch.py",
                {
                    "use_sim_time": context.launch_configurations["use_sim_time"],
                },
            )
        )

    if _is_true(context, "launch_imu_filter") or (
        environment == "real" and _is_true(context, "launch_mid360")
    ):
        actions.append(
            _include_launch(
                "core_launch",
                "sensing.launch.py",
                {
                    "environment": context.launch_configurations["environment"],
                    "launch_mid360": context.launch_configurations["launch_mid360"],
                    "launch_imu_filter": context.launch_configurations[
                        "launch_imu_filter"
                    ],
                    "imu_use_mag": context.launch_configurations["imu_use_mag"],
                    "imu_publish_tf": context.launch_configurations["imu_publish_tf"],
                    "imu_world_frame": context.launch_configurations["imu_world_frame"],
                },
            )
        )

    if environment == "real" and _is_true(context, "launch_usb_camera"):
        actions.append(_include_launch("core_camera", "usb_cam.launch.py"))

    if _is_true(context, "launch_navigation"):
        actions.append(
            _include_launch(
                "core_launch",
                "navigation.launch.py",
                {
                    "environment": context.launch_configurations["environment"],
                    "odom_source": context.launch_configurations["odom_source"],
                    "map_name": context.launch_configurations["map_name"],
                    "init_yaw": context.launch_configurations["init_yaw"],
                    "use_rviz": context.launch_configurations["use_rviz"],
                    "use_smoother": context.launch_configurations["use_smoother"],
                    "use_localization": context.launch_configurations["use_localization"],
                    "launch_mid360": "false",
                },
            )
        )

    if _is_true(context, "launch_mode"):
        actions.append(_include_launch("core_mode", "mode.launch.py"))

    if environment == "real" and _is_true(context, "launch_status_gui"):
        actions.append(
            _include_launch("core_status_gui", "status_display_gui.launch.py")
        )

    if _is_true(context, "launch_wireless_parser"):
        actions.append(
            _include_launch(
                "core_ros_player_controller",
                "wireless_parser_node.launch.py",
            )
        )

    if _is_true(context, "launch_shooter"):
        actions.append(_include_launch("core_shooter", "shooter.launch.py"))

    if _is_true(context, "launch_enemy_detection"):
        actions.append(
            _include_launch("core_enemy_detection", "detection.launch.py")
        )

    if environment == "real" and _is_true(context, "launch_hardware"):
        actions.append(_include_launch("core_hardware", "core_hardware.launch.py"))

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "environment",
                default_value="sim",
                description="'sim' or 'real'. Forwarded to navigation.launch.py",
            ),
            DeclareLaunchArgument(
                "odom_source",
                default_value="sim",
                description="'sim' or 'fastlio'. Forwarded to navigation.launch.py",
            ),
            DeclareLaunchArgument(
                "map_name",
                default_value="core1_field",
                description="Map preset forwarded to navigation.launch.py",
            ),
            DeclareLaunchArgument(
                "init_yaw",
                default_value="0.0",
                description="Initial yaw [rad] forwarded to navigation.launch.py",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz through navigation.launch.py",
            ),
            DeclareLaunchArgument(
                "use_smoother",
                default_value="true",
                description="Launch cmd_vel smoother through navigation.launch.py",
            ),
            DeclareLaunchArgument(
                "use_localization",
                default_value="false",
                description="Enable localization through navigation.launch.py",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Forwarded to state_publisher.launch.py",
            ),
            DeclareLaunchArgument(
                "imu_use_mag",
                default_value="false",
                description="Forwarded to imu_filter.launch.py",
            ),
            DeclareLaunchArgument(
                "imu_publish_tf",
                default_value="false",
                description="Forwarded to imu_filter.launch.py",
            ),
            DeclareLaunchArgument(
                "imu_world_frame",
                default_value="enu",
                description="Forwarded to imu_filter.launch.py",
            ),
            DeclareLaunchArgument(
                "launch_state_publisher",
                default_value="true",
                description="Include state_publisher.launch.py",
            ),
            DeclareLaunchArgument(
                "launch_imu_filter",
                default_value="true",
                description="Include imu_filter.launch.py",
            ),
            DeclareLaunchArgument(
                "launch_navigation",
                default_value="true",
                description="Include navigation.launch.py",
            ),
            DeclareLaunchArgument(
                "launch_mid360",
                default_value="true",
                description="Include sensing.launch.py Mid-360 path when environment:=real",
            ),
            DeclareLaunchArgument(
                "launch_usb_camera",
                default_value="true",
                description="Include usb_cam.launch.py when environment:=real",
            ),
            DeclareLaunchArgument(
                "launch_status_gui",
                default_value="true",
                description="Include status_display_gui.launch.py when environment:=real",
            ),
            DeclareLaunchArgument(
                "launch_hardware",
                default_value="true",
                description="Include core_hardware.launch.py when environment:=real",
            ),
            DeclareLaunchArgument(
                "launch_mode",
                default_value="true",
                description="Include mode.launch.py",
            ),
            DeclareLaunchArgument(
                "launch_wireless_parser",
                default_value="true",
                description="Include wireless_parser_node.launch.py",
            ),
            DeclareLaunchArgument(
                "launch_shooter",
                default_value="true",
                description="Include shooter.launch.py",
            ),
            DeclareLaunchArgument(
                "launch_enemy_detection",
                default_value="true",
                description="Include detection.launch.py",
            ),
            OpaqueFunction(function=_launch_all),
        ]
    )
