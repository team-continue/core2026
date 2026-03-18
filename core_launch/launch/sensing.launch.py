"""Launch file for the sensing stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _launch_sensing(context):
    environment = context.launch_configurations.get("environment", "sim").lower()
    launch_mid360 = context.launch_configurations.get("launch_mid360", "true").lower()
    launch_imu_filter = context.launch_configurations.get(
        "launch_imu_filter", "true"
    ).lower()

    actions = []
    core_launch_share = get_package_share_directory("core_launch")

    if launch_imu_filter == "true":
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(core_launch_share, "launch", "imu_filter.launch.py")
                ),
                launch_arguments={
                    "use_mag": context.launch_configurations["imu_use_mag"],
                    "publish_tf": context.launch_configurations["imu_publish_tf"],
                    "world_frame": context.launch_configurations["imu_world_frame"],
                }.items(),
            )
        )

    if environment == "real" and launch_mid360 == "true":
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("livox_ros2_driver"),
                        "launch",
                        "livox_lidar_rviz_launch.py",
                    )
                )
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "environment",
                default_value="sim",
                description="'sim' or 'real'",
            ),
            DeclareLaunchArgument(
                "launch_mid360",
                default_value="true",
                description="Launch Livox Mid-360 when environment:=real",
            ),
            DeclareLaunchArgument(
                "launch_imu_filter",
                default_value="true",
                description="Include imu_filter.launch.py",
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
            OpaqueFunction(function=_launch_sensing),
        ]
    )
