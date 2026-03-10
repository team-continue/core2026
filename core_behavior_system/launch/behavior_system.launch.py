from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("core_behavior_system")
    params_file = os.path.join(pkg_share, "config", "behavior_system.yaml")

    selector_node = Node(
        package="core_behavior_system",
        executable="waypoint_selector_node",
        name="waypoint_selector",
        parameters=[params_file],
        output="screen",
    )

    manager_node = Node(
        package="core_behavior_system",
        executable="behavior_system_node",
        name="behavior_system",
        output="screen",
    )

    return LaunchDescription([manager_node, selector_node])
