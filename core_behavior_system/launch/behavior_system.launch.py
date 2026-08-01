from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("core_behavior_system")
    params_file = os.path.join(pkg_share, "config", "behavior_system.yaml")

    shoot_manager_node = Node(
        package="core_behavior_system",
        executable="attack_shoot_manager_node",
        name="attack_shoot_manager",
        parameters=[params_file],
        output="screen",
    )

    return LaunchDescription([shoot_manager_node])
