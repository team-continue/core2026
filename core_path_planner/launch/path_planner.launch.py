from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    path_planner_node = Node(
        package="core_path_planner",
        executable="path_planner_node",
        name="core_path_planner_node",
        output="screen",
    )

    return LaunchDescription([path_planner_node])
