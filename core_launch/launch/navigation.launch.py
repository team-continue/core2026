from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    costmap = Node(
        package='core_costmap_builder',
        executable='costmap_build_node',
        name='costmap_build_node',
        output='screen',
    )

    mppi = Node(
        package='core_mppi',
        executable='core_mppi_node',
        name='core_mppi_node',
        output='screen',
    )

    return LaunchDescription([
        costmap,
        mppi,
    ])
