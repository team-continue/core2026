from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:

    costmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory('core_costmap_builder'),
                '/launch/costmap_build.launch.py',
            ]
        )
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
