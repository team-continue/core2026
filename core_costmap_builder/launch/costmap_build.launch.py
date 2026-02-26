import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    this_dir = os.path.dirname(__file__)
    pkg_dir = os.path.abspath(os.path.join(this_dir, ".."))
    costmap_params = os.path.join(pkg_dir, "config", "costmap_build_node.yaml")

    costmap_build = Node(
        package="core_costmap_builder",
        executable="costmap_build_node",
        name="costmap_build_node",
        output="screen",
        parameters=[costmap_params],
    )

    debug_odom_to_base = Node(  # noqa: F841
        package="tf2_ros",
        executable="static_transform_publisher",
        name="debug_odom_to_base",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "odom",
            "--child-frame-id", "base_link",
        ],
    )

    debug_base_to_livox = Node(  # noqa: F841
        package="tf2_ros",
        executable="static_transform_publisher",
        name="debug_base_to_livox",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0.6",
            "--roll", "0", "--pitch", "3.141592653589793", "--yaw", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "livox_frame",
        ],
    )

    return LaunchDescription([
        # debug_odom_to_base,
        # debug_base_to_livox,
        costmap_build,
    ])
