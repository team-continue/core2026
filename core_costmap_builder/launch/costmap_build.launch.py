import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    this_dir = os.path.dirname(__file__)
    pkg_dir = os.path.abspath(os.path.join(this_dir, ".."))
    static_map_params = os.path.join(pkg_dir, "config", "static_global_map.yaml")
    costmap_params = os.path.join(pkg_dir, "config", "costmap_build_node.yaml")

    static_map = Node(
        package="core_costmap_builder",
        executable="static_global_map_publisher",
        name="static_global_map_publisher",
        output="screen",
        parameters=[static_map_params],
    )

    costmap_build = Node(
        package="core_costmap_builder",
        executable="costmap_build_node",
        name="costmap_build_node",
        output="screen",
        parameters=[costmap_params],
    )

    debug_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="debug_map_to_odom",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "map",
            "--child-frame-id", "odom",
        ],
    )

    debug_odom_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="debug_odom_to_base",
        output="screen",
        # x y z roll pitch yaw parent child
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
    )

    debug_base_to_livox = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="debug_base_to_livox",
        output="screen",
        # x y z roll pitch yaw parent child
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "livox_frame"],
    )

    return LaunchDescription([
        static_map,
        debug_map_to_odom,
        debug_odom_to_base,
        debug_base_to_livox,
        costmap_build,
    ])
