from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    wireless_topic_arg = DeclareLaunchArgument(
        "wireless_topic",
        default_value="/wireless",
        description="Input topic to remap node subscription target /wireless",
    )

    node = Node(
        package="core_ros_player_contoller",
        executable="wireless_parser_node",
        name="wireless_parser_node",
        output="screen",
        remappings=[
            ("/wireless", LaunchConfiguration("wireless_topic")),
        ],
    )

    return LaunchDescription([
        wireless_topic_arg,
        node,
    ])
