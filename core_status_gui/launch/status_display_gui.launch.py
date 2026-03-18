from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    behavior_topic_arg = DeclareLaunchArgument(
        "behavior_topic",
        default_value="/behavior_system/state_name",
        description="Behavior state topic to display",
    )
    hazard_status_topic_arg = DeclareLaunchArgument(
        "hazard_status_topic",
        default_value="/system/emergency/hazard_status",
        description="Primary hazard status topic",
    )
    hazard_status_compat_topic_arg = DeclareLaunchArgument(
        "hazard_status_compat_topic",
        default_value="/system/emergency/hazard_state",
        description="Compatibility hazard status topic",
    )
    hazard_label_topic_arg = DeclareLaunchArgument(
        "hazard_label_topic",
        default_value="/system/emergency/hazard_label",
        description="Hazard label topic",
    )
    fullscreen_arg = DeclareLaunchArgument(
        "fullscreen",
        default_value="true",
        description="Launch the GUI in fullscreen mode",
    )
    screen_index_arg = DeclareLaunchArgument(
        "screen_index",
        default_value="0",
        description="Screen index parameter for the GUI",
    )
    window_title_arg = DeclareLaunchArgument(
        "window_title",
        default_value="ROS Status Display",
        description="Window title for the GUI",
    )

    node = Node(
        package="core_status_gui",
        executable="status_display_gui",
        name="status_display_gui",
        output="screen",
        parameters=[
            {
                "behavior_topic": LaunchConfiguration("behavior_topic"),
                "hazard_status_topic": LaunchConfiguration("hazard_status_topic"),
                "hazard_status_compat_topic": LaunchConfiguration("hazard_status_compat_topic"),
                "hazard_label_topic": LaunchConfiguration("hazard_label_topic"),
                "fullscreen": LaunchConfiguration("fullscreen"),
                "screen_index": LaunchConfiguration("screen_index"),
                "window_title": LaunchConfiguration("window_title"),
            }
        ],
    )

    return LaunchDescription([
        behavior_topic_arg,
        hazard_status_topic_arg,
        hazard_status_compat_topic_arg,
        hazard_label_topic_arg,
        fullscreen_arg,
        screen_index_arg,
        window_title_arg,
        node,
    ])
