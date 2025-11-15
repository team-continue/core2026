import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("core_launch")
    default_model_path = PathJoinSubstitution(
        [package_share, "urdf", "core2025_attacker.urdf"]
    )

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=default_model_path,
        description="Absolute path to robot description file (URDF/Xacro)",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        arguments=[default_model_path],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription(
        [
            model_arg,
            use_sim_time_arg,
            joint_state_publisher_node,
            robot_state_publisher_node,
        ]
    )
