from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    acceleration_arg = DeclareLaunchArgument(
        "acceleration",
        default_value="2.0",
        description="Linear acceleration limit [m/s^2]",
    )
    rotation_acceleration_arg = DeclareLaunchArgument(
        "rotation_acceleration",
        default_value="3.141592653589793",
        description="Angular acceleration limit [rad/s^2]",
    )
    auto_rotation_velocity_arg = DeclareLaunchArgument(
        "auto_rotation_velocity",
        default_value="-3.141592653589793",
        description="Auto rotation angular velocity [rad/s]",
    )
    high_rotation_velocity_arg = DeclareLaunchArgument(
        "high_rotation_velocity",
        default_value="3.141592653589793",
        description="High rotation angular velocity [rad/s]",
    )
    yaw_rotation_velocity_arg = DeclareLaunchArgument(
        "yaw_rotation_velocity",
        default_value="6.28",
        description="Max yaw angular velocity [rad/s]",
    )

    body_control = Node(
        package="core_body_controller",
        executable="body_control_node",
        name="body_control_node",
        output="screen",
        parameters=[
            {
                "acceleration": LaunchConfiguration("acceleration"),
                "rotation_acceleration": LaunchConfiguration("rotation_acceleration"),
                "auto_rotation_velocity": LaunchConfiguration("auto_rotation_velocity"),
                "high_rotation_velocity": LaunchConfiguration("high_rotation_velocity"),
            }
        ],
    )

    target_angle = Node(
        package="core_body_controller",
        executable="target_angle_node",
        name="target_angle_node",
        output="screen",
        remappings=[("imu", "livox/imu")],
        parameters=[
            {
                "yaw_rotation_velocity": LaunchConfiguration("yaw_rotation_velocity"),
            }
        ],
    )

    return LaunchDescription(
        [
            acceleration_arg,
            rotation_acceleration_arg,
            yaw_rotation_velocity_arg,
            auto_rotation_velocity_arg,
            high_rotation_velocity_arg,
            body_control,
            target_angle,
        ]
    )
