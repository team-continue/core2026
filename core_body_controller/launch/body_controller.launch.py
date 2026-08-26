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
        default_value="-6.28",
        description="High rotation angular velocity [rad/s]",
    )
    yaw_rotation_velocity_arg = DeclareLaunchArgument(
        "yaw_rotation_velocity",
        default_value="6.28",
        description="Max yaw angular velocity [rad/s]",
    )
    yaw_rotation_acceleration_arg = DeclareLaunchArgument(
        "yaw_rotation_acceleration",
        default_value="9.42477796076938",
        description="Max yaw angular acceleration [rad/s^2]",
    )
    cmd_vel_timeout_arg = DeclareLaunchArgument(
        "cmd_vel_timeout_sec",
        default_value="0.2",
        description="Maximum accepted age of cmd_vel [s]",
    )
    imu_timeout_arg = DeclareLaunchArgument(
        "imu_timeout_sec",
        default_value="0.2",
        description="Maximum accepted age of IMU data [s]",
    )
    body_omega_timeout_arg = DeclareLaunchArgument(
        "body_omega_timeout_sec",
        default_value="0.2",
        description="Maximum accepted age of body_omega feedforward [s]",
    )
    imu_yaw_bias_arg = DeclareLaunchArgument(
        "imu_yaw_bias",
        default_value="0.031415926535897934",
        description="Yaw gyro bias compensation [rad/s]",
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
                "cmd_vel_timeout_sec": LaunchConfiguration("cmd_vel_timeout_sec"),
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
                "yaw_rotation_acceleration": LaunchConfiguration(
                    "yaw_rotation_acceleration"
                ),
                "cmd_vel_timeout_sec": LaunchConfiguration("cmd_vel_timeout_sec"),
                "imu_timeout_sec": LaunchConfiguration("imu_timeout_sec"),
                "body_omega_timeout_sec": LaunchConfiguration(
                    "body_omega_timeout_sec"
                ),
                "imu_yaw_bias": LaunchConfiguration("imu_yaw_bias"),
            }
        ],
    )

    return LaunchDescription(
        [
            acceleration_arg,
            rotation_acceleration_arg,
            yaw_rotation_velocity_arg,
            yaw_rotation_acceleration_arg,
            auto_rotation_velocity_arg,
            high_rotation_velocity_arg,
            cmd_vel_timeout_arg,
            imu_timeout_arg,
            body_omega_timeout_arg,
            imu_yaw_bias_arg,
            body_control,
            target_angle,
        ]
    )
