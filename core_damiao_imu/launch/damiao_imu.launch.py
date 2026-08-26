from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("imu_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("imu_baudrate", default_value="921600"),
            DeclareLaunchArgument(
                "imu_frame_id", default_value="damiao_imu_link"
            ),
            DeclareLaunchArgument("imu_output_rate_hz", default_value="200"),
            Node(
                package="core_damiao_imu",
                executable="damiao_imu_node",
                name="damiao_imu_node",
                output="screen",
                parameters=[
                    {
                        "port": LaunchConfiguration("imu_port"),
                        "baudrate": ParameterValue(
                            LaunchConfiguration("imu_baudrate"), value_type=int
                        ),
                        "frame_id": LaunchConfiguration("imu_frame_id"),
                        "output_rate_hz": ParameterValue(
                            LaunchConfiguration("imu_output_rate_hz"), value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
