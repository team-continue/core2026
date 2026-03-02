from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_mag_arg = DeclareLaunchArgument(
        "use_mag",
        default_value="false",
        description="Use magnetometer data if available",
    )

    publish_tf_arg = DeclareLaunchArgument(
        "publish_tf",
        default_value="false",
        description="Publish IMU orientation transform",
    )

    world_frame_arg = DeclareLaunchArgument(
        "world_frame",
        default_value="enu",
        description="World frame for orientation output",
    )

    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick",
        output="screen",
        parameters=[
            {
                "use_mag": LaunchConfiguration("use_mag"),
                "publish_tf": LaunchConfiguration("publish_tf"),
                "world_frame": LaunchConfiguration("world_frame"),
            }
        ],
        remappings=[
            ("imu/data_raw", "imu"),
            ("imu/data", "filtered_imu"),
        ],
    )

    return LaunchDescription(
        [
            use_mag_arg,
            publish_tf_arg,
            world_frame_arg,
            imu_filter_node,
        ]
    )
