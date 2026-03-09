from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    wireless_arg = DeclareLaunchArgument(
        "wireless",
        default_value="/wireless",
        description="Input topic to remap node subscription target /wireless",
    )
    rotation_flag_arg = DeclareLaunchArgument(
        "rotation_flag",
        default_value="/rotation_flag",
        description="Output topic to remap /rotation_flag",
    )
    cmd_vel_arg = DeclareLaunchArgument(
        "cmd_vel",
        default_value="/cmd_vel",
        description="Output topic to remap /cmd_vel",
    )
    manual_mode_arg = DeclareLaunchArgument(
        "manual_mode",
        default_value="/manual_mode",
        description="Output topic to remap /manual_mode",
    )
    manual_pitch_arg = DeclareLaunchArgument(
        "manual_pitch",
        default_value="/manual_pitch",
        description="Output topic to remap /manual_pitch",
    )
    shoot_motor_arg = DeclareLaunchArgument(
        "shoot_motor",
        default_value="/shoot_motor",
        description="Output topic to remap /shoot_motor",
    )
    left_shoot_once_arg = DeclareLaunchArgument(
        "left_shoot_once",
        default_value="/left/shoot_once",
        description="Output topic to remap /left/shoot_once",
    )
    test_mode_arg = DeclareLaunchArgument(
        "test_mode",
        default_value="/test_mode",
        description="Output topic to remap /test_mode",
    )

    node = Node(
        package="core_ros_player_controller",
        executable="wireless_parser_node",
        name="wireless_parser_node",
        output="screen",
        remappings=[
            ("/wireless", LaunchConfiguration("wireless")),
            ("/rotation_flag", LaunchConfiguration("rotation_flag")),
            ("/cmd_vel", LaunchConfiguration("cmd_vel")),
            ("/manual_mode", LaunchConfiguration("manual_mode")),
            ("/manual_pitch", LaunchConfiguration("manual_pitch")),
            ("/shoot_motor", LaunchConfiguration("shoot_motor")),
            ("/left/shoot_once", LaunchConfiguration("left_shoot_once")),
            ("/test_mode", LaunchConfiguration("test_mode")),
        ],
    )

    return LaunchDescription([
        wireless_arg,
        rotation_flag_arg,
        cmd_vel_arg,
        manual_mode_arg,
        manual_pitch_arg,
        shoot_motor_arg,
        left_shoot_once_arg,
        test_mode_arg,
        node,
    ])
