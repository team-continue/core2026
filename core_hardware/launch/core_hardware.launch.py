from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    wireless_topic_arg = DeclareLaunchArgument(
        'wireless_topic',
        default_value='/hardware/wireless',
        description='Topic used for wireless data published by core_hardware',
    )

    core_hardware_bridge = Node(
        package='core_hardware',
        executable='core_hardware',
        output='screen',
        parameters=[{'socket_path': '/tmp/core_hardware.sock'}],
        remappings=[
            ('wireless', LaunchConfiguration('wireless_topic')),
        ],
    )
    # USB(Teensy)直結版。socket版と排他なので通常は無効。
    # 使う場合はこの定義と下の GroupAction 内の行を同時に有効化する。
    # core_hardware_usb = Node(
    #     package='core_hardware',
    #     executable='core_hardware_usb',
    #     output='screen',
    #     parameters=[{'port': '/dev/teensy'}],
    # )
    return LaunchDescription([
        wireless_topic_arg,
        GroupAction([
            PushRosNamespace('hardware'),
            core_hardware_bridge,
            # core_hardware_usb,
        ]),
    ])
