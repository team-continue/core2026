from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    core_hardware_bridge = Node(
        package='core_hardware',
        executable='core_hardware',
        output='screen',
        parameters=[{'socket_path': '/tmp/core_hardware.sock'}],
    )
    core_hardware_usb = Node(
        package='core_hardware',
        executable='core_hardware_usb',
        output='screen',
        parameters=[{'port': '/dev/teensy'}],
    )
    return LaunchDescription([
        core_hardware_bridge,
        # core_hardware_usb,
    ])
