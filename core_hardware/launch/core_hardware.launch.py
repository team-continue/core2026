from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    core_hardware_ecat = Node(
            package='core_hardware',
            executable='core_hardware',
            prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
            shell=True,
            output='screen',
            parameters=[{'if_name': "eth0"}]
        )
    core_hardware_usb = Node(
            package='core_hardware',
            executable='core_hardware_usb',
            output="screen",
            parameters=[{'port': "/dev/teensy"}]
        )
    return LaunchDescription([
        # core_hardware_ecat,
        core_hardware_usb
    ])