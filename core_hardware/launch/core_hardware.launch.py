from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    core_hardware = Node(
            package='core_hardware',
            executable='core_hardware',
            prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
            shell=True,
            output='screen',
            parameters=[{'if_name': "eth0"}]
        )
    return LaunchDescription([
        core_hardware
    ])