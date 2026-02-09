from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('core_path_follower')
    param_file = os.path.join(pkg_share, 'config', 'default_params.yaml')

    node = Node(
        package='core_path_follower',
        executable='path_follower_node',
        name='core_path_follower',
        output='screen',
        parameters=[param_file]
    )

    return LaunchDescription([node])
