from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('core_enemy_detection')

    left_turret_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'enemy_detection.launch.py')
        ),
        launch_arguments={
            'turret_name':'left',
            'input_topic':'/left/turret_camera_left/color/image',
            'output_topic':'/left/target_pose'
        }.items()
    )

    right_turret_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'enemy_detection.launch.py')
        ),
        launch_arguments={
            'turret_name':'right',
            'input_topic':'/turret_camera_right/color/image',
            'output_topic':'/right/target_pose'
        }.items()
    )

    return LaunchDescription([
        left_turret_launch,
        right_turret_launch
    ])