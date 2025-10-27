from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_camera',
            executable='usb_camera',
            name='usb_camera',
            output='screen'
        ),
        
        Node(
            package='core_enemy_detection',
            executable='enemy_detection',
            name='enemy_detection',
            output='screen',
            parameters=[
                {'image_fps': [120]},   # 未使用
                {'team': [0]},
                {'image_size': [1080, 980]},    # 未使用
                {'red_range_lower1]},': [0, 128, 200]},
                {'red_range_upper1': [1, 255, 255]},
                {'red_range_lower2': [170, 128, 125]},
                {'red_range_upper2': [180, 225, 255]},
                {'blue_range_lower': [105, 64,  255]},  #未調整
                {'blue_range_upper': [135, 255, 255]},  #未調整
                {'black_range_lower': [95, 50, 50]},
                {'black_range_upper': [115, 153, 100]},
                {'kernel_matrix_size': [3, 3]},
                {'calibration_file_path': '/home/hauki/Core2025/src/enemy_detection/config/Calibratin-HD.xml'}
            ]
        ),

        Node(
            package='enemy_detection',
            executable='enemy_selector',
            name='enemy_selector',
            output='screen',
            parameters=[
                {'calibration_file_path': '/home/hauki/Core2025/src/enemy_detection/config/Calibration-HD.xml'}
            ]
        )

    ])