from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _usb_cam_node(
    camera_name: str,
    video_device: LaunchConfiguration,
    remappings=None,
) -> Node:
    return Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        namespace=camera_name,
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': video_device,
            'camera_name': camera_name,
            'frame_id': f'{camera_name}_frame',
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'framerate': LaunchConfiguration('framerate'),
            'pixel_format': LaunchConfiguration('pixel_format'),
            'io_method': LaunchConfiguration('io_method'),
        }],
        remappings=remappings or [],
    )


def generate_launch_description():
    left_device = LaunchConfiguration('left_device')
    tps_device = LaunchConfiguration('tps_device')
    right_device = LaunchConfiguration('right_device')

    return LaunchDescription([
        DeclareLaunchArgument(
            'left_device',
            default_value='/dev/camera_left',
            description='Video device path for camera_left',
        ),
        DeclareLaunchArgument(
            'tps_device',
            default_value='/dev/camera_tps',
            description='Video device path for camera_tps',
        ),
        DeclareLaunchArgument(
            'right_device',
            default_value='/dev/camera_right',
            description='Video device path for camera_right',
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='640',
            description='Image width for all usb_cam nodes',
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='480',
            description='Image height for all usb_cam nodes',
        ),
        DeclareLaunchArgument(
            'framerate',
            default_value='30.0',
            description='Frame rate for all usb_cam nodes',
        ),
        DeclareLaunchArgument(
            'pixel_format',
            default_value='yuyv',
            description='Pixel format for all usb_cam nodes',
        ),
        DeclareLaunchArgument(
            'io_method',
            default_value='mmap',
            description='I/O method for all usb_cam nodes',
        ),
        _usb_cam_node(
            'camera_left',
            left_device,
            remappings=[
                ('image_raw', '/turret_camera_left/color/image'),
                ('camera_info', '/turret_camera_left/color/camera_info'),
            ],
        ),
        _usb_cam_node('camera_tps', tps_device),
        _usb_cam_node(
            'camera_right',
            right_device,
            remappings=[
                ('image_raw', '/turret_camera_right/color/image'),
                ('camera_info', '/turret_camera_right/color/camera_info'),
            ],
        ),
    ])
