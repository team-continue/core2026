from launch import LaunchDescription
from launch_ros.actions import Node


def _cam(
    name: str,
    device: str,
    width: int,
    height: int,
    fps: float,
    image_topic: str,
    info_topic: str,
):
    return Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        namespace=name,
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': device,
            'camera_name': name,
            'frame_id': f'{name}_frame',
            'image_width': width,
            'image_height': height,
            'framerate': fps,
            'pixel_format': 'mjpeg2rgb',
            'io_method': 'mmap',
            'autofocus': False,
            'focus': -1,
        }],
        remappings=[
            ('image_raw', image_topic),
            ('camera_info', info_topic),
        ],
    )


def generate_launch_description():
    return LaunchDescription([
        _cam(
            'camera_left',
            '/dev/camera_left',
            640,
            480,
            30.0,
            '/turret_camera_left/color/image',
            '/turret_camera_left/color/camera_info',
        ),
        _cam(
            'camera_tps',
            '/dev/camera_tps',
            1280,
            720,
            30.0,
            '/turret_camera_tps/color/image',
            '/turret_camera_tps/color/camera_info',
        ),
        _cam(
            'camera_right',
            '/dev/camera_right',
            640,
            480,
            30.0,
            '/turret_camera_right/color/image',
            '/turret_camera_right/color/camera_info',
        ),
    ])