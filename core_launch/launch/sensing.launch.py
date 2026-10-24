"""
Launch the sensors (LiDAR, turret cameras, IMU) for the real robot.

センサ (LiDAR・砲塔カメラ・IMU) の起動 (実機用)。

名前空間は launch で付ける (PushRosNamespace('sensing'))。各ノードは相対名で出すので、
トピックは次のようになる。
  LiDAR : /sensing/livox/lidar, /sensing/livox/imu
  カメラ: /sensing/turret_camera_{left,tps,right}/color/{image,camera_info}
  IMU   : /sensing/imu
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _turret_camera(name: str, device: str, width: int, height: int, fps: float) -> Node:
    # namespace の turret_camera_<name>/color の下に image と camera_info を出す
    return Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        namespace=f'turret_camera_{name}/color',
        name='usb_cam',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_cameras')),
        parameters=[{
            'video_device': device,
            'camera_name': f'camera_{name}',
            'frame_id': f'camera_{name}_frame',
            'image_width': width,
            'image_height': height,
            'framerate': fps,
            'pixel_format': 'mjpeg2rgb',
            'io_method': 'mmap',
            'autofocus': False,
            'focus': -1,
        }],
        remappings=[('image_raw', 'image')],
    )


def generate_launch_description() -> LaunchDescription:
    livox = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        parameters=[{
            'xfer_format': 0,  # PointCloud2 (MID-360)
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'lvx_file_path': '/home/livox/livox_test.lvx',
            'user_config_path': PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'), 'config', 'MID360_config.json',
            ]),
            'cmdline_input_bd_code': 'livox0000000001',
        }],
    )

    damiao_imu = Node(
        package='core_damiao_imu',
        executable='damiao_imu_node',
        name='damiao_imu_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_imu')),
        parameters=[{
            'port': LaunchConfiguration('imu_port'),
            'baudrate': ParameterValue(LaunchConfiguration('imu_baudrate'), value_type=int),
            'frame_id': LaunchConfiguration('imu_frame_id'),
            'output_rate_hz': ParameterValue(
                LaunchConfiguration('imu_output_rate_hz'), value_type=int),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_lidar', default_value='true', description='Launch the Livox MID-360 driver'),
        DeclareLaunchArgument(
            'use_cameras', default_value='true', description='Launch the three turret cameras'),
        DeclareLaunchArgument(
            'use_imu', default_value='true', description='Launch the DM-IMU-L1 driver'),
        DeclareLaunchArgument(
            'imu_port',
            default_value='/dev/serial/by-id/usb-DM-IMU_DM-IMU_USB_CDC_2025021200-if00',
            description='DM-IMU-L1 USB serial device',
        ),
        DeclareLaunchArgument(
            'imu_baudrate', default_value='921600', description='DM-IMU-L1 USB serial baud rate'),
        DeclareLaunchArgument(
            'imu_frame_id', default_value='damiao_imu_link',
            description='Frame ID assigned to DM-IMU-L1 messages'),
        DeclareLaunchArgument(
            'imu_output_rate_hz', default_value='200', description='DM-IMU-L1 output rate [Hz]'),
        GroupAction([
            PushRosNamespace('sensing'),
            livox,
            damiao_imu,
            _turret_camera('left', '/dev/camera_left', 640, 480, 30.0),
            _turret_camera('tps', '/dev/camera_tps', 1280, 720, 30.0),
            _turret_camera('right', '/dev/camera_right', 640, 480, 30.0),
        ]),
    ])
