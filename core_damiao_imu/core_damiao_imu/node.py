"""ROS 2 node for a USB-connected Damiao DM-IMU-L1."""

import math
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial

from core_damiao_imu.protocol import configuration_commands
from core_damiao_imu.protocol import DamiaoFrame
from core_damiao_imu.protocol import DamiaoFrameParser
from core_damiao_imu.protocol import euler_zyx_degrees_to_quaternion
from core_damiao_imu.protocol import RID_ACCELERATION
from core_damiao_imu.protocol import RID_ANGULAR_VELOCITY
from core_damiao_imu.protocol import RID_EULER


Vector3 = Tuple[float, float, float]


class DamiaoImuNode(Node):
    """Read DM-IMU-L1 serial frames and publish ``sensor_msgs/Imu``."""

    def __init__(self) -> None:
        super().__init__("damiao_imu_node")
        self._port = self.declare_parameter("port", "/dev/ttyACM0").value
        self._baudrate = int(self.declare_parameter("baudrate", 921600).value)
        self._frame_id = self.declare_parameter(
            "frame_id", "damiao_imu_link"
        ).value
        self._output_rate_hz = int(
            self.declare_parameter("output_rate_hz", 200).value
        )
        self._reconnect_interval_sec = float(
            self.declare_parameter("reconnect_interval_sec", 1.0).value
        )

        if not self._port:
            raise ValueError("port must not be empty")
        if self._baudrate <= 0:
            raise ValueError("baudrate must be positive")
        if not self._frame_id:
            raise ValueError("frame_id must not be empty")
        if not math.isfinite(self._reconnect_interval_sec):
            raise ValueError("reconnect_interval_sec must be finite")
        if self._reconnect_interval_sec <= 0.0:
            raise ValueError("reconnect_interval_sec must be positive")
        configuration_commands(self._output_rate_hz)

        self._publisher = self.create_publisher(Imu, "imu", 10)
        self._serial: Optional[serial.Serial] = None
        self._parser = DamiaoFrameParser()
        self._acceleration: Optional[Vector3] = None
        self._angular_velocity: Optional[Vector3] = None
        self._next_connect_time = 0.0
        self._timer = self.create_timer(0.001, self._poll)

    def _poll(self) -> None:
        if self._serial is None:
            if time.monotonic() >= self._next_connect_time:
                self._connect()
            return

        try:
            readable = self._serial.in_waiting
            if readable <= 0:
                return
            data = self._serial.read(readable)
            if not data:
                return
            for frame in self._parser.feed(data):
                self._handle_frame(frame)
        except (OSError, serial.SerialException) as error:
            self._disconnect(f"serial read failed: {error}")

    def _connect(self) -> None:
        device = None
        try:
            device = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.0,
                write_timeout=0.2,
            )
            device.reset_input_buffer()
            self._configure(device)
            device.reset_input_buffer()
        except (OSError, serial.SerialException) as error:
            try:
                if device is not None:
                    device.close()
            except (OSError, serial.SerialException):
                pass
            self._next_connect_time = time.monotonic() + self._reconnect_interval_sec
            self.get_logger().warn(
                f"Could not open DM-IMU-L1 at {self._port}: {error}; retrying"
            )
            return

        self._serial = device
        self._parser.reset()
        self._acceleration = None
        self._angular_velocity = None
        self.get_logger().info(
            f"Connected to DM-IMU-L1 at {self._port} @ {self._baudrate} baud"
        )

    def _configure(self, device: serial.Serial) -> None:
        for command in configuration_commands(self._output_rate_hz):
            written = device.write(command)
            if written != len(command):
                raise serial.SerialException("short write while configuring IMU")
            device.flush()
            time.sleep(0.01)

    def _disconnect(self, reason: str) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except (OSError, serial.SerialException):
                pass
        self._serial = None
        self._parser.reset()
        self._acceleration = None
        self._angular_velocity = None
        self._next_connect_time = time.monotonic() + self._reconnect_interval_sec
        self.get_logger().warn(f"DM-IMU-L1 disconnected ({reason}); retrying")

    def _handle_frame(self, frame: DamiaoFrame) -> None:
        if frame.rid == RID_ACCELERATION:
            self._acceleration = frame.values
        elif frame.rid == RID_ANGULAR_VELOCITY:
            self._angular_velocity = frame.values
        elif (
            frame.rid == RID_EULER
            and self._acceleration is not None
            and self._angular_velocity is not None
        ):
            self._publish(frame.values)
            self._acceleration = None
            self._angular_velocity = None

    def _publish(self, euler_degrees: Vector3) -> None:
        if self._acceleration is None or self._angular_velocity is None:
            return

        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id

        quaternion = euler_zyx_degrees_to_quaternion(*euler_degrees)
        message.orientation.x = quaternion[0]
        message.orientation.y = quaternion[1]
        message.orientation.z = quaternion[2]
        message.orientation.w = quaternion[3]

        message.angular_velocity.x = self._angular_velocity[0]
        message.angular_velocity.y = self._angular_velocity[1]
        message.angular_velocity.z = self._angular_velocity[2]

        message.linear_acceleration.x = self._acceleration[0]
        message.linear_acceleration.y = self._acceleration[1]
        message.linear_acceleration.z = self._acceleration[2]

        self._publisher.publish(message)

    def destroy_node(self) -> bool:
        if self._serial is not None:
            try:
                self._serial.close()
            except (OSError, serial.SerialException):
                pass
            self._serial = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DamiaoImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
