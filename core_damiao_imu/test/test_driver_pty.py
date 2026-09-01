import errno
import math
import os
import pty
import select
import struct
import threading
import time

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Imu

from core_damiao_imu.node import DamiaoImuNode
from core_damiao_imu.protocol import configuration_commands
from core_damiao_imu.protocol import crc16


def make_frame(rid, values, slave_id=1):
    payload = bytearray((0x55, 0xAA, slave_id, rid))
    payload.extend(struct.pack("<fff", *values))
    checksum = crc16(payload)
    payload.extend((checksum & 0xFF, checksum >> 8, 0x0A))
    return bytes(payload)


def read_configuration(master_fd, timeout=2.0):
    expected = b"".join(configuration_commands(200))
    received = bytearray()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and expected not in received:
        readable, _, _ = select.select([master_fd], [], [], 0.05)
        if not readable:
            continue
        try:
            received.extend(os.read(master_fd, 4096))
        except OSError as error:
            if error.errno != errno.EIO:
                raise
    assert expected in received


def replace_symlink(link_path, target_path):
    replacement = link_path.with_name(link_path.name + ".new")
    replacement.symlink_to(target_path)
    os.replace(replacement, link_path)


def test_pty_configuration_publish_disconnect_and_reconnect(tmp_path):
    first_master, first_slave = pty.openpty()
    device_link = tmp_path / "damiao_imu"
    device_link.symlink_to(os.ttyname(first_slave))

    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            f"port:={device_link}",
            "-p",
            "frame_id:=damiao_test_link",
            "-p",
            "output_rate_hz:=200",
            "-p",
            "reconnect_interval_sec:=0.1",
        ]
    )
    driver = DamiaoImuNode()
    listener = rclpy.create_node("damiao_imu_pty_listener")
    messages = []
    listener.create_subscription(Imu, "/imu", messages.append, 100)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(driver)
    executor.add_node(listener)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    second_master = None
    second_slave = None
    try:
        read_configuration(first_master)
        time.sleep(0.1)

        sample = (
            make_frame(1, (1.0, 2.0, 9.0))
            + make_frame(2, (0.1, 0.2, 0.3))
            + make_frame(3, (0.0, 0.0, 90.0))
        )
        sample_count = 20
        for _ in range(sample_count):
            os.write(first_master, sample)
            time.sleep(0.005)

        deadline = time.monotonic() + 2.0
        while len(messages) < sample_count and time.monotonic() < deadline:
            time.sleep(0.01)
        assert len(messages) == sample_count
        first_message = messages[-1]
        assert first_message.header.frame_id == "damiao_test_link"
        assert first_message.header.stamp.sec > 0
        assert first_message.linear_acceleration.x == pytest.approx(1.0)
        assert first_message.angular_velocity.z == pytest.approx(0.3)
        assert first_message.orientation.z == pytest.approx(math.sqrt(0.5), abs=1e-6)
        assert first_message.orientation.w == pytest.approx(math.sqrt(0.5), abs=1e-6)

        os.close(first_master)
        os.close(first_slave)
        first_master = None
        first_slave = None
        message_count_at_disconnect = len(messages)

        second_master, second_slave = pty.openpty()
        replace_symlink(device_link, os.ttyname(second_slave))
        read_configuration(second_master, timeout=3.0)
        time.sleep(0.1)
        assert len(messages) == message_count_at_disconnect

        os.write(second_master, make_frame(3, (0.0, 0.0, -90.0)))
        time.sleep(0.1)
        assert len(messages) == message_count_at_disconnect

        resumed_sample = (
            make_frame(1, (4.0, 5.0, 6.0))
            + make_frame(2, (0.4, 0.5, 0.6))
            + make_frame(3, (0.0, 0.0, -90.0))
        )
        os.write(second_master, resumed_sample)
        deadline = time.monotonic() + 2.0
        while len(messages) == message_count_at_disconnect and time.monotonic() < deadline:
            time.sleep(0.01)
        assert len(messages) == message_count_at_disconnect + 1
        resumed_message = messages[-1]
        assert resumed_message.linear_acceleration.x == pytest.approx(4.0)
        assert resumed_message.angular_velocity.z == pytest.approx(0.6)
        assert resumed_message.orientation.z == pytest.approx(-math.sqrt(0.5), abs=1e-6)
        assert resumed_message.orientation.w == pytest.approx(math.sqrt(0.5), abs=1e-6)
    finally:
        executor.shutdown(timeout_sec=1.0)
        thread.join(timeout=1.0)
        executor.remove_node(listener)
        executor.remove_node(driver)
        listener.destroy_node()
        driver.destroy_node()
        rclpy.shutdown()
        for descriptor in (
            first_master,
            first_slave,
            second_master,
            second_slave,
        ):
            if descriptor is not None:
                os.close(descriptor)
