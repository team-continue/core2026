"""Check the opt-in velocity input at the outgoing CAN interface."""
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from core_msgs.msg import CANArray
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64


@pytest.mark.launch_test
def generate_test_description():
    controller = launch_ros.actions.Node(
        package='core_body_controller', executable='target_angle_node', namespace='waist_test',
        parameters=[{'sysid_velocity_mode': True, 'cmd_vel_timeout_sec': 0.12,
                     'imu_timeout_sec': 0.12}],
        remappings=[('/system/emergency/hazard_status', '/waist_test/hazard')])
    return launch.LaunchDescription([controller, launch_testing.actions.ReadyToTest()])


class TestWaistSysid(unittest.TestCase):
    def test_only_fresh_safe_commands_reach_id4(self):
        rclpy.init()
        node = rclpy.create_node('driver', namespace='waist_test')
        try:
            velocity = node.create_publisher(Float64, 'sysid_velocity', 1)
            hazard = node.create_publisher(Bool, 'hazard', 1)
            imu = node.create_publisher(Imu, 'imu', 1)
            output = []
            node.create_subscription(
                CANArray, 'can/tx', lambda msg: output.extend(
                    m.data[-1] for m in msg.array if m.id == 4), 10)
            deadline = time.monotonic() + 5
            while not all(pub.get_subscription_count() for pub in (velocity, hazard, imu)):
                self.assertLess(time.monotonic(), deadline)
                rclpy.spin_once(node, timeout_sec=0.02)

            orientation = Imu()
            orientation.orientation.w = 1.0

            def check(speed, stop, send_imu, expected):
                output.clear()
                end = time.monotonic() + 0.60
                while time.monotonic() < end:
                    if stop is not None:
                        hazard.publish(Bool(data=stop))
                    if send_imu:
                        imu.publish(orientation)
                    if speed is not None:
                        velocity.publish(Float64(data=speed))
                    rclpy.spin_once(node, timeout_sec=0.02)
                    time.sleep(0.005)
                self.assertGreaterEqual(len(output), 5)
                for value in output[-5:]:
                    self.assertAlmostEqual(value, expected, places=5)

            check(0.15, None, True, 0.0)   # no initial clearance
            check(0.15, False, True, 0.15)
            check(None, False, True, 0.0)  # command expires
            check(-0.15, False, True, -0.15)
            check(-0.15, True, True, 0.0)  # e-stop wins over repeated commands
            check(-0.15, False, True, -0.15)
            check(-0.15, None, True, 0.0)  # missing hazard even if commands continue
            check(0.15, False, False, 0.0)
            check(0.36, False, True, 0.0)
        finally:
            node.destroy_node()
            rclpy.shutdown()
