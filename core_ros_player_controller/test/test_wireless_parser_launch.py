import os
import threading
import time
import unittest
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, Int32, UInt8MultiArray


@pytest.mark.launch_test
def generate_test_description():
    executable_override = os.environ.get("WIRELESS_PARSER_NODE_EXECUTABLE")
    param_file = Path(__file__).resolve().parents[1] / "config" / "wireless_parser_params.yaml"
    if executable_override:
        node = launch_ros.actions.Node(
            executable=executable_override,
            name="wireless_parser_node",
            output="screen",
            parameters=[str(param_file)],
        )
    else:
        node = launch_ros.actions.Node(
            package="core_ros_player_controller",
            executable="wireless_parser_node",
            name="wireless_parser_node",
            output="screen",
            parameters=[str(param_file)],
        )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {"node": node}


class TestWirelessParserNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("wireless_parser_protocol_test")
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.stop = False
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()
        self.pub = self.node.create_publisher(UInt8MultiArray, "/wireless", 10)
        self.values = {}
        self.events = {}
        for name, topic, msg_type in (
            ("cmd_vel", "/cmd_vel", Twist),
            ("rotation", "/rotation", Int32),
            ("ads", "/ads", Bool),
            ("manual_mode", "/manual_mode", Bool),
            ("manual_pitch", "/manual_pitch", Float32),
            ("roller", "/shoot_motor", Bool),
            ("shoot", "/right/shoot_fullauto", Bool),
            ("reload", "/reloading", Bool),
            ("estop", "/system/emergency/hazard_status", Bool),
        ):
            self.events[name] = threading.Event()
            self.node.create_subscription(
                msg_type, topic,
                lambda msg, n=name: self._received(n, msg), 10)

    def tearDown(self):
        self.stop = True
        self.thread.join(timeout=1.0)
        self.executor.remove_node(self.node)
        self.node.destroy_node()

    def _spin(self):
        while not self.stop:
            self.executor.spin_once(timeout_sec=0.1)

    def _received(self, name, msg):
        self.values[name] = msg
        self.events[name].set()

    def _publish(self, data):
        deadline = time.time() + 2.0
        while self.pub.get_subscription_count() == 0 and time.time() < deadline:
            time.sleep(0.02)
        msg = UInt8MultiArray()
        msg.data = data
        self.pub.publish(msg)

    def _publish_movement_and_wait_for_cmd_vel(self, movement):
        self.events["cmd_vel"].clear()
        self._publish([0, 0, 0, movement, 0, 0, 0])
        self.assertTrue(self.events["cmd_vel"].wait(2.0), "missing /cmd_vel")
        return self.values["cmd_vel"]

    def test_new_7byte_mapping(self):
        # EStop, Roller, Reload, Shoot, ADS, RightTurretAuto;
        # mouse x=-127, y=+127; W+A and InfiniteRotate=R2.
        self._publish([0b01011111, 0x81, 0x7F, 0b00100011, 0, 0, 0])
        for name in ("cmd_vel", "rotation", "ads", "manual_mode", "manual_pitch", "roller", "shoot", "estop"):
            self.assertTrue(self.events[name].wait(2.0), f"missing /{name}")

        self.assertAlmostEqual(self.values["cmd_vel"].linear.x, 0.25)
        self.assertAlmostEqual(self.values["cmd_vel"].linear.y, 0.25)
        self.assertAlmostEqual(self.values["cmd_vel"].angular.z, 2.0)
        self.assertEqual(self.values["rotation"].data, 2)
        self.assertTrue(self.values["ads"].data)
        self.assertFalse(self.values["manual_mode"].data)
        self.assertAlmostEqual(self.values["manual_pitch"].data, -1.0)
        self.assertTrue(self.values["roller"].data)
        self.assertTrue(self.values["shoot"].data)
        self.assertTrue(self.values["estop"].data)

    def test_wasd_velocity_scale_and_opposing_keys(self):
        for movement, expected_x, expected_y in (
            (0b0001, 0.25, 0.0),   # W
            (0b0010, 0.0, 0.25),   # A
            (0b0100, -0.25, 0.0),  # S
            (0b1000, 0.0, -0.25),  # D
            (0b0101, 0.0, 0.0),    # W + S
            (0b1010, 0.0, 0.0),    # A + D
        ):
            with self.subTest(movement=movement):
                cmd_vel = self._publish_movement_and_wait_for_cmd_vel(movement)
                self.assertAlmostEqual(cmd_vel.linear.x, expected_x)
                self.assertAlmostEqual(cmd_vel.linear.y, expected_y)
                self.assertAlmostEqual(cmd_vel.angular.z, 0.0)

    def test_reload_is_published_on_rising_edge(self):
        self._publish([0, 0, 0, 0, 0, 0, 0])
        self.events["reload"].clear()
        self._publish([0b00000100, 0, 0, 0, 0, 0, 0])
        self.assertTrue(self.events["reload"].wait(2.0))
        self.events["reload"].clear()
        self._publish([0b00000100, 0, 0, 0, 0, 0, 0])
        self.assertFalse(self.events["reload"].wait(0.4))


@launch_testing.post_shutdown_test()
class TestWirelessParserNodeAfterShutdown(unittest.TestCase):
    def test_process_exited(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
