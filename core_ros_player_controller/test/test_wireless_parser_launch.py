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
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import UInt8MultiArray


@pytest.mark.launch_test
def generate_test_description():
    executable_override = os.environ.get("WIRELESS_PARSER_NODE_EXECUTABLE")
    local_build_executable = Path.cwd() / "wireless_parser_node"
    param_file = Path(__file__).resolve().parents[1] / "config" / "wireless_parser_params.yaml"
    if executable_override:
        wireless_parser_node = launch_ros.actions.Node(
            executable=executable_override,
            name="wireless_parser_node",
            output="screen",
            parameters=[str(param_file)],
        )
    elif local_build_executable.exists():
        wireless_parser_node = launch_ros.actions.Node(
            executable=str(local_build_executable),
            name="wireless_parser_node",
            output="screen",
            parameters=[str(param_file)],
        )
    else:
        wireless_parser_node = launch_ros.actions.Node(
            package="core_ros_player_controller",
            executable="wireless_parser_node",
            name="wireless_parser_node",
            output="screen",
            parameters=[str(param_file)],
        )

    return (
        launch.LaunchDescription([
            wireless_parser_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {"wireless_parser_node": wireless_parser_node},
    )


class TestWirelessParserNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("wireless_parser_launch_test")
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)

        self.spin_thread_stop = False
        self.spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self.spin_thread.start()

        self.wireless_pub = self.node.create_publisher(UInt8MultiArray, "/wireless", 10)

        self.test_mode_msg = None
        self.test_mode_event = threading.Event()
        self.node.create_subscription(Bool, "/test_mode", self._on_test_mode, 10)

        self.cmd_vel_msg = None
        self.cmd_vel_event = threading.Event()
        self.node.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

        self.manual_mode_msg = None
        self.manual_mode_event = threading.Event()
        self.node.create_subscription(Bool, "/manual_mode", self._on_manual_mode, 10)

        self.manual_pitch_msg = None
        self.manual_pitch_event = threading.Event()
        self.node.create_subscription(Float32, "/manual_pitch", self._on_manual_pitch, 10)

        self.shoot_motor_msg = None
        self.shoot_motor_event = threading.Event()
        self.node.create_subscription(Bool, "/shoot_motor", self._on_shoot_motor, 10)

        self.shoot_once_msg = None
        self.shoot_once_event = threading.Event()
        self.node.create_subscription(Bool, "/left/shoot_once", self._on_shoot_once, 10)

        self.rotation_msg = None
        self.rotation_event = threading.Event()
        self.node.create_subscription(Bool, "/rotation", self._on_rotation, 10)

        self.ads_msg = None
        self.ads_event = threading.Event()
        self.node.create_subscription(Bool, "/ads", self._on_ads, 10)

        self.reloading_msg = None
        self.reloading_event = threading.Event()
        self.node.create_subscription(Bool, "/reloading", self._on_reloading, 10)

        self.hazard_status_msg = None
        self.hazard_status_event = threading.Event()
        self.node.create_subscription(Bool, "/system/emergency/hazard_status", self._on_hazard_status, 10)

    def tearDown(self):
        self.spin_thread_stop = True
        self.spin_thread.join(timeout=1.0)
        self.executor.remove_node(self.node)
        self.node.destroy_node()

    def _spin_loop(self):
        while not self.spin_thread_stop:
            self.executor.spin_once(timeout_sec=0.1)

    def _on_test_mode(self, msg: Bool):
        self.test_mode_msg = msg
        self.test_mode_event.set()

    def _on_cmd_vel(self, msg: Twist):
        self.cmd_vel_msg = msg
        self.cmd_vel_event.set()

    def _on_manual_mode(self, msg: Bool):
        self.manual_mode_msg = msg
        self.manual_mode_event.set()

    def _on_manual_pitch(self, msg: Float32):
        self.manual_pitch_msg = msg
        self.manual_pitch_event.set()

    def _on_shoot_motor(self, msg: Bool):
        self.shoot_motor_msg = msg
        self.shoot_motor_event.set()

    def _on_shoot_once(self, msg: Bool):
        self.shoot_once_msg = msg
        self.shoot_once_event.set()

    def _on_rotation(self, msg: Bool):
        self.rotation_msg = msg
        self.rotation_event.set()

    def _on_ads(self, msg: Bool):
        self.ads_msg = msg
        self.ads_event.set()

    def _on_reloading(self, msg: Bool):
        self.reloading_msg = msg
        self.reloading_event.set()

    def _on_hazard_status(self, msg: Bool):
        self.hazard_status_msg = msg
        self.hazard_status_event.set()

    def test_test_mode_is_false(self):
        msg = UInt8MultiArray()
        msg.data = [0, 0, 0, 0, 0, 0, 0]

        deadline = time.time() + 2.0
        while self.wireless_pub.get_subscription_count() == 0 and time.time() < deadline:
            time.sleep(0.05)

        self.wireless_pub.publish(msg)

        received = self.test_mode_event.wait(timeout=2.0)
        self.assertTrue(received, "Did not receive /test_mode")
        self.assertIsNotNone(self.test_mode_msg)
        self.assertFalse(self.test_mode_msg.data)

    def test_manual_mode_outputs_and_new_bit_mapping(self):
        msg = UInt8MultiArray()
        # values[0]: b7 roller, b6 click, b5 reload, b4 d, b3 a, b2 s, b1 w, b0 space
        # values[4]: b1 rotation, b0 ADS
        msg.data = [0b11001010, 127, 64, 0b00000000, 0b00000011, 0, 0]

        deadline = time.time() + 2.0
        while self.wireless_pub.get_subscription_count() == 0 and time.time() < deadline:
            time.sleep(0.05)

        self.cmd_vel_event.clear()
        self.manual_mode_event.clear()
        self.manual_pitch_event.clear()
        self.shoot_motor_event.clear()
        self.shoot_once_event.clear()
        self.rotation_event.clear()
        self.ads_event.clear()
        self.wireless_pub.publish(msg)

        self.assertTrue(self.cmd_vel_event.wait(timeout=2.0), "Did not receive /cmd_vel")
        self.assertTrue(self.manual_mode_event.wait(timeout=2.0), "Did not receive /manual_mode")
        self.assertTrue(self.manual_pitch_event.wait(timeout=2.0), "Did not receive /manual_pitch")
        self.assertTrue(self.shoot_motor_event.wait(timeout=2.0), "Did not receive /shoot_motor")
        self.assertTrue(self.shoot_once_event.wait(timeout=2.0), "Did not receive /left/shoot_once")
        self.assertTrue(self.rotation_event.wait(timeout=2.0), "Did not receive /rotation")
        self.assertTrue(self.ads_event.wait(timeout=2.0), "Did not receive /ads")

        self.assertIsNotNone(self.cmd_vel_msg)
        self.assertEqual(self.cmd_vel_msg.linear.x, 0.5)
        self.assertEqual(self.cmd_vel_msg.linear.y, 0.5)
        self.assertAlmostEqual(self.cmd_vel_msg.angular.z, 1.0, places=4)

        self.assertIsNotNone(self.manual_mode_msg)
        self.assertTrue(self.manual_mode_msg.data)

        self.assertIsNotNone(self.manual_pitch_msg)
        self.assertAlmostEqual(self.manual_pitch_msg.data, 64.0 / 127.0, places=4)

        self.assertIsNotNone(self.shoot_motor_msg)
        self.assertTrue(self.shoot_motor_msg.data)

        self.assertIsNotNone(self.shoot_once_msg)
        self.assertTrue(self.shoot_once_msg.data)

        self.assertIsNotNone(self.rotation_msg)
        self.assertTrue(self.rotation_msg.data)

        self.assertIsNotNone(self.ads_msg)
        self.assertTrue(self.ads_msg.data)

    def test_manual_mode_rotation_follows_key_rotation(self):
        msg = UInt8MultiArray()
        # values[4]: b1 rotation=0
        msg.data = [0, 0, 0, 0b00000000, 0b00000000, 0, 0]

        deadline = time.time() + 2.0
        while self.wireless_pub.get_subscription_count() == 0 and time.time() < deadline:
            time.sleep(0.05)

        self.rotation_event.clear()
        self.wireless_pub.publish(msg)

        self.assertTrue(self.rotation_event.wait(timeout=2.0), "Did not receive /rotation")
        self.assertIsNotNone(self.rotation_msg)
        self.assertFalse(self.rotation_msg.data)

    def test_auto_mode_suppresses_non_manual_and_non_test_topics(self):
        msg = UInt8MultiArray()
        # ui_auto_flag=1 (values[3] b1)
        msg.data = [0b11111010, 127, 64, 0b00000010, 0, 0, 0]

        deadline = time.time() + 2.0
        while self.wireless_pub.get_subscription_count() == 0 and time.time() < deadline:
            time.sleep(0.05)

        self.cmd_vel_event.clear()
        self.manual_mode_event.clear()
        self.manual_pitch_event.clear()
        self.shoot_motor_event.clear()
        self.shoot_once_event.clear()
        self.rotation_event.clear()
        self.ads_event.clear()
        self.reloading_event.clear()
        self.wireless_pub.publish(msg)

        self.assertTrue(self.manual_mode_event.wait(timeout=2.0), "Did not receive /manual_mode")
        self.assertIsNotNone(self.manual_mode_msg)
        self.assertFalse(self.manual_mode_msg.data)

        self.assertFalse(self.cmd_vel_event.wait(timeout=0.5), "Unexpected /cmd_vel in auto mode")
        self.assertFalse(self.manual_pitch_event.wait(timeout=0.5), "Unexpected /manual_pitch in auto mode")
        self.assertFalse(self.shoot_motor_event.wait(timeout=0.5), "Unexpected /shoot_motor in auto mode")
        self.assertFalse(self.shoot_once_event.wait(timeout=0.5), "Unexpected /left/shoot_once in auto mode")
        self.assertFalse(self.rotation_event.wait(timeout=0.5), "Unexpected /rotation in auto mode")
        self.assertFalse(self.ads_event.wait(timeout=0.5), "Unexpected /ads in auto mode")
        self.assertFalse(self.reloading_event.wait(timeout=0.5), "Unexpected /reloading in auto mode")

    def test_reloading_publishes_only_on_rising_edge(self):
        msg_low = UInt8MultiArray()
        msg_low.data = [0b00000000, 0, 0, 0b00000000, 0, 0, 0]
        msg_high = UInt8MultiArray()
        # key_reload=1 (values[0] b5), ui_auto_flag=0
        msg_high.data = [0b00100000, 0, 0, 0b00000000, 0, 0, 0]

        deadline = time.time() + 2.0
        while self.wireless_pub.get_subscription_count() == 0 and time.time() < deadline:
            time.sleep(0.05)

        self.reloading_event.clear()
        self.wireless_pub.publish(msg_low)
        time.sleep(0.1)
        self.wireless_pub.publish(msg_high)

        self.assertTrue(self.reloading_event.wait(timeout=2.0), "Did not receive /reloading on rising edge")
        self.assertIsNotNone(self.reloading_msg)
        self.assertTrue(self.reloading_msg.data)

        self.reloading_event.clear()
        self.wireless_pub.publish(msg_high)
        self.assertFalse(self.reloading_event.wait(timeout=0.5), "Unexpected /reloading without rising edge")

    def test_ui_auto_flag_toggle_for_log_generation(self):
        msg_manual = UInt8MultiArray()
        msg_manual.data = [0, 0, 0, 0b00000000, 0, 0, 0]
        msg_auto = UInt8MultiArray()
        msg_auto.data = [0, 0, 0, 0b00000010, 0, 0, 0]

        deadline = time.time() + 2.0
        while self.wireless_pub.get_subscription_count() == 0 and time.time() < deadline:
            time.sleep(0.05)

        self.wireless_pub.publish(msg_manual)
        time.sleep(0.05)
        self.wireless_pub.publish(msg_auto)
        time.sleep(0.05)
        self.wireless_pub.publish(msg_manual)

    def test_hazard_status_follows_space_emergency_bit(self):
        msg_off = UInt8MultiArray()
        msg_off.data = [0b00000000, 0, 0, 0b00000000, 0, 0, 0]
        msg_on = UInt8MultiArray()
        msg_on.data = [0b00000001, 0, 0, 0b00000000, 0, 0, 0]

        deadline = time.time() + 2.0
        while self.wireless_pub.get_subscription_count() == 0 and time.time() < deadline:
            time.sleep(0.05)

        self.hazard_status_event.clear()
        self.wireless_pub.publish(msg_off)
        self.assertTrue(self.hazard_status_event.wait(timeout=2.0), "Did not receive /system/emergency/hazard_status")
        self.assertIsNotNone(self.hazard_status_msg)
        self.assertFalse(self.hazard_status_msg.data)

        self.hazard_status_event.clear()
        self.wireless_pub.publish(msg_on)
        self.assertTrue(self.hazard_status_event.wait(timeout=2.0), "Did not receive /system/emergency/hazard_status")
        self.assertIsNotNone(self.hazard_status_msg)
        self.assertTrue(self.hazard_status_msg.data)

@launch_testing.post_shutdown_test()
class TestWirelessParserNodeAfterShutdown(unittest.TestCase):
    def test_ui_auto_flag_transition_logs_present(self, proc_output, wireless_parser_node):
        launch_testing.asserts.assertInStderr(
            proc_output, "ui_auto_flag changed: false -> true", wireless_parser_node
        )
        launch_testing.asserts.assertInStderr(
            proc_output, "ui_auto_flag changed: true -> false", wireless_parser_node
        )

    def test_process_exited(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
