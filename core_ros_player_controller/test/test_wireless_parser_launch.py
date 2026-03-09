import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import UInt8MultiArray


@pytest.mark.launch_test
def generate_test_description():
    wireless_parser_node = launch_ros.actions.Node(
        package="core_ros_player_controller",
        executable="wireless_parser_node",
        name="wireless_parser_node",
        output="screen",
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

        self.node.create_subscription(Bool, "/shoot_motor", lambda _msg: None, 10)
        self.node.create_subscription(Bool, "/left/shoot_once", lambda _msg: None, 10)

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

    def test_template_for_more_checks(self):
        msg = UInt8MultiArray()
        msg.data = [0b00001010, 127, 64, 0b00000000, 0, 0, 0]

        deadline = time.time() + 2.0
        while self.wireless_pub.get_subscription_count() == 0 and time.time() < deadline:
            time.sleep(0.05)

        self.cmd_vel_event.clear()
        self.manual_mode_event.clear()
        self.manual_pitch_event.clear()
        self.wireless_pub.publish(msg)

        self.assertTrue(self.cmd_vel_event.wait(timeout=2.0), "Did not receive /cmd_vel")
        self.assertTrue(self.manual_mode_event.wait(timeout=2.0), "Did not receive /manual_mode")
        self.assertTrue(self.manual_pitch_event.wait(timeout=2.0), "Did not receive /manual_pitch")

        self.assertIsNotNone(self.cmd_vel_msg)
        self.assertEqual(self.cmd_vel_msg.linear.x, 1.0)
        self.assertEqual(self.cmd_vel_msg.linear.y, 1.0)
        self.assertAlmostEqual(self.cmd_vel_msg.angular.z, 1.0, places=4)

        self.assertIsNotNone(self.manual_mode_msg)
        self.assertTrue(self.manual_mode_msg.data)

        self.assertIsNotNone(self.manual_pitch_msg)
        self.assertAlmostEqual(self.manual_pitch_msg.data, 64.0 / 127.0, places=4)


@launch_testing.post_shutdown_test()
class TestWirelessParserNodeAfterShutdown(unittest.TestCase):
    def test_process_exited(self, proc_info, wireless_parser_node):
        proc_info.assertWaitForShutdown(process=wireless_parser_node, timeout=5.0)
