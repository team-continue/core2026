import math
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from core_msgs.msg import CANArray
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64, Int32


@pytest.mark.launch_test
def generate_test_description():
    body_node = launch_ros.actions.Node(
        package="core_body_controller",
        executable="body_control_node",
        name="body_control_node_test",
        output="screen",
        parameters=[
            {
                "acceleration": 100.0,
                "rotation_acceleration": 100.0,
                "cmd_vel_timeout_sec": 0.15,
            }
        ],
        remappings=[
            ("cmd_vel", "/body_test/cmd_vel"),
            ("joint_states", "/body_test/joint_states"),
            ("body_omega", "/body_test/body_omega"),
            ("can/tx", "/body_test/can_tx"),
            ("/rotation", "/body_test/rotation"),
            ("/system/emergency/hazard_status", "/body_test/hazard_status"),
        ],
    )
    target_node = launch_ros.actions.Node(
        package="core_body_controller",
        executable="target_angle_node",
        name="target_angle_node_test",
        output="screen",
        parameters=[
            {
                "yaw_rotation_velocity": 1.0,
                "yaw_rotation_acceleration": 1000.0,
                "cmd_vel_timeout_sec": 0.15,
                "imu_timeout_sec": 0.15,
                "body_omega_timeout_sec": 0.15,
            }
        ],
        remappings=[
            ("cmd_vel", "/target_test/cmd_vel"),
            ("imu", "/target_test/imu"),
            ("yaw_target_angle", "/target_test/yaw_target_angle"),
            ("body_omega", "/target_test/body_omega"),
            ("target_omega", "/target_test/target_omega"),
            ("can/tx", "/target_test/can_tx"),
            ("/rotation", "/target_test/rotation"),
            ("/system/emergency/hazard_status", "/target_test/hazard_status"),
        ],
    )
    return launch.LaunchDescription(
        [body_node, target_node, launch_testing.actions.ReadyToTest()]
    ), {"body_node": body_node, "target_node": target_node}


class TestBodyControllerNodes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("body_controller_integration_test")
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.stop = False
        self.lock = threading.Lock()
        self.can_messages = {"body": [], "target": []}
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

        self.body_cmd_pub = self.node.create_publisher(Twist, "/body_test/cmd_vel", 10)
        self.body_joint_pub = self.node.create_publisher(
            JointState, "/body_test/joint_states", 10
        )
        self.body_hazard_pub = self.node.create_publisher(
            Bool, "/body_test/hazard_status", 10
        )
        self.target_cmd_pub = self.node.create_publisher(
            Twist, "/target_test/cmd_vel", 10
        )
        self.target_imu_pub = self.node.create_publisher(Imu, "/target_test/imu", 10)
        self.target_body_omega_pub = self.node.create_publisher(
            Float64, "/target_test/body_omega", 10
        )
        self.target_rotation_pub = self.node.create_publisher(
            Int32, "/target_test/rotation", 10
        )
        self.target_hazard_pub = self.node.create_publisher(
            Bool, "/target_test/hazard_status", 10
        )
        self.node.create_subscription(
            CANArray,
            "/body_test/can_tx",
            lambda msg: self._record_can("body", msg),
            10,
        )
        self.node.create_subscription(
            CANArray,
            "/target_test/can_tx",
            lambda msg: self._record_can("target", msg),
            10,
        )

    def tearDown(self):
        self.stop = True
        self.thread.join(timeout=1.0)
        self.executor.remove_node(self.node)
        self.node.destroy_node()

    def _spin(self):
        while not self.stop:
            self.executor.spin_once(timeout_sec=0.05)

    def _record_can(self, source, msg):
        with self.lock:
            self.can_messages[source].append(msg)
            self.can_messages[source] = self.can_messages[source][-200:]

    def _clear_can(self, source):
        with self.lock:
            self.can_messages[source].clear()

    def _wait_for_can(self, source, predicate, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                messages = list(self.can_messages[source])
            for msg in reversed(messages):
                if predicate(msg):
                    return msg
            time.sleep(0.01)
        self.fail(f"timed out waiting for matching {source} CAN message")

    def _wait_for_can_count(self, source, predicate, count=3, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                messages = list(self.can_messages[source])
            if sum(1 for msg in messages if predicate(msg)) >= count:
                return
            time.sleep(0.01)
        self.fail(f"timed out waiting for {count} matching {source} CAN messages")

    @staticmethod
    def _target_value(msg):
        if len(msg.array) != 1 or msg.array[0].id != 4 or len(msg.array[0].data) < 2:
            return None
        return msg.array[0].data[1]

    @staticmethod
    def _body_values(msg):
        if len(msg.array) != 4 or any(len(frame.data) < 2 for frame in msg.array):
            return None
        return [frame.data[1] for frame in msg.array]

    @staticmethod
    def _publish_repeatedly(publisher, msg, count=3):
        for _ in range(count):
            publisher.publish(msg)
            time.sleep(0.02)

    def _publish_fresh_imu(self, count=3, yaw=0.0):
        for _ in range(count):
            msg = Imu()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.orientation.z = math.sin(yaw * 0.5)
            msg.orientation.w = math.cos(yaw * 0.5)
            self.target_imu_pub.publish(msg)
            time.sleep(0.02)

    def test_runtime_contracts_and_fail_safe_behavior(self):
        self._wait_for_can_count(
            "target", lambda msg: self._target_value(msg) == 0.0
        )

        parameter_client = self.node.create_client(
            GetParameters, "/target_angle_node_test/get_parameters"
        )
        self.assertTrue(parameter_client.wait_for_service(timeout_sec=2.0))
        request = GetParameters.Request()
        request.names = [
            "yaw_rotation_velocity",
            "yaw_rotation_acceleration",
            "body_omega_timeout_sec",
        ]
        future = parameter_client.call_async(request)
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(future.done())
        values = future.result().values
        self.assertAlmostEqual(values[0].double_value, 1.0)
        self.assertAlmostEqual(values[1].double_value, 1000.0)
        self.assertAlmostEqual(values[2].double_value, 0.15)

        false_msg = Bool(data=False)
        zero_twist = Twist()
        self._publish_repeatedly(self.target_hazard_pub, false_msg)
        self._publish_repeatedly(self.target_cmd_pub, zero_twist)
        self._publish_repeatedly(self.target_body_omega_pub, Float64(data=0.5))

        for mode, expected in ((0, 0.0), (1, 0.5), (2, 0.5)):
            self._clear_can("target")
            self._publish_repeatedly(self.target_rotation_pub, Int32(data=mode))
            self._publish_repeatedly(
                self.target_body_omega_pub, Float64(data=0.5)
            )
            self._publish_fresh_imu()
            self._publish_repeatedly(self.target_cmd_pub, zero_twist)
            self._wait_for_can(
                "target",
                lambda msg, value=expected: (
                    self._target_value(msg) is not None
                    and math.isclose(self._target_value(msg), value, abs_tol=0.08)
                ),
            )

        self._clear_can("target")
        self._publish_repeatedly(self.target_rotation_pub, Int32(data=0))
        self._publish_fresh_imu(yaw=0.3)
        tracked = self._wait_for_can(
            "target",
            lambda msg: self._target_value(msg) is not None
            and math.isclose(self._target_value(msg), 0.6, abs_tol=0.1),
        )
        self.assertGreater(self._target_value(tracked), 0.0)
        self._publish_fresh_imu(yaw=0.0)

        self._clear_can("target")
        self._publish_repeatedly(self.target_body_omega_pub, Float64(data=5.0))
        self._publish_repeatedly(self.target_rotation_pub, Int32(data=1))
        self._publish_fresh_imu()
        limited = self._wait_for_can(
            "target",
            lambda msg: self._target_value(msg) is not None
            and abs(self._target_value(msg)) > 0.9,
        )
        self.assertLessEqual(abs(self._target_value(limited)), 1.0 + 1e-6)

        deadline = time.monotonic() + 0.22
        while time.monotonic() < deadline:
            self.target_cmd_pub.publish(zero_twist)
            imu_msg = Imu()
            imu_msg.header.stamp = self.node.get_clock().now().to_msg()
            self.target_imu_pub.publish(imu_msg)
            time.sleep(0.02)

        self._clear_can("target")
        for _ in range(5):
            self.target_cmd_pub.publish(zero_twist)
            imu_msg = Imu()
            imu_msg.header.stamp = self.node.get_clock().now().to_msg()
            self.target_imu_pub.publish(imu_msg)
            time.sleep(0.02)
        self._wait_for_can_count(
            "target",
            lambda msg: self._target_value(msg) == 0.0,
            timeout=0.1,
        )

        self._clear_can("target")
        time.sleep(0.2)
        self._wait_for_can_count(
            "target", lambda msg: self._target_value(msg) == 0.0
        )

        self._publish_repeatedly(self.target_cmd_pub, zero_twist)
        self._clear_can("target")
        resumed_imu = Imu()
        resumed_imu.header.stamp = self.node.get_clock().now().to_msg()
        resumed_imu.orientation.z = math.sin(1.0)
        resumed_imu.orientation.w = math.cos(1.0)
        self.target_imu_pub.publish(resumed_imu)
        self._wait_for_can_count(
            "target",
            lambda msg: self._target_value(msg) == 0.0,
            timeout=0.1,
        )

        self._publish_repeatedly(self.body_hazard_pub, false_msg)
        short_joint_state = JointState()
        short_joint_state.position = [0.0]
        self._publish_repeatedly(self.body_joint_pub, short_joint_state)
        nan_joint_state = JointState()
        nan_joint_state.position = [0.0, 0.0, 0.0, 0.0, float("nan")]
        self._publish_repeatedly(self.body_joint_pub, nan_joint_state)

        body_twist = Twist()
        body_twist.linear.x = 0.2
        self._clear_can("body")
        self._publish_repeatedly(self.body_cmd_pub, body_twist)
        self._wait_for_can(
            "body",
            lambda msg: self._body_values(msg) is not None
            and any(abs(value) > 0.01 for value in self._body_values(msg)),
        )

        self._clear_can("body")
        time.sleep(0.2)
        self._wait_for_can(
            "body",
            lambda msg: self._body_values(msg) is not None
            and all(abs(value) < 1e-6 for value in self._body_values(msg)),
        )

        self._publish_fresh_imu()
        self._publish_repeatedly(self.target_hazard_pub, Bool(data=True))
        self._clear_can("target")
        self._wait_for_can_count(
            "target", lambda msg: self._target_value(msg) == 0.0
        )


@launch_testing.post_shutdown_test()
class TestBodyControllerNodesAfterShutdown(unittest.TestCase):
    def test_processes_exited(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
