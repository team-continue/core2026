"""Regression tests for health_monitor using an isolated diagnostic topic."""

import math
import os
import resource
import signal
import subprocess
import time
import uuid

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState


EXECUTABLE = os.environ['HEALTH_MONITOR_EXECUTABLE']


def disable_core_dumps():
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def command(*parameters, namespace):
    args = [
        EXECUTABLE, '--ros-args', '-r', f'__ns:={namespace}',
        '--disable-external-lib-logs', '--disable-rosout-logs',
    ]
    for parameter in parameters:
        args.extend(['-p', parameter])
    return args


@pytest.mark.parametrize('period_ms', [0, -1])
def test_rejects_nonpositive_publish_period(period_ms):
    namespace = '/health_test_' + uuid.uuid4().hex
    process = subprocess.Popen(
        command(f'publish_period_ms:={period_ms}', namespace=namespace),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, preexec_fn=disable_core_dumps,
    )
    try:
        output, _ = process.communicate(timeout=5)
        assert process.returncode != 0
        assert 'publish_period_ms must be > 0' in output
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()


@pytest.fixture(scope='module')
def battery_monitor():
    namespace = '/health_test_' + uuid.uuid4().hex
    context = Context()
    rclpy.init(
        args=[
            '--ros-args', '--disable-external-lib-logs',
            '--disable-rosout-logs',
        ],
        context=context,
    )
    node = rclpy.create_node(
        'battery_probe', namespace=namespace, context=context,
        enable_rosout=False,
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    received = []
    subscription = node.create_subscription(
        DiagnosticArray, namespace + '/diagnostics', received.append, 10,
    )
    publisher = node.create_publisher(
        BatteryState, namespace + '/battery', qos_profile_sensor_data,
    )
    process = subprocess.Popen(
        command(
            'publish_period_ms:=20', f'battery.topic:={namespace}/battery',
            namespace=namespace,
        ),
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=disable_core_dumps,
    )
    try:
        deadline = time.monotonic() + 5
        while publisher.get_subscription_count() == 0:
            assert process.poll() is None, 'health_monitor exited'
            assert time.monotonic() < deadline, 'battery discovery timed out'
            executor.spin_once(timeout_sec=0.02)
        yield executor, publisher, received, process
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        executor.shutdown()
        node.destroy_subscription(subscription)
        node.destroy_node()
        context.shutdown()


@pytest.mark.parametrize(
    'voltage, expected_level, expected_message',
    [
        (24.0, 0, 'OK'),
        (22.0, 1, 'voltage low'),
        (20.0, 2, 'voltage critical'),
        (math.nan, 3, 'invalid voltage'),
        (math.inf, 3, 'invalid voltage'),
        (-math.inf, 3, 'invalid voltage'),
        (24.0, 0, 'OK'),  # Recover after an invalid measurement.
    ],
)
def test_battery_diagnostics(
    battery_monitor, voltage, expected_level, expected_message,
):
    executor, publisher, received, process = battery_monitor
    received.clear()
    message = BatteryState()
    message.present = True
    message.voltage = voltage
    deadline = time.monotonic() + 5
    last_status = None
    while time.monotonic() < deadline:
        assert process.poll() is None, 'health_monitor exited'
        publisher.publish(message)
        executor.spin_once(timeout_sec=0.02)
        while received:
            for status in received.pop(0).status:
                if status.name != 'power/battery':
                    continue
                values = {entry.key: entry.value for entry in status.values}
                if 'voltage' not in values:
                    continue
                measured = float(values['voltage'])
                same_voltage = (
                    math.isnan(measured) if math.isnan(voltage)
                    else measured == voltage
                )
                if not same_voltage:
                    continue
                level = (status.level[0] if isinstance(status.level, bytes)
                         else status.level)
                last_status = (level, status.message)
                if last_status == (expected_level, expected_message):
                    return
    pytest.fail(
        f'voltage={voltage}: expected {(expected_level, expected_message)}, '
        f'last diagnostic was {last_status}',
    )
