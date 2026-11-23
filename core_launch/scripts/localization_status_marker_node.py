#!/usr/bin/env python3
"""自己位置の状態（/localization/health）を、RViz で機体に重ねて出す（visualization_msgs/MarkerArray）.

nav.rviz の Localization > Status が /localization/status_markers を表示する。機体（base_link）の床に
状態の色の円（OK 緑・DEGRADED 黄・FAILED 赤・届かない 灰）を、上に評価値などの文字を出す。
時刻 0（最新の TF）で出すので、自己位置の TF が止まっても、最後に分かった機体の位置に出し続ける。
"""

import math
import sys

from builtin_interfaces.msg import Time as TimeMsg
from core_msgs.msg import LocalizationHealth
from core_nav_launch.localization_status import HealthSample, status_label
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray


class LocalizationStatusMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__('localization_status_marker')
        self._frame_id = str(self.declare_parameter('frame_id', 'base_link').value)
        self._text_height_m = float(self.declare_parameter('text_height_m', 1.4).value)
        self._text_size_m = float(self.declare_parameter('text_size_m', 0.3).value)
        self._disc_radius_m = float(self.declare_parameter('disc_radius_m', 0.75).value)
        self._stale_after_s = float(self.declare_parameter('stale_after_s', 1.0).value)
        rate_hz = float(self.declare_parameter('rate_hz', 5.0).value)
        positives = {
            'rate_hz': rate_hz, 'stale_after_s': self._stale_after_s,
            'disc_radius_m': self._disc_radius_m, 'text_size_m': self._text_size_m,
        }
        invalid = {name: value for name, value in positives.items() if not value > 0.0}
        if invalid:
            raise ValueError(f'正の値にしてください: {invalid}')
        self._sample: HealthSample | None = None
        self._received: Time | None = None
        self._publisher = self.create_publisher(MarkerArray, '/localization/status_markers', 1)
        self.create_subscription(LocalizationHealth, '/localization/health', self._on_health, 10)
        self.create_timer(1.0 / rate_hz, self._publish)

    def _on_health(self, msg: LocalizationHealth) -> None:
        self._sample = HealthSample(
            level=int(msg.level), fitness=float(msg.fitness), inlier_ratio=float(msg.inlier_ratio),
            gyro_difference=float(msg.gyro_difference), time_since_match_s=float(msg.time_since_match_s),
        )
        self._received = self.get_clock().now()

    def _age_s(self) -> float:
        if self._received is None:
            return math.inf
        # シミュレーションをやり直して時刻が戻ったときは、受けたばかりとみなす
        return max(0.0, (self.get_clock().now() - self._received).nanoseconds * 1e-9)

    def _marker(self, marker_id: int, marker_type: int, rgba: tuple[float, float, float, float]) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._frame_id
        # 時刻 0 は「最新の TF」
        marker.header.stamp = TimeMsg()
        marker.ns = 'localization_status'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.frame_locked = True
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        return marker

    def _publish(self) -> None:
        label = status_label(self._sample, self._age_s(), self._stale_after_s)
        red, green, blue, _ = label.rgba
        disc = self._marker(0, Marker.CYLINDER, (red, green, blue, 0.35))
        disc.scale.x = disc.scale.y = 2.0 * self._disc_radius_m
        disc.scale.z = 0.02
        text = self._marker(1, Marker.TEXT_VIEW_FACING, label.rgba)
        text.pose.position.z = self._text_height_m
        text.scale.z = self._text_size_m
        text.text = label.text
        self._publisher.publish(MarkerArray(markers=[disc, text]))


def main() -> int:
    rclpy.init()
    try:
        node = LocalizationStatusMarkerNode()
    except ValueError as error:
        print(f'[localization_status_marker] {error}', file=sys.stderr, flush=True)
        rclpy.try_shutdown()
        return 1
    try:
        rclpy.spin(node)
    # Ctrl+C（launch の SIGINT）や SIGTERM では、rclpy が止めて ExternalShutdownException になる
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
