"""確認スクリプト共通のデータ型、記録ノード、走行の補助."""

from dataclasses import dataclass
import math
import os
from pathlib import Path
import threading
import time
from typing import Callable, Final, Sequence

from geometry_msgs.msg import Twist
from core_nav_launch.composition import CONTAINER_OF_EXECUTABLE
from nav_msgs.msg import Odometry
import numpy as np
from numpy.typing import NDArray
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

FloatArray = NDArray[np.float64]

TRUTH_TOPIC: Final = '/ground_truth/odom'
# ナビゲーションを動かすときは /cmd_vel_nav などを指定できるよう、既定値だけを持つ
CMD_TOPIC: Final = '/control/cmd_vel'

# 機体の加速度の上限（5m/s2）より少し小さくして、目標の手前で確実に減速する
BRAKE_ACCEL_MPS2: Final = 4.0
ARRIVAL_RADIUS_M: Final = 0.03
COMMAND_PERIOD_S: Final = 0.02
WALL_TIMEOUT_S: Final = 120.0


class CheckError(RuntimeError):
    """確認の続行に必要なデータが得られないときの例外."""


@dataclass(frozen=True)
class Pose:
    t: float
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class ProcessUsage:
    wall_s: float
    cpu_s: float
    rss_mib: float


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def stamp_to_sec(stamp) -> float:  # noqa: ANN001 builtin_interfaces/Time
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def to_pose(msg: Odometry) -> Pose:
    q = msg.pose.pose.orientation
    p = msg.pose.pose.position
    return Pose(
        t=stamp_to_sec(msg.header.stamp),
        x=p.x,
        y=p.y,
        z=p.z,
        yaw=math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
    )


def find_process(name: str) -> int | None:
    """同じ ROS_DOMAIN_ID で動く、name のプロセスの PID を返す.

    コマンドの先頭の引数（実行ファイル）に name を含むか、ノード名を name にした（__node:=name）プロセスを探す。
    composition:=true のコンテナは、ノード名（core_nav_launch/composition.py）で見つかる。
    """
    domain = os.environ.get('ROS_DOMAIN_ID', '').encode()
    node_argument = f'__node:={name}'
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            arguments = [
                item.decode(errors='replace')
                for item in (entry / 'cmdline').read_bytes().split(b'\0')
            ]
            environment = (entry / 'environ').read_bytes().split(b'\0')
        except OSError:
            continue
        if name not in arguments[0] and node_argument not in arguments[1:]:
            continue
        variables = dict(item.split(b'=', 1) for item in environment if b'=' in item)
        if variables.get(b'ROS_DOMAIN_ID', b'') == domain:
            return int(entry.name)
    return None


def find_stack_processes(names: Sequence[str]) -> dict[str, int]:
    """ノード（別のプロセスで動かすときの実行ファイル名）のプロセスを、測る単位の名前と PID で返す.

    composition:=true でコンテナに入ったノードは、コンテナのプロセスで測る。同じコンテナのノードは 1 つにまとまる。
    見つからないノードは含めない。
    """
    found: dict[str, int] = {}
    for name in names:
        label = name
        pid = find_process(name)
        if pid is None and name in CONTAINER_OF_EXECUTABLE:
            label = CONTAINER_OF_EXECUTABLE[name]
            pid = find_process(label)
        if pid is not None:
            found[label] = pid
    return found


def missing_processes(names: Sequence[str], found: dict[str, int]) -> list[str]:
    """find_stack_processes で、ノードのプロセスもコンテナも見つからなかったノード."""
    return [
        name for name in names
        if name not in found and CONTAINER_OF_EXECUTABLE.get(name) not in found
    ]


def read_usage(pid: int) -> ProcessUsage:
    """プロセスの累積 CPU 時間と使用メモリを読む."""
    try:
        # comm に空白が含まれても崩れないよう、閉じ括弧より後ろを使う
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        status = Path(f'/proc/{pid}/status').read_text()
    except OSError as error:
        raise CheckError(f'プロセス {pid} の情報を読めません: {error}') from error
    ticks = float(fields[11]) + float(fields[12])  # utime, stime
    rss_kib = next(float(line.split()[1]) for line in status.splitlines() if line.startswith('VmRSS'))
    return ProcessUsage(time.monotonic(), ticks / os.sysconf('SC_CLK_TCK'), rss_kib / 1024.0)


def cpu_percent(start: ProcessUsage, end: ProcessUsage) -> float:
    elapsed = end.wall_s - start.wall_s
    return 100.0 * (end.cpu_s - start.cpu_s) / elapsed if elapsed > 0.0 else 0.0


class DriveNode(Node):
    """真値を記録し、真値を使って機体を動かすノード.

    確認スクリプトの走行に使う。ナビゲーションの出力は使わない。
    """

    def __init__(self, name: str, cmd_topic: str = CMD_TOPIC) -> None:
        super().__init__(name)
        self.truth: list[Pose] = []
        self._cmd = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(
            Odometry, TRUTH_TOPIC, self._on_truth,
            QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

    def _on_truth(self, msg: Odometry) -> None:
        self.truth.append(to_pose(msg))

    def now_sim(self) -> float:
        if not self.truth:
            raise CheckError(f'{TRUTH_TOPIC} を受信していません')
        return self.truth[-1].t

    def wait_for_truth(self) -> None:
        wall_start = time.monotonic()
        while not self.truth:
            if time.monotonic() - wall_start > WALL_TIMEOUT_S:
                raise CheckError(f'{WALL_TIMEOUT_S}秒待っても {TRUTH_TOPIC} を受信できません')
            time.sleep(0.1)

    def command_world(self, vx: float, vy: float, wz: float) -> None:
        """世界座標の速度を、現在の真値の向きで機体座標の指令に直して出す."""
        yaw = self.truth[-1].yaw
        msg = Twist()
        msg.linear.x = math.cos(yaw) * vx + math.sin(yaw) * vy
        msg.linear.y = -math.sin(yaw) * vx + math.cos(yaw) * vy
        msg.angular.z = wz
        self._cmd.publish(msg)

    def hold(self, vx: float, vy: float, wz: float, duration_s: float) -> None:
        start = self.now_sim()
        wall_start = time.monotonic()
        while self.now_sim() - start < duration_s:
            if time.monotonic() - wall_start > WALL_TIMEOUT_S:
                raise CheckError(f'{duration_s}秒待つ間にシミュレーション時間が進みませんでした')
            self.command_world(vx, vy, wz)
            time.sleep(COMMAND_PERIOD_S)

    def go_to(self, x: float, y: float, speed: float, wz: float = 0.0) -> None:
        """目標まで走り、手前で減速して止まる."""
        wall_start = time.monotonic()
        while True:
            now = self.truth[-1]
            dx, dy = x - now.x, y - now.y
            distance = math.hypot(dx, dy)
            if distance < ARRIVAL_RADIUS_M:
                return
            if time.monotonic() - wall_start > WALL_TIMEOUT_S:
                raise CheckError(f'目標 ({x}, {y}) に到達できません（現在 ({now.x:.2f}, {now.y:.2f})）')
            v = min(speed, math.sqrt(2.0 * BRAKE_ACCEL_MPS2 * distance))
            self.command_world(v * dx / distance, v * dy / distance, wz)
            time.sleep(COMMAND_PERIOD_S)

    def stop(self, duration_s: float = 1.0) -> None:
        self.hold(0.0, 0.0, 0.0, duration_s)


def standard_phases(
    node: DriveNode, lap_seconds: float
) -> list[tuple[str, Callable[[], None]]]:
    """自己位置の評価に使う走行の区間を、名前と動きの組で返す."""

    def straight(speed: float) -> Callable[[], None]:
        def run() -> None:
            node.go_to(10.0, -2.5, speed)
            node.stop()
            node.go_to(-10.0, -2.5, speed)
            node.stop()
        return run

    def spin(rate: float) -> Callable[[], None]:
        # 2 回転して止まる
        return lambda: node.hold(0.0, 0.0, rate, 4.0 * math.pi / rate)

    def translate_and_spin() -> None:
        for x in (10.0, -10.0, 0.0):
            node.go_to(x, 0.0, 2.0, 1.0)
        node.stop()

    def stop_and_go() -> None:
        # 1 サイクルで約 3m 進むので、柵に当たらないよう端から始める
        node.go_to(-9.0, 0.0, 2.0)
        node.stop()
        for direction in (1.0, -1.0):
            for _ in range(6):
                node.hold(direction * 3.0, 0.0, 0.0, 1.0)
                node.stop(1.0)

    def fence_collision() -> None:
        # 実機でも起こりうる衝突の後に、自己位置が戻るかを見る。約 3.9 秒で柵に当たる
        node.go_to(0.0, 0.0, 2.0)
        node.stop()
        node.hold(3.0, 0.0, 0.0, 5.0)
        node.stop()
        node.go_to(0.0, 0.0, 2.0)
        node.stop()

    def laps() -> None:
        corners = ((10.0, -5.5), (10.0, 5.5), (-10.0, 5.5), (-10.0, -5.5))
        start = node.now_sim()
        while node.now_sim() - start < lap_seconds:
            for x, y in corners:
                node.go_to(x, y, 3.0, 0.3)
        node.stop()

    return [
        ('静止', lambda: node.stop(20.0)),
        ('直進 1m/s', straight(1.0)),
        ('直進 2m/s', straight(2.0)),
        ('直進 3m/s', straight(3.0)),
        ('その場旋回 1rad/s', spin(1.0)),
        ('その場旋回 2rad/s', spin(2.0)),
        ('並進と旋回の同時動作', translate_and_spin),
        ('急な加減速', stop_and_go),
        (COLLISION_PHASE, fence_collision),
        ('周回', laps),
        ('停止後', lambda: node.stop(5.0)),
    ]


COLLISION_PHASE: Final = '柵への衝突 3m/s'
# 機体の角までの距離（約 0.57m）を柵の位置から引いた範囲。ここを出たら柵に当たった可能性がある
SAFE_X_M: Final = 11.4
SAFE_Y_M: Final = 6.9


def left_safe_area(truth: Sequence[Pose], start: float, end: float) -> bool:
    return any(
        abs(p.x) > SAFE_X_M or abs(p.y) > SAFE_Y_M
        for p in truth
        if start <= p.t <= end
    )


def rms(values: FloatArray) -> float:
    return float(np.sqrt(np.mean(np.square(values))))


class SpinThread:
    """ノードを別スレッドで回し、終了時に確実に止める."""

    def __init__(self, node: Node) -> None:
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._node = node

    def __enter__(self) -> 'SpinThread':
        self._thread.start()
        return self

    def __exit__(self, *_exc: object) -> None:
        self._executor.shutdown()
        self._thread.join(timeout=5.0)
        self._node.destroy_node()


def run_with_node(node_factory: Callable[[], Node], body: Callable[[Node], bool]) -> int:
    """ノードを作って body を実行し、終了コードを返す."""
    rclpy.init()
    node = node_factory()
    ok = False
    with SpinThread(node):
        try:
            ok = body(node)
        except CheckError as error:
            print(f'[NG] 確認を中断しました: {error}', flush=True)
        finally:
            # 異常終了しても機体を走らせたままにしない。シグナルで止められたときは送れない
            if rclpy.ok() and isinstance(node, DriveNode) and node.truth:
                node.command_world(0.0, 0.0, 0.0)
                time.sleep(0.1)
    rclpy.try_shutdown()
    print('結果: 完了条件を満たしました' if ok else '結果: 完了条件を満たしていません', flush=True)
    return 0 if ok else 1
