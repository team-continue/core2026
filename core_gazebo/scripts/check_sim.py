#!/usr/bin/env python3
"""シミュレーションの完了条件（手順1）を確認する.

sim.launch.py を起動した状態で実行する。全項目が合格なら終了コード 0、
不合格があれば 1 を返す。ロボットを動かすので、初期位置の周囲に 3m の空きが必要。
"""

import argparse
from collections import deque
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import threading
import time
from typing import Callable, Final, Sequence
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from numpy.typing import NDArray
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2

FloatArray = NDArray[np.float64]

ODOM_TOPIC: Final = '/ground_truth/odom'
CLOUD_TOPIC: Final = '/sensing/sim/points'
CMD_TOPIC: Final = '/control/cmd_vel'
IMU_TOPICS: Final = {'/sensing/livox/imu': 'livox_frame', '/sensing/imu': 'damiao_imu_link'}

GRAVITY_MPS2: Final = 9.81
EXPECTED_CLOUD_POINTS: Final = 720 * 112
# 距離ノイズ 2cm（1σ）に対して十分広い値
SURFACE_TOLERANCE_M: Final = 0.1
MIN_ON_SURFACE_RATIO: Final = 0.99
BODY_LENGTH_M: Final = 0.8
# 機体の箱（core2026.urdf.xacro の見た目の形。base_link は床に投影した機体中心）
BODY_HALF_M: Final = 0.4
BODY_HEIGHT_M: Final = 0.8
# 自機の除去の箱（footprint.yaml の機体の形と self_filter_margin_m 0.05m。FAST-LIO と検出と同じ判定）
SELF_HALF_M: Final = BODY_HALF_M + 0.05
SELF_TOP_M: Final = BODY_HEIGHT_M + 0.05
# robot.gazebo.xacro の gpu_lidar の格子と距離の範囲
LIDAR_AZIMUTH_SAMPLES: Final = 720
LIDAR_ELEVATION_MIN_DEG: Final = -7.0
LIDAR_ELEVATION_MAX_DEG: Final = 52.0
LIDAR_ELEVATION_SAMPLES: Final = 112
LIDAR_MIN_RANGE_M: Final = 0.1
# 機体でさえぎられる向きの幅 [度] の、形状から求めた値との差の許容（1 度ごとの区切りの端の分）
OCCLUSION_TOLERANCE_DEG: Final = 4.0
RETURN_S: Final = 5.0
RETURN_GAIN_PER_S: Final = 2.0
MIN_HITS_PER_SURFACE: Final = 20
MIN_VALID_RATIO: Final = 0.9
# robot.gazebo.xacro の加減速度の上限
MAX_LINEAR_ACCEL_MPS2: Final = 5.0
MAX_ANGULAR_ACCEL_RADPS2: Final = 10.0
# 指令の遅れを評価から除く時間
COMMAND_DELAY_S: Final = 0.1
# 3m/s から止まるのに 0.6 秒かかる
STOP_S: Final = 1.0
WALL_TIMEOUT_S: Final = 60.0


class CheckError(RuntimeError):
    """確認の続行に必要なデータが得られないときの例外."""


@dataclass(frozen=True)
class CheckResult:
    name: str
    ok: bool
    detail: str


@dataclass(frozen=True)
class Box:
    name: str
    center: FloatArray
    yaw: float
    half: FloatArray


@dataclass(frozen=True)
class Cylinder:
    name: str
    center: FloatArray
    radius: float
    half_height: float


@dataclass(frozen=True)
class Plane:
    name: str


Surface = Box | Cylinder | Plane


@dataclass(frozen=True)
class Mount:
    """base_link での LiDAR の取り付け位置（回転なし）."""

    x: float
    height: float

    def to_base(self, points_lidar: FloatArray) -> FloatArray:
        return points_lidar + np.array([self.x, 0.0, self.height])


@dataclass(frozen=True)
class BodyOcclusion:
    """形状から求めた、LiDAR の格子のうち機体に当たる光線."""

    # 機体に当たり、最短距離より近いので点にならない光線の数
    too_close: int
    # 機体に当たり、自機の点になる光線の数
    self_hits: int
    # どの仰角でも機体に当たる方位の幅 [度]
    blocked_deg: float
    # 1 度ごとの方位の区切りのうち、すべての光線が機体に当たるもの（-180〜179 度）
    blocked_bins: frozenset[int]


@dataclass(frozen=True)
class OdomSample:
    t: float
    x: float
    y: float
    z: float
    yaw: float
    tilt: float


@dataclass(frozen=True)
class ImuSample:
    t: float
    frame_id: str
    gyro: tuple[float, float, float]
    accel: tuple[float, float, float]


@dataclass(frozen=True)
class Motion:
    name: str
    vx: float
    vy: float
    wz: float
    duration_s: float


def stamp_to_sec(sec: int, nanosec: int) -> float:
    return float(sec) + float(nanosec) * 1e-9


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def rotate_z(points: FloatArray, yaw: float) -> FloatArray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((
        c * points[:, 0] - s * points[:, 1],
        s * points[:, 0] + c * points[:, 1],
        points[:, 2],
    ))


def load_surfaces(world_path: Path) -> list[Surface]:
    """ワールドの静的モデルから、箱・円柱・平面の形状を読み出す."""
    models = ET.parse(world_path).getroot().findall('./world/model')

    def parse(model: ET.Element) -> Surface | None:
        name = model.attrib['name']
        pose_text = model.findtext('pose', default='0 0 0 0 0 0')
        pose = np.array([float(v) for v in pose_text.split()], dtype=np.float64)
        geometry = model.find('./link/collision/geometry')
        if geometry is None:
            raise CheckError(f'{world_path}: モデル {name} に衝突形状がありません')
        box = geometry.find('box')
        cylinder = geometry.find('cylinder')
        if geometry.find('plane') is not None:
            return Plane(name)
        if box is not None:
            size = np.array([float(v) for v in box.findtext('size', '').split()], dtype=np.float64)
            return Box(name, pose[:3], float(pose[5]), size / 2.0)
        if cylinder is not None:
            return Cylinder(
                name,
                pose[:3],
                float(cylinder.findtext('radius', '')),
                float(cylinder.findtext('length', '')) / 2.0,
            )
        return None

    return [surface for surface in map(parse, models) if surface is not None]


def surface_distance(surface: Surface, points: FloatArray) -> FloatArray:
    """各点から形状の表面までの距離を返す."""
    if isinstance(surface, Plane):
        return np.abs(points[:, 2])
    if isinstance(surface, Box):
        local = rotate_z(points - surface.center, -surface.yaw)
        q = np.abs(local) - surface.half
    else:
        local = points - surface.center
        q = np.column_stack((
            np.hypot(local[:, 0], local[:, 1]) - surface.radius,
            np.abs(local[:, 2]) - surface.half_height,
        ))
    outside = np.linalg.norm(np.maximum(q, 0.0), axis=1)
    inside = np.minimum(np.max(q, axis=1), 0.0)
    return np.abs(outside + inside)


def interpolate_odom(samples: Sequence[OdomSample], t: float) -> OdomSample:
    """真値を時刻 t に線形補間する."""
    times = np.array([s.t for s in samples])
    if t < times[0] or t > times[-1]:
        raise CheckError(f'時刻 {t:.3f} の真値がありません（{times[0]:.3f}〜{times[-1]:.3f}）')
    i = int(np.clip(np.searchsorted(times, t), 1, len(samples) - 1))
    a, b = samples[i - 1], samples[i]
    ratio = 0.0 if b.t == a.t else (t - a.t) / (b.t - a.t)
    return OdomSample(
        t=t,
        x=a.x + ratio * (b.x - a.x),
        y=a.y + ratio * (b.y - a.y),
        z=a.z + ratio * (b.z - a.z),
        yaw=a.yaw + ratio * wrap_angle(b.yaw - a.yaw),
        tilt=a.tilt,
    )


def cloud_to_lidar_points(cloud: PointCloud2) -> FloatArray:
    """有効な距離の点だけを LiDAR 座標系で取り出す."""
    raw = point_cloud2.read_points_numpy(cloud, field_names=('x', 'y', 'z'), skip_nans=False)
    points = np.asarray(raw, dtype=np.float64)
    ranges = np.linalg.norm(points, axis=1)
    return points[np.isfinite(ranges) & (ranges >= 0.1) & (ranges <= 40.0)]


def azimuth_bins(azimuth_rad: FloatArray) -> NDArray[np.int64]:
    """方位を 1 度ごとの区切り（-180〜179）にする。180 度は -180 度と同じ区切り."""
    return (np.floor(np.degrees(azimuth_rad)).astype(np.int64) + 180) % 360 - 180


def lidar_rays() -> tuple[FloatArray, FloatArray]:
    """gpu_lidar の格子の光線の (方位, 仰角) [rad]。どちらも 1 次元で、全組み合わせが光線になる."""
    azimuth = np.linspace(-math.pi, math.pi, LIDAR_AZIMUTH_SAMPLES)
    elevation = np.radians(
        np.linspace(LIDAR_ELEVATION_MIN_DEG, LIDAR_ELEVATION_MAX_DEG, LIDAR_ELEVATION_SAMPLES)
    )
    return azimuth, elevation


def body_occlusion(mount: Mount) -> BodyOcclusion:
    """LiDAR の格子の光線と機体の箱の交差を求める（スラブ法）."""
    azimuth, elevation = lidar_rays()
    az, el = np.meshgrid(azimuth, elevation, indexing='ij')
    direction = np.stack(
        (np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)), axis=-1
    )
    origin = np.array([mount.x, 0.0, mount.height])
    low = np.array([-BODY_HALF_M, -BODY_HALF_M, 0.0])
    high = np.array([BODY_HALF_M, BODY_HALF_M, BODY_HEIGHT_M])
    with np.errstate(divide='ignore', invalid='ignore'):
        t1 = (low - origin) / direction
        t2 = (high - origin) / direction
    # 軸に平行な光線は、その軸で箱の範囲の中なら制約なし、外なら当たらない
    parallel = direction == 0.0
    inside = (origin >= low) & (origin <= high)
    near = np.where(parallel, np.where(inside, -np.inf, np.inf), np.minimum(t1, t2))
    far = np.where(parallel, np.where(inside, np.inf, -np.inf), np.maximum(t1, t2))
    entry = np.max(near, axis=-1)
    leave = np.min(far, axis=-1)
    hit = (leave >= np.maximum(entry, 0.0))
    distance = np.where(hit, np.maximum(entry, 0.0), np.inf)
    blocked_azimuth = np.all(hit, axis=1)
    bins = azimuth_bins(azimuth)
    # 区切りの中のすべての方位が、どの仰角でも当たるものだけを遮られた区切りとする
    blocked_bins = frozenset(
        int(b) for b in np.unique(bins)
        if bool(np.all(blocked_azimuth[bins == b]))
    )
    step_deg = 360.0 / (LIDAR_AZIMUTH_SAMPLES - 1)
    return BodyOcclusion(
        too_close=int(np.count_nonzero(hit & (distance < LIDAR_MIN_RANGE_M))),
        self_hits=int(np.count_nonzero(hit & (distance >= LIDAR_MIN_RANGE_M))),
        blocked_deg=float(np.count_nonzero(blocked_azimuth)) * step_deg,
        blocked_bins=blocked_bins,
    )


def self_mask(points_base: FloatArray) -> NDArray[np.bool_]:
    """自機の除去の箱（FAST-LIO と検出と同じ判定）の中の点."""
    return (
        (np.abs(points_base[:, 0]) <= SELF_HALF_M)
        & (np.abs(points_base[:, 1]) <= SELF_HALF_M)
        & (points_base[:, 2] <= SELF_TOP_M)
    )


def measured_blocked_bins(points_lidar: FloatArray, mount: Mount) -> frozenset[int]:
    """自機以外の点が 1 つもない 1 度ごとの方位の区切り（機体でさえぎられた向き）."""
    outside = points_lidar[~self_mask(mount.to_base(points_lidar))]
    seen = azimuth_bins(np.arctan2(outside[:, 1], outside[:, 0]))
    return frozenset(range(-180, 180)) - frozenset(int(b) for b in np.unique(seen))


def evaluate_cloud(
    points_lidar: FloatArray,
    pose: OdomSample,
    mount: Mount,
    surfaces: Sequence[Surface],
) -> tuple[float, float, dict[str, int], int]:
    """点群を世界座標に変換し、地図形状との一致度を計算する.

    戻り値は (許容内の割合, 距離の95パーセンタイル, 形状ごとの点数, 自機の点数)。
    """
    points_base = mount.to_base(points_lidar)
    is_self = self_mask(points_base)
    outside = points_base[~is_self]
    world = rotate_z(outside, pose.yaw) + np.array([pose.x, pose.y, pose.z])
    distances = np.stack([surface_distance(s, world) for s in surfaces])
    nearest = np.min(distances, axis=0)
    on_surface = nearest <= SURFACE_TOLERANCE_M
    owners = np.argmin(distances, axis=0)
    hits = {
        s.name: int(np.count_nonzero(on_surface & (owners == i)))
        for i, s in enumerate(surfaces)
    }
    return (
        float(np.count_nonzero(on_surface)) / float(len(nearest)),
        float(np.percentile(nearest, 95.0)),
        hits,
        int(np.count_nonzero(is_self)),
    )


def settle_time(motion: Motion) -> float:
    """指令の速度に達するまでの時間."""
    return COMMAND_DELAY_S + max(
        math.hypot(motion.vx, motion.vy) / MAX_LINEAR_ACCEL_MPS2,
        abs(motion.wz) / MAX_ANGULAR_ACCEL_RADPS2,
    )


def predict_displacement(motion: Motion, yaw0: float, duration_s: float) -> tuple[float, float]:
    """機体座標系の一定速度指令で進む世界座標の変位を厳密に積分する."""
    theta = motion.wz * duration_s
    if abs(motion.wz) < 1e-9:
        px, py = motion.vx * duration_s, motion.vy * duration_s
    else:
        a = math.sin(theta) / motion.wz
        b = (1.0 - math.cos(theta)) / motion.wz
        px, py = a * motion.vx - b * motion.vy, b * motion.vx + a * motion.vy
    c, s = math.cos(yaw0), math.sin(yaw0)
    return c * px - s * py, s * px + c * py


def moving_average(values: FloatArray, window: int) -> FloatArray:
    return np.convolve(values, np.ones(window) / float(window), mode='valid')


def signal_match(measured: FloatArray, expected: FloatArray) -> tuple[float, float]:
    """expected に対する measured のゲインと相関係数を返す."""
    denominator = float(np.dot(expected, expected))
    if denominator < 1e-9:
        raise CheckError('基準信号がほぼ 0 なのでゲインを計算できません')
    return float(np.dot(measured, expected)) / denominator, float(np.corrcoef(measured, expected)[0, 1])


class Recorder(Node):
    """ブリッジのトピックを記録し、速度指令を出すノード."""

    def __init__(self) -> None:
        super().__init__('check_sim')
        qos = QoSProfile(
            depth=1000,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.odom: list[OdomSample] = []
        self.imu: dict[str, list[ImuSample]] = {topic: [] for topic in IMU_TOPICS}
        self.clouds: deque[PointCloud2] = deque(maxlen=50)
        self.cloud_stamps: list[float] = []
        self._cmd = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, qos)
        # 点群は 1 メッセージが大きく、Best Effort だと欠落する
        cloud_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PointCloud2, CLOUD_TOPIC, self._on_cloud, cloud_qos)
        for topic in IMU_TOPICS:
            self.create_subscription(Imu, topic, self._imu_callback(topic), qos)

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom.append(OdomSample(
            t=stamp_to_sec(msg.header.stamp.sec, msg.header.stamp.nanosec),
            x=p.x, y=p.y, z=p.z, yaw=yaw,
            # 機体の z 軸と鉛直のなす角
            tilt=math.acos(min(1.0, abs(1.0 - 2.0 * (q.x * q.x + q.y * q.y)))),
        ))

    def _on_cloud(self, msg: PointCloud2) -> None:
        self.clouds.append(msg)
        self.cloud_stamps.append(stamp_to_sec(msg.header.stamp.sec, msg.header.stamp.nanosec))

    def _imu_callback(self, topic: str) -> Callable[[Imu], None]:
        def callback(msg: Imu) -> None:
            w, a = msg.angular_velocity, msg.linear_acceleration
            self.imu[topic].append(ImuSample(
                t=stamp_to_sec(msg.header.stamp.sec, msg.header.stamp.nanosec),
                frame_id=msg.header.frame_id,
                gyro=(w.x, w.y, w.z),
                accel=(a.x, a.y, a.z),
            ))
        return callback

    def now_sim(self) -> float:
        if not self.odom:
            raise CheckError(f'{ODOM_TOPIC} を受信していません')
        return self.odom[-1].t

    def publish_cmd(self, vx: float, vy: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self._cmd.publish(msg)

    def wait_sim(self, duration_s: float, on_tick: Callable[[float], None] | None = None) -> None:
        """シミュレーション時間で待つ。on_tick には経過時間を渡す."""
        start = self.now_sim()
        wall_start = time.monotonic()
        while (elapsed := self.now_sim() - start) < duration_s:
            if time.monotonic() - wall_start > WALL_TIMEOUT_S:
                raise CheckError(f'{duration_s}秒待つ間にシミュレーション時間が進みませんでした')
            if on_tick is not None:
                on_tick(elapsed)
            time.sleep(0.02)

    def drive(self, motion: Motion) -> tuple[float, float]:
        """一定の速度指令を出し、指令を出していた時刻の範囲を返す."""
        start = self.now_sim()
        self.wait_sim(
            motion.duration_s,
            lambda _: self.publish_cmd(motion.vx, motion.vy, motion.wz),
        )
        end = self.now_sim()
        self.stop()
        return start, end

    def stop(self) -> None:
        self.wait_sim(STOP_S, lambda _: self.publish_cmd(0.0, 0.0, 0.0))

    def next_cloud_after(self, t: float) -> PointCloud2:
        wall_start = time.monotonic()
        while time.monotonic() - wall_start < WALL_TIMEOUT_S:
            for cloud in list(self.clouds):
                if stamp_to_sec(cloud.header.stamp.sec, cloud.header.stamp.nanosec) > t:
                    return cloud
            time.sleep(0.01)
        raise CheckError(f'時刻 {t:.3f} より新しい {CLOUD_TOPIC} を受信できません')


def wait_for_topics(recorder: Recorder) -> None:
    wall_start = time.monotonic()
    while time.monotonic() - wall_start < WALL_TIMEOUT_S:
        missing = [
            name
            for name, received in (
                (ODOM_TOPIC, bool(recorder.odom)),
                (CLOUD_TOPIC, bool(recorder.clouds)),
                *((topic, bool(recorder.imu[topic])) for topic in IMU_TOPICS),
            )
            if not received
        ]
        if not missing:
            return
        time.sleep(0.1)
    raise CheckError(f'{WALL_TIMEOUT_S}秒待っても受信できないトピック: {missing}')


def rate_hz(stamps: Sequence[float], start: float, end: float) -> float:
    selected = [t for t in stamps if start <= t <= end]
    if len(selected) < 2:
        return 0.0
    return float(len(selected) - 1) / (selected[-1] - selected[0])


def check_rates_and_static_imu(recorder: Recorder) -> list[CheckResult]:
    recorder.stop()
    wall_start = time.monotonic()
    start = recorder.now_sim()
    recorder.wait_sim(5.0)
    end = recorder.now_sim()
    rtf = (end - start) / (time.monotonic() - wall_start)

    def rate_result(topic: str, stamps: Sequence[float], expected: float) -> CheckResult:
        rate = rate_hz(stamps, start, end)
        return CheckResult(
            f'周期 {topic}',
            abs(rate - expected) <= 0.05 * expected,
            f'{rate:.1f}Hz（期待値 {expected:.0f}Hz、実時間比 {rtf:.2f}）',
        )

    def static_result(topic: str, frame_id: str) -> CheckResult:
        samples = [s for s in recorder.imu[topic] if start <= s.t <= end]
        accel = np.mean([s.accel for s in samples], axis=0)
        gyro = np.mean([s.gyro for s in samples], axis=0)
        frames = {s.frame_id for s in samples}
        ok = (
            abs(accel[2] - GRAVITY_MPS2) < 0.05
            and float(np.linalg.norm(accel[:2])) < 0.05
            and float(np.linalg.norm(gyro)) < 0.01
            and frames == {frame_id}
        )
        return CheckResult(
            f'静止時 {topic}',
            ok,
            f'加速度 {np.round(accel, 3).tolist()} 角速度 {np.round(gyro, 4).tolist()} '
            f'フレーム {sorted(frames)}',
        )

    return [
        rate_result(CLOUD_TOPIC, recorder.cloud_stamps, 10.0),
        rate_result(ODOM_TOPIC, [s.t for s in recorder.odom], 200.0),
        *(rate_result(t, [s.t for s in recorder.imu[t]], 200.0) for t in IMU_TOPICS),
        *(static_result(t, f) for t, f in IMU_TOPICS.items()),
    ]


def hidden_walls(
    surfaces: Sequence[Surface], pose: OdomSample, mount: Mount, occlusion: BodyOcclusion
) -> set[str]:
    """中心の向きが機体でさえぎられる方位にあるアリーナの壁（見えなくてよい）."""
    sensor = np.array([pose.x, pose.y]) + rotate_z(
        np.array([[mount.x, 0.0, 0.0]]), pose.yaw
    )[0, :2]
    hidden = set()
    for surface in surfaces:
        if not surface.name.startswith('arena_wall_') or isinstance(surface, Plane):
            continue
        dx, dy = surface.center[0] - sensor[0], surface.center[1] - sensor[1]
        bearing = wrap_angle(math.atan2(dy, dx) - pose.yaw)
        if int(azimuth_bins(np.array([bearing]))[0]) in occlusion.blocked_bins:
            hidden.add(surface.name)
    return hidden


def check_static_cloud(
    recorder: Recorder, surfaces: Sequence[Surface], mount: Mount
) -> list[CheckResult]:
    cloud = recorder.next_cloud_after(recorder.now_sim())
    points = cloud_to_lidar_points(cloud)
    t = stamp_to_sec(cloud.header.stamp.sec, cloud.header.stamp.nanosec)
    pose = interpolate_odom(recorder.odom, t)
    ratio, p95, hits, self_points = evaluate_cloud(points, pose, mount, surfaces)
    elevation = np.degrees(np.arctan2(points[:, 2], np.hypot(points[:, 0], points[:, 1])))
    seen = {name for name, count in hits.items() if count >= MIN_HITS_PER_SURFACE}
    names = {s.name for s in surfaces}
    fences = {name for name in names if name.startswith('fence_')}
    occlusion = body_occlusion(mount)
    hidden = hidden_walls(surfaces, pose, mount, occlusion)
    arena = ({'ground', 'ceiling'} | {name for name in names if name.startswith('arena_wall_')})
    landmarks = names - fences - arena
    expected_arena = arena - hidden
    # 機体に当たって最短距離より近い光線は点にならない
    expected_valid = EXPECTED_CLOUD_POINTS - occlusion.too_close
    measured_blocked = measured_blocked_bins(points, mount)
    # 機体でさえぎられた向きの幅。形状から求めた区切りとの差（多い側と少ない側）も見る
    extra = measured_blocked - occlusion.blocked_bins
    missing = occlusion.blocked_bins - measured_blocked
    structure_ok = cloud.width * cloud.height == EXPECTED_CLOUD_POINTS
    field_names = [f.name for f in cloud.fields]
    return [
        CheckResult(
            '点群の形式',
            structure_ok and cloud.header.frame_id == 'livox_frame',
            f'{cloud.width}x{cloud.height} フレーム {cloud.header.frame_id} '
            f'フィールド {field_names}',
        ),
        CheckResult(
            '点群の視野',
            -7.5 <= float(elevation.min()) <= -6.0
            and 50.0 <= float(elevation.max()) <= 52.5
            # 屋内なのでほぼ全点が返る。距離上限を超えるアリーナの隅と、機体に当たって最短距離より近い
            # 光線だけが欠ける
            and len(points) >= MIN_VALID_RATIO * expected_valid,
            f'仰角 {elevation.min():.1f}〜{elevation.max():.1f}度、有効 {len(points)}点'
            f'（機体が近すぎて欠ける光線 {occlusion.too_close} 本を除いた {expected_valid} 本の'
            f'{len(points) / expected_valid * 100.0:.1f}%）',
        ),
        CheckResult(
            '機体による遮り',
            abs(len(measured_blocked) - len(occlusion.blocked_bins)) <= OCCLUSION_TOLERANCE_DEG
            and len(extra) <= OCCLUSION_TOLERANCE_DEG and len(missing) <= OCCLUSION_TOLERANCE_DEG,
            f'自機以外の点がない方位 {len(measured_blocked)}度（形状から {occlusion.blocked_deg:.1f}度、'
            f'1度の区切りで {len(occlusion.blocked_bins)}個。多い側 {len(extra)}、少ない側 {len(missing)}）、'
            f'自機の箱の中の点 {self_points}点（形状から {occlusion.self_hits}本が機体に当たる）',
        ),
        CheckResult(
            '静止時の点群と地図の一致',
            ratio >= MIN_ON_SURFACE_RATIO,
            f'{SURFACE_TOLERANCE_M}m以内 {ratio * 100.0:.2f}%、95%点 {p95 * 100.0:.1f}cm',
        ),
        CheckResult(
            '床・柵・アリーナ・固定物が見える',
            expected_arena <= seen and bool(fences & seen) and bool(landmarks & seen),
            f'{dict(sorted(hits.items()))}'
            + (f'（機体でさえぎられる向きの壁 {sorted(hidden)} は除く）' if hidden else ''),
        ),
    ]


def check_motion(recorder: Recorder) -> list[CheckResult]:
    motions = [
        Motion('+x 3m/s', 3.0, 0.0, 0.0, 1.5),
        Motion('-x 3m/s', -3.0, 0.0, 0.0, 1.5),
        Motion('+y 3m/s', 0.0, 3.0, 0.0, 1.5),
        Motion('-y 3m/s', 0.0, -3.0, 0.0, 1.5),
        Motion('+yaw 1rad/s', 0.0, 0.0, 1.0, math.pi / 2.0),
        # 90度回した状態で +x を指令すると世界座標の +y に進むこと（機体座標系の確認）
        Motion('90度回転後の+x', 1.0, 0.0, 0.0, 1.0),
        Motion('90度回転後の-x', -1.0, 0.0, 0.0, 1.0),
        Motion('-yaw 1rad/s', 0.0, 0.0, -1.0, math.pi / 2.0),
        Motion('x+y+yaw 同時', 1.0, 1.0, 1.0, 1.5),
        Motion('x+y+yaw 逆向き', -1.0, -1.0, -1.0, 1.5),
    ]

    def run(motion: Motion) -> CheckResult:
        start, end = recorder.drive(motion)
        a = interpolate_odom(recorder.odom, start + settle_time(motion))
        b = interpolate_odom(recorder.odom, end)
        dt = b.t - a.t
        expected_x, expected_y = predict_displacement(motion, a.yaw, dt)
        dx, dy = b.x - a.x, b.y - a.y
        dyaw = wrap_angle(b.yaw - a.yaw)
        expected_yaw = wrap_angle(motion.wz * dt)
        path = math.hypot(motion.vx, motion.vy) * dt
        position_error = math.hypot(dx - expected_x, dy - expected_y)
        yaw_error = abs(wrap_angle(dyaw - expected_yaw))
        window = [s for s in recorder.odom if start <= s.t <= end]
        max_z = max(abs(s.z) for s in window)
        max_tilt = max(s.tilt for s in window)
        ok = (
            position_error <= 0.02 * path + 0.02
            and yaw_error <= 0.02
            and max_z <= 0.005
            and max_tilt <= 0.005
        )
        return CheckResult(
            f'動き {motion.name}',
            ok,
            f'位置誤差 {position_error * 100.0:.1f}cm/{path:.2f}m、'
            f'yaw誤差 {math.degrees(yaw_error):.2f}度、'
            f'z {max_z * 1000.0:.2f}mm、傾き {math.degrees(max_tilt):.2f}度',
        )

    return [run(motion) for motion in motions]


def check_moving_cloud(
    recorder: Recorder, surfaces: Sequence[Surface], mount: Mount
) -> list[CheckResult]:
    """走行中の点群と真値を比べる。点の取得時刻がばらついていれば一致しなくなる."""

    def evaluate_while(name: str, forward: Motion, backward: Motion) -> CheckResult:
        start, end = recorder.drive(forward)
        clouds = [
            c for c in list(recorder.clouds)
            if start + settle_time(forward)
            <= stamp_to_sec(c.header.stamp.sec, c.header.stamp.nanosec) <= end
        ]
        if not clouds:
            raise CheckError(f'{name} の走行中に点群を受信できませんでした')
        results = [
            evaluate_cloud(
                cloud_to_lidar_points(c),
                interpolate_odom(
                    recorder.odom,
                    stamp_to_sec(c.header.stamp.sec, c.header.stamp.nanosec),
                ),
                mount,
                surfaces,
            )
            for c in clouds
        ]
        recorder.drive(backward)
        worst_ratio = min(r[0] for r in results)
        worst_p95 = max(r[1] for r in results)
        return CheckResult(
            f'走行中の点群 {name}',
            worst_ratio >= MIN_ON_SURFACE_RATIO,
            f'{len(clouds)}フレーム、{SURFACE_TOLERANCE_M}m以内 最小 {worst_ratio * 100.0:.2f}%、'
            f'95%点 最大 {worst_p95 * 100.0:.1f}cm',
        )

    return [
        evaluate_while(
            '3m/s',
            Motion('', 3.0, 0.0, 0.0, 1.5),
            Motion('', -3.0, 0.0, 0.0, 1.5),
        ),
        evaluate_while(
            '1rad/s',
            Motion('', 0.0, 0.0, 1.0, 1.0),
            Motion('', 0.0, 0.0, -1.0, 1.0),
        ),
    ]


def check_imu_motion(recorder: Recorder) -> list[CheckResult]:
    """正弦波の速度指令で、IMU と真値から求めた加速度・角速度を比べる."""
    amplitude_mps, amplitude_radps, frequency_hz, duration_s = 1.5, 1.0, 0.5, 4.0
    omega = 2.0 * math.pi * frequency_hz
    window = 40  # 200Hz で 0.2 秒の移動平均

    def resample(times: FloatArray, values: FloatArray, grid: FloatArray) -> FloatArray:
        return np.interp(grid, times, values)

    def compare_linear(topic: str, start: float, end: float) -> CheckResult:
        odom = [s for s in recorder.odom if start <= s.t <= end]
        imu = [s for s in recorder.imu[topic] if start <= s.t <= end]
        odom_t = np.array([s.t for s in odom])
        velocity = np.gradient(np.array([s.x for s in odom]), odom_t)
        expected = moving_average(np.gradient(velocity, odom_t), window)
        measured = moving_average(np.array([s.accel[0] for s in imu]), window)
        expected_t = moving_average(odom_t, window)
        measured_t = moving_average(np.array([s.t for s in imu]), window)
        grid = np.arange(start + 0.5, end - 0.5, 0.05)
        gain, corr = signal_match(
            resample(measured_t, measured, grid), resample(expected_t, expected, grid)
        )
        return CheckResult(
            f'加速度の追従 {topic}',
            0.8 <= gain <= 1.2 and corr >= 0.9,
            f'ゲイン {gain:.3f}、相関 {corr:.3f}（真値の微分との比較）',
        )

    def compare_angular(topic: str, start: float, end: float) -> CheckResult:
        odom = [s for s in recorder.odom if start <= s.t <= end]
        imu = [s for s in recorder.imu[topic] if start <= s.t <= end]
        odom_t = np.array([s.t for s in odom])
        expected = moving_average(np.gradient(np.unwrap([s.yaw for s in odom]), odom_t), window)
        measured = moving_average(np.array([s.gyro[2] for s in imu]), window)
        expected_t = moving_average(odom_t, window)
        measured_t = moving_average(np.array([s.t for s in imu]), window)
        grid = np.arange(start + 0.5, end - 0.5, 0.05)
        gain, corr = signal_match(
            resample(measured_t, measured, grid), resample(expected_t, expected, grid)
        )
        return CheckResult(
            f'角速度の追従 {topic}',
            0.95 <= gain <= 1.05 and corr >= 0.98,
            f'ゲイン {gain:.3f}、相関 {corr:.3f}（真値の微分との比較）',
        )

    start = recorder.now_sim()
    recorder.wait_sim(
        duration_s,
        lambda elapsed: recorder.publish_cmd(amplitude_mps * math.sin(omega * elapsed), 0.0, 0.0),
    )
    end = recorder.now_sim()
    recorder.stop()
    linear = [compare_linear(topic, start, end) for topic in IMU_TOPICS]

    start = recorder.now_sim()
    recorder.wait_sim(
        duration_s,
        lambda elapsed: recorder.publish_cmd(0.0, 0.0, amplitude_radps * math.sin(omega * elapsed)),
    )
    end = recorder.now_sim()
    recorder.stop()
    return [*linear, *(compare_angular(topic, start, end) for topic in IMU_TOPICS)]


def check_fence_contact(recorder: Recorder, surfaces: Sequence[Surface]) -> list[CheckResult]:
    """柵に押し付けても、止まるだけで浮いたり傾いたりしないことを確かめる."""
    fence = next((s for s in surfaces if s.name == 'fence_west'), None)
    if not isinstance(fence, Box):
        raise CheckError('ワールドに箱形状の fence_west がありません')
    # 機体の前面が柵の内側の面に接する位置
    contact_x = float(fence.center[0] + fence.half[0]) + BODY_LENGTH_M / 2.0
    home = recorder.odom[-1]
    # 接触までに約 2.2 秒かかるので、その後も押し続ける
    start, _ = recorder.drive(Motion('', -3.0, 0.0, 0.0, 4.0))
    pushed = [s for s in recorder.odom if s.t >= start]
    imu = [s for s in recorder.imu['/sensing/livox/imu'] if s.t >= start]
    min_x = min(s.x for s in pushed)
    max_z = max(abs(s.z) for s in pushed)
    max_tilt = max(s.tilt for s in pushed)
    # 向きが少しずれていると角から当たって回るが、押し続ければ柵に沿って揃う（yaw 0度）
    max_yaw_change = max(abs(wrap_angle(s.yaw - home.yaw)) for s in pushed)
    final_yaw = abs(wrap_angle(pushed[-1].yaw))
    max_accel = max(abs(s.accel[0]) for s in imu)

    # 次回の確認に備えて、元の位置へ戻す
    recorder.wait_sim(
        RETURN_S,
        lambda _: recorder.publish_cmd(
            float(np.clip(RETURN_GAIN_PER_S * (home.x - recorder.odom[-1].x), -3.0, 3.0)), 0.0, 0.0
        ),
    )
    recorder.stop()
    final = recorder.odom[-1]
    return [
        CheckResult(
            '柵への接触',
            min_x >= contact_x - 0.02
            and max_z <= 0.02
            and max_tilt <= math.radians(1.0)
            and final_yaw <= math.radians(1.0),
            f'最接近 x={min_x:.3f}m（接触位置 {contact_x:.3f}m）、z {max_z * 1000.0:.1f}mm、'
            f'傾き {math.degrees(max_tilt):.2f}度、yaw変化 最大 {math.degrees(max_yaw_change):.2f}度、'
            f'押し付け後のyaw {math.degrees(final_yaw):.2f}度、'
            f'IMU ax 最大 {max_accel:.1f}m/s2',
        ),
        CheckResult(
            '接触後に離れられる',
            abs(final.x - home.x) <= 0.05 and abs(final.z) <= 0.02 and final.tilt <= math.radians(1.0),
            f'戻り位置の誤差 {abs(final.x - home.x) * 100.0:.1f}cm、z {final.z * 1000.0:.1f}mm',
        ),
    ]


def print_results(results: Sequence[CheckResult]) -> None:
    for result in results:
        print(f"[{'OK' if result.ok else 'NG'}] {result.name}: {result.detail}", flush=True)


def run_checks(recorder: Recorder, surfaces: Sequence[Surface], mount: Mount) -> bool:
    stages: list[Callable[[], list[CheckResult]]] = [
        lambda: check_rates_and_static_imu(recorder),
        lambda: check_static_cloud(recorder, surfaces, mount),
        lambda: check_motion(recorder),
        lambda: check_moving_cloud(recorder, surfaces, mount),
        lambda: check_imu_motion(recorder),
        lambda: check_fence_contact(recorder, surfaces),
    ]
    wait_for_topics(recorder)
    all_ok = True
    for stage in stages:
        results = stage()
        print_results(results)
        all_ok = all_ok and all(r.ok for r in results)
    return all_ok


def main() -> int:
    default_world = Path(get_package_share_directory('core_gazebo')) / 'worlds' / 'field.sdf'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', type=Path, default=default_world)
    # sim.launch.py の lidar_x・lidar_height と同じ値にする（既定は機体の前の端、床から 0.25m）
    parser.add_argument('--lidar-x', type=float, default=0.44)
    parser.add_argument('--lidar-height', type=float, default=0.25)
    args = parser.parse_args()

    surfaces = load_surfaces(args.world)
    rclpy.init()
    recorder = Recorder()
    executor = SingleThreadedExecutor()
    executor.add_node(recorder)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        ok = run_checks(recorder, surfaces, Mount(args.lidar_x, args.lidar_height))
    except CheckError as error:
        print(f'[NG] 確認を中断しました: {error}', file=sys.stderr)
        ok = False
    finally:
        # 異常終了しても機体を走らせたままにしない
        recorder.publish_cmd(0.0, 0.0, 0.0)
        time.sleep(0.1)
        # スピン中のスレッドが残ったまま rclpy を閉じると異常終了する
        executor.shutdown()
        spinner.join(timeout=5.0)
        recorder.destroy_node()
        rclpy.try_shutdown()
    print('結果: 全項目合格' if ok else '結果: 不合格あり', flush=True)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
