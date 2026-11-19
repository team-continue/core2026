#!/usr/bin/env python3
"""シナリオファイルどおりに機体を走らせて、結果を記録する（scenario_test 機能）.

既定では sim_nav.launch.py を起動し、ゴールを順に出して到達を待ち、相手との距離や接触を
真値から計算する。終了コードは、全ゴールに到達し、自機に原因のある接触が 0 なら 0。

他のスクリプトからは関数 run_scenario() を呼べる。シナリオの形式:
    start: [x, y, yaw]
    goals: [[x, y, yaw], ...]   # 順に回る
    timeout_s: 120              # 1 回り（repeat の 1 回分）の制限時間（シミュレーション時間）
    repeat: 1
    world: field.sdf            # field.sdf か field_static.sdf
    obstacles: [...]            # core_nav_check.scenario と同じ形式
"""

import argparse
import bisect
from dataclasses import asdict, dataclass, replace
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from typing import Final, Mapping, Sequence
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from core_nav_check.common import (
    CheckError,
    DriveNode,
    Pose,
    SpinThread,
)
from core_nav_check.scenario import (
    field_obstacles,
    footprint_gap,
    Obstacle,
    parse_obstacles,
    state_at,
)
from core_msgs.msg import (
    ContactStatus,
    LocalizationHealth,
    NavigationStatus,
    ProcessStats,
    SafetyStatus,
)
import numpy as np
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.parameter import Parameter
from tf2_msgs.msg import TFMessage
import yaml

ParamValue = bool | int | float | str
# JSON に書ける値だけを入れる
ResultDict = dict[str, object]

# config/field.yaml と footprint.yaml の値（柵までの距離の計算に使う）
FIELD_X_M: Final = 12.0
FIELD_Y_M: Final = 7.5
OWN_HALF_M: Final = 0.4
# 腰のある機体（sim_nav.launch.py の waist:=true）では、外周（足回り）は上半身（base_link）から腰の角度だけ
# 回っている。/tf のこの 2 つの間の向きを、真値の向きに足して外周を置く
BASE_FRAME: Final = 'base_link'
CHASSIS_FRAME: Final = 'chassis_link'

# 接触の直前の見る範囲と、自機に原因があるとみなす速度
FAULT_WINDOW_S: Final = 0.5
FAULT_SPEED_MPS: Final = 0.5
# 速く当たったとみなす、当たるときの近づく速さ（safety_stop.mitigation.min_impact_speed_mps と同じ）
HARD_IMPACT_SPEED_MPS: Final = 1.5
# 柵との距離がこれ以下なら触れたとみなす
FENCE_TOUCH_M: Final = 0.01
# 静止物（試験用の箱）との距離がこれ以下なら触れたとみなす。物理がめり込みを止めるので、当たっても距離は
# 0 より下がらない（obstacle_goals で箱に 1.2m/s で当たった記録で最小 0.0006m）。柵と同じしきいにする
STATIC_TOUCH_M: Final = FENCE_TOUCH_M
# 真値は 200Hz。距離の計算は 50Hz で十分
DEFAULT_TRUTH_STRIDE: Final = 4
READY_TIMEOUT_S: Final = 180.0
STOP_TIMEOUT_S: Final = 40.0
REPUBLISH_S: Final = 3.0
# 前のゴールの REACHED を今のゴールの到達と取り違えないよう、この時間は無視する
STALE_REACHED_S: Final = 2.0
PLANNER_FAILED_S: Final = 20.0

WORLD_TO_MAP: Final = {
    'field.sdf': 'arena_sim.pcd',
    'field_static.sdf': 'arena_static.pcd',
    'field_obstacles.sdf': 'arena_obstacles.pcd',
    'field_narrow.sdf': 'arena_narrow.pcd',
    # CoRE-1 の試験のフィールド（core_launch の make_core1_map.py と core_gazebo の make_core1_field_model.py で作る）
    'core1_2027.sdf': 'core1_2027.pcd',
    # 構造物をフィールドの 3D データから作ったもの（STL がある手元でだけ作れる）
    'core1_2027_mesh.sdf': 'core1_2027_mesh.pcd',
}
# ワールドごとのフィールドの設定（範囲・静止物の高さ・入らない範囲）。sim_nav.launch.py の field_file に渡す
WORLD_TO_FIELD: Final = {
    'core1_2027.sdf': 'field_core1_2027.yaml',
    'core1_2027_mesh.sdf': 'field_core1_2027.yaml',
}
# 点群を除いて記録する
BAG_TOPICS: Final = (
    '/behavior/goal_pose', '/ui/auto/start', '/control/mode', '/planning/plan', '/control/cmd_vel', '/planning/cmd_vel_nav', '/ground_truth/odom', '/localization/lio/odom', '/tf',
    '/tf_static', '/perception/tracks', '/perception/stats', '/planning/navigation/status',
    '/planning/planner/status', '/planning/safety/status', '/planning/contact/status', '/localization/health', '/clock',
)
SAFETY_STATE_NAMES: Final = {
    SafetyStatus.STATE_PASS: 'PASS',
    SafetyStatus.STATE_SLOW: 'SLOW',
    SafetyStatus.STATE_STOP: 'STOP',
    SafetyStatus.STATE_WATCHDOG_STOP: 'WATCHDOG_STOP',
    SafetyStatus.STATE_CONTACT_STOP: 'CONTACT_STOP',
    SafetyStatus.STATE_LOCALIZATION_STOP: 'LOCALIZATION_STOP',
}
NAVIGATION_STATE_NAMES: Final = {
    NavigationStatus.STATE_IDLE: 'IDLE',
    NavigationStatus.STATE_DRIVING: 'DRIVING',
    NavigationStatus.STATE_STUCK_WAIT: 'STUCK_WAIT',
    NavigationStatus.STATE_BACKOFF: 'BACKOFF',
    NavigationStatus.STATE_GOAL_WAIT: 'GOAL_WAIT',
    NavigationStatus.STATE_REACHED: 'REACHED',
    NavigationStatus.STATE_FAILED: 'FAILED',
}
ACTIVE_NAVIGATION_STATES: Final = (
    NavigationStatus.STATE_DRIVING,
    NavigationStatus.STATE_STUCK_WAIT,
    NavigationStatus.STATE_BACKOFF,
    NavigationStatus.STATE_GOAL_WAIT,
)


@dataclass(frozen=True)
class ScenarioSpec:
    name: str
    start: tuple[float, float, float]
    goals: tuple[tuple[float, float, float], ...]
    timeout_s: float
    repeat: int
    world: str
    obstacles: tuple[Obstacle, ...]


@dataclass(frozen=True)
class GoalResult:
    x: float
    y: float
    yaw: float
    reached: bool
    final_state: str
    duration_s: float
    final_distance_m: float


@dataclass(frozen=True)
class StatusSample:
    t: float
    state: int
    retries: int
    speed_limit_mps: float
    planning_ms: float
    distance_to_goal_m: float


@dataclass(frozen=True)
class StaticBox:
    name: str
    x: float
    y: float
    yaw: float
    half_x: float
    half_y: float


def load_spec(path: Path) -> ScenarioSpec:
    """シナリオファイルを読む."""
    try:
        with path.open(encoding='utf-8') as file:
            root = yaml.safe_load(file)
    except (OSError, yaml.YAMLError) as error:
        raise CheckError(f'シナリオ {path} を読めません: {error}') from error
    if not isinstance(root, dict) or 'start' not in root or not root.get('goals'):
        raise CheckError(f'シナリオ {path} に start と goals が必要です')
    start = root['start']
    return ScenarioSpec(
        name=path.stem,
        start=(float(start[0]), float(start[1]), float(start[2])),
        goals=tuple((float(g[0]), float(g[1]), float(g[2])) for g in root['goals']),
        timeout_s=float(root.get('timeout_s', 120.0)),
        repeat=int(root.get('repeat', 1)),
        world=str(root.get('world', 'field.sdf')),
        obstacles=tuple(parse_obstacles(root.get('obstacles') or [])),
    )


def write_temp_scenario(content: Mapping[str, object], name: str) -> Path:
    """試験用のシナリオを一時ファイルに書く."""
    directory = Path(tempfile.mkdtemp(prefix='core_nav_scenario_'))
    path = directory / f'{name}.yaml'
    path.write_text(yaml.safe_dump(dict(content), allow_unicode=True), encoding='utf-8')
    return path


def resolve_world(world: str) -> tuple[Path, Path]:
    """ワールド名から SDF と地図の場所を返す."""
    if world not in WORLD_TO_MAP:
        raise CheckError(f'未知のワールドです: {world}（{sorted(WORLD_TO_MAP)} から選んでください）')
    world_path = Path(get_package_share_directory('core_gazebo')) / 'worlds' / world
    map_path = (
        Path(get_package_share_directory('core_launch')) / 'maps' / WORLD_TO_MAP[world]
    )
    return world_path, map_path


def parse_param_value(text: str) -> ParamValue:
    if text.lower() in ('true', 'false'):
        return text.lower() == 'true'
    for convert in (int, float):
        try:
            return convert(text)
        except ValueError:
            continue
    return text


def parse_param_spec(text: str) -> tuple[str, str, ParamValue]:
    """NODE:NAME=VALUE を分解する."""
    target, separator, rest = text.partition(':')
    name, equals, value = rest.partition('=')
    if not separator or not equals or not target or not name:
        raise CheckError(f'--param は NODE:NAME=VALUE の形で指定してください: {text}')
    return target, name, parse_param_value(value)


def start_stack(
    spec: ScenarioSpec, scenario_path: Path, launch_args: Sequence[str], log_path: Path,
) -> subprocess.Popen[bytes]:
    """sim_nav.launch.py を起動する。ROS_DOMAIN_ID と GZ_PARTITION はそのまま引き継ぐ."""
    world_path, map_path = resolve_world(spec.world)
    arguments = {
        'x': str(spec.start[0]),
        'y': str(spec.start[1]),
        'yaw': str(spec.start[2]),
        'world': str(world_path),
        'map_path': str(map_path),
        'scenario_file': str(scenario_path.resolve()),
    }
    field_file = field_config(spec.world)
    if field_file is not None:
        arguments['field_file'] = str(field_file)
    for item in launch_args:
        name, separator, value = item.partition('=')
        if not separator:
            raise CheckError(f'--launch-arg は NAME=VALUE の形で指定してください: {item}')
        arguments[name] = value
    command = [
        'ros2', 'launch', 'core_gazebo', 'sim_nav.launch.py',
        *(f'{name}:={value}' for name, value in arguments.items()),
    ]
    with log_path.open('wb') as log:
        return subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy())


def stop_process(process: subprocess.Popen[bytes], timeout_s: float = STOP_TIMEOUT_S) -> None:
    """SIGINT で止める。止まらなければ順に強く止める."""
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=timeout_s)
        return
    except subprocess.TimeoutExpired:
        print(f'[警告] プロセス {process.pid} が SIGINT で止まらないので SIGTERM を送ります', flush=True)
    process.terminate()
    try:
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        print(f'[警告] プロセス {process.pid} を強制終了します', flush=True)
        process.kill()
        process.wait(timeout=10.0)


def log_tail(path: Path, lines: int = 15) -> str:
    try:
        text = path.read_text(errors='replace')
    except OSError as error:
        return f'ログを読めません: {error}'
    return '\n'.join(text.splitlines()[-lines:])


class ScenarioNode(DriveNode):
    """真値と各処理の状態を記録し、ゴールを出すノード.

    DriveNode の走行関数は /cmd_vel_nav に出す。ナビゲーションを止めて直接動かす試験で使う。
    """

    def __init__(self, name: str = 'run_scenario') -> None:
        super().__init__(name, cmd_topic='/planning/cmd_vel_nav')
        self.navigation: list[StatusSample] = []
        self.planner_states: list[tuple[float, int]] = []
        self.safety: list[tuple[float, int]] = []
        self.contact: list[tuple[float, bool]] = []
        self.health_level: int | None = None
        self.perception_ms: list[float] = []
        # 腰の角度（BASE_FRAME から見た CHASSIS_FRAME の向き）。腰のない機体では空のまま
        self.waist: list[tuple[float, float]] = []
        # 操縦の UI と同じく、目標を渡して auto を押す（control_mode_manager が自動に入り、計画へ渡す）
        self._goal_publisher = self.create_publisher(PoseStamped, '/ui/auto/start', 10)
        self.create_subscription(NavigationStatus, '/planning/navigation/status', self._on_navigation, 50)
        self.create_subscription(NavigationStatus, '/planning/planner/status', self._on_planner, 50)
        self.create_subscription(SafetyStatus, '/planning/safety/status', self._on_safety, 50)
        self.create_subscription(ContactStatus, '/planning/contact/status', self._on_contact, 50)
        self.create_subscription(LocalizationHealth, '/localization/health', self._on_health, 50)
        self.create_subscription(ProcessStats, '/perception/stats', self._on_perception, 50)
        self.create_subscription(TFMessage, '/tf', self._on_tf, 100)

    def received_at(self) -> float:
        """受信した時点の真値の時刻。各メッセージの時刻の付け方に依存しないようにする."""
        return self.truth[-1].t if self.truth else float('nan')

    def _on_navigation(self, msg: NavigationStatus) -> None:
        self.navigation.append(StatusSample(
            t=self.received_at(),
            state=msg.state,
            retries=msg.retries,
            speed_limit_mps=msg.speed_limit_mps,
            planning_ms=msg.planning_ms,
            distance_to_goal_m=msg.distance_to_goal_m,
        ))

    def _on_planner(self, msg: NavigationStatus) -> None:
        self.planner_states.append((self.received_at(), msg.state))

    def _on_safety(self, msg: SafetyStatus) -> None:
        self.safety.append((self.received_at(), msg.state))

    def _on_contact(self, msg: ContactStatus) -> None:
        self.contact.append((self.received_at(), msg.in_contact))

    def _on_health(self, msg: LocalizationHealth) -> None:
        self.health_level = msg.level

    def _on_perception(self, msg: ProcessStats) -> None:
        self.perception_ms.append(msg.processing_ms)

    def _on_tf(self, msg: TFMessage) -> None:
        # 真値を受ける前は時刻が付けられない
        if not self.truth:
            return
        for transform in msg.transforms:
            if transform.header.frame_id == BASE_FRAME and transform.child_frame_id == CHASSIS_FRAME:
                q = transform.transform.rotation
                self.waist.append((
                    self.received_at(),
                    math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
                ))

    def wait_sim(self, duration_s: float) -> None:
        """指令を出さずにシミュレーション時間で待つ."""
        start = self.now_sim()
        wall_start = time.monotonic()
        while self.now_sim() - start < duration_s:
            if time.monotonic() - wall_start > duration_s * 3.0 + 60.0:
                raise CheckError(f'{duration_s}秒待つ間にシミュレーション時間が進みませんでした')
            time.sleep(0.02)

    def latest_state(self) -> int | None:
        return self.navigation[-1].state if self.navigation else None

    def publish_goal(self, x: float, y: float, yaw: float) -> None:
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        now = self.received_at()
        if not math.isnan(now):
            msg.header.stamp.sec = int(now)
            msg.header.stamp.nanosec = int((now - int(now)) * 1e9)
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self._goal_publisher.publish(msg)

    def goal_subscribers(self) -> int:
        return self._goal_publisher.get_subscription_count()

    def set_parameter(
        self, target: str, name: str, value: ParamValue, timeout_s: float = 15.0,
    ) -> SetParametersResult:
        """別のノードのパラメータを設定し、受け入れられたかを返す."""
        client = self.create_client(SetParameters, f'/{target}/set_parameters')
        try:
            if not client.wait_for_service(timeout_sec=timeout_s):
                raise CheckError(f'ノード {target} のパラメータサービスに接続できません')
            request = SetParameters.Request()
            request.parameters = [Parameter(name, value=value).to_parameter_msg()]
            future = client.call_async(request)
            wall_start = time.monotonic()
            while not future.done():
                if time.monotonic() - wall_start > timeout_s:
                    raise CheckError(f'{target}:{name} の設定の応答がありません')
                time.sleep(0.01)
            result = future.result()
            if result is None:
                raise CheckError(f'{target}:{name} の設定に失敗しました: {future.exception()}')
            return result.results[0]
        finally:
            self.destroy_client(client)


def wait_until_ready(
    node: ScenarioNode, process: subprocess.Popen[bytes] | None, log_path: Path | None,
    timeout_s: float = READY_TIMEOUT_S,
) -> None:
    """/navigation/status が届き、自己位置が正常になるまで待つ."""
    wall_start = time.monotonic()
    while True:
        if process is not None and process.poll() is not None:
            tail = log_tail(log_path) if log_path is not None else ''
            raise CheckError(f'launch が終了しました（終了コード {process.returncode}）\n{tail}')
        ready = (
            bool(node.truth) and bool(node.navigation)
            and node.health_level == LocalizationHealth.LEVEL_OK and node.goal_subscribers() > 0
        )
        if ready:
            return
        if time.monotonic() - wall_start > timeout_s:
            raise CheckError(
                f'{timeout_s}秒待っても準備が整いません（真値 {bool(node.truth)}、'
                f'状態 {bool(node.navigation)}、自己位置 {node.health_level}、'
                f'ゴールの購読者 {node.goal_subscribers()}）'
            )
        time.sleep(0.1)


def drive_goal(
    node: ScenarioNode, goal: tuple[float, float, float], time_limit_s: float,
    process: subprocess.Popen[bytes] | None = None,
) -> GoalResult:
    """ゴールを出して、到達か失敗か時間切れまで待つ."""
    x, y, yaw = goal
    start = node.now_sim()
    wall_start = time.monotonic()
    wall_limit = time_limit_s * 3.0 + 60.0
    node.publish_goal(x, y, yaw)
    last_publish = start
    navigation_active = False
    accepted = False
    failed_since: float | None = None
    reached = False
    final_state = 'TIMEOUT'
    while True:
        now = node.now_sim()
        if process is not None and process.poll() is not None:
            raise CheckError(f'走行中に launch が終了しました（終了コード {process.returncode}）')
        state = node.latest_state()
        planner_time, planner_state = (
            node.planner_states[-1] if node.planner_states else (float('-inf'), None)
        )
        planner_current = planner_time >= start
        navigation_active = navigation_active or state in ACTIVE_NAVIGATION_STATES
        accepted = accepted or navigation_active or (
            planner_current and planner_state == NavigationStatus.STATE_GOAL_WAIT
        )
        # 計画側が経路なしと言い続けるなら、制限時間まで待たない
        if planner_current and planner_state == NavigationStatus.STATE_FAILED:
            failed_since = now if failed_since is None else failed_since
        else:
            failed_since = None
        if failed_since is not None and now - failed_since > PLANNER_FAILED_S:
            final_state = 'NO_PATH'
            break
        # 前のゴールの REACHED と取り違えないよう、動き出すか一定時間経つまで到達としない
        settled = navigation_active or now - start > STALE_REACHED_S
        if state == NavigationStatus.STATE_REACHED and settled:
            reached = True
            final_state = 'REACHED'
            break
        if state == NavigationStatus.STATE_FAILED and settled:
            final_state = 'FAILED'
            break
        if now - start > time_limit_s or time.monotonic() - wall_start > wall_limit:
            final_state = 'TIMEOUT'
            break
        # 受け付けられていないようなら 1 度だけ出し直す。出し直すと待ち時間が巻き戻る処理があるので繰り返さない
        if not accepted and last_publish == start and now - start > REPUBLISH_S:
            node.publish_goal(x, y, yaw)
            last_publish = now
        time.sleep(0.05)
    end = node.truth[-1]
    return GoalResult(
        x=x, y=y, yaw=yaw, reached=reached, final_state=final_state, duration_s=now - start,
        final_distance_m=math.hypot(end.x - x, end.y - y),
    )


def field_config(world: str) -> Path | None:
    """ワールドのフィールドの設定（WORLD_TO_FIELD）。なければ None."""
    name = WORLD_TO_FIELD.get(world)
    if name is None:
        return None
    return Path(get_package_share_directory('core_launch')) / 'config' / name


def structure_outlines_path(map_path: Path) -> Path:
    """地図の隣の構造物の外形（core_launch/maps/<地図の名前>_outlines.yaml。make_core1_map.py が作る）."""
    return map_path.with_name(f'{map_path.stem}_outlines.yaml')


def load_static_polygons(world_path: Path, outlines_path: Path) -> list[tuple[str, np.ndarray]]:
    """構造物の外形（CoRE-1 の試験のフィールド）を読む.

    outlines_path（make_core1_map.py が作る外形。Gazebo のフィールドはこれを押し出したものか 3D データ）が
    あればそれを、なければ polyline の形の静止物を SDF から読む。
    """
    if outlines_path.exists():
        return load_outlines(outlines_path)
    polygons = []
    for model in ET.parse(world_path).getroot().findall('./world/model'):
        polyline = model.find('./link/collision/geometry/polyline')
        if polyline is None:
            continue
        points = [[float(v) for v in (point.text or '').split()] for point in polyline.findall('point')]
        if len(points) < 3 or any(len(point) != 2 for point in points):
            raise CheckError(f'{world_path}: {model.attrib["name"]} の polyline の頂点が正しくありません')
        polygons.append((model.attrib['name'], np.array(points, dtype=np.float64)))
    return polygons


def load_outlines(path: Path) -> list[tuple[str, np.ndarray]]:
    """make_core1_map.py が作った構造物の外形（outlines: [{name, height_m, points: [[x, y], ...]}]）を読む."""
    try:
        document = yaml.safe_load(path.read_text(encoding='utf-8'))
        outlines = document['outlines']
    except (OSError, yaml.YAMLError, KeyError, TypeError) as error:
        raise CheckError(f'構造物の外形 {path} を読めません: {error}') from error
    polygons = []
    for outline in outlines:
        points = np.array(outline.get('points', []), dtype=np.float64)
        if points.ndim != 2 or points.shape[0] < 3 or points.shape[1] != 2 or not np.all(np.isfinite(points)):
            raise CheckError(f'{path}: {outline.get("name")} の頂点が正しくありません')
        polygons.append((str(outline['name']), points))
    return polygons


def load_keep_out(world: str) -> list[np.ndarray]:
    """ワールドのフィールドの設定から、入らない範囲（進入禁止ゾーン）の多角形を読む."""
    path = field_config(world)
    if path is None:
        return []
    try:
        document = yaml.safe_load(path.read_text(encoding='utf-8'))
        keep_out = document['/**']['ros__parameters'].get('keep_out', {})
    except (OSError, yaml.YAMLError, KeyError, TypeError) as error:
        raise CheckError(f'フィールドの設定 {path} を読めません: {error}') from error
    xs, ys, counts = keep_out.get('x', []), keep_out.get('y', []), keep_out.get('counts', [])
    if len(xs) != len(ys) or sum(counts) != len(xs):
        raise CheckError(f'{path} の keep_out の数が合いません')
    polygons, start = [], 0
    for count in counts:
        polygons.append(np.array(list(zip(xs[start:start + count], ys[start:start + count])), dtype=np.float64))
        start += count
    return polygons


def inside_polygon(px: np.ndarray, py: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    """点が多角形の中にあるか（偶奇規則）."""
    result = np.zeros(px.shape, dtype=bool)
    for (ax, ay), (bx, by) in zip(polygon, np.roll(polygon, 1, axis=0)):
        crosses = (ay > py) != (by > py)
        with np.errstate(divide='ignore', invalid='ignore'):
            at = ax + (py - ay) * (bx - ax) / (by - ay)
        result ^= crosses & (px < at)
    return result


def polygon_distance(px: np.ndarray, py: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    """点から多角形までの符号付き距離（内側は負）."""
    distance = np.full(px.shape, np.inf)
    for (ax, ay), (bx, by) in zip(polygon, np.roll(polygon, 1, axis=0)):
        ex, ey = bx - ax, by - ay
        length2 = ex * ex + ey * ey
        t = np.clip(((px - ax) * ex + (py - ay) * ey) / length2, 0.0, 1.0) if length2 > 0.0 else 0.0
        distance = np.minimum(distance, np.hypot(px - (ax + t * ex), py - (ay + t * ey)))
    return np.where(inside_polygon(px, py, polygon), -distance, distance)


def outline_points(samples: Sequence['Pose']) -> tuple[np.ndarray, np.ndarray]:
    """各時刻の機体の外周の点（map 座標）。形は (時刻, 点)."""
    outline = np.array(footprint_outline(OWN_HALF_M))
    xs = np.array([p.x for p in samples])[:, None]
    ys = np.array([p.y for p in samples])[:, None]
    yaws = np.array([p.yaw for p in samples])[:, None]
    c, s = np.cos(yaws), np.sin(yaws)
    return xs + c * outline[:, 0] - s * outline[:, 1], ys + s * outline[:, 0] + c * outline[:, 1]


def analyze_structures(
    samples: Sequence['Pose'], world_path: Path, world: str, outlines_path: Path,
) -> ResultDict:
    """構造物までの距離と接触、入らない範囲に機体がかかった回数と時間."""
    result: ResultDict = {
        'min_structure_gap_m': None, 'structure_contacts': 0, 'keep_out_entries': 0, 'keep_out_time_s': 0.0,
    }
    polygons = load_static_polygons(world_path, outlines_path)
    keep_out = load_keep_out(world)
    if not samples or (not polygons and not keep_out):
        return result
    px, py = outline_points(samples)
    times = np.array([p.t for p in samples])
    if polygons:
        gaps = np.min([polygon_distance(px, py, polygon).min(axis=1) for _, polygon in polygons], axis=0)
        touching = gaps <= STATIC_TOUCH_M
        result['min_structure_gap_m'] = float(gaps.min())
        result['structure_contacts'] = int(np.count_nonzero(touching[1:] & ~touching[:-1]) + int(touching[0]))
    if keep_out:
        inside = np.any([inside_polygon(px, py, polygon).any(axis=1) for polygon in keep_out], axis=0)
        result['keep_out_entries'] = int(np.count_nonzero(inside[1:] & ~inside[:-1]) + int(inside[0]))
        result['keep_out_time_s'] = float(np.sum(np.diff(times)[inside[:-1]])) if len(times) > 1 else 0.0
    return result


def load_static_boxes(world_path: Path) -> list[StaticBox]:
    """試験用に置いた静止物（static_box_*）を SDF から読む."""
    boxes = []
    for model in ET.parse(world_path).getroot().findall('./world/model'):
        name = model.attrib['name']
        if not name.startswith('static_box'):
            continue
        pose = [float(v) for v in model.findtext('pose', default='0 0 0 0 0 0').split()]
        size = model.find('./link/collision/geometry/box/size')
        if size is None or size.text is None:
            raise CheckError(f'{world_path}: {name} に箱の大きさがありません')
        sx, sy, _ = (float(v) for v in size.text.split())
        boxes.append(StaticBox(name, pose[0], pose[1], pose[5], sx / 2.0, sy / 2.0))
    return boxes


def footprint_outline(half: float, points_per_side: int = 8) -> list[tuple[float, float]]:
    """機体の外周の点を、機体座標系で返す."""
    steps = [-half + 2.0 * half * i / points_per_side for i in range(points_per_side + 1)]
    return (
        [(half, s) for s in steps] + [(-half, s) for s in steps]
        + [(s, half) for s in steps] + [(s, -half) for s in steps]
    )


def box_distance(x: float, y: float, box: StaticBox) -> float:
    """点から箱までの符号付き距離（内側は負）."""
    c, s = math.cos(box.yaw), math.sin(box.yaw)
    local_x = c * (x - box.x) + s * (y - box.y)
    local_y = -s * (x - box.x) + c * (y - box.y)
    dx = abs(local_x) - box.half_x
    dy = abs(local_y) - box.half_y
    return math.hypot(max(dx, 0.0), max(dy, 0.0)) + min(max(dx, dy), 0.0)


def fence_clearance(pose: Pose) -> float:
    """機体の角から柵の内側の面までの最小距離."""
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    clearance = float('inf')
    for corner_x, corner_y in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        x = pose.x + OWN_HALF_M * (c * corner_x - s * corner_y)
        y = pose.y + OWN_HALF_M * (s * corner_x + c * corner_y)
        clearance = min(clearance, FIELD_X_M - abs(x), FIELD_Y_M - abs(y))
    return clearance


def thin_samples(truth: Sequence[Pose], stride: int) -> list[Pose]:
    return list(truth[::stride])


def chassis_samples(samples: Sequence[Pose], waist: Sequence[tuple[float, float]]) -> list[Pose]:
    """真値の向きに腰の角度を足して、外周（足回り）の向きにする.

    腰の角度は各時刻の直前に受けた値（最初に受けるより前は最初の値）。腰の角度がなければそのまま返す
    """
    if not waist:
        return list(samples)
    times = [t for t, _ in waist]

    def chassis_yaw(pose: Pose) -> float:
        q = waist[max(bisect.bisect_right(times, pose.t) - 1, 0)][1]
        return math.atan2(math.sin(pose.yaw + q), math.cos(pose.yaw + q))
    return [replace(pose, yaw=chassis_yaw(pose)) for pose in samples]


def analyze_obstacles(
    samples: Sequence[Pose], obstacles: Sequence[Obstacle], warn_m: float,
) -> ResultDict:
    """相手との距離と接触を計算する."""
    per_obstacle: dict[str, float] = {}
    contacts: list[ResultDict] = []
    warn_time = 0.0
    min_gap = float('inf')
    times = np.array([s.t for s in samples])
    xs = np.array([s.x for s in samples])
    ys = np.array([s.y for s in samples])
    gaps_now = np.full(len(samples), float('inf'))
    for obstacle in obstacles:
        previous_gap = float('inf')
        obstacle_min = float('inf')
        for index, pose in enumerate(samples):
            state = state_at(obstacle, pose.t)
            gap = footprint_gap(pose.x, pose.y, pose.yaw, OWN_HALF_M, obstacle, state)
            obstacle_min = min(obstacle_min, gap)
            gaps_now[index] = min(gaps_now[index], gap)
            if gap <= 0.0 < previous_gap:
                contacts.append(classify_contact(obstacle, pose.t, times, xs, ys))
            previous_gap = gap
        per_obstacle[obstacle.name] = obstacle_min
        min_gap = min(min_gap, obstacle_min)
    if len(times) >= 2:
        steps = np.diff(times)
        warn_time = float(np.sum(steps[gaps_now[:-1] < warn_m]))
    own = [c for c in contacts if c['own_fault']]
    impacts = [float(c['impact_speed_mps']) for c in contacts]
    return {
        'min_gap_m': min_gap if obstacles else None,
        'min_gap_by_obstacle_m': per_obstacle,
        'min_distance_warn_m': warn_m,
        'warn_time_s': warn_time,
        'contacts': len(contacts),
        'own_fault_contacts': len(own),
        'other_fault_contacts': len(contacts) - len(own),
        # 当たるときの近づく速さ（中心を結ぶ向きの相対速度）の平均と、HARD_IMPACT_SPEED_MPS 以上の回数
        'mean_impact_speed_mps': float(np.mean(impacts)) if impacts else None,
        'hard_contacts': sum(1 for impact in impacts if impact >= HARD_IMPACT_SPEED_MPS),
        'contact_details': contacts,
    }


def classify_contact(
    obstacle: Obstacle, t: float, times: np.ndarray, xs: np.ndarray, ys: np.ndarray,
) -> ResultDict:
    """接触の直前に、自機が相手に近づく向きに動いていたかを調べる."""
    before = t - FAULT_WINDOW_S
    x0, y0 = float(np.interp(before, times, xs)), float(np.interp(before, times, ys))
    x1, y1 = float(np.interp(t, times, xs)), float(np.interp(t, times, ys))
    span = min(FAULT_WINDOW_S, t - float(times[0])) or FAULT_WINDOW_S
    vx, vy = (x1 - x0) / span, (y1 - y0) / span
    state = state_at(obstacle, t)
    closing = vx * (state.x - x1) + vy * (state.y - y1)
    speed = math.hypot(vx, vy)
    # 当たるときの近づく速さ: 相対速度の、中心を結ぶ向きの成分（近づくときに正）
    distance = math.hypot(state.x - x1, state.y - y1)
    impact = max(
        ((vx - state.vx) * (state.x - x1) + (vy - state.vy) * (state.y - y1)) / distance, 0.0,
    ) if distance > 1e-6 else 0.0
    return {
        'obstacle': obstacle.name,
        'time_s': t,
        'own_speed_mps': speed,
        'own_fault': speed >= FAULT_SPEED_MPS and closing > 0.0,
        'impact_speed_mps': impact,
    }


def fence_touches(samples: Sequence[Pose]) -> list[dict[str, float]]:
    """柵に触れ始めた時刻と位置の一覧を返す。原因を後から調べられるようにする."""
    touches: list[dict[str, float]] = []
    previous = float('inf')
    for pose in samples:
        clearance = fence_clearance(pose)
        if clearance <= FENCE_TOUCH_M < previous:
            touches.append({'time_s': pose.t, 'x': pose.x, 'y': pose.y})
        previous = clearance
    return touches


def static_gaps(samples: Sequence[Pose], boxes: Sequence[StaticBox]) -> tuple[np.ndarray, np.ndarray]:
    """各時刻の、機体の外周と静止物の最小の符号付き距離と、最も近い静止物の番号を返す."""
    outline = np.array(footprint_outline(OWN_HALF_M))
    xs = np.array([p.x for p in samples])[:, None]
    ys = np.array([p.y for p in samples])[:, None]
    yaws = np.array([p.yaw for p in samples])[:, None]
    c, s = np.cos(yaws), np.sin(yaws)
    px = xs + c * outline[:, 0] - s * outline[:, 1]
    py = ys + s * outline[:, 0] + c * outline[:, 1]
    per_box = []
    for box in boxes:
        bc, bs = math.cos(box.yaw), math.sin(box.yaw)
        local_x = bc * (px - box.x) + bs * (py - box.y)
        local_y = -bs * (px - box.x) + bc * (py - box.y)
        dx = np.abs(local_x) - box.half_x
        dy = np.abs(local_y) - box.half_y
        distance = (
            np.hypot(np.maximum(dx, 0.0), np.maximum(dy, 0.0)) + np.minimum(np.maximum(dx, dy), 0.0)
        )
        per_box.append(distance.min(axis=1))
    gaps = np.array(per_box)
    return gaps.min(axis=0), gaps.argmin(axis=0)


def analyze_static(samples: Sequence[Pose], world_path: Path) -> ResultDict:
    """試験用の静止物と柵までの距離を計算する.

    静止物は、距離が STATIC_TOUCH_M 以下になった回数を接触として数え、時刻、静止物、位置、当たる前の速さを
    static_touches に残す。物理がめり込みを止めるので、当たっても距離は 0 より下がらない。間引かずに
    samples（50Hz）のまま見る（10Hz では、箱に 1.2m/s で当たって 0.04 秒で止まった接触を見落とした）
    """
    fence = min((fence_clearance(p) for p in samples), default=float('inf'))
    boxes = load_static_boxes(world_path)
    result: ResultDict = {
        'min_fence_clearance_m': fence, 'fence_touches': fence_touches(samples),
        'min_static_gap_m': None, 'static_contacts': 0, 'static_touches': [],
    }
    if not boxes or not samples:
        return result
    gaps, nearest = static_gaps(samples, boxes)
    touches: list[dict[str, object]] = []
    previous = float('inf')
    for index, (pose, gap) in enumerate(zip(samples, gaps)):
        if gap <= STATIC_TOUCH_M < previous:
            # 当たる前の速さ。触れた標本の 1 つ前までの区間で求める（触れた標本までの区間は止まり始めを含む）
            first, last = samples[max(index - 2, 0)], samples[max(index - 1, 0)]
            span = last.t - first.t
            speed = math.hypot(last.x - first.x, last.y - first.y) / span if span > 0.0 else 0.0
            touches.append({
                'time_s': pose.t, 'box': boxes[int(nearest[index])].name, 'x': pose.x, 'y': pose.y,
                'speed_mps': speed,
            })
        previous = float(gap)
    result['min_static_gap_m'] = float(gaps.min())
    result['static_contacts'] = len(touches)
    result['static_touches'] = touches
    return result


def state_durations(
    samples: Sequence[tuple[float, int]], names: Mapping[int, str],
) -> dict[str, float]:
    """状態ごとの時間を、次のメッセージまでの間隔で積算する."""
    durations: dict[str, float] = {}
    valid = [s for s in samples if not math.isnan(s[0])]
    for (t0, state), (t1, _) in zip(valid, valid[1:]):
        name = names.get(state, str(state))
        durations[name] = durations.get(name, 0.0) + max(t1 - t0, 0.0)
    return durations


def percentile_summary(values: Sequence[float]) -> tuple[float | None, float | None]:
    if not values:
        return None, None
    array = np.array(values, dtype=np.float64)
    return float(np.mean(array)), float(np.percentile(array, 99.0))


def analyze_run(
    node: ScenarioNode, spec: ScenarioSpec, goal_results: Sequence[GoalResult], start: float,
    end: float, warn_m: float, stride: int = DEFAULT_TRUTH_STRIDE,
) -> ResultDict:
    """記録から結果をまとめる。受信は別スレッドで続くので、先に固定してから計算する."""
    truth = [p for p in list(node.truth) if start <= p.t <= end]
    if len(truth) < 2:
        raise CheckError('走行中の真値がほとんどありません')
    # 機体の外周での距離は足回りの向きで測る（腰のない機体では真値の向きのまま）
    waist = list(node.waist)
    samples = chassis_samples(thin_samples(truth, stride), waist)
    times = np.array([p.t for p in samples])
    speeds = np.hypot(
        np.gradient(np.array([p.x for p in samples]), times),
        np.gradient(np.array([p.y for p in samples]), times),
    )
    navigation = [s for s in list(node.navigation) if start <= s.t <= end]
    planning = [s.planning_ms for s in navigation if s.state in ACTIVE_NAVIGATION_STATES]
    planning_mean, planning_p99 = percentile_summary(planning)
    perception_mean, perception_p99 = percentile_summary(list(node.perception_ms))
    contact_samples = [c for c in list(node.contact) if start <= c[0] <= end]
    contact_events = sum(
        1 for (_, a), (_, b) in zip(contact_samples, contact_samples[1:]) if b and not a
    ) + int(bool(contact_samples) and contact_samples[0][1])
    world_path, map_path = resolve_world(spec.world)
    reached = sum(1 for g in goal_results if g.reached)
    limits = [s.speed_limit_mps for s in navigation if s.state == NavigationStatus.STATE_DRIVING]

    result: ResultDict = {
        'scenario': spec.name,
        'world': spec.world,
        'goals_total': len(goal_results),
        'goals_reached': reached,
        'goal_results': [asdict(g) for g in goal_results],
        'duration_s': end - start,
        'max_speed_mps': float(np.max(speeds)),
        'mean_speed_mps': float(np.mean(speeds)),
        'planning_ms_mean': planning_mean,
        'planning_ms_p99': planning_p99,
        'perception_ms_mean': perception_mean,
        'perception_ms_p99': perception_p99,
        'safety_time_s': state_durations(list(node.safety), SAFETY_STATE_NAMES),
        'navigation_time_s': state_durations(
            [(s.t, s.state) for s in navigation], NAVIGATION_STATE_NAMES,
        ),
        'max_retries': max((s.retries for s in navigation), default=0),
        'speed_limit_min_mps': min(limits) if limits else None,
        'speed_limit_max_mps': max(limits) if limits else None,
        'contact_events': contact_events,
        'footprint_frame': CHASSIS_FRAME if waist else BASE_FRAME,
    }
    # 観客（フィールドの外の人）は柵の向こうにいて機体と当たらないので、接触と最小距離から除く
    result.update(analyze_obstacles(samples, field_obstacles(spec.obstacles), warn_m))
    result['spectators'] = len(spec.obstacles) - len(field_obstacles(spec.obstacles))
    result.update(analyze_static(samples, world_path))
    result.update(analyze_structures(samples, world_path, spec.world, structure_outlines_path(map_path)))
    fence_touched = as_float(result['min_fence_clearance_m']) <= 0.0
    result['ok'] = (
        reached == len(goal_results)
        and as_float(result['own_fault_contacts']) == 0.0
        and as_float(result['static_contacts']) == 0.0
        and as_float(result['structure_contacts']) == 0.0
        and as_float(result['keep_out_entries']) == 0.0
        and not fence_touched
    )
    return result


def as_float(value: object) -> float:
    if isinstance(value, (int, float)):
        return float(value)
    raise CheckError(f'数値ではありません: {value}')


def format_optional(value: object, digits: int = 2) -> str:
    return f'{value:.{digits}f}' if isinstance(value, float) else str(value)


def print_summary(result: Mapping[str, object]) -> None:
    print(f"--- シナリオ {result['scenario']}（{result['world']}）", flush=True)
    print(f"ゴール到達: {result['goals_reached']}/{result['goals_total']}、走行時間 {format_optional(result['duration_s'], 1)}秒")
    goal_results = result['goal_results']
    if isinstance(goal_results, list):
        for goal in goal_results:
            print(
                f"  ({goal['x']:.1f}, {goal['y']:.1f}): {goal['final_state']}、"
                f"{goal['duration_s']:.1f}秒、最終距離 {goal['final_distance_m']:.2f}m"
            )
    print(
        f"相手との最小距離: {format_optional(result['min_gap_m'])}m、"
        f"{format_optional(result['min_distance_warn_m'])}m 未満の時間 {format_optional(result['warn_time_s'], 1)}秒"
    )
    print(
        f"接触: {result['contacts']} 回（自機が原因 {result['own_fault_contacts']}、"
        f"相手から {result['other_fault_contacts']}）、静止物 {result['static_contacts']} 回、"
        f"接触検知 {result['contact_events']} 回、"
        f"当たる速さ 平均 {format_optional(result['mean_impact_speed_mps'])}m/s・"
        f"{HARD_IMPACT_SPEED_MPS}m/s 以上 {result['hard_contacts']} 回"
    )
    print(
        f"速度: 最高 {format_optional(result['max_speed_mps'])}m/s、平均 {format_optional(result['mean_speed_mps'])}m/s、"
        f"柵までの最小距離 {format_optional(result['min_fence_clearance_m'])}m、"
        f"静止物までの最小距離 {format_optional(result['min_static_gap_m'])}m"
    )
    if result.get('min_structure_gap_m') is not None or result.get('keep_out_entries'):
        print(
            f"構造物: 最小距離 {format_optional(result['min_structure_gap_m'])}m、接触 {result['structure_contacts']} 回、"
            f"進入禁止ゾーン {result['keep_out_entries']} 回（{format_optional(result['keep_out_time_s'], 1)}秒）"
        )
    print(
        f"計算時間[ms]: 計画 平均 {format_optional(result['planning_ms_mean'])}、99%点 {format_optional(result['planning_ms_p99'])}、"
        f"検出 平均 {format_optional(result['perception_ms_mean'])}、99%点 {format_optional(result['perception_ms_p99'])}"
    )
    print(f"安全停止の状態[秒]: {result['safety_time_s']}")
    print(f"計画の状態[秒]: {result['navigation_time_s']}、復帰の試行 {result['max_retries']} 回", flush=True)


def load_test_config(path: Path | None) -> tuple[str, bool, float]:
    """scenario_test.yaml の値を読む."""
    config_path = path or (
        Path(get_package_share_directory('core_launch')) / 'config' / 'scenario_test.yaml'
    )
    try:
        with config_path.open(encoding='utf-8') as file:
            root = yaml.safe_load(file)
        values = root['/**']['ros__parameters']['scenario_test']
    except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
        raise CheckError(f'{config_path} を読めません: {error}') from error
    return (
        str(values.get('scenario_file', '')),
        bool(values.get('record_bag', False)),
        float(values.get('min_distance_warn_m', 0.3)),
    )


def start_bag(directory: Path) -> subprocess.Popen[bytes]:
    directory.parent.mkdir(parents=True, exist_ok=True)
    log_path = directory.with_suffix('.log')
    command = ['ros2', 'bag', 'record', '-o', str(directory), '--topics', *BAG_TOPICS]
    with log_path.open('wb') as log:
        return subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy())


def default_bag_directory(name: str) -> Path:
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    return Path.home() / '.ros' / 'core_nav_scenario_bags' / f'{name}_{stamp}'


def run_scenario(
    scenario_path: Path,
    json_path: Path | None = None,
    params: Sequence[str] = (),
    launch_args: Sequence[str] = (),
    no_launch: bool = False,
    record_bag: bool = False,
    bag_directory: Path | None = None,
    warn_m: float = 0.3,
    log_path: Path | None = None,
) -> ResultDict:
    """シナリオを 1 回実行して結果を返す."""
    spec = load_spec(scenario_path)
    launch_log = log_path or Path(tempfile.mkstemp(prefix=f'{spec.name}_', suffix='.launch.log')[1])
    print(f'launch のログ: {launch_log}', flush=True)
    process = None if no_launch else start_stack(spec, scenario_path, launch_args, launch_log)
    bag_process: subprocess.Popen[bytes] | None = None
    directory: Path | None = None
    rclpy.init()
    node = ScenarioNode()
    try:
        with SpinThread(node):
            wait_until_ready(node, process, launch_log)
            for item in params:
                target, name, value = parse_param_spec(item)
                outcome = node.set_parameter(target, name, value)
                if not outcome.successful:
                    raise CheckError(f'{item} が拒否されました: {outcome.reason}')
                print(f'パラメータ {item} を設定しました', flush=True)
            if record_bag:
                directory = bag_directory or default_bag_directory(spec.name)
                bag_process = start_bag(directory)
                print(f'記録先: {directory}', flush=True)
            # 自己位置が落ち着くまで待つ
            node.wait_sim(2.0)
            start = node.now_sim()
            goal_results: list[GoalResult] = []
            for _ in range(spec.repeat):
                deadline = node.now_sim() + spec.timeout_s
                for goal in spec.goals:
                    remaining = max(deadline - node.now_sim(), 1.0)
                    goal_results.append(drive_goal(node, goal, remaining, process))
                    print(f'ゴール {goal[:2]}: {goal_results[-1].final_state}', flush=True)
            end = node.now_sim()
            result = analyze_run(node, spec, goal_results, start, end, warn_m)
    finally:
        if bag_process is not None:
            stop_process(bag_process)
        if process is not None:
            stop_process(process)
        rclpy.try_shutdown()
    result['launch_log'] = str(launch_log)
    if directory is not None:
        result['bag_directory'] = str(directory)
    print_summary(result)
    if json_path is not None:
        json_path.write_text(json.dumps(result, ensure_ascii=False, indent=1), encoding='utf-8')
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario-file', type=Path, default=None)
    parser.add_argument('--json', type=Path, default=None, help='結果を書き出すファイル')
    parser.add_argument(
        '--param', action='append', default=[], metavar='NODE:NAME=VALUE',
        help='走行前に設定するパラメータ（複数指定可）',
    )
    parser.add_argument(
        '--launch-arg', action='append', default=[], metavar='NAME=VALUE',
        help='sim_nav.launch.py に追加で渡す引数（複数指定可）',
    )
    parser.add_argument('--no-launch', action='store_true', help='起動済みのシミュレーションを使う')
    parser.add_argument(
        '--record-bag', action=argparse.BooleanOptionalAction, default=None,
        help='rosbag に記録する（既定は scenario_test.yaml の値）',
    )
    parser.add_argument('--bag-dir', type=Path, default=None, help='rosbag の出力先')
    parser.add_argument('--min-distance-warn-m', type=float, default=None)
    parser.add_argument('--log-file', type=Path, default=None, help='launch のログの出力先')
    parser.add_argument('--config', type=Path, default=None, help='scenario_test.yaml の場所')
    args = parser.parse_args()

    try:
        config_scenario, config_bag, config_warn = load_test_config(args.config)
        scenario_text = str(args.scenario_file) if args.scenario_file else config_scenario
        if not scenario_text:
            raise CheckError('--scenario-file か scenario_test.scenario_file で指定してください')
        result = run_scenario(
            Path(scenario_text),
            json_path=args.json,
            params=args.param,
            launch_args=args.launch_arg,
            no_launch=args.no_launch,
            record_bag=config_bag if args.record_bag is None else args.record_bag,
            bag_directory=args.bag_dir,
            warn_m=config_warn if args.min_distance_warn_m is None else args.min_distance_warn_m,
            log_path=args.log_file,
        )
    except CheckError as error:
        print(f'[NG] シナリオを実行できませんでした: {error}', flush=True)
        return 1
    ok = bool(result['ok'])
    print('結果: 全ゴールに到達し、自機に原因のある接触はありません' if ok else '結果: 不合格です', flush=True)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
