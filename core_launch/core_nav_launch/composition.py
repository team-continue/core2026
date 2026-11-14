"""ノードを別々のプロセスで起動するか、コンテナ（1 つのプロセス）にまとめて起動するかを切り替える.

composition:=true では、ノードを component としてコンテナに入れる。同じコンテナのノードの間のトピックは、
DDS を通さずにポインタで渡す（use_intra_process_comms）。コンテナは component_container_isolated で、
ノードごとに別のスレッド（シングルスレッドの executor）で動くので、重い処理が同じコンテナのほかのノードを
待たせない。1 つのコンテナに負荷が偏らないよう、コンテナは処理の段ごとに分ける。
- lidar_container: 点群の入力（シミュレーションでは points_to_livox）と検出（perception_node）
- localization_container: LIO の出力の変換、地図照合の入力のフィルタ、地図照合、自己位置の異常判定、
  （state_source:=ekf のとき）車輪の速度と角速度の合わせ方と EKF
- planning_container: グローバル計画、局所計画、接触の検知
FAST-LIO（component にできない）と安全停止（計画が止まっても機体を止める）は、いつも別のプロセスにする。

名前空間（/sensing・/localization・/perception・/planning）はコンテナとは別に、ノードごとに NodeSpec.namespace で
決める。1 つのコンテナに違う名前空間のノードが入ってよい。コンテナの名前は直下に置く（/lidar_container など）。
"""

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Final

from launch import LaunchDescriptionEntity
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

COMPOSITION_MODES: Final = ('true', 'false')
# Gazebo の crowd_goals（各 6 分、スタックを 4 コアに固定、別々のプロセスと交互に 2 回ずつ）で、CPU の合計は
# 92〜93% から 78% に、使用メモリの合計は 約740MiB から 約500MiB に減った。ゴール、接触、自己位置の誤差、
# 計画と検出の計算時間は変わらなかった（2026-11-11）
DEFAULT_COMPOSITION: Final = 'true'

# 名前空間（設計書 3 節）
SENSING_NS: Final = 'sensing'
LOCALIZATION_NS: Final = 'localization'
PERCEPTION_NS: Final = 'perception'
PLANNING_NS: Final = 'planning'

# FAST-LIO は絶対名でトピックを出すので、名前空間に入れても動かない。/localization の下へ移す。
# /path は地図照合の /localization/path とぶつけないよう、lio/path にする
FAST_LIO_REMAPPINGS: Final = (
    ('/Odometry', '/localization/Odometry'),
    ('/cloud_registered', '/localization/cloud_registered'),
    ('/cloud_registered_body', '/localization/cloud_registered_body'),
    ('/cloud_effected', '/localization/cloud_effected'),
    ('/Laser_map', '/localization/Laser_map'),
    ('/path', '/localization/lio/path'),
)

LIDAR_CONTAINER: Final = 'lidar_container'
LOCALIZATION_CONTAINER: Final = 'localization_container'
PLANNING_CONTAINER: Final = 'planning_container'

# composition:=true で、どのノード（別のプロセスで動かすときの実行ファイル）がどのコンテナで動くか。
# 確認のスクリプトが、ノードの CPU をコンテナのプロセスで測るのに使う
CONTAINER_OF_EXECUTABLE: Final = {
    'points_to_livox': LIDAR_CONTAINER,
    'perception_node': LIDAR_CONTAINER,
    'lio_odom_adapter': LOCALIZATION_CONTAINER,
    'scan_filter': LOCALIZATION_CONTAINER,
    'lidar_localization_node': LOCALIZATION_CONTAINER,
    'localization_health': LOCALIZATION_CONTAINER,
    'gyro_odometer': LOCALIZATION_CONTAINER,
    'ekf_localizer': LOCALIZATION_CONTAINER,
    'global_planner': PLANNING_CONTAINER,
    'mppi_lite': PLANNING_CONTAINER,
    'contact_monitor': PLANNING_CONTAINER,
}

# 同じプロセスのノードの間は、DDS を通さずにポインタで渡す
_INTRA_PROCESS: Final = {'use_intra_process_comms': True}


@dataclass(frozen=True)
class NodeSpec:
    """別のプロセスでもコンテナの中でも、同じ名前・パラメータ・リマップで起動するノード.

    name は省けない。コンテナに渡すパラメータファイル（/** の設定）は、ノード名がないと当てはまらない。
    """

    package: str
    executable: str
    plugin: str
    name: str
    parameters: tuple[object, ...] = ()
    remappings: tuple[tuple[str, str], ...] = ()
    # ノードを置く名前空間（先頭の / は付けない。例: 'localization'）。空なら直下
    namespace: str = ''


def parse_composition(value: str) -> bool:
    """launch の引数 composition を読む."""
    if value not in COMPOSITION_MODES:
        raise RuntimeError(f'composition は true か false を指定してください: {value}')
    return value == 'true'


def standalone_node(spec: NodeSpec) -> Node:
    """ノードを別のプロセスで起動する."""
    return Node(
        package=spec.package,
        executable=spec.executable,
        name=spec.name,
        namespace=spec.namespace,
        output='screen',
        parameters=list(spec.parameters),
        remappings=list(spec.remappings),
    )


def composable_node(spec: NodeSpec) -> ComposableNode:
    """コンテナに入れるノード."""
    return ComposableNode(
        package=spec.package,
        plugin=spec.plugin,
        name=spec.name,
        namespace=spec.namespace,
        parameters=list(spec.parameters),
        remappings=list(spec.remappings),
        extra_arguments=[_INTRA_PROCESS],
    )


def container(name: str, specs: Sequence[NodeSpec]) -> ComposableNodeContainer:
    """ノードを入れたコンテナを起動する."""
    return ComposableNodeContainer(
        name=name,
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        output='screen',
        composable_node_descriptions=[composable_node(spec) for spec in specs],
    )


def load_into(container_name: str, specs: Sequence[NodeSpec]) -> LoadComposableNodes:
    """ほかの launch が起動したコンテナにノードを入れる。コンテナが起動するまで待つ."""
    return LoadComposableNodes(
        target_container=f'/{container_name.lstrip("/")}',
        composable_node_descriptions=[composable_node(spec) for spec in specs],
    )


def launch_nodes(
    composition: bool, container_name: str, specs: Sequence[NodeSpec],
) -> list[LaunchDescriptionEntity]:
    """composition なら 1 つのコンテナに、そうでなければノードごとに別のプロセスで起動する."""
    if not composition:
        return [standalone_node(spec) for spec in specs]
    return [container(container_name, specs)]
