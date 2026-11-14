"""自己位置の出し方（launch 引数 state_source）.

- lio: FAST-LIO と地図照合（今まで）
- ekf: さらに、車輪の速度と IMU の角速度を合わせた速度（gyro_odometer）と、地図照合の姿勢（/pcl_pose）を
  EKF（ekf_localizer）で合わせ、/localization/kinematic_state に出す（Autoware の gyro_odometer と ekf_localizer と
  同じ分け方）。角速度は 2 つの IMU の融合（/imu/fused）を使うので、imu_source が fused でなくても融合のノードを
  起動する。シミュレーションでは、車輪の速度（/vehicle/twist）を wheel_odometry_sim が出す。
  地図照合の初期値はいままでどおり LIO から作り、制御（MPPI と安全停止）もまだ LIO の姿勢を使う
"""

from pathlib import Path
from typing import Final

from core_nav_launch.composition import LOCALIZATION_NS, NodeSpec

STATE_SOURCES: Final = ('lio', 'ekf')
DEFAULT_STATE_SOURCE: Final = 'lio'
EKF: Final = 'ekf'


def parse_state_source(value: str) -> bool:
    """launch の引数 state_source を読み、EKF を起動するかを返す."""
    if value not in STATE_SOURCES:
        raise RuntimeError(f'state_source は {" / ".join(STATE_SOURCES)} のどれかにしてください: {value}')
    return value == EKF


def ekf_node_specs(config_dir: Path, imu_topic: str, use_sim_time: bool) -> list[NodeSpec]:
    """車輪の速度と角速度を合わせるノードと、EKF のノード."""
    return [
        NodeSpec(
            package='core_localization',
            executable='gyro_odometer',
            plugin='core_localization::GyroOdometerNode',
            name='gyro_odometer',
            namespace=LOCALIZATION_NS,
            parameters=(
                str(config_dir / 'gyro_odometer.yaml'),
                {'use_sim_time': use_sim_time, 'imu_topic': imu_topic},
            ),
        ),
        NodeSpec(
            package='core_localization',
            executable='ekf_localizer',
            plugin='core_localization::EkfLocalizerNode',
            name='ekf_localizer',
            namespace=LOCALIZATION_NS,
            parameters=(str(config_dir / 'ekf_localizer.yaml'), {'use_sim_time': use_sim_time}),
        ),
    ]
