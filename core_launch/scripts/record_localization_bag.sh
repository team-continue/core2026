#!/usr/bin/env bash
# 自己位置を後から再生して調べるための rosbag を録る（実機でも、シミュレーションでも使う）。
#
# 入力（LiDAR、IMU、固定の TF）は必ず録る。再生（replay_localization.py）では、これだけで FAST-LIO と
# 地図照合を動かし直せる。--with-outputs を付けると、走っている間の自己位置の出力（TF、/lio/odom、照合の状態、
# 自己位置の状態、追跡）も録る。走ったときの指標をそのまま出したり、再生の結果と比べたりできる。
#
# 使い方: record_localization_bag.sh <出力のディレクトリ> [--with-outputs] [ros2 bag record の引数 ...]
# シミュレーションで録るときは --use-sim-time を付ける。付けないと受けた時刻が実時間で録られ、再生の /clock が
# ヘッダの時刻（シミュレーションの時刻）と合わない。
# 止めるときは Ctrl-C（ros2 bag record が SIGINT で書き終える）。
set -euo pipefail
# トピックは replay_localization.py の INPUT_TOPICS と、check_localization_jumps.py が読むものに合わせる

if [[ $# -lt 1 ]]; then
  echo "使い方: $0 <出力のディレクトリ> [--with-outputs] [ros2 bag record の引数 ...]" >&2
  exit 2
fi
output="$1"
shift
topics=(
  /sensing/livox/lidar  # Livox の点群（CustomMsg、10Hz）
  /sensing/livox/imu    # Mid-360 の内蔵 IMU（200Hz、FAST-LIO の既定の入力）
  /sensing/imu          # Damiao IMU（200Hz、imu_source:=fused と、異常の判定に使う）
  /tf_static            # 機体の固定の変換（robot_state_publisher）
  /control/cmd_vel      # 速度指令（速さや旋回の場面を探すため）
)
if [[ "${1:-}" == --with-outputs ]]; then
  shift
  topics+=(
    /tf                     # map→odom と odom→base_link
    /localization/lio/odom  # LIO の姿勢と速度
    /localization/alignment_status  # 地図照合の採否と補正の大きさ
    /localization/health    # 自己位置の状態
    /perception/tracks      # 追跡（混雑の判定に使う）
  )
fi
if [[ -e "${output}" ]]; then
  echo "出力のディレクトリがすでにあります: ${output}" >&2
  exit 1
fi
# ros2 bag record は、各トピックを出しているノードの QoS（点群は Reliable）に合わせて受ける
exec ros2 bag record --output "${output}" --storage mcap "${topics[@]}" "$@"
