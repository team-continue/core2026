#!/usr/bin/env bash
# 外部パッケージをワークスペースの src/thirdparty（core2026 の外）に取得し、Jazzy で動かすためのパッチを当てる。
# livox_ros_driver2 がワークスペースに無ければ、メッセージ定義だけのパッケージ（シミュレーション用）を置く。
# 何度実行してもよい。
set -euo pipefail

bringup_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# core_launch → core2026 → src
thirdparty_dir="$(cd "${bringup_dir}/../.." && pwd)/thirdparty"
fast_lio_dir="${thirdparty_dir}/FAST_LIO"
localization_dir="${thirdparty_dir}/lidar_localization_ros2"
mkdir -p "${thirdparty_dir}"
vcs import --recursive --input "${bringup_dir}/thirdparty/core_nav.repos" "${thirdparty_dir}"

src_dir="$(cd "${thirdparty_dir}/.." && pwd)"
# core_launch/thirdparty の雛形（COLCON_IGNORE 付き）は数えない
if ! grep -rqs --include=package.xml --exclude-dir=livox_ros_driver2_msgs '<name>livox_ros_driver2</name>' "${src_dir}"; then
  mkdir -p "${thirdparty_dir}/livox_ros_driver2"
  cp -r "${bringup_dir}/thirdparty/livox_ros_driver2_msgs/." "${thirdparty_dir}/livox_ros_driver2/"
  rm -f "${thirdparty_dir}/livox_ros_driver2/COLCON_IGNORE"
  echo "livox_ros_driver2: メッセージ定義だけのパッケージを置きました"
fi

# パッチは前のパッチの行を文脈に含むので、1 枚ずつでは適用済みかを判定できない。
# 作業ツリーがきれいならすべて順に当てる。変更があれば、きれいな取り出しに当てた結果と同じかを確かめる。
apply_patches() {
  local target_dir="$1"
  shift
  local target_name
  target_name="$(basename "${target_dir}")"
  if git -C "${target_dir}" diff --quiet; then
    for patch in "$@"; do
      git -C "${target_dir}" apply "${bringup_dir}/thirdparty/${patch}"
      echo "${target_name}: ${patch} を適用しました"
    done
    return
  fi
  local scratch
  scratch="$(mktemp -d)"
  git -C "${target_dir}" worktree add --quiet --detach "${scratch}/tree" HEAD
  for patch in "$@"; do
    git -C "${scratch}/tree" apply "${bringup_dir}/thirdparty/${patch}"
  done
  local expected actual
  expected="$(git -C "${scratch}/tree" diff)"
  actual="$(git -C "${target_dir}" diff)"
  git -C "${target_dir}" worktree remove --force "${scratch}/tree"
  rm -rf "${scratch}"
  if [[ "${expected}" == "${actual}" ]]; then
    echo "${target_name}: パッチは適用済みです"
  else
    echo "${target_name}: パッチと違う変更があります。git -C ${target_dir} diff を確かめてください" >&2
    return 1
  fi
}

# FAST-LIO（順に当てる）:
# - cxx17: FAST-LIO は C++14 固定だが、Jazzy の rclcpp のヘッダは C++17 を必要とする
# - odom_and_imu_queue: /Odometry に速度を出す（制御に使う）。処理が遅れても IMU を失わないよう受信の深さを増やす
# - self_filter: 自機の点を機体の箱（footprint.yaml の長さ・幅・上面と URDF の LiDAR の取り付け）で除く。値は launch が
#   preprocess.self_filter.* で渡す。地図のボクセルにある点を置き換えない設定（mapping.keep_existing_map_points）も足す
# - process_on_arrival: 点群と IMU がそろったら、10ms のタイマーを待たずにすぐ処理する
apply_patches "${fast_lio_dir}" \
  fast_lio_cxx17.patch \
  fast_lio_odom_and_imu_queue.patch \
  fast_lio_self_filter.patch \
  fast_lio_process_on_arrival.patch

# lidar_localization_ros2（順に当てる）:
# - lifecycle_retry: 起動時の通信の確立中に change_state の応答が失われると、60 秒待って起動に失敗する
# - degenerate_correction: 点群が水平のある向きを決められないときは、その向きの補正を捨てて LIO の姿勢のままにする
# - component: composition:=true で、地図照合を点群のフィルタなどと同じコンテナに入れられるよう component として登録する
# - robust_gate: 速いときや密集したときに自己位置が跳ばないようにする（どれも既定は切ってあり、lidar_localization.yaml で入れる）
apply_patches "${localization_dir}" \
  lidar_localization_lifecycle_retry.patch \
  lidar_localization_degenerate_correction.patch \
  lidar_localization_component.patch \
  lidar_localization_robust_gate.patch
