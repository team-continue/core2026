// 実機の Mid-360 の 1 走査の時間の広がりを、Gazebo の一度に撮った点群に付ける計算。ROS に依存しない。
//
// 実機（2026-03 の bag）では、1 走査（100ms）の中で点の時刻が散らばり、方位角を約 18 周する（1 周は約 5.5ms）。
// 走っている間はその間の機体の動きで点がずれ、FAST-LIO と検出が点ごとの時刻で補正する。Gazebo の gpu_lidar は
// 全点を同じ時刻に撮るので、点ごとに時刻を振り、その時刻の LiDAR の位置から見えたはずの位置に点を移す。
// LiDAR の位置は真値の姿勢の記録（200Hz）を補間して求めるので、加速している間の動きもそのまま入る。
// 動いている相手の、走査の間の動きは扱わない。自機の車体に当たった点は動かさない（BodyBox）。

#ifndef CORE_GAZEBO__SCAN_MOTION_HPP_
#define CORE_GAZEBO__SCAN_MOTION_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Geometry>

namespace core_gazebo
{

struct StampedPose
{
  std::uint64_t stamp_ns;
  Eigen::Isometry3d pose;
};

// 方位角 azimuth_rad（LiDAR の座標系、-π〜π）の点を、何周目（turn、0〜turns_per_scan-1）に撮ったかから、
// 走査の始めからの時刻 [s] を出す。方位角は時刻とともに増える（実機と同じ向き）。結果は [0, scan_period_s)
inline double sweepTime(
  const double azimuth_rad, const int turn, const int turns_per_scan, const double scan_period_s)
{
  double phase = (azimuth_rad + M_PI) / (2.0 * M_PI);
  phase -= std::floor(phase);
  return (static_cast<double>(turn) + phase) / static_cast<double>(turns_per_scan) * scan_period_s;
}

// 時刻の順に並んだ姿勢 poses の、時刻 stamp_ns の姿勢（位置は線形、向きは球面線形の補間）。
// 範囲の外なら nullopt
inline std::optional<Eigen::Isometry3d> interpolatePose(
  const std::vector<StampedPose> & poses, const std::uint64_t stamp_ns)
{
  if (poses.empty() || stamp_ns < poses.front().stamp_ns || stamp_ns > poses.back().stamp_ns) {
    return std::nullopt;
  }
  const auto upper = std::lower_bound(
    poses.begin(), poses.end(), stamp_ns,
    [](const StampedPose & pose, const std::uint64_t t) {return pose.stamp_ns < t;});
  if (upper->stamp_ns == stamp_ns || upper == poses.begin()) {
    return upper->pose;
  }
  const auto lower = std::prev(upper);
  const double ratio = static_cast<double>(stamp_ns - lower->stamp_ns) /
    static_cast<double>(upper->stamp_ns - lower->stamp_ns);
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() =
    lower->pose.translation() + ratio * (upper->pose.translation() - lower->pose.translation());
  pose.linear() = Eigen::Quaterniond(lower->pose.rotation())
    .slerp(ratio, Eigen::Quaterniond(upper->pose.rotation())).toRotationMatrix();
  return pose;
}

// 自機の車体の箱（base_link で、床に投影した機体中心からの前後・左右の半分と、床からの高さ）。
// 車体に当たった点は LiDAR と一緒に動くので、走査の間の動きでずらさない
struct BodyBox
{
  double half_length_m{0.4};
  double half_width_m{0.4};
  double height_m{0.8};
  // Gazebo の点の雑音（標準偏差 2cm）の分の余裕
  double margin_m{0.05};
};

inline bool insideBody(const Eigen::Vector3d & point_in_base, const BodyBox & box)
{
  return std::abs(point_in_base.x()) <= box.half_length_m + box.margin_m &&
         std::abs(point_in_base.y()) <= box.half_width_m + box.margin_m &&
         point_in_base.z() >= -box.margin_m && point_in_base.z() <= box.height_m + box.margin_m;
}

// 撮った時刻の LiDAR（世界から見た姿勢 world_from_sensor_capture）の座標系の点 point が、別の時刻の LiDAR
// （world_from_sensor_at）からはどこに見えるか。止まっている物の点なので、世界での位置は変わらない
inline Eigen::Vector3d pointSeenAt(
  const Eigen::Vector3d & point, const Eigen::Isometry3d & world_from_sensor_capture,
  const Eigen::Isometry3d & world_from_sensor_at)
{
  return world_from_sensor_at.inverse() * (world_from_sensor_capture * point);
}

}  // namespace core_gazebo

#endif  // CORE_GAZEBO__SCAN_MOTION_HPP_
