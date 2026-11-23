// 接触の検知の判定（ROS に依存しない部分）。contact_monitor_node が使う。

#ifndef CORE_PLANNING__CONTACT_DETECTION_HPP_
#define CORE_PLANNING__CONTACT_DETECTION_HPP_

#include <algorithm>

namespace core_planning
{
// IMU の水平加速度を、衝突の衝撃として数えるか。
//
// 機体が止まっていて（実際の速さも指令の速さも min_speed_mps 未満）、止まったままの間の衝撃は数えない。
// 止まっている間に接触を検知しても、安全停止は止めるだけで何も変わらず、行き詰まりから下がって抜け出す動きを
// 遅らせるだけになる。止まっている間の IMU の振れ（Gazebo の駆動の揺れ。実機では射出の反動など）で、
// 接触していないのに安全停止が働いていた。min_speed_mps が 0 なら、速さによらず数える（前の動き）
inline bool countsAsImpact(
  const double horizontal_accel_mps2, const double threshold_mps2, const double speed_mps,
  const double command_speed_mps, const double min_speed_mps)
{
  if (!(horizontal_accel_mps2 > threshold_mps2)) {
    return false;
  }
  return !(min_speed_mps > 0.0) || std::max(speed_mps, command_speed_mps) >= min_speed_mps;
}
}  // namespace core_planning

#endif  // CORE_PLANNING__CONTACT_DETECTION_HPP_
