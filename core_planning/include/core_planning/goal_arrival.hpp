// 局所計画の「着いた」の通知を、大域計画がゴールの到達として受けるかの判定。ROS に依存しない。
//
// ゴールが占有（膨らませた静止物・進入禁止ゾーン）の中にあると、大域計画は近くの空きへ移した所まで経路を
// 出す。局所計画はそこへも固定物の余裕で近づけないと、手前で着いたことにする。元のゴールからの距離だけで
// 確かめると、その位置はゴールから到達の半径の外になることがあり、到達にも失敗にもならずに止まり続けた
// （sim で、進入禁止ゾーンから 0.08m のゴールに 1.14m 手前で止まり、120 秒の時間切れ）。
// 出した経路の終わり（移したゴール）からの距離でも確かめる。

#ifndef CORE_PLANNING__GOAL_ARRIVAL_HPP_
#define CORE_PLANNING__GOAL_ARRIVAL_HPP_

#include <cmath>
#include <optional>

namespace core_planning
{
struct ArrivalPoint
{
  double x{0.0};
  double y{0.0};
};

// 機体 (robot_x, robot_y) が、目標（ゴールか代わりのゴール）か、出した経路の終わり（あれば）から
// radius 以内か
inline bool arrivedNearTarget(
  const double robot_x, const double robot_y, const ArrivalPoint & target,
  const std::optional<ArrivalPoint> & path_end, const double radius)
{
  const auto within = [&](const ArrivalPoint & point) {
      return std::hypot(robot_x - point.x, robot_y - point.y) <= radius;
    };
  return within(target) || (path_end.has_value() && within(*path_end));
}
}  // namespace core_planning

#endif  // CORE_PLANNING__GOAL_ARRIVAL_HPP_
