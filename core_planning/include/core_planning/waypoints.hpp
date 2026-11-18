// 通過点（ゴールの前に順に通る点）を扱う。ROS に依存しない。
//
// 経路は「機体 → 通過点 1 → … → ゴール」の区間ごとに作ってつなぐ。機体が次の通過点から決まった距離の中に
// 入ったら、その点は通過済みにして列から除く（計画し直しても、通過済みの点へは戻らない）。
// 構造物や進入禁止ゾーンに近すぎる通過点は、局所計画が余裕を取るので機体がその距離まで近づけない。
// 受けたときに、余裕のある一番近い所へ移す（withClearance）。

#ifndef CORE_PLANNING__WAYPOINTS_HPP_
#define CORE_PLANNING__WAYPOINTS_HPP_

#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace core_planning
{
struct Waypoint
{
  double x{0.0};
  double y{0.0};
};

// 先頭から順に、機体が radius 以内に入った通過点を除く。除いた数を返す。
// 順に通るので、先頭の点に入るまで後ろの点は見ない
inline std::size_t dropReachedWaypoints(
  std::vector<Waypoint> & waypoints, const double robot_x, const double robot_y, const double radius)
{
  std::size_t count = 0U;
  while (count < waypoints.size() &&
    std::hypot(waypoints[count].x - robot_x, waypoints[count].y - robot_y) <= radius)
  {
    ++count;
  }
  waypoints.erase(waypoints.begin(), waypoints.begin() + static_cast<std::ptrdiff_t>(count));
  return count;
}

// 静止物までの距離 distance(x, y) が clearance 以上の所のうち、point に一番近い所（search_radius の中を step の
// 格子で探す）。point がもう clearance 以上なら point のまま。見つからなければ nullopt
template<typename DistanceFn>
std::optional<Waypoint> withClearance(
  const Waypoint & point, const double clearance, const double search_radius, const double step,
  const DistanceFn & distance)
{
  if (distance(point.x, point.y) >= clearance) {
    return point;
  }
  std::optional<Waypoint> best;
  double best_squared = search_radius * search_radius;
  const int cells = static_cast<int>(std::ceil(search_radius / step));
  for (int i = -cells; i <= cells; ++i) {
    for (int j = -cells; j <= cells; ++j) {
      const double dx = i * step;
      const double dy = j * step;
      const double squared = dx * dx + dy * dy;
      if (squared > best_squared) {
        continue;
      }
      if (distance(point.x + dx, point.y + dy) >= clearance) {
        best_squared = squared;
        best = Waypoint{point.x + dx, point.y + dy};
      }
    }
  }
  return best;
}

// 機体から、残りの通過点を順に通ってゴールまでの直線の距離の和
inline double routeDistance(
  const double robot_x, const double robot_y, const std::vector<Waypoint> & waypoints,
  const double goal_x, const double goal_y)
{
  double distance = 0.0;
  double x = robot_x;
  double y = robot_y;
  for (const auto & waypoint : waypoints) {
    distance += std::hypot(waypoint.x - x, waypoint.y - y);
    x = waypoint.x;
    y = waypoint.y;
  }
  return distance + std::hypot(goal_x - x, goal_y - y);
}

// 区間ごとの点の列を 1 本につなぐ。前の区間の終点と次の区間の始点が同じ位置なら 1 つにする
inline std::vector<std::pair<double, double>> joinSegments(
  const std::vector<std::vector<std::pair<double, double>>> & segments)
{
  constexpr double kSamePointM = 1e-6;
  std::vector<std::pair<double, double>> points;
  for (const auto & segment : segments) {
    for (const auto & point : segment) {
      if (!points.empty() &&
        std::hypot(point.first - points.back().first, point.second - points.back().second) <
        kSamePointM)
      {
        continue;
      }
      points.push_back(point);
    }
  }
  return points;
}
}  // namespace core_planning

#endif  // CORE_PLANNING__WAYPOINTS_HPP_
