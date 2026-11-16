#include "core_planning/hybrid_astar.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

#include "core_planning/jps.hpp"

namespace core_planning
{
namespace
{
using Point = std::pair<double, double>;

constexpr double kSqrt2 = 1.4142135623730951;
// 円弧の上で占有を確かめる間隔 [m]（格子 0.1m の半分）
constexpr double kSampleM = 0.05;

double wrapAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// 向き heading・曲率 curvature で長さ length 進んだ姿勢
struct Pose
{
  double x;
  double y;
  double heading;
};

Pose advance(const Pose & pose, const double curvature, const double length)
{
  if (std::abs(curvature) < 1e-9) {
    return Pose{
      pose.x + length * std::cos(pose.heading), pose.y + length * std::sin(pose.heading),
      pose.heading};
  }
  const double heading = pose.heading + curvature * length;
  return Pose{
    pose.x + (std::sin(heading) - std::sin(pose.heading)) / curvature,
    pose.y - (std::cos(heading) - std::cos(pose.heading)) / curvature, wrapAngle(heading)};
}

// 円弧の途中の点（kSampleM おき、終点を含む）
std::vector<Point> arcSamples(const Pose & pose, const double curvature, const double length)
{
  const int count = std::max(1, static_cast<int>(std::ceil(length / kSampleM)));
  std::vector<Point> samples;
  samples.reserve(static_cast<std::size_t>(count));
  for (int k = 1; k <= count; ++k) {
    const Pose p = advance(pose, curvature, length * k / count);
    samples.emplace_back(p.x, p.y);
  }
  return samples;
}

bool freeAt(const OccupancyGrid & grid, const Point & point)
{
  return grid.isFree(grid.info().cellAt(point.first, point.second));
}

// ゴールのセルから 8 方向で各セルまでの最短距離 [m]（届かなければ無限大）
std::vector<double> distanceToGoal(const OccupancyGrid & grid, const GridCell & goal)
{
  const GridInfo & info = grid.info();
  const auto size = static_cast<std::size_t>(info.width * info.height);
  std::vector<double> distance(size, std::numeric_limits<double>::infinity());
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  distance[static_cast<std::size_t>(info.index(goal))] = 0.0;
  open.emplace(0.0, info.index(goal));
  constexpr std::array<std::array<int, 2>, 8> kSteps{{
    {{1, 0}}, {{-1, 0}}, {{0, 1}}, {{0, -1}}, {{1, 1}}, {{1, -1}}, {{-1, 1}}, {{-1, -1}}}};
  while (!open.empty()) {
    const auto [d, index] = open.top();
    open.pop();
    if (d > distance[static_cast<std::size_t>(index)]) {
      continue;
    }
    const GridCell cell{index % info.width, index / info.width};
    for (const auto & step : kSteps) {
      const GridCell next{cell.x + step[0], cell.y + step[1]};
      if (!grid.isFree(next)) {
        continue;
      }
      const bool diagonal = step[0] != 0 && step[1] != 0;
      if (diagonal && (!grid.isFree(GridCell{cell.x + step[0], cell.y}) ||
        !grid.isFree(GridCell{cell.x, cell.y + step[1]})))
      {
        continue;
      }
      const double nd = d + (diagonal ? kSqrt2 : 1.0) * info.resolution;
      const auto position = static_cast<std::size_t>(info.index(next));
      if (nd < distance[position]) {
        distance[position] = nd;
        open.emplace(nd, info.index(next));
      }
    }
  }
  return distance;
}

struct Node
{
  Pose pose;
  double curvature;
  double cost;
  int parent;
};

struct QueueEntry
{
  double priority;
  int node;
  bool operator>(const QueueEntry & other) const {return priority > other.priority;}
};
}  // namespace

void validateHybridAStarParams(const HybridAStarParams & params)
{
  const auto require = [](const char * name, const double value, const double min, const double max) {
      if (!std::isfinite(value) || value < min || value > max) {
        std::ostringstream message;
        message << "Hybrid A* の " << name << " は " << min << "〜" << max << " にしてください: " << value;
        throw std::invalid_argument(message.str());
      }
    };
  require("step_m", params.step_m, 0.01, 5.0);
  require("min_radius_m", params.min_radius_m, 0.05, 100.0);
  require("bin_m", params.bin_m, 0.01, 5.0);
  require("heading_bins", params.heading_bins, 4, 720);
  require("curvature_weight", params.curvature_weight, 0.0, 100.0);
  require("curvature_change_weight", params.curvature_change_weight, 0.0, 100.0);
  require("shot_angle_deg", params.shot_angle_deg, 0.0, 180.0);
  require("max_expansions", params.max_expansions, 1, 1e8);
}

std::optional<std::vector<Point>> findPathHybridAStar(
  const OccupancyGrid & inflated, const Point & start, const Point & goal,
  const HybridAStarParams & params)
{
  validateHybridAStarParams(params);
  const GridInfo & info = inflated.info();
  const GridCell start_cell = info.cellAt(start.first, start.second);
  const GridCell goal_cell = info.cellAt(goal.first, goal.second);
  if (!inflated.isFree(start_cell) || !inflated.isFree(goal_cell)) {
    return std::nullopt;
  }
  const std::vector<double> heuristic = distanceToGoal(inflated, goal_cell);
  const auto heuristicAt = [&](const double x, const double y) {
      const GridCell cell = info.cellAt(x, y);
      return info.contains(cell) ? heuristic[static_cast<std::size_t>(info.index(cell))] :
             std::numeric_limits<double>::infinity();
    };
  if (std::isinf(heuristicAt(start.first, start.second))) {
    return std::nullopt;
  }

  const double max_curvature = 1.0 / params.min_radius_m;
  const std::array<double, 5> curvatures{
    0.0, 0.5 * max_curvature, -0.5 * max_curvature, max_curvature, -max_curvature};
  const double heading_step = 2.0 * M_PI / params.heading_bins;
  const auto keyOf = [&](const Pose & pose) {
      const auto ix = static_cast<std::int64_t>(std::floor((pose.x - info.origin_x) / params.bin_m));
      const auto iy = static_cast<std::int64_t>(std::floor((pose.y - info.origin_y) / params.bin_m));
      const auto ih = static_cast<std::int64_t>(
        std::floor(wrapAngle(pose.heading) / heading_step + params.heading_bins * 0.5 + 0.5)) %
        params.heading_bins;
      return (ix * 100000 + iy) * 1000 + ih;
    };

  std::vector<Node> nodes;
  std::unordered_map<std::int64_t, double> best_cost;
  std::unordered_map<std::int64_t, bool> closed;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;
  const double start_h = heuristicAt(start.first, start.second);
  for (int k = 0; k < params.heading_bins; ++k) {
    const Pose pose{start.first, start.second, wrapAngle(k * heading_step)};
    nodes.push_back(Node{pose, 0.0, 0.0, -1});
    open.push(QueueEntry{start_h, static_cast<int>(nodes.size()) - 1});
  }

  const double shot_angle = params.shot_angle_deg * M_PI / 180.0;
  const auto reconstruct = [&](int index, const bool add_goal) {
      std::vector<Point> reversed;
      if (add_goal) {
        reversed.push_back(goal);
      }
      while (index >= 0 && nodes[static_cast<std::size_t>(index)].parent >= 0) {
        const Node & node = nodes[static_cast<std::size_t>(index)];
        const Node & parent = nodes[static_cast<std::size_t>(node.parent)];
        auto samples = arcSamples(parent.pose, node.curvature, params.step_m);
        reversed.insert(reversed.end(), samples.rbegin(), samples.rend());
        index = node.parent;
      }
      reversed.push_back(start);
      std::reverse(reversed.begin(), reversed.end());
      // 重なった点を除く
      std::vector<Point> path;
      for (const Point & point : reversed) {
        if (path.empty() || std::hypot(point.first - path.back().first, point.second - path.back().second) > 1e-6) {
          path.push_back(point);
        }
      }
      return path;
    };

  int expansions = 0;
  while (!open.empty() && expansions < params.max_expansions) {
    const int index = open.top().node;
    open.pop();
    const Node current = nodes[static_cast<std::size_t>(index)];
    const std::int64_t key = keyOf(current.pose);
    if (closed[key]) {
      continue;
    }
    closed[key] = true;
    ++expansions;
    // ゴールへ直線でつなげれば終える
    const double to_goal = std::hypot(goal.first - current.pose.x, goal.second - current.pose.y);
    const GridCell cell = info.cellAt(current.pose.x, current.pose.y);
    if (to_goal < 1e-6) {
      return reconstruct(index, false);
    }
    const double goal_direction = std::atan2(goal.second - current.pose.y, goal.first - current.pose.x);
    if ((std::abs(wrapAngle(goal_direction - current.pose.heading)) <= shot_angle ||
      to_goal <= params.step_m) && hasLineOfSight(inflated, cell, goal_cell))
    {
      return reconstruct(index, true);
    }
    for (const double curvature : curvatures) {
      const auto samples = arcSamples(current.pose, curvature, params.step_m);
      if (!std::all_of(samples.begin(), samples.end(), [&](const Point & p) {return freeAt(inflated, p);})) {
        continue;
      }
      const Pose next = advance(current.pose, curvature, params.step_m);
      const double h = heuristicAt(next.x, next.y);
      if (std::isinf(h)) {
        continue;
      }
      const double cost = current.cost + params.step_m +
        params.curvature_weight * std::abs(curvature) * params.step_m +
        (current.parent >= 0 ? params.curvature_change_weight * std::abs(curvature - current.curvature) : 0.0);
      const std::int64_t next_key = keyOf(next);
      if (closed.count(next_key) != 0U && closed[next_key]) {
        continue;
      }
      const auto found = best_cost.find(next_key);
      if (found != best_cost.end() && found->second <= cost) {
        continue;
      }
      best_cost[next_key] = cost;
      nodes.push_back(Node{next, curvature, cost, index});
      open.push(QueueEntry{cost + h, static_cast<int>(nodes.size()) - 1});
    }
  }
  return std::nullopt;
}
}  // namespace core_planning
