#include "core_planning/jps.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <queue>
#include <string>
#include <stdexcept>
#include <utility>

namespace core_planning
{
namespace
{
constexpr double kSqrt2 = 1.4142135623730951;

double octile(const GridCell & a, const GridCell & b)
{
  const int dx = std::abs(a.x - b.x);
  const int dy = std::abs(a.y - b.y);
  return static_cast<double>(std::max(dx, dy)) +
         (kSqrt2 - 1.0) * static_cast<double>(std::min(dx, dy));
}

int sign(const int value)
{
  return (value > 0) - (value < 0);
}

// (x, y) から (dx, dy) 方向へ進めるか。斜めは両隣が空いているときだけ通る
bool canMove(const OccupancyGrid & grid, const int x, const int y, const int dx, const int dy)
{
  if (grid.isOccupied(GridCell{x + dx, y + dy})) {
    return false;
  }
  if (dx != 0 && dy != 0) {
    return grid.isFree(GridCell{x + dx, y}) && grid.isFree(GridCell{x, y + dy});
  }
  return true;
}

// 直線方向の移動で、(x, y) に必ず立ち寄る必要があるか。
// 角を横切れないので、後ろの側面が塞がっていて真横が空いていれば曲がれるようになる
bool hasForcedNeighbor(
  const OccupancyGrid & grid, const int x, const int y, const int dx, const int dy)
{
  if (dx != 0) {
    return (grid.isOccupied(GridCell{x - dx, y + 1}) && grid.isFree(GridCell{x, y + 1})) ||
           (grid.isOccupied(GridCell{x - dx, y - 1}) && grid.isFree(GridCell{x, y - 1}));
  }
  return (grid.isOccupied(GridCell{x + 1, y - dy}) && grid.isFree(GridCell{x + 1, y})) ||
         (grid.isOccupied(GridCell{x - 1, y - dy}) && grid.isFree(GridCell{x - 1, y}));
}

// (x, y) から (dx, dy) 方向へ進み、次のジャンプポイントを返す
std::optional<GridCell> jump(
  const OccupancyGrid & grid, int x, int y, const int dx, const int dy, const GridCell & goal)
{
  while (true) {
    if (!canMove(grid, x, y, dx, dy)) {
      return std::nullopt;
    }
    x += dx;
    y += dy;
    const GridCell current{x, y};
    if (current == goal) {
      return current;
    }
    if (dx != 0 && dy != 0) {
      if (jump(grid, x, y, dx, 0, goal).has_value() || jump(grid, x, y, 0, dy, goal).has_value()) {
        return current;
      }
    } else if (hasForcedNeighbor(grid, x, y, dx, dy)) {
      return current;
    }
  }
}

// 親からの向きで、調べる方向を絞る
std::vector<std::pair<int, int>> pruneDirections(
  const OccupancyGrid & grid, const GridCell & cell, const std::optional<GridCell> & parent)
{
  if (!parent.has_value()) {
    return {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  }
  const int dx = sign(cell.x - parent->x);
  const int dy = sign(cell.y - parent->y);
  std::vector<std::pair<int, int>> directions;
  if (dx != 0 && dy != 0) {
    directions = {{dx, 0}, {0, dy}, {dx, dy}};
    return directions;
  }
  if (dx != 0) {
    directions.emplace_back(dx, 0);
    for (const int side : {1, -1}) {
      if (grid.isOccupied(GridCell{cell.x - dx, cell.y + side}) &&
        grid.isFree(GridCell{cell.x, cell.y + side}))
      {
        directions.emplace_back(0, side);
        directions.emplace_back(dx, side);
      }
    }
    return directions;
  }
  directions.emplace_back(0, dy);
  for (const int side : {1, -1}) {
    if (grid.isOccupied(GridCell{cell.x + side, cell.y - dy}) &&
      grid.isFree(GridCell{cell.x + side, cell.y}))
    {
      directions.emplace_back(side, 0);
      directions.emplace_back(side, dy);
    }
  }
  return directions;
}

struct QueueEntry
{
  double priority;
  int index;

  bool operator>(const QueueEntry & other) const {return priority > other.priority;}
};

using OpenList = std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>>;

double euclidean(const GridCell & a, const GridCell & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

// parent をたどって始点から goal_index までの列にする
std::vector<GridCell> tracePath(
  const GridInfo & info, const std::vector<int> & parent, const int goal_index)
{
  std::vector<GridCell> path;
  for (int index = goal_index; index != -1; index = parent[static_cast<std::size_t>(index)]) {
    path.push_back(GridCell{index % info.width, index / info.width});
  }
  std::reverse(path.begin(), path.end());
  return path;
}

// Theta* の距離の更新。current から next へ進むとき、current の見通しの親から next が見えれば
// 親から直線で進んだとする。更新したら true
bool relaxAnyAngle(
  const OccupancyGrid & grid, const GridInfo & info, const int current_index, const int next_index,
  std::vector<double> & cost, std::vector<int> & parent)
{
  const GridCell current{current_index % info.width, current_index / info.width};
  const GridCell next{next_index % info.width, next_index / info.width};
  const auto current_position = static_cast<std::size_t>(current_index);
  const auto next_position = static_cast<std::size_t>(next_index);
  int from_index = current_index;
  const int grand_index = parent[current_position];
  if (grand_index >= 0) {
    const GridCell grand{grand_index % info.width, grand_index / info.width};
    if (hasLineOfSight(grid, grand, next)) {
      from_index = grand_index;
    }
  }
  const GridCell from{from_index % info.width, from_index / info.width};
  const double tentative = cost[static_cast<std::size_t>(from_index)] + euclidean(from, next);
  if (tentative >= cost[next_position]) {
    return false;
  }
  cost[next_position] = tentative;
  parent[next_position] = from_index;
  return true;
}
}  // namespace

std::optional<std::vector<GridCell>> findPath(
  const OccupancyGrid & grid, const GridCell & start, const GridCell & goal)
{
  const GridInfo & info = grid.info();
  if (grid.isOccupied(start) || grid.isOccupied(goal)) {
    return std::nullopt;
  }
  if (start == goal) {
    return std::vector<GridCell>{start};
  }
  const auto size = static_cast<std::size_t>(info.width * info.height);
  std::vector<double> cost(size, std::numeric_limits<double>::infinity());
  std::vector<int> parent(size, -1);
  std::vector<std::uint8_t> closed(size, 0U);
  const auto cellOf = [&info](const int index) {
      return GridCell{index % info.width, index / info.width};
    };

  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;
  const int start_index = info.index(start);
  cost[static_cast<std::size_t>(start_index)] = 0.0;
  open.push(QueueEntry{octile(start, goal), start_index});
  const int goal_index = info.index(goal);

  while (!open.empty()) {
    const int current_index = open.top().index;
    open.pop();
    const auto current_position = static_cast<std::size_t>(current_index);
    if (closed[current_position] != 0U) {
      continue;
    }
    closed[current_position] = 1U;
    if (current_index == goal_index) {
      std::vector<GridCell> path;
      for (int index = goal_index; index != -1; index = parent[static_cast<std::size_t>(index)]) {
        path.push_back(cellOf(index));
      }
      std::reverse(path.begin(), path.end());
      return path;
    }
    const GridCell current = cellOf(current_index);
    const int parent_index = parent[current_position];
    const std::optional<GridCell> parent_cell =
      parent_index >= 0 ? std::optional<GridCell>(cellOf(parent_index)) : std::nullopt;
    for (const auto & [dx, dy] : pruneDirections(grid, current, parent_cell)) {
      const auto next = jump(grid, current.x, current.y, dx, dy, goal);
      if (!next.has_value()) {
        continue;
      }
      const int next_index = info.index(*next);
      const auto next_position = static_cast<std::size_t>(next_index);
      const double tentative = cost[current_position] + octile(current, *next);
      if (tentative < cost[next_position]) {
        cost[next_position] = tentative;
        parent[next_position] = current_index;
        open.push(QueueEntry{tentative + octile(*next, goal), next_index});
      }
    }
  }
  return std::nullopt;
}

bool hasLineOfSight(const OccupancyGrid & grid, const GridCell & from, const GridCell & to)
{
  // 線が通るセルをすべて調べる
  const int dx = std::abs(to.x - from.x);
  const int dy = std::abs(to.y - from.y);
  const int step_x = sign(to.x - from.x);
  const int step_y = sign(to.y - from.y);
  int x = from.x;
  int y = from.y;
  int error = dx - dy;
  for (int remaining = dx + dy; remaining > 0; --remaining) {
    if (grid.isOccupied(GridCell{x, y})) {
      return false;
    }
    if (error > 0) {
      x += step_x;
      error -= 2 * dy;
    } else if (error < 0) {
      y += step_y;
      error += 2 * dx;
    } else {
      // 格子点をちょうど通るので、両隣も確かめる
      if (grid.isOccupied(GridCell{x + step_x, y}) || grid.isOccupied(GridCell{x, y + step_y})) {
        return false;
      }
      x += step_x;
      y += step_y;
      error += 2 * dx - 2 * dy;
      --remaining;
    }
  }
  return grid.isFree(to);
}

std::vector<GridCell> simplifyPath(const OccupancyGrid & grid, const std::vector<GridCell> & path)
{
  if (path.size() < 3U) {
    return path;
  }
  std::vector<GridCell> simplified{path.front()};
  std::size_t anchor = 0U;
  while (anchor + 1U < path.size()) {
    // 見通しのある最も遠い点まで飛ぶ
    std::size_t next = path.size() - 1U;
    while (next > anchor + 1U && !hasLineOfSight(grid, path[anchor], path[next])) {
      --next;
    }
    simplified.push_back(path[next]);
    anchor = next;
  }
  return simplified;
}

std::vector<GridCell> expandPath(const std::vector<GridCell> & path)
{
  if (path.empty()) {
    return path;
  }
  std::vector<GridCell> cells{path.front()};
  for (std::size_t i = 1U; i < path.size(); ++i) {
    const GridCell & from = path[i - 1U];
    const GridCell & to = path[i];
    const int dx = to.x - from.x;
    const int dy = to.y - from.y;
    if (dx != 0 && dy != 0 && std::abs(dx) != std::abs(dy)) {
      throw std::invalid_argument("expandPath: 区間が 8 方向の直線ではありません");
    }
    const int steps = std::max(std::abs(dx), std::abs(dy));
    for (int k = 1; k <= steps; ++k) {
      cells.push_back(GridCell{from.x + sign(dx) * k, from.y + sign(dy) * k});
    }
  }
  return cells;
}

std::vector<GridCell> shortcutPath(const OccupancyGrid & grid, const std::vector<GridCell> & path)
{
  return simplifyPath(grid, expandPath(path));
}

std::optional<GridCell> nearestFreeCell(
  const OccupancyGrid & grid, const GridCell & cell, const int max_radius_cells)
{
  if (grid.isFree(cell)) {
    return cell;
  }
  std::optional<GridCell> best;
  double best_distance = std::numeric_limits<double>::infinity();
  for (int dy = -max_radius_cells; dy <= max_radius_cells; ++dy) {
    for (int dx = -max_radius_cells; dx <= max_radius_cells; ++dx) {
      const GridCell candidate{cell.x + dx, cell.y + dy};
      const double distance = std::hypot(dx, dy);
      if (distance < best_distance && distance <= max_radius_cells && grid.isFree(candidate)) {
        best = candidate;
        best_distance = distance;
      }
    }
  }
  return best;
}

std::optional<std::vector<GridCell>> findPathThetaStar(
  const OccupancyGrid & grid, const GridCell & start, const GridCell & goal)
{
  const GridInfo & info = grid.info();
  if (grid.isOccupied(start) || grid.isOccupied(goal)) {
    return std::nullopt;
  }
  if (start == goal) {
    return std::vector<GridCell>{start};
  }
  const auto size = static_cast<std::size_t>(info.width * info.height);
  std::vector<double> cost(size, std::numeric_limits<double>::infinity());
  std::vector<int> parent(size, -1);
  std::vector<std::uint8_t> closed(size, 0U);
  OpenList open;
  const int start_index = info.index(start);
  const int goal_index = info.index(goal);
  cost[static_cast<std::size_t>(start_index)] = 0.0;
  open.push(QueueEntry{euclidean(start, goal), start_index});
  constexpr std::array<std::pair<int, int>, 8> kNeighbors{{
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};

  while (!open.empty()) {
    const int current_index = open.top().index;
    open.pop();
    const auto current_position = static_cast<std::size_t>(current_index);
    if (closed[current_position] != 0U) {
      continue;
    }
    closed[current_position] = 1U;
    if (current_index == goal_index) {
      return tracePath(info, parent, goal_index);
    }
    const GridCell current{current_index % info.width, current_index / info.width};
    for (const auto & [dx, dy] : kNeighbors) {
      const GridCell next{current.x + dx, current.y + dy};
      if (!info.contains(next) || !canMove(grid, current.x, current.y, dx, dy)) {
        continue;
      }
      const int next_index = info.index(next);
      if (closed[static_cast<std::size_t>(next_index)] != 0U) {
        continue;
      }
      if (relaxAnyAngle(grid, info, current_index, next_index, cost, parent)) {
        open.push(
          QueueEntry{cost[static_cast<std::size_t>(next_index)] + euclidean(next, goal), next_index});
      }
    }
  }
  return std::nullopt;
}

std::optional<std::vector<GridCell>> findPathJpsTheta(
  const OccupancyGrid & grid, const GridCell & start, const GridCell & goal)
{
  const GridInfo & info = grid.info();
  if (grid.isOccupied(start) || grid.isOccupied(goal)) {
    return std::nullopt;
  }
  if (start == goal) {
    return std::vector<GridCell>{start};
  }
  const auto size = static_cast<std::size_t>(info.width * info.height);
  std::vector<double> cost(size, std::numeric_limits<double>::infinity());
  // 経路と距離に使う見通しの親
  std::vector<int> parent(size, -1);
  // 枝刈りの向きに使う JPS の親（ジャンプで来た元）
  std::vector<int> jump_parent(size, -1);
  std::vector<std::uint8_t> closed(size, 0U);
  OpenList open;
  const int start_index = info.index(start);
  const int goal_index = info.index(goal);
  cost[static_cast<std::size_t>(start_index)] = 0.0;
  open.push(QueueEntry{euclidean(start, goal), start_index});

  while (!open.empty()) {
    const int current_index = open.top().index;
    open.pop();
    const auto current_position = static_cast<std::size_t>(current_index);
    if (closed[current_position] != 0U) {
      continue;
    }
    closed[current_position] = 1U;
    if (current_index == goal_index) {
      return tracePath(info, parent, goal_index);
    }
    const GridCell current{current_index % info.width, current_index / info.width};
    const int jump_parent_index = jump_parent[current_position];
    const std::optional<GridCell> jump_parent_cell = jump_parent_index >= 0 ?
      std::optional<GridCell>(
      GridCell{jump_parent_index % info.width, jump_parent_index / info.width}) :
      std::nullopt;
    for (const auto & [dx, dy] : pruneDirections(grid, current, jump_parent_cell)) {
      const auto next = jump(grid, current.x, current.y, dx, dy, goal);
      if (!next.has_value()) {
        continue;
      }
      const int next_index = info.index(*next);
      if (closed[static_cast<std::size_t>(next_index)] != 0U) {
        continue;
      }
      if (relaxAnyAngle(grid, info, current_index, next_index, cost, parent)) {
        jump_parent[static_cast<std::size_t>(next_index)] = current_index;
        open.push(
          QueueEntry{
            cost[static_cast<std::size_t>(next_index)] + euclidean(*next, goal), next_index});
      }
    }
  }
  return std::nullopt;
}

SearchMethod parseSearchMethod(const std::string & name)
{
  if (name == "jps") {
    return SearchMethod::kJps;
  }
  if (name == "theta_star") {
    return SearchMethod::kThetaStar;
  }
  if (name == "jps_theta") {
    return SearchMethod::kJpsTheta;
  }
  if (name == "hybrid_astar") {
    return SearchMethod::kHybridAStar;
  }
  throw std::invalid_argument(
          "探索の方法は jps / theta_star / jps_theta / hybrid_astar のどれかにしてください: " + name);
}

std::optional<std::vector<GridCell>> searchPath(
  const OccupancyGrid & grid, const GridCell & start, const GridCell & goal,
  const SearchMethod method, const bool any_angle_shortcut)
{
  switch (method) {
    case SearchMethod::kJps:
    case SearchMethod::kHybridAStar: {
        const auto path = findPath(grid, start, goal);
        if (!path.has_value()) {
          return std::nullopt;
        }
        return any_angle_shortcut ? shortcutPath(grid, *path) : simplifyPath(grid, *path);
      }
    case SearchMethod::kThetaStar:
    case SearchMethod::kJpsTheta: {
        const auto path = method == SearchMethod::kThetaStar ?
          findPathThetaStar(grid, start, goal) : findPathJpsTheta(grid, start, goal);
        if (!path.has_value()) {
          return std::nullopt;
        }
        return simplifyPath(grid, *path);
      }
  }
  throw std::invalid_argument("searchPath: 不明な探索の方法です");
}

double pathLength(const std::vector<GridCell> & path)
{
  double length = 0.0;
  for (std::size_t i = 1U; i < path.size(); ++i) {
    length += std::hypot(path[i].x - path[i - 1U].x, path[i].y - path[i - 1U].y);
  }
  return length;
}
}  // namespace core_planning
