#include "path_planner/path_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace path_planner
{

PathPlanner::PathPlanner(const Settings & settings)
: settings_(settings) {}

void PathPlanner::setSettings(const Settings & settings)
{
  settings_ = settings;
}

void PathPlanner::setLocalCostmap(
  const nav_msgs::msg::OccupancyGrid & local_costmap)
{
  local_costmap_ = local_costmap;
}

PathPlanner::PlanResult PathPlanner::plan(
  const nav_msgs::msg::OccupancyGrid & map,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  GridIndex start_cell;
  GridIndex goal_cell;
  if (!worldToGrid(
      map, start.pose.position.x, start.pose.position.y,
      start_cell) ||
    !worldToGrid(
      map, goal.pose.position.x, goal.pose.position.y,
      goal_cell))
  {
    return PlanResult{Status::kStartOrGoalOutOfBounds, {}};
  }

  if (isOccupied(map, start_cell) || isOccupied(map, goal_cell)) {
    return PlanResult{Status::kStartOrGoalOccupied, {}};
  }

  std::vector<GridIndex> path_cells;
  if (!computePath(map, start_cell, goal_cell, path_cells)) {
    return PlanResult{Status::kNoPath, {}};
  }

  return PlanResult{Status::kOk, std::move(path_cells)};
}

bool PathPlanner::computePath(
  const nav_msgs::msg::OccupancyGrid & map,
  const GridIndex & start, const GridIndex & goal,
  std::vector<GridIndex> & out_path) const
{
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const int map_size = width * height;

  auto indexOf = [width](int x, int y) {return y * width + x;};
  auto toGrid = [width](int index) -> GridIndex {
      return GridIndex{index % width, index / width};
    };

  std::vector<double> g_score(map_size,
    std::numeric_limits<double>::infinity());
  std::vector<int> came_from(map_size, -1);
  std::vector<bool> closed(map_size, false);
  std::priority_queue<OpenItem> open;

  const int start_index = indexOf(start.x, start.y);
  const int goal_index = indexOf(goal.x, goal.y);
  g_score[start_index] = 0.0;
  open.push(OpenItem{heuristic(start, goal), 0.0, start_index});

  const std::vector<GridIndex> neighbors =
    settings_.use_diagonal ?
    std::vector<GridIndex>{{1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {-1, 1}, {1, -1}, {-1, -1}} :
  std::vector<GridIndex>{{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

  while (!open.empty()) {
    auto current = open.top();
    open.pop();
    if (closed[current.index]) {
      continue;
    }
    if (current.index == goal_index) {
      reconstructPath(map, came_from, goal_index, out_path);
      return true;
    }
    closed[current.index] = true;

    GridIndex current_cell = toGrid(current.index);
    for (const auto & offset : neighbors) {
      GridIndex next{current_cell.x + offset.x, current_cell.y + offset.y};
      if (next.x < 0 || next.y < 0 || next.x >= width || next.y >= height) {
        continue;
      }
      if (isOccupied(map, next)) {
        continue;
      }
      const int next_index = indexOf(next.x, next.y);
      if (closed[next_index]) {
        continue;
      }
      const double step_cost =
        (offset.x != 0 && offset.y != 0) ? kDiagonalCost : 1.0;
      const double tentative_g = g_score[current.index] + step_cost;
      if (tentative_g < g_score[next_index]) {
        came_from[next_index] = current.index;
        g_score[next_index] = tentative_g;
        const double f_score = tentative_g + heuristic(next, goal);
        open.push(OpenItem{f_score, tentative_g, next_index});
      }
    }
  }

  return false;
}

void PathPlanner::reconstructPath(
  const nav_msgs::msg::OccupancyGrid & map,
  const std::vector<int> & came_from,
  int goal_index,
  std::vector<GridIndex> & out_path) const
{
  const int width = static_cast<int>(map.info.width);
  auto toGrid = [width](int index) -> GridIndex {
      return GridIndex{index % width, index / width};
    };

  out_path.clear();
  int current = goal_index;
  while (current >= 0) {
    out_path.push_back(toGrid(current));
    current = came_from[current];
  }
  std::reverse(out_path.begin(), out_path.end());
}

double PathPlanner::heuristic(const GridIndex & a, const GridIndex & b) const
{
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  return std::hypot(dx, dy);
}

bool PathPlanner::worldToGrid(
  const nav_msgs::msg::OccupancyGrid & map,
  double wx, double wy, GridIndex & cell) const
{
  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;
  const double resolution = map.info.resolution;
  if (wx < origin_x || wy < origin_y) {
    return false;
  }
  const int mx = static_cast<int>(std::floor((wx - origin_x) / resolution));
  const int my = static_cast<int>(std::floor((wy - origin_y) / resolution));
  if (mx < 0 || my < 0 || mx >= static_cast<int>(map.info.width) ||
    my >= static_cast<int>(map.info.height))
  {
    return false;
  }
  cell = GridIndex{mx, my};
  return true;
}

void PathPlanner::gridToWorld(
  const nav_msgs::msg::OccupancyGrid & map,
  const GridIndex & cell, double & wx,
  double & wy) const
{
  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;
  const double resolution = map.info.resolution;
  wx = origin_x + (static_cast<double>(cell.x) + 0.5) * resolution;
  wy = origin_y + (static_cast<double>(cell.y) + 0.5) * resolution;
}

bool PathPlanner::isOccupied(
  const nav_msgs::msg::OccupancyGrid & map,
  const GridIndex & cell) const
{
  const int width = static_cast<int>(map.info.width);
  const int index = cell.y * width + cell.x;
  if (index < 0 || index >= static_cast<int>(map.data.size())) {
    return true;
  }
  const int8_t global_value = map.data.at(index);
  if (global_value < 0 && !settings_.allow_unknown) {
    return true;
  }
  if (global_value >= settings_.occupied_threshold) {
    return true;
  }

  if (local_costmap_.has_value()) {
    const auto & local_map = local_costmap_.value();
    double wx = 0.0;
    double wy = 0.0;
    gridToWorld(map, cell, wx, wy);
    GridIndex local_cell;
    if (worldToGrid(local_map, wx, wy, local_cell)) {
      const int local_index =
        local_cell.y * static_cast<int>(local_map.info.width) + local_cell.x;
      if (local_index >= 0 &&
        local_index < static_cast<int>(local_map.data.size()))
      {
        const int8_t local_value = local_map.data.at(local_index);
        if (local_value < 0 && !settings_.allow_unknown) {
          return true;
        }
        if (local_value >= settings_.occupied_threshold) {
          return true;
        }
      }
    }
  }

  return false;
}

}  // namespace path_planner
