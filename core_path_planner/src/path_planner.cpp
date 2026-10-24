#include "path_planner/path_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

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
  const geometry_msgs::msg::PoseStamped & goal)
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

  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  if (map.data.size() <
    static_cast<size_t>(width) * static_cast<size_t>(height))
  {
    // Malformed grid: treat like the fully occupied map it would have
    // been under per-cell bounds checking.
    return PlanResult{Status::kStartOrGoalOccupied, {}};
  }

  buildLocalPatch(map);

  const int start_index = start_cell.y * width + start_cell.x;
  const int goal_index = goal_cell.y * width + goal_cell.x;
  if (isBlocked(cellValue(map, start_index, start_cell.x, start_cell.y)) ||
    isBlocked(cellValue(map, goal_index, goal_cell.x, goal_cell.y)))
  {
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
  std::vector<GridIndex> & out_path)
{
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);

  const int margin = settings_.search_window_margin;
  if (margin >= 0) {
    const Window window{
      std::max(0, std::min(start.x, goal.x) - margin),
      std::max(0, std::min(start.y, goal.y) - margin),
      std::min(width - 1, std::max(start.x, goal.x) + margin),
      std::min(height - 1, std::max(start.y, goal.y) + margin)};
    const bool covers_full_map = window.x0 == 0 && window.y0 == 0 &&
      window.x1 == width - 1 && window.y1 == height - 1;
    if (!covers_full_map &&
      searchWindow(map, start, goal, window, out_path))
    {
      return true;
    }
    // Windowed attempt failed: fall back to the full map to preserve
    // completeness.
  }

  const Window full_map{0, 0, width - 1, height - 1};
  return searchWindow(map, start, goal, full_map, out_path);
}

bool PathPlanner::searchWindow(
  const nav_msgs::msg::OccupancyGrid & map,
  const GridIndex & start, const GridIndex & goal,
  const Window & window, std::vector<GridIndex> & out_path)
{
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  beginSearchEpoch(width * height);

  auto indexOf = [width](int x, int y) {return y * width + x;};
  auto toGrid = [width](int index) -> GridIndex {
      return GridIndex{index % width, index / width};
    };

  const int start_index = indexOf(start.x, start.y);
  const int goal_index = indexOf(goal.x, goal.y);
  g_score_[start_index] = 0.0;
  came_from_[start_index] = -1;
  visit_gen_[start_index] = generation_;
  open_heap_.clear();
  open_heap_.push_back(OpenItem{heuristic(start, goal), start_index});

  static constexpr GridIndex kDiagonalOffsets[8] =
  {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}};
  static constexpr GridIndex kCardinalOffsets[4] =
  {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  const GridIndex * neighbors =
    settings_.use_diagonal ? kDiagonalOffsets : kCardinalOffsets;
  const int neighbor_count = settings_.use_diagonal ? 8 : 4;

  while (!open_heap_.empty()) {
    std::pop_heap(open_heap_.begin(), open_heap_.end(), OpenItemGreater{});
    const OpenItem current = open_heap_.back();
    open_heap_.pop_back();
    if (closed_gen_[current.index] == generation_) {
      continue;
    }
    if (current.index == goal_index) {
      reconstructPath(map, came_from_, goal_index, out_path);
      return true;
    }
    closed_gen_[current.index] = generation_;

    const GridIndex current_cell = toGrid(current.index);
    const double current_g = g_score_[current.index];
    for (int i = 0; i < neighbor_count; ++i) {
      const GridIndex & offset = neighbors[i];
      const GridIndex next{current_cell.x + offset.x,
        current_cell.y + offset.y};
      if (next.x < window.x0 || next.y < window.y0 || next.x > window.x1 ||
        next.y > window.y1)
      {
        continue;
      }
      const int next_index = indexOf(next.x, next.y);
      if (closed_gen_[next_index] == generation_) {
        continue;
      }
      const int8_t cell_value = cellValue(map, next_index, next.x, next.y);
      if (isBlocked(cell_value)) {
        continue;
      }
      double step_cost = (offset.x != 0 && offset.y != 0) ?
        kDiagonalCost : 1.0;
      if (settings_.cost_weight > 0.0 && cell_value > 0) {
        step_cost +=
          settings_.cost_weight * (static_cast<double>(cell_value) / 100.0);
      }
      const double tentative_g = current_g + step_cost;
      if (visit_gen_[next_index] != generation_ ||
        tentative_g < g_score_[next_index])
      {
        visit_gen_[next_index] = generation_;
        came_from_[next_index] = current.index;
        g_score_[next_index] = tentative_g;
        open_heap_.push_back(
          OpenItem{tentative_g + heuristic(next, goal), next_index});
        std::push_heap(
          open_heap_.begin(), open_heap_.end(),
          OpenItemGreater{});
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
  const double dx = static_cast<double>(std::abs(a.x - b.x));
  const double dy = static_cast<double>(std::abs(a.y - b.y));
  if (!settings_.use_diagonal) {
    // Manhattan distance: tight lower bound for 4-connected moves.
    return dx + dy;
  }
  // Octile distance: tight lower bound for 8-connected moves with
  // diagonal cost kDiagonalCost.
  return std::max(dx, dy) + (kDiagonalCost - 1.0) * std::min(dx, dy);
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

void PathPlanner::buildLocalPatch(const nav_msgs::msg::OccupancyGrid & map)
{
  patch_active_ = false;
  if (!local_costmap_.has_value()) {
    return;
  }
  const auto & local = local_costmap_.value();

  // Bounding box of the local costmap on the global grid.
  const double resolution = map.info.resolution;
  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;
  const double local_x0 = local.info.origin.position.x;
  const double local_y0 = local.info.origin.position.y;
  const double local_x1 = local_x0 +
    static_cast<double>(local.info.width) * local.info.resolution;
  const double local_y1 = local_y0 +
    static_cast<double>(local.info.height) * local.info.resolution;

  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const int x0 = std::max(
    0, static_cast<int>(std::floor((local_x0 - origin_x) / resolution)));
  const int y0 = std::max(
    0, static_cast<int>(std::floor((local_y0 - origin_y) / resolution)));
  const int x1 = std::min(
    width - 1,
    static_cast<int>(std::floor((local_x1 - origin_x) / resolution)));
  const int y1 = std::min(
    height - 1,
    static_cast<int>(std::floor((local_y1 - origin_y) / resolution)));
  if (x0 > x1 || y0 > y1) {
    return;
  }

  patch_x0_ = x0;
  patch_y0_ = y0;
  patch_x1_ = x1;
  patch_y1_ = y1;
  patch_width_ = x1 - x0 + 1;
  patch_values_.resize(
    static_cast<size_t>(patch_width_) * static_cast<size_t>(y1 - y0 + 1));

  const int local_width = static_cast<int>(local.info.width);
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      const int8_t global_value = map.data[y * width + x];
      int8_t merged = global_value;
      double wx = 0.0;
      double wy = 0.0;
      gridToWorld(map, GridIndex{x, y}, wx, wy);
      GridIndex local_cell;
      if (worldToGrid(local, wx, wy, local_cell)) {
        const int local_index = local_cell.y * local_width + local_cell.x;
        if (local_index >= 0 &&
          local_index < static_cast<int>(local.data.size()))
        {
          const int8_t local_value = local.data[local_index];
          if (!settings_.allow_unknown &&
            (global_value < 0 || local_value < 0))
          {
            // Unknown in either grid blocks the cell; keep it marked
            // unknown so isBlocked() rejects it.
            merged = -1;
          } else {
            merged = std::max(global_value, local_value);
          }
        }
      }
      patch_values_[(y - y0) * patch_width_ + (x - x0)] = merged;
    }
  }
  patch_active_ = true;
}

int8_t PathPlanner::cellValue(
  const nav_msgs::msg::OccupancyGrid & map, int index, int x, int y) const
{
  if (patch_active_ && x >= patch_x0_ && x <= patch_x1_ && y >= patch_y0_ &&
    y <= patch_y1_)
  {
    return patch_values_[(y - patch_y0_) * patch_width_ + (x - patch_x0_)];
  }
  return map.data[index];
}

bool PathPlanner::isBlocked(int8_t value) const
{
  if (value < 0) {
    return !settings_.allow_unknown;
  }
  return value >= settings_.occupied_threshold;
}

void PathPlanner::beginSearchEpoch(int map_size)
{
  if (static_cast<int>(visit_gen_.size()) != map_size) {
    g_score_.assign(map_size, std::numeric_limits<double>::infinity());
    came_from_.assign(map_size, -1);
    visit_gen_.assign(map_size, 0);
    closed_gen_.assign(map_size, 0);
    generation_ = 0;
  }
  ++generation_;
  if (generation_ == 0) {
    // The 32-bit stamp wrapped: old stamps would alias the new epoch,
    // so reset them once.
    std::fill(visit_gen_.begin(), visit_gen_.end(), 0);
    std::fill(closed_gen_.begin(), closed_gen_.end(), 0);
    generation_ = 1;
  }
}

}  // namespace path_planner
