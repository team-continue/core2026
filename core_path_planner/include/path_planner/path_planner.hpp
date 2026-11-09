#pragma once

#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <optional>
#include <vector>

namespace path_planner
{

class PathPlanner
{
public:
  struct GridIndex
  {
    int x;
    int y;
  };

  struct Settings
  {
    int occupied_threshold{50};
    bool allow_unknown{false};
    bool use_diagonal{true};
    double cost_weight{0.0};
    // Margin (in cells) added around the start-goal bounding box for the
    // first search attempt; falls back to a full-map search only when the
    // windowed attempt finds NO path, so a path found inside the window
    // may be costlier than the global optimum (speed/optimality
    // trade-off). Negative disables windowing (always globally optimal).
    int search_window_margin{-1};
  };

  enum class Status
  {
    kOk,
    kStartOrGoalOutOfBounds,
    kStartOrGoalOccupied,
    kNoPath
  };

  struct PlanResult
  {
    Status status{Status::kNoPath};
    std::vector<GridIndex> path;
  };

  explicit PathPlanner(const Settings & settings);

  void setSettings(const Settings & settings);
  void setLocalCostmap(const nav_msgs::msg::OccupancyGrid & local_costmap);

  PlanResult plan(
    const nav_msgs::msg::OccupancyGrid & map,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  void gridToWorld(
    const nav_msgs::msg::OccupancyGrid & map,
    const GridIndex & cell, double & wx, double & wy) const;

private:
  struct OpenItem
  {
    double f;
    int index;
  };

  struct OpenItemGreater
  {
    bool operator()(const OpenItem & a, const OpenItem & b) const
    {
      return a.f > b.f;
    }
  };

  struct Window
  {
    int x0;
    int y0;
    int x1;
    int y1;
  };

  bool computePath(
    const nav_msgs::msg::OccupancyGrid & map,
    const GridIndex & start, const GridIndex & goal,
    std::vector<GridIndex> & out_path);
  bool searchWindow(
    const nav_msgs::msg::OccupancyGrid & map,
    const GridIndex & start, const GridIndex & goal,
    const Window & window, std::vector<GridIndex> & out_path);
  void reconstructPath(
    const nav_msgs::msg::OccupancyGrid & map,
    const std::vector<int> & came_from, int goal_index,
    std::vector<GridIndex> & out_path) const;
  double heuristic(const GridIndex & a, const GridIndex & b) const;
  bool worldToGrid(
    const nav_msgs::msg::OccupancyGrid & map, double wx,
    double wy, GridIndex & cell) const;

  // Precomputes a merged (max of global and local) value patch over the
  // region of the global grid covered by the local costmap, so the search
  // loop never does world<->grid transforms per neighbor.
  void buildLocalPatch(const nav_msgs::msg::OccupancyGrid & map);
  int8_t cellValue(
    const nav_msgs::msg::OccupancyGrid & map, int index, int x, int y) const;
  bool isBlocked(int8_t value) const;

  // Advances the generation stamp so scratch buffers become logically
  // clear in O(1); buffers are only re-filled when the map size changes
  // or the 32-bit stamp wraps.
  void beginSearchEpoch(int map_size);

  Settings settings_;
  std::optional<nav_msgs::msg::OccupancyGrid> local_costmap_;

  // Per-plan scratch, reused across calls. A cell's g_score_/came_from_
  // (or closed state) is valid only when its stamp equals generation_.
  std::vector<double> g_score_;
  std::vector<int> came_from_;
  std::vector<uint32_t> visit_gen_;
  std::vector<uint32_t> closed_gen_;
  uint32_t generation_{0};
  std::vector<OpenItem> open_heap_;

  // Merged local-costmap patch, in global grid coordinates.
  bool patch_active_{false};
  int patch_x0_{0};
  int patch_y0_{0};
  int patch_x1_{-1};
  int patch_y1_{-1};
  int patch_width_{0};
  std::vector<int8_t> patch_values_;

  static constexpr double kDiagonalCost = 1.41421356237;
};

}  // namespace path_planner
