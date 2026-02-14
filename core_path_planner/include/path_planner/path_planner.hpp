#pragma once

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
    const geometry_msgs::msg::PoseStamped & goal) const;

  void gridToWorld(
    const nav_msgs::msg::OccupancyGrid & map,
    const GridIndex & cell, double & wx, double & wy) const;

private:
  struct OpenItem
  {
    double f;
    double g;
    int index;
    bool operator<(const OpenItem & other) const {return f > other.f;}
  };

  bool computePath(
    const nav_msgs::msg::OccupancyGrid & map,
    const GridIndex & start, const GridIndex & goal,
    std::vector<GridIndex> & out_path) const;
  void reconstructPath(
    const nav_msgs::msg::OccupancyGrid & map,
    const std::vector<int> & came_from, int goal_index,
    std::vector<GridIndex> & out_path) const;
  double heuristic(const GridIndex & a, const GridIndex & b) const;
  bool worldToGrid(
    const nav_msgs::msg::OccupancyGrid & map, double wx,
    double wy, GridIndex & cell) const;
  bool isOccupied(
    const nav_msgs::msg::OccupancyGrid & map,
    const GridIndex & cell) const;

  Settings settings_;
  std::optional<nav_msgs::msg::OccupancyGrid> local_costmap_;

  static constexpr double kDiagonalCost = 1.41421356237;
};

}  // namespace path_planner
