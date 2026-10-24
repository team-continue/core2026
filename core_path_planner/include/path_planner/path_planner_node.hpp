#pragma once

#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "path_planner/path_planner.hpp"

namespace path_planner
{

class PathPlannerNode : public rclcpp::Node
{
public:
  explicit PathPlannerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onGlobalMapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onLocalCostmapReceived(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onStartPoseReceived(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Runs at replan_rate_hz and plans only when an input changed since the
  // last plan, decoupling planning cost from topic rates.
  void onReplanTimer();

  void tryPlan();

  // Transforms the cells with the CURRENT start pose and publishes; cheap
  // (no search), so it also runs on every start-pose message to keep the
  // local-frame path aligned with the moving robot between replans.
  void publishPath(const std::vector<PathPlanner::GridIndex> & path_cells);

  // True when the pose differs from the start pose used for the last plan
  // by more than the configured tolerances.
  bool startPoseChanged(const geometry_msgs::msg::PoseStamped & pose) const;

  // Extract yaw from quaternion
  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q) const;

  // Planner instance
  PathPlanner planner_;

  // Parameters
  std::string global_map_topic_;
  std::string local_costmap_topic_;
  std::string start_topic_;
  std::string goal_topic_;
  std::string path_topic_;
  std::string local_frame_id_;
  bool publish_in_global_frame_;
  std::string global_frame_id_;
  int occupied_threshold_;
  bool allow_unknown_;
  bool use_diagonal_;
  double cost_weight_;
  int search_window_margin_;
  double replan_rate_hz_;
  double replan_position_tolerance_;
  double replan_yaw_tolerance_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
    local_costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr replan_timer_;

  // State
  std::optional<nav_msgs::msg::OccupancyGrid> global_map_;
  std::optional<geometry_msgs::msg::PoseStamped> start_pose_;
  std::optional<geometry_msgs::msg::PoseStamped> goal_pose_;
  std::optional<geometry_msgs::msg::PoseStamped> last_planned_start_;
  std::vector<PathPlanner::GridIndex> last_path_cells_;
  bool plan_pending_{false};
};

}  // namespace path_planner
