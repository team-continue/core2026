#pragma once

#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "path_planner/path_planner.hpp"

namespace path_planner {

class PathPlannerNode : public rclcpp::Node {
 public:
  explicit PathPlannerNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void onGlobalMapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onLocalCostmapReceived(
      const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onStartPoseReceived(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void tryPlan();
  void publishPath(const std::vector<PathPlanner::GridIndex>& path_cells);

  // Transform a point from global to local (robot) coordinates
  void transformToLocal(double global_x, double global_y,
                        const geometry_msgs::msg::Pose& robot_pose,
                        double& local_x, double& local_y) const;

  // Extract yaw from quaternion
  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) const;

  // Planner instance
  PathPlanner planner_;

  // Parameters
  std::string global_map_topic_;
  std::string local_costmap_topic_;
  std::string start_topic_;
  std::string goal_topic_;
  std::string path_topic_;
  std::string local_frame_id_;
  int occupied_threshold_;
  bool allow_unknown_;
  bool use_diagonal_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
      local_costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // State
  std::optional<nav_msgs::msg::OccupancyGrid> global_map_;
  std::optional<geometry_msgs::msg::PoseStamped> start_pose_;
  std::optional<geometry_msgs::msg::PoseStamped> goal_pose_;
};

}  // namespace path_planner
