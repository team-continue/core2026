#include "path_planner/path_planner_node.hpp"

namespace path_planner
{

PathPlannerNode::PathPlannerNode(const rclcpp::NodeOptions & options)
: Node("core_path_planner_node", options),
  planner_(PathPlanner::Settings{})
{
  global_map_topic_ =
    declare_parameter<std::string>("global_map_topic", "/map");
  local_costmap_topic_ =
    declare_parameter<std::string>("local_costmap_topic", "/local_costmap");
  start_topic_ = declare_parameter<std::string>("start_topic", "/start_pose");
  goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
  path_topic_ = declare_parameter<std::string>("path_topic", "/planned_path");
  local_frame_id_ =
    declare_parameter<std::string>("local_frame_id", "chassis_link");
  publish_in_global_frame_ =
    declare_parameter<bool>("publish_in_global_frame", false);
  global_frame_id_ =
    declare_parameter<std::string>("global_frame_id", "odom");
  occupied_threshold_ = declare_parameter<int>("occupied_threshold", 50);
  allow_unknown_ = declare_parameter<bool>("allow_unknown", true);
  use_diagonal_ = declare_parameter<bool>("use_diagonal", true);
  cost_weight_ = declare_parameter<double>("cost_weight", 0.0);

  global_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    global_map_topic_, rclcpp::QoS(1).transient_local().reliable(),
    std::bind(
      &PathPlannerNode::onGlobalMapReceived, this,
      std::placeholders::_1));

  local_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    local_costmap_topic_, rclcpp::QoS(1).best_effort(),
    std::bind(
      &PathPlannerNode::onLocalCostmapReceived, this,
      std::placeholders::_1));

  start_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    start_topic_, rclcpp::QoS(5),
    std::bind(
      &PathPlannerNode::onStartPoseReceived, this,
      std::placeholders::_1));

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_, rclcpp::QoS(5),
    std::bind(
      &PathPlannerNode::onGoalPoseReceived, this,
      std::placeholders::_1));

  path_pub_ =
    create_publisher<nav_msgs::msg::Path>(path_topic_, rclcpp::QoS(5));

  planner_.setSettings(
    PathPlanner::Settings{occupied_threshold_,
      allow_unknown_, use_diagonal_, cost_weight_});

  RCLCPP_INFO(get_logger(), "Path Planner Node initialized");
}

void PathPlannerNode::onGlobalMapReceived(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  bool first_map = !global_map_.has_value();
  global_map_ = *msg;
  if (first_map) {
    RCLCPP_INFO(
      get_logger(), "Global map received: %ux%u, res=%.3f",
      msg->info.width, msg->info.height, msg->info.resolution);
  }
  // tryPlan();
}

void PathPlannerNode::onLocalCostmapReceived(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  planner_.setLocalCostmap(*msg);
  RCLCPP_DEBUG(get_logger(), "Local costmap received");
  tryPlan();
}

void PathPlannerNode::onStartPoseReceived(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  start_pose_ = *msg;
  RCLCPP_DEBUG(get_logger(), "Start pose received");
  tryPlan();
}

void PathPlannerNode::onGoalPoseReceived(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_pose_ = *msg;
  RCLCPP_INFO(get_logger(), "Goal pose received");
  // tryPlan();
}

void PathPlannerNode::tryPlan()
{
  RCLCPP_DEBUG(get_logger(), "Attempting to plan path");

  if (!global_map_.has_value() || !start_pose_.has_value() ||
    !goal_pose_.has_value())
  {
    return;
  }

  const auto result = planner_.plan(*global_map_, *start_pose_, *goal_pose_);

  switch (result.status) {
    case PathPlanner::Status::kStartOrGoalOutOfBounds:
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Start or goal outside global map bounds. "
        "start=(%.2f, %.2f) goal=(%.2f, %.2f) "
        "map_origin=(%.2f, %.2f) map_size=%ux%u res=%.5f",
        start_pose_->pose.position.x, start_pose_->pose.position.y,
        goal_pose_->pose.position.x, goal_pose_->pose.position.y,
        global_map_->info.origin.position.x,
        global_map_->info.origin.position.y,
        global_map_->info.width, global_map_->info.height,
        global_map_->info.resolution);
      return;
    case PathPlanner::Status::kStartOrGoalOccupied:
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Start or goal is occupied. "
        "start=(%.2f, %.2f) goal=(%.2f, %.2f)",
        start_pose_->pose.position.x, start_pose_->pose.position.y,
        goal_pose_->pose.position.x, goal_pose_->pose.position.y);
      return;
    case PathPlanner::Status::kNoPath:
      RCLCPP_WARN(get_logger(), "Failed to find path.");
      return;
    case PathPlanner::Status::kOk:
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Path planned: %zu waypoints, start=(%.2f, %.2f) goal=(%.2f, %.2f)",
        result.path.size(),
        start_pose_->pose.position.x, start_pose_->pose.position.y,
        goal_pose_->pose.position.x, goal_pose_->pose.position.y);
      publishPath(result.path);
      break;
  }
}

void PathPlannerNode::publishPath(
  const std::vector<PathPlanner::GridIndex> & path_cells)
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = now();
  path_msg.header.frame_id =
    publish_in_global_frame_ ? global_frame_id_ : local_frame_id_;
  path_msg.poses.reserve(path_cells.size());

  const auto & map = *global_map_;
  const auto & robot_pose = start_pose_->pose;

  for (const auto & cell : path_cells) {
    double global_x = 0.0;
    double global_y = 0.0;
    planner_.gridToWorld(map, cell, global_x, global_y);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;

    if (publish_in_global_frame_) {
      pose.pose.position.x = global_x;
      pose.pose.position.y = global_y;
    } else {
      double local_x = 0.0;
      double local_y = 0.0;
      transformToLocal(global_x, global_y, robot_pose, local_x, local_y);
      pose.pose.position.x = local_x;
      pose.pose.position.y = local_y;
    }

    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }

  path_pub_->publish(path_msg);
}

void PathPlannerNode::transformToLocal(
  double global_x, double global_y,
  const geometry_msgs::msg::Pose & robot_pose, double & local_x,
  double & local_y) const
{
  // Translate to robot origin
  const double dx = global_x - robot_pose.position.x;
  const double dy = global_y - robot_pose.position.y;

  // Get robot yaw and rotate by negative yaw
  const double yaw = getYawFromQuaternion(robot_pose.orientation);
  const double cos_yaw = std::cos(-yaw);
  const double sin_yaw = std::sin(-yaw);

  // Apply rotation to get local coordinates
  local_x = dx * cos_yaw - dy * sin_yaw;
  local_y = dx * sin_yaw + dy * cos_yaw;
}

double PathPlannerNode::getYawFromQuaternion(
  const geometry_msgs::msg::Quaternion & q) const
{
  // yaw (z-axis rotation)
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace path_planner
