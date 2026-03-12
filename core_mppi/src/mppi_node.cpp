// Copyright 2026 team-continue
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core_mppi/mppi_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

namespace core_mppi
{

MppiNode::MppiNode(const rclcpp::NodeOptions & options)
: Node("core_mppi_node", options), controller_(MppiParams{})
{
  path_topic_ = declare_parameter<std::string>("path_topic", "/planned_path");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
  local_costmap_topic_ = declare_parameter<std::string>("local_costmap_topic", "/costmap/local");
  global_costmap_topic_ = declare_parameter<std::string>("global_costmap_topic", "/costmap/global");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
  control_rate_ = declare_parameter<double>("control_rate", 20.0);
  goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.15);

  MppiParams params;
  params.samples = declare_parameter<int>("mppi.samples", params.samples);
  params.horizon_steps = declare_parameter<int>("mppi.horizon_steps", params.horizon_steps);
  params.dt = declare_parameter<double>("mppi.dt", params.dt);
  params.temperature = declare_parameter<double>("mppi.temperature", params.temperature);
  params.noise_vx = declare_parameter<double>("mppi.noise_vx", params.noise_vx);
  params.noise_vy = declare_parameter<double>("mppi.noise_vy", params.noise_vy);
  params.noise_wz = declare_parameter<double>("mppi.noise_wz", params.noise_wz);
  params.max_vx = declare_parameter<double>("mppi.max_vx", params.max_vx);
  params.max_vy = declare_parameter<double>("mppi.max_vy", params.max_vy);
  params.max_wz = declare_parameter<double>("mppi.max_wz", params.max_wz);
  params.w_path = declare_parameter<double>("mppi.w_path", params.w_path);
  params.w_goal = declare_parameter<double>("mppi.w_goal", params.w_goal);
  params.w_obstacle = declare_parameter<double>("mppi.w_obstacle", params.w_obstacle);
  params.w_control = declare_parameter<double>("mppi.w_control", params.w_control);
  params.w_smooth = declare_parameter<double>("mppi.w_smooth", params.w_smooth);
  params.w_heading = declare_parameter<double>("mppi.w_heading", params.w_heading);
  params.unknown_cost = declare_parameter<double>("mppi.unknown_cost", params.unknown_cost);
  params.heading_lookahead =
    declare_parameter<int>("mppi.heading_lookahead", params.heading_lookahead);
  controller_.setParams(params);

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    path_topic_, 10, std::bind(&MppiNode::onPath, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 50, std::bind(&MppiNode::onOdom, this, std::placeholders::_1));
  goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_, 10, std::bind(&MppiNode::onGoalPose, this, std::placeholders::_1));
  local_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    local_costmap_topic_, 10, std::bind(&MppiNode::onLocalCostmap, this, std::placeholders::_1));
  global_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    global_costmap_topic_, 10, std::bind(&MppiNode::onGlobalCostmap, this, std::placeholders::_1));

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  goal_reached_pub_ = create_publisher<std_msgs::msg::Bool>("/goal_reached", 10);

  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_));
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&MppiNode::onTimer, this));

  RCLCPP_INFO(
    get_logger(), "core_mppi_node started: path=%s local=%s global=%s cmd=%s", path_topic_.c_str(),
    local_costmap_topic_.c_str(), global_costmap_topic_.c_str(), cmd_vel_topic_.c_str());
}

void MppiNode::onPath(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  path_.clear();
  path_.reserve(msg->poses.size());
  for (const auto & p : msg->poses) {
    path_.push_back(p.pose);
  }
  have_path_ = !path_.empty();
}

void MppiNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_pose_ = msg->pose.pose;
  odom_frame_ = msg->header.frame_id;
  have_odom_ = true;
}

void MppiNode::onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  goal_pose_ = msg->pose;
  have_goal_ = true;
  goal_reached_ = false;
  RCLCPP_INFO(
    get_logger(), "New goal received: (%.2f, %.2f)", goal_pose_.position.x, goal_pose_.position.y);
}

void MppiNode::onLocalCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  local_costmap_ = msg;
}

void MppiNode::onGlobalCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  global_costmap_ = msg;
}

void MppiNode::onTimer()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!have_path_ || !have_odom_) {
    publishStop();
    return;
  }

  // Goal reached — stay stopped
  if (goal_reached_) {
    publishStop();
    publishGoalReached();
    return;
  }

  // Check goal proximity
  if (have_goal_) {
    const double dx = goal_pose_.position.x - current_pose_.position.x;
    const double dy = goal_pose_.position.y - current_pose_.position.y;
    if (std::hypot(dx, dy) < goal_tolerance_) {
      goal_reached_ = true;
      publishStop();
      publishGoalReached();
      RCLCPP_INFO(
        get_logger(), "Goal reached (%.2f, %.2f)", goal_pose_.position.x, goal_pose_.position.y);
      return;
    }
  }

  if (local_costmap_ && !odom_frame_.empty() && local_costmap_->header.frame_id != odom_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Local costmap frame (%s) differs from odom frame (%s); local costmap ignored for scoring.",
      local_costmap_->header.frame_id.c_str(), odom_frame_.c_str());
  }
  if (global_costmap_ && !odom_frame_.empty() && global_costmap_->header.frame_id != odom_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Global costmap frame (%s) differs from odom frame (%s); global costmap ignored for scoring.",
      global_costmap_->header.frame_id.c_str(), odom_frame_.c_str());
  }

  const nav_msgs::msg::OccupancyGrid * local_map =
    (local_costmap_ && local_costmap_->header.frame_id == odom_frame_) ? local_costmap_.get()
                                                                       : nullptr;
  const nav_msgs::msg::OccupancyGrid * global_map =
    (global_costmap_ && global_costmap_->header.frame_id == odom_frame_) ? global_costmap_.get()
                                                                         : nullptr;

  geometry_msgs::msg::Twist cmd;
  if (!controller_.compute(current_pose_, path_, last_cmd_, local_map, global_map, cmd)) {
    publishStop();
    return;
  }

  last_cmd_ = cmd;
  cmd_pub_->publish(cmd);
}

void MppiNode::publishStop()
{
  geometry_msgs::msg::Twist stop;
  last_cmd_ = stop;
  cmd_pub_->publish(stop);
}

void MppiNode::publishGoalReached()
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  goal_reached_pub_->publish(msg);
}

}  // namespace core_mppi
