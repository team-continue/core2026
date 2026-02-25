#include "core_path_follower/path_follower_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>

#include "core_path_follower/interpolation.hpp"

namespace core_path_follower
{

PathFollowerNode::PathFollowerNode(const rclcpp::NodeOptions & options)
: Node("core_path_follower", options),
  controller_(ControllerParam{})
{
  // topics
  path_topic_ = declare_parameter<std::string>("path_topic", "/planned_path");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  // frame
  use_local_frame_ = declare_parameter<bool>("use_local_frame", true);
  reset_on_new_path_ = declare_parameter<bool>("reset_on_new_path", false);
  // controller
  const auto controller_type = declare_parameter<std::string>("controller_type", "cascade");
  const auto linear_speed = declare_parameter<double>("linear_speed", 0.5);
  const auto lookahead_dist = declare_parameter<double>("lookahead_dist", 0.5);
  const auto goal_tolerance = declare_parameter<double>("goal_tolerance", 0.15);
  control_rate_ = declare_parameter<double>("control_rate", 20.0);
  const auto outer_kp = declare_parameter<double>("outer_kp", 1.0);
  const auto outer_ki = declare_parameter<double>("outer_ki", 0.0);
  const auto outer_kd = declare_parameter<double>("outer_kd", 0.1);
  const auto inner_kp = declare_parameter<double>("inner_kp", 2.0);
  const auto inner_ki = declare_parameter<double>("inner_ki", 0.0);
  const auto inner_kd = declare_parameter<double>("inner_kd", 0.05);
  const auto pure_k = declare_parameter<double>("pure_k", 1.0);
  // interpolation
  interpolation_type_ = declare_parameter<std::string>("interpolation", "none");
  spline_samples_per_segment_ = declare_parameter<int>("spline_samples_per_segment", 10);
  bezier_samples_ = declare_parameter<int>("bezier_samples", 100);

  // init controller
  ControllerParam cparam;
  cparam.controller_type = controller_type;
  cparam.linear_speed = linear_speed;
  cparam.lookahead_dist = lookahead_dist;
  cparam.goal_tolerance = goal_tolerance;
  cparam.outer_kp = outer_kp;
  cparam.outer_ki = outer_ki;
  cparam.outer_kd = outer_kd;
  cparam.inner_kp = inner_kp;
  cparam.inner_ki = inner_ki;
  cparam.inner_kd = inner_kd;
  cparam.pure_k = pure_k;
  controller_.setParam(cparam);

  // subscriptions
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    path_topic_, 10,
    std::bind(&PathFollowerNode::onPath, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 50,
    std::bind(&PathFollowerNode::onOdom, this, std::placeholders::_1));

  // publishers
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  goal_reached_pub_ = create_publisher<std_msgs::msg::Bool>("/goal_reached", 10);

  // timer
  const auto period = std::chrono::duration<double>(1.0 / control_rate_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&PathFollowerNode::onTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "core_path_follower started (controller=%s, local_frame=%s, path_topic=%s)",
    controller_type.c_str(),
    use_local_frame_ ? "true" : "false",
    path_topic_.c_str());
}

void PathFollowerNode::onPath(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(path_mutex_);

  // Extract poses
  std::vector<geometry_msgs::msg::Pose> incoming;
  incoming.reserve(msg->poses.size());
  for (const auto & ps : msg->poses) {
    incoming.push_back(ps.pose);
  }

  // Interpolate if requested
  if (interpolation_type_ == "spline") {
    path_poses_ = interp::splineCatmullRom(incoming, spline_samples_per_segment_);
  } else if (interpolation_type_ == "bezier") {
    path_poses_ = interp::bezierGlobal(incoming, bezier_samples_);
  } else {
    path_poses_ = std::move(incoming);
  }

  // Reset state
  goal_reached_ = false;

  if (use_local_frame_) {
    current_target_idx_ = 0;
  } else {
    // World-frame: find closest point to current odom pose
    if (have_odom_ && !path_poses_.empty()) {
      const double cx = current_pose_.position.x;
      const double cy = current_pose_.position.y;
      size_t best = 0;
      double best_d = 1e9;
      for (size_t i = 0; i < path_poses_.size(); ++i) {
        const double dx = path_poses_[i].position.x - cx;
        const double dy = path_poses_[i].position.y - cy;
        const double d = std::hypot(dx, dy);
        if (d < best_d) {
          best_d = d;
          best = i;
        }
      }
      current_target_idx_ = best;
    } else {
      current_target_idx_ = 0;
    }
  }

  if (reset_on_new_path_) {
    controller_.reset();
  }

  RCLCPP_INFO(
    get_logger(), "Received path with %zu poses (local_frame=%s), start_idx=%zu",
    path_poses_.size(), use_local_frame_ ? "true" : "false", current_target_idx_);
}

void PathFollowerNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_pose_ = msg->pose.pose;
  latest_angular_z_ = msg->twist.twist.angular.z;
  have_odom_ = true;
}

void PathFollowerNode::onTimer()
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  if (path_poses_.empty()) {
    return;
  }

  if (goal_reached_) {
    publishStop();
    return;
  }

  const double dt = 1.0 / control_rate_;
  ControlOutput output;

  if (use_local_frame_) {
    output = controller_.computeLocalFrame(
      path_poses_, latest_angular_z_, dt, current_target_idx_, goal_reached_);
  } else {
    if (!have_odom_) {
      return;
    }
    output = controller_.computeWorldFrame(
      path_poses_, current_pose_, latest_angular_z_, dt,
      current_target_idx_, goal_reached_);
  }

  if (goal_reached_) {
    publishStop();
    publishGoalReached();
    RCLCPP_INFO(
      get_logger(), "Goal reached (%s).",
      use_local_frame_ ? "local frame" : "world frame");
    return;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = output.vx;
  cmd.linear.y = output.vy;
  cmd.angular.z = output.omega;
  cmd_pub_->publish(cmd);
}

void PathFollowerNode::publishStop()
{
  geometry_msgs::msg::Twist stop;
  cmd_pub_->publish(stop);
}

void PathFollowerNode::publishGoalReached()
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  goal_reached_pub_->publish(msg);
}

} // namespace core_path_follower
