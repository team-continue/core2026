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

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "core_path_follower/controller.hpp"

namespace core_path_follower
{

class PathFollowerNode : public rclcpp::Node
{
public:
  explicit PathFollowerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onPath(const nav_msgs::msg::Path::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onTimer();

  void publishStop();
  void publishGoalReached();

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  Controller controller_;

  std::vector<geometry_msgs::msg::Pose> path_poses_;
  std::mutex path_mutex_;
  size_t current_target_idx_{0};
  bool goal_reached_{false};

  geometry_msgs::msg::Pose current_pose_;
  double latest_angular_z_{0.0};
  bool have_odom_{false};

  std::string path_topic_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  bool use_local_frame_;
  bool reset_on_new_path_;
  double control_rate_;
  std::string interpolation_type_;
  int spline_samples_per_segment_;
  int bezier_samples_;
};

}  // namespace core_path_follower
