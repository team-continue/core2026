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

#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "core_mppi/mppi_controller.hpp"

namespace core_mppi
{

class MppiNode : public rclcpp::Node
{
public:
  explicit MppiNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onPath(const nav_msgs::msg::Path::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onLocalCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onGlobalCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onTimer();
  void publishStop();
  void publishGoalReached();

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  std::vector<geometry_msgs::msg::Pose> path_;
  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Twist last_cmd_;
  nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap_;
  nav_msgs::msg::OccupancyGrid::SharedPtr global_costmap_;
  std::string odom_frame_;
  geometry_msgs::msg::Pose goal_pose_;
  bool have_path_{false};
  bool have_odom_{false};
  bool have_goal_{false};
  bool goal_reached_{false};

  std::string path_topic_;
  std::string odom_topic_;
  std::string goal_topic_;
  std::string local_costmap_topic_;
  std::string global_costmap_topic_;
  std::string cmd_vel_topic_;
  double control_rate_{20.0};
  double goal_tolerance_{0.15};

  MppiController controller_;
};

}  // namespace core_mppi
