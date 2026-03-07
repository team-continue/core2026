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

#include <random>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace core_mppi
{

struct MppiParams
{
  int samples{160};
  int horizon_steps{12};
  double dt{0.08};
  double temperature{1.0};

  double noise_vx{0.25};
  double noise_vy{0.25};
  double noise_wz{0.8};

  double max_vx{1.0};
  double max_vy{1.0};
  double max_wz{3.14};

  double w_path{3.0};
  double w_goal{8.0};
  double w_obstacle{18.0};
  double w_control{0.3};
  double w_smooth{0.8};
  double unknown_cost{0.5};
};

class MppiController
{
public:
  explicit MppiController(const MppiParams & params);

  void setParams(const MppiParams & params);

  bool compute(
    const geometry_msgs::msg::Pose & current_pose,
    const std::vector<geometry_msgs::msg::Pose> & path,
    const geometry_msgs::msg::Twist & prev_cmd,
    const nav_msgs::msg::OccupancyGrid * local_costmap,
    const nav_msgs::msg::OccupancyGrid * global_costmap,
    geometry_msgs::msg::Twist & out_cmd);

private:
  struct State
  {
    double x;
    double y;
    double yaw;
  };

  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q);
  static double clamp(double value, double lo, double hi);
  static double sqr(double x);

  State rolloutStep(const State & s, const geometry_msgs::msg::Twist & u) const;
  double nearestPathDistance(
    const std::vector<geometry_msgs::msg::Pose> & path, double x, double y) const;
  double occupancyCost(
    const nav_msgs::msg::OccupancyGrid * map, double x, double y) const;

  MppiParams params_;
  std::mt19937 rng_;
};

}  // namespace core_mppi
