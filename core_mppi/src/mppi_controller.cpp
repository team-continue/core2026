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

#include "core_mppi/mppi_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace core_mppi
{

MppiController::MppiController(const MppiParams & params) : params_(params), rng_(0x434F5245) {}

void MppiController::setParams(const MppiParams & params) { params_ = params; }

double MppiController::yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double MppiController::clamp(double value, double lo, double hi)
{
  return std::min(hi, std::max(lo, value));
}

double MppiController::sqr(double x) { return x * x; }

MppiController::State MppiController::rolloutStep(
  const State & s, const geometry_msgs::msg::Twist & u) const
{
  const double world_vx = u.linear.x * std::cos(s.yaw) - u.linear.y * std::sin(s.yaw);
  const double world_vy = u.linear.x * std::sin(s.yaw) + u.linear.y * std::cos(s.yaw);
  return {
    s.x + world_vx * params_.dt,
    s.y + world_vy * params_.dt,
    s.yaw + u.angular.z * params_.dt,
  };
}

double MppiController::nearestPathDistance(
  const std::vector<geometry_msgs::msg::Pose> & path, double x, double y) const
{
  double best = std::numeric_limits<double>::infinity();
  for (const auto & pose : path) {
    const double d = std::hypot(pose.position.x - x, pose.position.y - y);
    if (d < best) {
      best = d;
    }
  }
  return std::isfinite(best) ? best : 0.0;
}

double MppiController::occupancyCost(
  const nav_msgs::msg::OccupancyGrid * map, double x, double y) const
{
  if (map == nullptr || map->info.resolution <= 0.0 || map->data.empty()) {
    return 0.0;
  }

  const double origin_x = map->info.origin.position.x;
  const double origin_y = map->info.origin.position.y;
  const double res = static_cast<double>(map->info.resolution);
  const int mx = static_cast<int>(std::floor((x - origin_x) / res));
  const int my = static_cast<int>(std::floor((y - origin_y) / res));

  if (
    mx < 0 || my < 0 || mx >= static_cast<int>(map->info.width) ||
    my >= static_cast<int>(map->info.height)) {
    return 1.0;
  }

  const int idx = my * static_cast<int>(map->info.width) + mx;
  const int8_t cell = map->data[static_cast<size_t>(idx)];
  if (cell < 0) {
    return params_.unknown_cost;
  }
  return static_cast<double>(cell) / 100.0;
}

double MppiController::headingCost(
  const std::vector<geometry_msgs::msg::Pose> & path, double x, double y, double yaw) const
{
  int nearest_idx = 0;
  double best_dist = std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(path.size()); ++i) {
    const double d = std::hypot(path[i].position.x - x, path[i].position.y - y);
    if (d < best_dist) {
      best_dist = d;
      nearest_idx = i;
    }
  }
  const int target_idx =
    std::min(nearest_idx + params_.heading_lookahead, static_cast<int>(path.size()) - 1);
  const double dx = path[target_idx].position.x - x;
  const double dy = path[target_idx].position.y - y;
  if (std::hypot(dx, dy) < 1e-6) {
    return 0.0;
  }
  const double target_yaw = std::atan2(dy, dx);
  double diff = target_yaw - yaw;
  while (diff > M_PI) {
    diff -= 2.0 * M_PI;
  }
  while (diff < -M_PI) {
    diff += 2.0 * M_PI;
  }
  return diff * diff;
}

bool MppiController::compute(
  const geometry_msgs::msg::Pose & current_pose, const std::vector<geometry_msgs::msg::Pose> & path,
  const geometry_msgs::msg::Twist & prev_cmd, const nav_msgs::msg::OccupancyGrid * local_costmap,
  const nav_msgs::msg::OccupancyGrid * global_costmap, geometry_msgs::msg::Twist & out_cmd)
{
  if (path.empty()) {
    return false;
  }

  const int samples = std::max(1, params_.samples);
  const int horizon = std::max(1, params_.horizon_steps);
  const double temp = std::max(1e-6, params_.temperature);

  std::normal_distribution<double> n_vx(0.0, params_.noise_vx);
  std::normal_distribution<double> n_vy(0.0, params_.noise_vy);
  std::normal_distribution<double> n_wz(0.0, params_.noise_wz);

  std::vector<geometry_msgs::msg::Twist> first_cmds(static_cast<size_t>(samples));
  std::vector<double> costs(static_cast<size_t>(samples), 0.0);

  const auto & goal = path.back().position;

  for (int k = 0; k < samples; ++k) {
    State s{
      current_pose.position.x, current_pose.position.y,
      yawFromQuaternion(current_pose.orientation)};
    geometry_msgs::msg::Twist u_prev = prev_cmd;

    double cost = 0.0;
    for (int t = 0; t < horizon; ++t) {
      geometry_msgs::msg::Twist u;
      u.linear.x = clamp(u_prev.linear.x + n_vx(rng_), -params_.max_vx, params_.max_vx);
      u.linear.y = clamp(u_prev.linear.y + n_vy(rng_), -params_.max_vy, params_.max_vy);
      u.angular.z = clamp(u_prev.angular.z + n_wz(rng_), -params_.max_wz, params_.max_wz);

      if (t == 0) {
        first_cmds[static_cast<size_t>(k)] = u;
      }

      s = rolloutStep(s, u);

      const double path_cost = nearestPathDistance(path, s.x, s.y);
      const double local_obs = occupancyCost(local_costmap, s.x, s.y);
      const double global_obs = occupancyCost(global_costmap, s.x, s.y);
      const double obs_cost = std::max(local_obs, global_obs);

      const double heading = headingCost(path, s.x, s.y, s.yaw);

      cost += params_.w_path * path_cost;
      cost += params_.w_obstacle * obs_cost;
      cost += params_.w_heading * heading;
      cost += params_.w_control * (sqr(u.linear.x) + sqr(u.linear.y) + sqr(u.angular.z));
      cost +=
        params_.w_smooth * (sqr(u.linear.x - u_prev.linear.x) + sqr(u.linear.y - u_prev.linear.y) +
                            sqr(u.angular.z - u_prev.angular.z));

      u_prev = u;
    }

    const double goal_dist = std::hypot(s.x - goal.x, s.y - goal.y);
    cost += params_.w_goal * goal_dist;
    costs[static_cast<size_t>(k)] = cost;
  }

  const auto min_it = std::min_element(costs.begin(), costs.end());
  const double min_cost = (min_it == costs.end()) ? 0.0 : *min_it;

  double sum_w = 0.0;
  geometry_msgs::msg::Twist weighted_cmd;
  for (int k = 0; k < samples; ++k) {
    const double w = std::exp(-(costs[static_cast<size_t>(k)] - min_cost) / temp);
    sum_w += w;
    weighted_cmd.linear.x += w * first_cmds[static_cast<size_t>(k)].linear.x;
    weighted_cmd.linear.y += w * first_cmds[static_cast<size_t>(k)].linear.y;
    weighted_cmd.angular.z += w * first_cmds[static_cast<size_t>(k)].angular.z;
  }

  if (sum_w <= 1e-12) {
    return false;
  }

  out_cmd.linear.x = weighted_cmd.linear.x / sum_w;
  out_cmd.linear.y = weighted_cmd.linear.y / sum_w;
  out_cmd.angular.z = weighted_cmd.angular.z / sum_w;
  return true;
}

}  // namespace core_mppi
