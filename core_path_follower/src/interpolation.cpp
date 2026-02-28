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

#include "core_path_follower/interpolation.hpp"

#include <cmath>

namespace core_path_follower
{
namespace interp
{
std::vector<geometry_msgs::msg::Pose> splineCatmullRom(
  const std::vector<geometry_msgs::msg::Pose> & waypoints, int samples_per_segment)
{
  std::vector<geometry_msgs::msg::Pose> out;
  if (waypoints.size() < 2) {
    return waypoints;
  }

  for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    geometry_msgs::msg::Pose p0 = (i == 0) ? waypoints[i] : waypoints[i - 1];
    geometry_msgs::msg::Pose p1 = waypoints[i];
    geometry_msgs::msg::Pose p2 = waypoints[i + 1];
    geometry_msgs::msg::Pose p3 = (i + 2 < waypoints.size()) ? waypoints[i + 2] : waypoints[i + 1];

    for (int s = 0; s < samples_per_segment; ++s) {
      double t = static_cast<double>(s) / static_cast<double>(samples_per_segment);
      double t2 = t * t;
      double t3 = t2 * t;

      double x0 = p0.position.x;
      double x1 = p1.position.x;
      double x2 = p2.position.x;
      double x3 = p3.position.x;
      double y0 = p0.position.y;
      double y1 = p1.position.y;
      double y2 = p2.position.y;
      double y3 = p3.position.y;

      double cx = 0.5 *
        ((2.0 * x1) + (-x0 + x2) * t + (2.0 * x0 - 5.0 * x1 + 4.0 * x2 - x3) * t2 +
        (-x0 + 3.0 * x1 - 3.0 * x2 + x3) * t3);
      double cy = 0.5 *
        ((2.0 * y1) + (-y0 + y2) * t + (2.0 * y0 - 5.0 * y1 + 4.0 * y2 - y3) * t2 +
        (-y0 + 3.0 * y1 - 3.0 * y2 + y3) * t3);

      geometry_msgs::msg::Pose np;
      np.position.x = cx;
      np.position.y = cy;
      np.position.z = 0.0;
      double dx = cx - p1.position.x;
      double dy = cy - p1.position.y;
      double yaw = std::atan2(dy, dx);
      np.orientation.w = std::cos(yaw * 0.5);
      np.orientation.x = 0.0;
      np.orientation.y = 0.0;
      np.orientation.z = std::sin(yaw * 0.5);
      out.push_back(np);
    }
  }
  out.push_back(waypoints.back());
  return out;
}

std::vector<geometry_msgs::msg::Pose> bezierGlobal(
  const std::vector<geometry_msgs::msg::Pose> & waypoints, int samples)
{
  std::vector<geometry_msgs::msg::Pose> out;
  if (waypoints.size() < 2) {
    return waypoints;
  }

  int n = static_cast<int>(waypoints.size()) - 1;
  std::vector<double> fact(n + 1, 1.0);
  for (int i = 1; i <= n; ++i) {
    fact[i] = fact[i - 1] * static_cast<double>(i);
  }

  auto binom = [&](int i, int n)
    {
      return fact[n] / (fact[i] * fact[n - i]);
    };

  for (int s = 0; s <= samples; ++s) {
    double t = static_cast<double>(s) / static_cast<double>(samples);
    double x = 0.0;
    double y = 0.0;
    for (int i = 0; i <= n; ++i) {
      double b = binom(i, n) * std::pow(1.0 - t, n - i) * std::pow(t, i);
      x += b * waypoints[i].position.x;
      y += b * waypoints[i].position.y;
    }
    geometry_msgs::msg::Pose np;
    np.position.x = x;
    np.position.y = y;
    np.position.z = 0.0;
    double t_a = std::min(1.0, t + 1e-3);
    double xa = 0.0, ya = 0.0;
    for (int i = 0; i <= n; ++i) {
      double b = binom(i, n) * std::pow(1.0 - t_a, n - i) * std::pow(t_a, i);
      xa += b * waypoints[i].position.x;
      ya += b * waypoints[i].position.y;
    }
    double yaw = std::atan2(ya - y, xa - x);
    np.orientation.w = std::cos(yaw * 0.5);
    np.orientation.x = 0.0;
    np.orientation.y = 0.0;
    np.orientation.z = std::sin(yaw * 0.5);
    out.push_back(np);
  }
  return out;
}
}     // namespace interp
}  // namespace core_path_follower
