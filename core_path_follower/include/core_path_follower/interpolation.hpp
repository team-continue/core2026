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

#include <vector>

#include <geometry_msgs/msg/pose.hpp>

namespace core_path_follower
{
namespace interp
{

std::vector<geometry_msgs::msg::Pose> splineCatmullRom(
  const std::vector<geometry_msgs::msg::Pose> & waypoints, int samples_per_segment);

std::vector<geometry_msgs::msg::Pose> bezierGlobal(
  const std::vector<geometry_msgs::msg::Pose> & waypoints, int samples);

}   // namespace interp
}  // namespace core_path_follower
