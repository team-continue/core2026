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
} // namespace core_path_follower
