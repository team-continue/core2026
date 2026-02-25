// Copyright 2026 team-continue
// SPDX-License-Identifier: Apache-2.0

#include "core_path_follower/path_follower_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<core_path_follower::PathFollowerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
