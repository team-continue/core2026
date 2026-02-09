#include "core_path_follower/path_follower_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<core_path_follower::PathFollowerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
