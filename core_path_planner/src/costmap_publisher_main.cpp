#include <rclcpp/rclcpp.hpp>

#include "path_planner/costmap_publisher_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<path_planner::CostmapPublisherNode>();
  RCLCPP_INFO(node->get_logger(), "Core Costmap Publisher Node has started.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
