#include <rclcpp/rclcpp.hpp>

#include "path_planner/path_planner_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<path_planner::PathPlannerNode>();
  RCLCPP_INFO(node->get_logger(), "Core Path Planner Node has started.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
