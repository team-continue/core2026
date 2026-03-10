#include <rclcpp/rclcpp.hpp>

class BehaviorSystemManager : public rclcpp::Node {
public:
  BehaviorSystemManager() : rclcpp::Node("behavior_system") {
    RCLCPP_INFO(get_logger(), "behavior_system manager node started");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BehaviorSystemManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
