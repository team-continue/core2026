#include <core_msgs/msg/can_array.hpp>
#include <rclcpp/rclcpp.hpp>

class InitNode : public rclcpp::Node {
 public:
  InitNode();

 private:
  rclcpp::Publisher<core_msgs::msg::CANArray>::SharedPtr can_pub_;
};

InitNode::InitNode() : Node("init_node") {
  can_pub_ = this->create_publisher<core_msgs::msg::CANArray>("can/tx", 10);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InitNode>());
  rclcpp::shutdown();
  return 0;
}