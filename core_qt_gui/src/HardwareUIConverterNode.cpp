#include "gui_qt/HardwareUIConverterNode.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardwareUIConverterNode>());
    rclcpp::shutdown();
    return 0;
}
