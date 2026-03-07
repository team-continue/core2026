#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "core_enemy_detection/target_selector.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<core_enemy_detection::targetSelector>());
    rclcpp::shutdown();
    return 0;
}
