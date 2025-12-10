#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "target_calculator.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<core_enemy_detection::targetCalculator>());
    rclcpp::shutdown();
    return 0;
}
