#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "target_detector.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<core_enemy_detection::targetDetector>());
    rclcpp::shutdown();
    return 0;
}
