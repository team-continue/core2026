#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "enemy_detection.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<enemy_detection::enemyDetection>());
    rclcpp::shutdown();
    return 0;
}
