#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "core_enemy_detection/oakd_panel_localizer.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<core_enemy_detection::oakdPanelLocalizer>());
    rclcpp::shutdown();
    return 0;
}
