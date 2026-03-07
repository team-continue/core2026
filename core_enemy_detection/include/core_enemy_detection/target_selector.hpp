#ifndef CORE_ENEMY_DETECTION__TARGET_SELECTOR_HPP_
#define CORE_ENEMY_DETECTION__TARGET_SELECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "core_enemy_detection_common.hpp"
#include "core_msgs/msg/damage_panel_info_array.hpp"


namespace core_enemy_detection
{
class targetSelector : public rclcpp::Node
{
public:
    targetSelector();

private:
    rclcpp::TimerBase::SharedPtr nodeTimer;
    rclcpp::Subscription<core_msgs::msg::DamagePanelInfoArray>::SharedPtr dpInfoSub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr targetPointPub;

    bool flag;
    std::vector<int> imageSize;
    void selectTarget(const core_msgs::msg::DamagePanelInfoArray);
    void declareIntArray(std::vector<int>&, std::vector<int64_t>);
    void publishTargetPoint();
    
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rcl_interfaces::msg::SetParametersResult changeParameter(const std::vector<rclcpp::Parameter>&);


    std::vector<core_msgs::msg::DamagePanelInfo> damagePanels;
    rclcpp::Time timeStamp;
    core_msgs::msg::DamagePanelInfo target;
};
}  // namespace core_enemy_detection
#endif  // CORE_ENEMY_DETECTION__TARGET_SELECTOR_HPP_