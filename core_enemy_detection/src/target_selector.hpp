#ifndef CORE_ENEMY_DETECTION__TARGET_SELECTOR_HPP_
#define CORE_ENEMY_DETECTION__TARGET_SELECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "core_msgs/msg/damage_panel_info_array.hpp"
#include "image2camera_vector_angle.hpp"


namespace core_enemy_detection
{
class targetSelector : public rclcpp::Node
{
public:
    targetSelector();

private:
    rclcpp::TimerBase::SharedPtr nodeTimer;
    rclcpp::Subscription<core_msgs::msg::DamagePanelInfoArray>::SharedPtr dpInfoSub;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr targetPosePub;

    void selectTarget(const core_msgs::msg::DamagePanelInfoArray);
    void publishTargetPose();

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf;

    std::vector<core_msgs::msg::DamagePanelInfo> damagePanels;
    core_msgs::msg::DamagePanelInfo target;
    geometry_msgs::msg::Pose dpPose;
    bool flag = false;
};
}  // namespace core_enemy_detection
#endif  // CORE_ENEMY_DETECTION__TARGET_SELECTOR_HPP_