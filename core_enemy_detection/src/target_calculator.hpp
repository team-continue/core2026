#ifndef CORE_ENEMY_DETECTION__TARGETCALCULATOR_HPP_
#define CORE_ENEMY_DETECTION__TARGETCALCULATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "core_msgs/msg/damage_panel_info_array.hpp"

namespace core_enemy_detection
{
class targetCalculator : public rclcpp::Node
{
public:
    targetCalculator();

private:
    rclcpp::Subscription<core_msgs::msg::DamagePanelInfoArray>::SharedPtr dpSub;

    std::vector<core_msgs::msg::DamagePanelInfo> dpInfo;
    
    float cameraHeight = 0;
    float targetHeight = 0;

    std::vector<geometry_msgs::msg::Pose> dpPose;

    std::string fileName = "None";
    cv::Mat matrixK;
    float k1, k2, p1, p2, k3;

    void setCalibrationParam();
    void calculateWorldPose(const core_msgs::msg::DamagePanelInfoArray);


};
}  // namespace core_enemy_detection
#endif  // CORE_ENEMY_DETECTION__TARGETCALCULATOR_HPP_
