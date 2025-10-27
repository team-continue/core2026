#ifndef ENEMY_SELECTOR__ENEMY_SELECTOR_HPP_
#define ENEMY_SELECTOR__ENEMY_SELECTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <limits>
#include <list>

#include "image2camera_vector_angle.hpp"

namespace enemy_selector
{
class enemySelector : public rclcpp::Node
{
public:
    enemySelector();

private:
    rclcpp::TimerBase::SharedPtr nodeTimer;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr coordSub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr selectSub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr coordsPub2ui;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr coordPub;

    void publishDirection();
    void updateEnemy(geometry_msgs::msg::PoseArray);
    void updateSelect(std_msgs::msg::Int32);
    std::vector<geometry_msgs::msg::Pose> enemyCoords;
    std::vector<geometry_msgs::msg::Pose>::iterator ecitr;
    image2camera_vector_angle::vec2agCalculator *calc;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf;
    std::string calibrationFilePath = "None";

};
}  // namespace enemy_selector
#endif  // ENEMY_SELECTOR__ENEMY_SELECTOR_HPP_
