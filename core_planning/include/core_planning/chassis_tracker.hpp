// 駆動部（chassis_link）の向きと回転モードを読み、衝突判定に使う駆動部の形を返す（ROS のノードの部品）。
//
// - 向き: TF の base_link → chassis_link（robot_state_publisher が腰の角度から出す）
// - 回転モード: /control/rotation（0 以外なら、駆動部が上部の下で回り続ける）
// footprint.rotate_with_chassis が false なら何もしない（機体の形は base_link の向きのまま）

#ifndef CORE_PLANNING__CHASSIS_TRACKER_HPP_
#define CORE_PLANNING__CHASSIS_TRACKER_HPP_

#include <cmath>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>

#include "core_planning/footprint.hpp"

namespace core_planning
{
class ChassisTracker
{
public:
  ChassisTracker(rclcpp::Node & node, const tf2_ros::Buffer & tf_buffer)
  : node_(node), tf_buffer_(tf_buffer)
  {
    enabled_ = node.declare_parameter<bool>(
      "footprint.rotate_with_chassis", false);
    base_frame_ = node.declare_parameter<std::string>("footprint.base_frame", "base_link");
    chassis_frame_ = node.declare_parameter<std::string>("footprint.chassis_frame", "chassis_link");
    if (enabled_) {
      rotation_subscription_ = node.create_subscription<std_msgs::msg::Int32>(
        "/control/rotation", rclcpp::QoS(10),
        [this](const std_msgs::msg::Int32::ConstSharedPtr & msg) {rotating_ = msg->data != 0;});
    }
  }

  // 衝突判定に使う駆動部の形。無効なら nullopt。向きが分からないときは外接円にする
  std::optional<ChassisShape> shape(const double half_length, const double half_width)
  {
    if (!enabled_) {
      return std::nullopt;
    }
    return chassisShape(half_length, half_width, lookupYaw(), rotating_);
  }

private:
  std::optional<double> lookupYaw()
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        base_frame_, chassis_frame_, tf2::TimePointZero);
      const auto & q = transform.transform.rotation;
      return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 5000,
        "%s から %s への変換を取得できないので、機体の形を外接円にします: %s", base_frame_.c_str(),
        chassis_frame_.c_str(), error.what());
      return std::nullopt;
    }
  }

  rclcpp::Node & node_;
  const tf2_ros::Buffer & tf_buffer_;
  bool enabled_{false};
  bool rotating_{false};
  std::string base_frame_;
  std::string chassis_frame_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rotation_subscription_;
};
}  // namespace core_planning

#endif  // CORE_PLANNING__CHASSIS_TRACKER_HPP_
