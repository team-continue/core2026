#include <cstddef>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace
{
constexpr std::size_t kPacketSize = 7;

bool bit(std::uint8_t value, unsigned int index)
{
  return ((value >> index) & 1U) != 0U;
}

// mouse values are transmitted as an 8-bit two's-complement delta.
float signed_mouse_delta(std::uint8_t value)
{
  return static_cast<float>(static_cast<std::int8_t>(value)) / 127.0F;
}
}  // namespace

class WirelessParserNode : public rclcpp::Node
{
public:
  WirelessParserNode()
  : Node("wireless_parser_node")
  {
    subscription_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/wireless", 10,
      std::bind(&WirelessParserNode::wireless_callback, this, std::placeholders::_1));

    ads_publisher_ = create_publisher<std_msgs::msg::Bool>("/ads", 10);
    left_turret_auto_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/left/turret_auto", 10);
    right_turret_auto_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/right/turret_auto", 10);
    rotation_publisher_ = create_publisher<std_msgs::msg::Int32>("/rotation", 10);
    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    manual_mode_publisher_ = create_publisher<std_msgs::msg::Bool>("/manual_mode", 10);
    manual_pitch_publisher_ = create_publisher<std_msgs::msg::Float32>("/manual_pitch", 10);
    shoot_motor_publisher_ = create_publisher<std_msgs::msg::Bool>("/shoot_motor", 10);
    left_fullauto_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/left/shoot_fullauto", 10);
    right_fullauto_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/right/shoot_fullauto", 10);
    reloading_publisher_ = create_publisher<std_msgs::msg::Bool>("/reloading", 10);
    hazard_status_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/system/emergency/hazard_status", 10);
    test_mode_publisher_ = create_publisher<std_msgs::msg::Bool>("/test_mode", 10);

    mouse_x_sensitivity_ = declare_parameter<double>("mouse_x_sensitivity", 1.0);
    mouse_y_sensitivity_ = declare_parameter<double>("mouse_y_sensitivity", 1.0);
    mouse_x_inverse_ = declare_parameter<bool>("mouse_x_inverse", false);
    mouse_y_inverse_ = declare_parameter<bool>("mouse_y_inverse", false);
    cmd_vel_xy_scale_ = declare_parameter<double>("cmd_vel_xy_scale", 0.25);
    manual_mode_target_side_ = declare_parameter<std::string>(
      "manual_mode_target_side", "right");
    if (manual_mode_target_side_ != "left" && manual_mode_target_side_ != "right") {
      throw std::invalid_argument(
        "manual_mode_target_side must be either 'left' or 'right'");
    }

    RCLCPP_INFO(get_logger(), "Subscribed to 7-byte /wireless controller packets");
  }

private:
  void wireless_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < kPacketSize) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Ignoring wireless packet shorter than 7 bytes");
      return;
    }

    const auto & data = msg->data;
    const std::uint8_t flags = data[0];
    const std::uint8_t mouse_x_raw = data[1];
    const std::uint8_t mouse_y_raw = data[2];
    const std::uint8_t movement = data[3];

    // byte 0: EStop, Roller, Reload, Shoot, ADS, LeftTurretAuto, RightTurretAuto, reserved
    const bool estop = bit(flags, 0);
    const bool roller = bit(flags, 1);
    const bool reload = bit(flags, 2);
    const bool shoot = bit(flags, 3);
    const bool ads = bit(flags, 4);
    const bool left_turret_auto = bit(flags, 5);
    const bool right_turret_auto = bit(flags, 6);

    // byte 3: W, A, S, D, InfiniteRotate (00=off, 01=R1, 10=R2)
    const bool key_w = bit(movement, 0);
    const bool key_a = bit(movement, 1);
    const bool key_s = bit(movement, 2);
    const bool key_d = bit(movement, 3);
    const std::int32_t infinite_rotate = static_cast<std::int32_t>((movement >> 4) & 0x03U);

    const double mouse_x = static_cast<double>(signed_mouse_delta(mouse_x_raw)) *
      mouse_x_sensitivity_ * (mouse_x_inverse_ ? -1.0 : 1.0);
    const double mouse_y = static_cast<double>(signed_mouse_delta(mouse_y_raw)) *
      mouse_y_sensitivity_ * (mouse_y_inverse_ ? -1.0 : 1.0);

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = (static_cast<double>(key_w) - static_cast<double>(key_s)) *
      cmd_vel_xy_scale_;
    cmd_vel.linear.y = (static_cast<double>(key_a) - static_cast<double>(key_d)) *
      cmd_vel_xy_scale_;
    cmd_vel.angular.z = mouse_x;
    cmd_vel_publisher_->publish(cmd_vel);

    std_msgs::msg::Bool bool_msg;
    bool_msg.data = ads;
    ads_publisher_->publish(bool_msg);

    bool_msg.data = left_turret_auto;
    left_turret_auto_publisher_->publish(bool_msg);
    bool_msg.data = right_turret_auto;
    right_turret_auto_publisher_->publish(bool_msg);

    std_msgs::msg::Int32 rotation_msg;
    rotation_msg.data = infinite_rotate;
    rotation_publisher_->publish(rotation_msg);

    // LeftTurretAuto and RightTurretAuto are independent.  The legacy single
    // /manual_mode topic uses the configured shooter target side.
    std_msgs::msg::Bool manual_mode_msg;
    const unsigned int target_auto_bit = manual_mode_target_side_ == "left" ? 5U : 6U;
    manual_mode_msg.data = !bit(flags, target_auto_bit);
    manual_mode_publisher_->publish(manual_mode_msg);

    std_msgs::msg::Float32 pitch_msg;
    pitch_msg.data = static_cast<float>(mouse_y);
    manual_pitch_publisher_->publish(pitch_msg);

    bool_msg.data = roller;
    shoot_motor_publisher_->publish(bool_msg);

    bool_msg.data = shoot;
    if (manual_mode_target_side_ == "left") {
      left_fullauto_publisher_->publish(bool_msg);
    } else {
      right_fullauto_publisher_->publish(bool_msg);
    }

    bool_msg.data = estop;
    hazard_status_publisher_->publish(bool_msg);

    std_msgs::msg::Bool test_mode_msg;
    test_mode_msg.data = false;
    test_mode_publisher_->publish(test_mode_msg);

    if (reload && !previous_reload_) {
      std_msgs::msg::Bool reload_msg;
      reload_msg.data = true;
      reloading_publisher_->publish(reload_msg);
    }
    previous_reload_ = reload;
  }

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ads_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_turret_auto_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_turret_auto_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rotation_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr manual_mode_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr manual_pitch_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shoot_motor_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_fullauto_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_fullauto_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reloading_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hazard_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr test_mode_publisher_;
  double mouse_x_sensitivity_{1.0};
  double mouse_y_sensitivity_{1.0};
  bool mouse_x_inverse_{false};
  bool mouse_y_inverse_{false};
  double cmd_vel_xy_scale_{0.25};
  std::string manual_mode_target_side_{"right"};
  bool previous_reload_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WirelessParserNode>());
  rclcpp::shutdown();
  return 0;
}
