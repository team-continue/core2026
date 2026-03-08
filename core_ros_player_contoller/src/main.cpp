#include <algorithm>
#include <cstdint>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

class WirelessParserNode : public rclcpp::Node
{
public:
  WirelessParserNode()
  : Node("wireless_parser_node")
  {
    //=================================
    // Subscribers
    //=================================
    subscription_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/wireless", 10,
      std::bind(&WirelessParserNode::wireless_callback, this, std::placeholders::_1));
    
    //=================================
    // Publishers
    //=================================
    parsed_publisher_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/wireless/parsed", 10);
    rotation_flag_publisher_ = create_publisher<std_msgs::msg::Bool>("/rotation_flag", 10);
    manual_mode_publisher_ = create_publisher<std_msgs::msg::Bool>("/manual_mode", 10);
    test_pitch_angle_publisher_ = create_publisher<std_msgs::msg::Float32>("/test_pitch_angle", 10);
    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
   
    //=================================
    // Parameters
    //=================================
    mouse_x_sensitivity_ = declare_parameter<double>("mouse_x_sensitivity", 1.0);
    mouse_y_sensitivity_ = declare_parameter<double>("mouse_y_sensitivity", 1.0);
    
    
    
    RCLCPP_INFO(
      get_logger(),
      "Subscribed: /wireless (std_msgs/msg/UInt8MultiArray), Publish: /wireless/parsed, /rotation_flag, /manual_mode, /test_pitch_angle, /cmd_vel");
  }

private:
  void wireless_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    const auto & values = msg->data;
    if (values.size() < 7) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "values.size() < 7");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "%d, %d, %d, %d, %d, %d, %d", values[0], values[1], values[2], values[3], values[4], values[5], values[6]);

    parsed_publisher_->publish(*msg);

    std_msgs::msg::Bool rotation_flag_msg;
    std_msgs::msg::Bool manual_mode_msg;
    std_msgs::msg::Float32 test_pitch_angle_msg;
    geometry_msgs::msg::Twist cmd_vel_msg;

    const uint8_t raw_flags = values[0];
    const uint8_t raw_mouse_x = values[1];
    const uint8_t raw_mouse_y = values[2];
    [[maybe_unused]] const uint8_t raw_unused_1 = values[3];
    [[maybe_unused]] const uint8_t raw_unused_2 = values[4];
    [[maybe_unused]] const uint8_t raw_unused_3 = values[5];
    [[maybe_unused]] const uint8_t raw_unused_4 = values[6];

    [[maybe_unused]] const uint8_t flag_emergency = (raw_flags >> 0) & 1;
    const uint8_t key_w =          (raw_flags >> 1) & 1;
    const uint8_t key_a =          (raw_flags >> 2) & 1;
    const uint8_t key_s =          (raw_flags >> 3) & 1;
    const uint8_t key_d =          (raw_flags >> 4) & 1;
    const uint8_t key_click =      (raw_flags >> 5) & 1;
    [[maybe_unused]] const uint8_t flag_unused = (raw_flags >> 6) & 1;
    const uint8_t flag_manual =    (raw_flags >> 7) & 1;

    // マウス入力の正規化、感度適用
    const float mouse_x = (static_cast<float>(static_cast<int8_t>(raw_mouse_x)) / 127.0f) *
      static_cast<float>(mouse_x_sensitivity_);
    const float mouse_y = (static_cast<float>(static_cast<int8_t>(raw_mouse_y)) / 127.0f) *
      static_cast<float>(mouse_y_sensitivity_);

    // キー入力とマウス入力からcmd_vel生成
    const double linear_x_from_wa = static_cast<double>(key_w) - static_cast<double>(key_s);
    const double linear_y_from_ad = static_cast<double>(key_a) - static_cast<double>(key_d);
    const double angular_from_mouse = static_cast<double>(mouse_x);

    cmd_vel_msg.linear.x = linear_x_from_wa;
    cmd_vel_msg.linear.y = linear_y_from_ad;
    cmd_vel_msg.angular.z = std::clamp(angular_from_mouse, -1.0, 1.0);

    // マウス入力でピッチ入力生成
    test_pitch_angle_msg.data = mouse_y;
    
    // body_controllerの回転フラグを設定、キー入力モード時胴体に追従させる
    rotation_flag_msg.data = flag_manual > 0;
    
    // shooterの照準を設定、キー入力モード時マニュアルモードに移行
    manual_mode_msg.data = flag_manual > 0;

    // For body_controller
    cmd_vel_publisher_->publish(cmd_vel_msg);
    rotation_flag_publisher_->publish(rotation_flag_msg);
    
    // For Shooter
    manual_mode_publisher_->publish(manual_mode_msg);
    test_pitch_angle_publisher_->publish(test_pitch_angle_msg);
  }

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr parsed_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rotation_flag_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr manual_mode_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr test_pitch_angle_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  double mouse_x_sensitivity_;
  double mouse_y_sensitivity_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WirelessParserNode>());
  rclcpp::shutdown();
  return 0;
}
