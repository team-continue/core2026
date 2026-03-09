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
    // For Body Controller
    rotation_flag_publisher_ = create_publisher<std_msgs::msg::Bool>("/rotation_flag", 10);
    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    // For Shooter
    manual_mode_publisher_ = create_publisher<std_msgs::msg::Bool>("/manual_mode", 10);
    manual_pitch_publisher_ = create_publisher<std_msgs::msg::Float32>("/manual_pitch", 10);
    shoot_motor_publisher_ = create_publisher<std_msgs::msg::Bool>("/shoot_motor", 10);
    shoot_once_publisher_ = create_publisher<std_msgs::msg::Bool>("/left/shoot_once", 10);
    test_mode_publisher_ = create_publisher<std_msgs::msg::Bool>("/test_mode", 10);
   
    //=================================
    // Parameters
    //=================================
    mouse_x_sensitivity_ = declare_parameter<double>("mouse_x_sensitivity", 1.0);
    mouse_y_sensitivity_ = declare_parameter<double>("mouse_y_sensitivity", 1.0);
    
    
    
    RCLCPP_INFO(
      get_logger(),
      "Subscribed: /wireless (std_msgs/msg/UInt8MultiArray), Publish: /rotation_flag, /manual_mode, /manual_pitch, /cmd_vel, /shoot_motor, /left/shoot_once, /test_mode");
  }

private:
  void wireless_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    // Check Size
    const auto & values = msg->data;
    if (values.size() < 7) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "values.size() < 7");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "%d, %d, %d, %d, %d, %d, %d", values[0], values[1], values[2], values[3], values[4], values[5], values[6]);

    std_msgs::msg::Bool rotation_flag_msg;
    std_msgs::msg::Bool manual_mode_msg;
    std_msgs::msg::Float32 manual_pitch_msg;
    geometry_msgs::msg::Twist cmd_vel_msg;
    std_msgs::msg::Bool shoot_motor_msg;
    std_msgs::msg::Bool shoot_once_msg;
    std_msgs::msg::Bool test_mode_msg;

    const uint8_t raw_flags    = values[0];
    const uint8_t raw_mouse_x  = values[1];
    const uint8_t raw_mouse_y  = values[2];
    const uint8_t raw_ui_flags = values[3];
    [[maybe_unused]]
    const uint8_t raw_unused_2 = values[4];
    [[maybe_unused]]
    const uint8_t raw_unused_3 = values[5];
    [[maybe_unused]]
    const uint8_t raw_unused_4 = values[6];

    //=================================
    // キー入力系グラグ
    //=================================
    [[maybe_unused]]
    const uint8_t flag_emergency   = (raw_flags >> 0) & 1;
    const uint8_t key_w            = (raw_flags >> 1) & 1;
    const uint8_t key_s            = (raw_flags >> 2) & 1;
    const uint8_t key_a            = (raw_flags >> 3) & 1;
    const uint8_t key_d            = (raw_flags >> 4) & 1;
    [[maybe_unused]]
    const uint8_t flag_unused      = (raw_flags >> 5) & 1;
    const uint8_t key_click        = (raw_flags >> 6) & 1;
    [[maybe_unused]]
    const uint8_t flag_lock        = (raw_flags >> 7) & 1;

    //=================================
    // UI系グラグ
    //=================================
    const uint8_t ui_auto_flag     = (raw_ui_flags >> 1) & 1;
    const uint8_t ui_roller_flag   = (raw_ui_flags >> 0) & 1;

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
    
    // body_controllerの回転フラグを設定、UIの自動フラグがOFFのとき胴体に追従させる
    rotation_flag_msg.data = ui_auto_flag == 0;
    
    // shooterの照準操作モードを設定、UIの自動フラグがOFFのときマニュアルモード
    manual_mode_msg.data = ui_auto_flag == 0;
    // shooterのピッチ入力をマウス入力で生成
    manual_pitch_msg.data = mouse_y;
    // shooterのローラーの動作グラグ
    shoot_motor_msg.data = ui_roller_flag > 0;
    // shooterの単発発射トリガー
    shoot_once_msg.data = key_click > 0;
    // shooterのtest_modeは常にfalse
    test_mode_msg.data = false;

    // For body_controller
    cmd_vel_publisher_->publish(cmd_vel_msg);
    rotation_flag_publisher_->publish(rotation_flag_msg);
    
    // For Shooter
    manual_mode_publisher_->publish(manual_mode_msg);
    manual_pitch_publisher_->publish(manual_pitch_msg);
    shoot_motor_publisher_->publish(shoot_motor_msg);
    shoot_once_publisher_->publish(shoot_once_msg);
    test_mode_publisher_->publish(test_mode_msg);
  }

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rotation_flag_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr manual_mode_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr manual_pitch_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shoot_motor_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shoot_once_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr test_mode_publisher_;
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
