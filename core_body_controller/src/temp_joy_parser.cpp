// include joy msg
#include "core_msgs/msg/can.hpp"
#include "core_msgs/msg/can_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

class TempJoyParser : public rclcpp::Node {
 public:
  TempJoyParser();

 private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  // rclcpp::Publisher<core_msgs::msg::CANArray>::SharedPtr can_pub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pad_r1_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pad_r2_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pad_l1_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pad_l2_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pad_triangle_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pad_circle_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pad_cross_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pad_square_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pad_up_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pad_down_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pad_left_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pad_right_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pad_ps_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pad_options_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pad_share_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pad_l3_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pad_r3_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
      pad_r_stick_vertical_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();

  sensor_msgs::msg::Joy latest_joy_msg_;

  std::vector<bool> button_flip_flag_;
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->axes.size() != latest_joy_msg_.axes.size() ||
        msg->buttons.size() != latest_joy_msg_.buttons.size()) {
      latest_joy_msg_ = *msg;
      button_flip_flag_.resize(msg->buttons.size());
      for (long unsigned int i = 0; i < button_flip_flag_.size(); i++) {
        button_flip_flag_[i] = false;
      }
    }
    for (long unsigned int i = 0; i < msg->buttons.size(); i++) {
      if (msg->buttons[i] != latest_joy_msg_.buttons[i]) {
        button_flip_flag_[i] = !button_flip_flag_[i];
      }
    }
    latest_joy_msg_ = *msg;
  }
};

TempJoyParser::TempJoyParser() : Node("temp_joy_parser") {
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&TempJoyParser::joy_callback, this, std::placeholders::_1));
  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  // can_pub_ = this->create_publisher<core_msgs::msg::CANArray>("can/tx",
  // 10);

  pad_r1_pub_ = this->create_publisher<std_msgs::msg::Bool>("pad/r1", 10);
  pad_r2_pub_ = this->create_publisher<std_msgs::msg::Bool>("pad/r2", 10);
  pad_l1_pub_ = this->create_publisher<std_msgs::msg::Bool>("pad/l1", 10);
  pad_l2_pub_ = this->create_publisher<std_msgs::msg::Bool>("pad/l2", 10);
  pad_triangle_pub_ =
      this->create_publisher<std_msgs::msg::Bool>("pad/triangle", 10);
  pad_circle_pub_ =
      this->create_publisher<std_msgs::msg::Bool>("pad/circle", 10);
  pad_cross_pub_ = this->create_publisher<std_msgs::msg::Bool>("pad/cross", 10);
  pad_square_pub_ =
      this->create_publisher<std_msgs::msg::Bool>("pad/square", 10);
  pad_up_pub_ = this->create_publisher<std_msgs::msg::Bool>("pad/up", 10);
  pad_down_pub_ = this->create_publisher<std_msgs::msg::Bool>("pad/down", 10);
  pad_left_pub_ = this->create_publisher<std_msgs::msg::Bool>("pad/left", 10);
  pad_right_pub_ = this->create_publisher<std_msgs::msg::Bool>("pad/right", 10);
  pad_ps_pub_ = this->create_publisher<std_msgs::msg::Bool>("pad/ps", 10);
  pad_options_pub_ =
      this->create_publisher<std_msgs::msg::Bool>("pad/options", 10);
  pad_share_pub_ = this->create_publisher<std_msgs::msg::Bool>("pad/share", 10);
  pad_l3_pub_ = this->create_publisher<std_msgs::msg::Bool>("pad/l3", 10);
  pad_r3_pub_ = this->create_publisher<std_msgs::msg::Bool>("pad/r3", 10);
  pad_r_stick_vertical_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "pad/r_stick_vertical", 10);

  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(10),
                              std::bind(&TempJoyParser::timer_callback, this));
  latest_joy_msg_.axes.resize(8);
  latest_joy_msg_.buttons.resize(11);
}

void TempJoyParser::timer_callback() {
  auto cmd_vel = geometry_msgs::msg::Twist();
  cmd_vel.linear.x = latest_joy_msg_.axes[1];
  cmd_vel.linear.y = latest_joy_msg_.axes[0];
  cmd_vel.angular.z = latest_joy_msg_.axes[3];
  cmd_vel_pub_->publish(cmd_vel);

  auto msg = std_msgs::msg::Float32();
  msg.data = latest_joy_msg_.axes[4];
  pad_r_stick_vertical_pub_->publish(msg);

  // RCLCPP_INFO(this->get_logger(),
  //             "cmd_vel: linear.x=%f, linear.y=%f, angular.z=%f",
  //             cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

  // if (latest_joy_msg_.buttons[5]) {
  //   auto can_array = core_msgs::msg::CANArray();
  //   auto can = core_msgs::msg::CAN();
  //   can.id = 7;
  //   can.data.push_back(12.56);
  //   can_array.array.push_back(can);
  //   can.id = 8;
  //   can.data[0] = -12.56;
  //   can_array.array.push_back(can);
  //   can_pub_->publish(can_array);
  //   RCLCPP_INFO(this->get_logger(), "Shooter angle vel: 1.57");
  // } else {
  //   auto can_array = core_msgs::msg::CANArray();
  //   auto can = core_msgs::msg::CAN();
  //   can.id = 7;
  //   can.data.push_back(0);
  //   can_array.array.push_back(can);
  //   can.id = 8;
  //   can_array.array.push_back(can);
  //   can_pub_->publish(can_array);
  //   RCLCPP_INFO(this->get_logger(), "Shooter angle vel: 0");
  // }

  // 9に送ると死ぬ
  // if (latest_joy_msg_.buttons[7]) {
  //   auto can_array = core_msgs::msg::CANArray();
  //   auto can = core_msgs::msg::CAN();
  //   can.id = 9;
  //   can.data.push_back((1-latest_joy_msg_.axes[5])*1000);
  //   can_array.array.push_back(can);
  //   can_pub_->publish(can_array);
  //   RCLCPP_INFO(this->get_logger(), "ESC: %f",
  //   (1-latest_joy_msg_.axes[5])*1000);
  // }
  // else {
  //   auto can_array = core_msgs::msg::CANArray();
  //   auto can = core_msgs::msg::CAN();
  //   can.id = 9;
  //   can.data.push_back(0);
  //   can_array.array.push_back(can);
  //   can_pub_->publish(can_array);
  // }

  // ###################################################
  // make it flip flop
  for (long unsigned int i = 0; i < button_flip_flag_.size(); i++) {
    std_msgs::msg::Bool msg;
    if (button_flip_flag_[i]) {
      msg.data = latest_joy_msg_.buttons[i];
      switch (i) {
        case 0:
          pad_cross_pub_->publish(msg);
          break;
        case 1:
          pad_circle_pub_->publish(msg);
          break;
        case 2:
          pad_square_pub_->publish(msg);
          break;
        case 3:
          pad_triangle_pub_->publish(msg);
          break;
        case 4:
          pad_l1_pub_->publish(msg);
          break;
        case 5:
          pad_r1_pub_->publish(msg);
          break;
        case 6:
          pad_l2_pub_->publish(msg);
          break;
        case 7:
          pad_r2_pub_->publish(msg);
          break;
        case 8:
          pad_share_pub_->publish(msg);
          break;
        case 9:
          pad_options_pub_->publish(msg);
          break;
        case 10:
          pad_ps_pub_->publish(msg);
          break;
        case 11:
          pad_l3_pub_->publish(msg);
          break;
        case 12:
          pad_r3_pub_->publish(msg);
          break;
        case 13:
          pad_up_pub_->publish(msg);
          break;
        case 14:
          pad_right_pub_->publish(msg);
          break;
        case 15:
          pad_down_pub_->publish(msg);
          break;
        case 16:
          pad_left_pub_->publish(msg);
          break;
        default:
          break;
      }
    }
    button_flip_flag_[i] = false;
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TempJoyParser>());
  rclcpp::shutdown();
  return 0;
}