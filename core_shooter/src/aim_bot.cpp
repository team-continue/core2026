#include <memory>
#include <cmath>
#include <algorithm>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

class AimBot : public rclcpp::Node
{
public:
  AimBot()
  : Node("aim_bot")
  {
    // ----------------------------
    // パラメータ宣言
    // ----------------------------
    this->declare_parameter<double>("rate", 30.0);
    this->declare_parameter<int>("pitch_motor_id", 10);
    this->declare_parameter<int>("yaw_motor_id", 7);
    this->declare_parameter<double>("pitch_offset", 0.0);
    this->declare_parameter<double>("yaw_min_angle", -3.14159265359);
    this->declare_parameter<double>("yaw_max_angle", 3.14159265359);
    this->declare_parameter<double>("pitch_min_angle", -3.14159265359);
    this->declare_parameter<double>("pitch_max_angle", 3.14159265359);
    this->declare_parameter<double>("image_center_x", 0.5);
    this->declare_parameter<double>("image_center_y", 0.5);
    this->declare_parameter<double>("image_tolerance_x", 0.02);
    this->declare_parameter<double>("image_tolerance_y", 0.02);
    this->declare_parameter<double>("yaw_image_gain", 0.5);
    this->declare_parameter<double>("pitch_image_gain", 0.5);
    this->declare_parameter<double>("yaw_direction", 1.0);
    this->declare_parameter<double>("pitch_direction", 1.0);
    this->declare_parameter<double>("target_timeout_sec", 0.2);
    this->declare_parameter<bool>("enable_test_mode", false);
    this->declare_parameter<double>("manual_mode_pitch_fixed_angle", 0.0);

    // ----------------------------
    // パラメータ取得
    // ----------------------------
    this->get_parameter("rate", rate_);
    this->get_parameter("pitch_offset", pitch_offset_);
    this->get_parameter("yaw_min_angle", yaw_min_angle_);
    this->get_parameter("yaw_max_angle", yaw_max_angle_);
    this->get_parameter("pitch_min_angle", pitch_min_angle_);
    this->get_parameter("pitch_max_angle", pitch_max_angle_);
    this->get_parameter("image_center_x", image_center_x_);
    this->get_parameter("image_center_y", image_center_y_);
    this->get_parameter("image_tolerance_x", image_tolerance_x_);
    this->get_parameter("image_tolerance_y", image_tolerance_y_);
    this->get_parameter("yaw_image_gain", yaw_image_gain_);
    this->get_parameter("pitch_image_gain", pitch_image_gain_);
    this->get_parameter("yaw_direction", yaw_direction_);
    this->get_parameter("pitch_direction", pitch_direction_);
    this->get_parameter("target_timeout_sec", target_timeout_sec_);
    this->get_parameter("enable_test_mode", enable_test_mode_);
    this->get_parameter("manual_mode_pitch_fixed_angle", manual_mode_pitch_fixed_angle_);
    this->get_parameter("pitch_motor_id", pitch_motor_id_);
    this->get_parameter("yaw_motor_id", yaw_motor_id_);

    if (rate_ <= 0.0) {
      RCLCPP_FATAL(get_logger(), "Invalid rate=%f (must be > 0)", rate_);
      throw std::runtime_error("invalid aimbot rate");
    }
    if (pitch_motor_id_ < 0 || yaw_motor_id_ < 0) {
      RCLCPP_FATAL(
        get_logger(), "Invalid motor ids: pitch_motor_id=%d, yaw_motor_id=%d",
        pitch_motor_id_, yaw_motor_id_);
      throw std::runtime_error("invalid aimbot motor ids");
    }
    if (yaw_min_angle_ > yaw_max_angle_) {
      RCLCPP_FATAL(
        get_logger(), "Invalid yaw angle caps: yaw_min_angle=%f > yaw_max_angle=%f",
        yaw_min_angle_, yaw_max_angle_);
      throw std::runtime_error("invalid yaw angle caps");
    }
    if (pitch_min_angle_ > pitch_max_angle_) {
      RCLCPP_FATAL(
        get_logger(), "Invalid pitch angle caps: pitch_min_angle=%f > pitch_max_angle=%f",
        pitch_min_angle_, pitch_max_angle_);
      throw std::runtime_error("invalid pitch angle caps");
    }
    if (image_tolerance_x_ < 0.0 || image_tolerance_y_ < 0.0 || target_timeout_sec_ < 0.0) {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid image params: image_tolerance_x=%f, image_tolerance_y=%f, target_timeout_sec=%f",
        image_tolerance_x_, image_tolerance_y_, target_timeout_sec_);
      throw std::runtime_error("invalid aimbot image parameters");
    }

    // ----------------------------
    // Subscriber
    // ----------------------------
    target_image_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "target_image_position", 10,
      std::bind(&AimBot::targetImageCallback, this, std::placeholders::_1));

    if (enable_test_mode_) {
      test_yaw_sub_ = create_subscription<std_msgs::msg::Float32>(
        "test_yaw_angle", 10,
        std::bind(&AimBot::testYawCallback, this, std::placeholders::_1));
      test_pitch_sub_ = create_subscription<std_msgs::msg::Float32>(
        "test_pitch_angle", 10,
        std::bind(&AimBot::testPitchCallback, this, std::placeholders::_1));
      RCLCPP_WARN(get_logger(), "Test mode enabled: subscribing test_yaw_angle / test_pitch_angle");
    }

    manual_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/manual_mode", 10, std::bind(&AimBot::manualModeCallback, this, std::placeholders::_1));

    hazard_state_sub_ = create_subscription<std_msgs::msg::Bool>(
      "hazard_status", 10, std::bind(&AimBot::hazardCallback, this, std::placeholders::_1));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&AimBot::jointStateCallback, this, std::placeholders::_1));

    // ----------------------------
    // Publisher
    // ----------------------------
    turret_motor_pub_ = create_publisher<std_msgs::msg::Float32>("/can/tx/turret_motor_angle", 10);
    pitch_motor_pub_ = create_publisher<std_msgs::msg::Float32>("/can/tx/pitch_motor_angle", 10);

    // ----------------------------
    // Timer
    // ----------------------------
    const auto period = std::chrono::duration<double>(1.0 / rate_);
    timer_ = create_wall_timer(period, std::bind(&AimBot::timerCallback, this));

    RCLCPP_INFO(
      get_logger(), "AimBot started. image_center=(%.3f, %.3f), image_tolerance=(%.3f, %.3f)",
      image_center_x_, image_center_y_, image_tolerance_x_, image_tolerance_y_);
  }

private:
  // ===== コールバック =====
  void hazardCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    hazard_detected_ = msg->data;
  }

  void manualModeCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool prev = manual_mode_active_;
    manual_mode_active_ = msg->data;

    if (manual_mode_active_ && !prev) {
      if (!has_joint_state_) {
        manual_mode_active_ = false;
        RCLCPP_WARN(this->get_logger(), "Manual mode ON ignored: joint_states not received yet");
        return;
      }
      manual_mode_yaw_hold_angle_ = std::clamp(yaw_angle_, yaw_min_angle_, yaw_max_angle_);
      RCLCPP_INFO(
        this->get_logger(), "Manual mode ON: hold yaw=%f, fixed pitch=%f",
        manual_mode_yaw_hold_angle_, manual_mode_pitch_fixed_angle_);
    } else if (!manual_mode_active_ && prev) {
      RCLCPP_INFO(this->get_logger(), "Manual mode OFF");
    }
  }

  void targetImageCallback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    target_image_x_ = msg->x;
    target_image_y_ = msg->y;
    has_target_ = true;
    last_target_time_ = this->now();
  }

  void testYawCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    test_yaw_target_ = msg->data;
    has_test_yaw_target_ = true;
  }

  void testPitchCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    test_pitch_target_ = msg->data;
    has_test_pitch_target_ = true;
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const size_t position_size = msg->position.size();
    if (yaw_motor_id_ < 0 || pitch_motor_id_ < 0 ||
      static_cast<size_t>(yaw_motor_id_) >= position_size ||
      static_cast<size_t>(pitch_motor_id_) >= position_size)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "joint_states size mismatch: position=%zu, yaw_id=%d, pitch_id=%d",
        position_size, yaw_motor_id_, pitch_motor_id_);
      return;
    }

    yaw_angle_ = msg->position[yaw_motor_id_];
    pitch_angle_ = msg->position[pitch_motor_id_];
    has_joint_state_ = true;
  }

  void timerCallback()
  {
    // 緊急停止時は現在角保持
    if (hazard_detected_) {
      publishHoldCurrent();
      return;
    }

    if (enable_test_mode_) {
      if (!has_test_yaw_target_ || !has_test_pitch_target_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "test mode enabled but test yaw/pitch target not received yet");
        publishHoldCurrent();
        return;
      }

      std_msgs::msg::Float32 yaw_msg;
      yaw_msg.data =
        static_cast<float>(std::clamp(test_yaw_target_, yaw_min_angle_, yaw_max_angle_));
      turret_motor_pub_->publish(yaw_msg);

      std_msgs::msg::Float32 pitch_msg;
      pitch_msg.data = static_cast<float>(
        std::clamp(test_pitch_target_, pitch_min_angle_, pitch_max_angle_));
      pitch_motor_pub_->publish(pitch_msg);
      return;
    }

    if (manual_mode_active_) {
      const double yaw_output = std::clamp(
        manual_mode_yaw_hold_angle_, yaw_min_angle_,
        yaw_max_angle_);
      const double pitch_output = std::clamp(
        manual_mode_pitch_fixed_angle_, pitch_min_angle_, pitch_max_angle_);

      std_msgs::msg::Float32 yaw_msg;
      yaw_msg.data = static_cast<float>(yaw_output);
      turret_motor_pub_->publish(yaw_msg);

      std_msgs::msg::Float32 pitch_msg;
      pitch_msg.data = static_cast<float>(pitch_output);
      pitch_motor_pub_->publish(pitch_msg);
      return;
    }

    if (!has_target_ || (this->now() - last_target_time_).seconds() > target_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "target_image_position timeout");
      publishHoldCurrent();
      return;
    }

    // 画像内位置（0.0-1.0想定）から中心との差を計算
    const double x_error = target_image_x_ - image_center_x_;
    const double y_error = target_image_y_ - image_center_y_;

    // モータ側が位置制御を行う前提で、現在角 + 画像偏差から目標角を算出
    double yaw_target = yaw_angle_;
    double pitch_target = pitch_angle_ + pitch_offset_;
    if (std::fabs(x_error) > image_tolerance_x_) {
      yaw_target += yaw_direction_ * yaw_image_gain_ * x_error;
    }
    if (std::fabs(y_error) > image_tolerance_y_) {
      pitch_target += pitch_direction_ * pitch_image_gain_ * y_error;
    }

    const double yaw_output = std::clamp(yaw_target, yaw_min_angle_, yaw_max_angle_);
    const double pitch_output = std::clamp(pitch_target, pitch_min_angle_, pitch_max_angle_);

    // --- 出力Publish ---
    std_msgs::msg::Float32 yaw_msg;
    yaw_msg.data = static_cast<float>(yaw_output);
    turret_motor_pub_->publish(yaw_msg);

    std_msgs::msg::Float32 pitch_msg;
    pitch_msg.data = static_cast<float>(pitch_output);
    pitch_motor_pub_->publish(pitch_msg);
  }

  void publishHoldCurrent()
  {
    std_msgs::msg::Float32 yaw_msg;
    yaw_msg.data = static_cast<float>(std::clamp(yaw_angle_, yaw_min_angle_, yaw_max_angle_));
    turret_motor_pub_->publish(yaw_msg);

    std_msgs::msg::Float32 pitch_msg;
    pitch_msg.data = static_cast<float>(
      std::clamp(pitch_angle_ + pitch_offset_, pitch_min_angle_, pitch_max_angle_));
    pitch_motor_pub_->publish(pitch_msg);
  }

  // ===== 内部変数 =====
  bool hazard_detected_ = true;
  bool has_joint_state_ = false;

  double yaw_angle_ = 0.0;
  double pitch_angle_ = 0.0;
  double target_image_x_ = 0.5;
  double target_image_y_ = 0.5;
  bool has_target_ = false;
  rclcpp::Time last_target_time_{0, 0, RCL_ROS_TIME};
  double test_yaw_target_ = 0.0;
  double test_pitch_target_ = 0.0;
  bool has_test_yaw_target_ = false;
  bool has_test_pitch_target_ = false;
  bool manual_mode_active_ = false;
  double manual_mode_yaw_hold_angle_ = 0.0;

  double rate_;
  double pitch_offset_;
  double yaw_min_angle_;
  double yaw_max_angle_;
  double pitch_min_angle_;
  double pitch_max_angle_;
  double image_center_x_;
  double image_center_y_;
  double image_tolerance_x_;
  double image_tolerance_y_;
  double yaw_image_gain_;
  double pitch_image_gain_;
  double yaw_direction_;
  double pitch_direction_;
  double target_timeout_sec_;
  bool enable_test_mode_ = false;
  double manual_mode_pitch_fixed_angle_ = 0.0;
  int pitch_motor_id_;
  int yaw_motor_id_;

  // ROS通信
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr turret_motor_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_motor_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_image_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr test_yaw_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr test_pitch_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hazard_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AimBot>());
  rclcpp::shutdown();
  return 0;
}
