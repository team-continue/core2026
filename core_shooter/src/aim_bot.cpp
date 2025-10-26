#include <memory>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

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
    this->declare_parameter<std::string>("source_frame", "base_link");
    this->declare_parameter<std::string>("target_frame", "enemy_tf");
    this->declare_parameter<double>("rate", 30.0);

    // ジョイント名
    this->declare_parameter<std::string>("yaw_joint_name", "yaw_joint");
    this->declare_parameter<std::string>("pitch_joint_name", "pitch_joint");

    // Yaw制御パラメータ
    this->declare_parameter<double>("yaw_kp", 1.2);
    this->declare_parameter<double>("yaw_max_output", 1.0);
    this->declare_parameter<double>("yaw_tolerance", 0.02);

    // Pitch制御パラメータ
    this->declare_parameter<double>("pitch_kp", 1.0);
    this->declare_parameter<double>("pitch_max_output", 1.0);
    this->declare_parameter<double>("pitch_tolerance", 0.02);
    this->declare_parameter<double>("pitch_offset", 0.0);

    // パラメータ取得
    this->get_parameter("source_frame", source_frame_);
    this->get_parameter("target_frame", target_frame_);
    this->get_parameter("rate", rate_);
    this->get_parameter("yaw_joint_name", yaw_joint_name_);
    this->get_parameter("pitch_joint_name", pitch_joint_name_);
    this->get_parameter("yaw_kp", yaw_kp_);
    this->get_parameter("yaw_max_output", yaw_max_output_);
    this->get_parameter("yaw_tolerance", yaw_tolerance_);
    this->get_parameter("pitch_kp", pitch_kp_);
    this->get_parameter("pitch_max_output", pitch_max_output_);
    this->get_parameter("pitch_tolerance", pitch_tolerance_);
    this->get_parameter("pitch_offset", pitch_offset_);

    // ----------------------------
    // Publisher
    // ----------------------------
    turret_dev_pub_ = this->create_publisher<std_msgs::msg::Float32>("turret_deviation", 10);
    turret_motor_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/can/tx/turret_motor_angle",
      10);
    pitch_motor_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/can/tx/pitch_motor_angle",
      10);

    // ----------------------------
    // Subscriber
    // ----------------------------
    aimbot_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "aimbot_state", 10, std::bind(&AimBot::aimbotStateCallback, this, std::placeholders::_1));

    hazard_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "hazard_status", 10, std::bind(&AimBot::hazardCallback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&AimBot::jointStateCallback, this, std::placeholders::_1));

    // ----------------------------
    // TF
    // ----------------------------
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ----------------------------
    // Timer
    // ----------------------------
    auto period = std::chrono::duration<double>(1.0 / rate_);
    timer_ = this->create_wall_timer(period, std::bind(&AimBot::timerCallback, this));

    RCLCPP_INFO(
      this->get_logger(), "AimBot started. TF: %s -> %s",
      source_frame_.c_str(), target_frame_.c_str());
  }

private:
  // ===== コールバック =====
  void aimbotStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    aimbot_enabled_ = msg->data;
  }

  void hazardCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    hazard_detected_ = msg->data;
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == yaw_joint_name_) {
        if (i < msg->position.size()) {yaw_angle_ = msg->position[i];}
      }
      if (msg->name[i] == pitch_joint_name_) {
        if (i < msg->position.size()) {pitch_angle_ = msg->position[i];}
      }
    }
  }

  void timerCallback()
  {
    // 緊急停止または無効時はゼロ出力
    if (!aimbot_enabled_ || hazard_detected_) {
      publishZero();
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        source_frame_, target_frame_, tf2::TimePointZero,
        200ms);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 2000, "TF lookup failed: %s", ex.what());
      publishZero();
      return;
    }

    // --- 敵座標を取得 ---
    double tx = transform.transform.translation.x;
    double ty = transform.transform.translation.y;
    double tz = transform.transform.translation.z;

    // --- 目標角度算出 ---
    double target_yaw = std::atan2(ty, tx);
    double dist_xy = std::sqrt(tx * tx + ty * ty);
    double target_pitch = std::atan2(-tz, dist_xy) + pitch_offset_;

    // --- 偏差計算（現在角度との差分） ---
    double yaw_error = target_yaw - yaw_angle_;
    double pitch_error = target_pitch - pitch_angle_;

    // --- 偏差を出力（デバッグ用） ---
    std_msgs::msg::Float32 dev_msg;
    dev_msg.data = static_cast<float>(yaw_error);
    turret_dev_pub_->publish(dev_msg);

    // --- 出力計算（比例制御） ---
    double yaw_output = std::clamp(yaw_kp_ * yaw_error, -yaw_max_output_, yaw_max_output_);
    double pitch_output =
      std::clamp(pitch_kp_ * pitch_error, -pitch_max_output_, pitch_max_output_);

    // --- 許容範囲以下なら0出力 ---
    if (std::fabs(yaw_error) < yaw_tolerance_) {yaw_output = 0.0;}
    if (std::fabs(pitch_error) < pitch_tolerance_) {pitch_output = 0.0;}

    // --- 出力Publish ---
    std_msgs::msg::Float32 yaw_msg;
    yaw_msg.data = static_cast<float>(yaw_output);
    turret_motor_pub_->publish(yaw_msg);

    std_msgs::msg::Float32 pitch_msg;
    pitch_msg.data = static_cast<float>(pitch_output);
    pitch_motor_pub_->publish(pitch_msg);
  }

  void publishZero()
  {
    std_msgs::msg::Float32 zero;
    zero.data = 0.0f;
    turret_dev_pub_->publish(zero);
    turret_motor_pub_->publish(zero);
    pitch_motor_pub_->publish(zero);
  }

  // ===== 内部変数 =====
  bool aimbot_enabled_ = false;
  bool hazard_detected_ = false;

  double yaw_angle_ = 0.0;
  double pitch_angle_ = 0.0;

  std::string source_frame_;
  std::string target_frame_;
  std::string yaw_joint_name_;
  std::string pitch_joint_name_;

  double rate_;
  double yaw_kp_, yaw_max_output_, yaw_tolerance_;
  double pitch_kp_, pitch_max_output_, pitch_tolerance_, pitch_offset_;

  // ROS通信
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr turret_dev_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr turret_motor_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_motor_pub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr aimbot_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hazard_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AimBot>());
  rclcpp::shutdown();
  return 0;
}
