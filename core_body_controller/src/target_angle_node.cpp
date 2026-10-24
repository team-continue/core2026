#include <algorithm>
#include <chrono>
#include <cmath>
#include <core_msgs/msg/can_array.hpp>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <stdexcept>

#include "core_body_controller/target_angle_controller.hpp"

class TargetAngleNode : public rclcpp::Node
{
public:
  TargetAngleNode();

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  static constexpr std::chrono::milliseconds TIMER_PERIOD{10};
  static constexpr double NOMINAL_TIMER_DT = 0.01;
  // joint_states の position のうち腰 (RoboStride 06、CAN ID 4) の添字
  static constexpr size_t WAIST_JOINT_INDEX = 4;

  rclcpp::Publisher<core_msgs::msg::CANArray>::SharedPtr can_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr world_target_angle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr body_omega_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr body_omega_manual_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_omega_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rotation_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr yaw_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_target_rate_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr yaw_center_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<core_body_controller::TargetAngleController> controller_;
  geometry_msgs::msg::Twist latest_twist_;
  double world_target_angle_ = 0.0;
  double latest_world_angle_ = 0.0;
  double latest_body_omega_ = 0.0;
  // 車体の回転のうち無限回転を除いた分 (body_control_node が加速度を制限したあとの値)
  double latest_body_omega_manual_ = 0.0;
  bool has_received_body_omega_manual_ = false;
  double cmd_vel_timeout_sec_ = 0.2;
  double imu_timeout_sec_ = 0.2;
  double body_omega_timeout_sec_ = 0.2;
  bool emergency_stop_flag_ = true;
  bool has_received_cmd_vel_ = false;
  bool has_received_imu_ = false;
  bool has_received_body_omega_ = false;
  int rotation_mode_ = 0;
  core_body_controller::YawMode yaw_mode_ = core_body_controller::YawMode::kFront;
  double yaw_rotation_velocity_ = 6.28;
  double latest_target_rate_ = 0.0;
  bool has_received_target_rate_ = false;
  double latest_waist_position_ = 0.0;
  bool has_received_joint_states_ = false;
  // 腰が車体の正面を向くときの角度 [rad] (実機の値。body_control_node と同じ)
  double waist_front_offset_ = 3.7822790145874023;
  double waist_joint_direction_ = 1.0;
  double joint_states_timeout_sec_ = 0.5;
  // CENTER で正面へ振り向くときの目標の向きの速さ [rad/s] (既定は 1 秒で 1 回転) と、振り向いている途中か
  double yaw_center_velocity_ = 2.0 * M_PI;
  bool centering_ = false;
  core_body_controller::YawTracker yaw_tracker_;
  SteadyTime last_cmd_vel_time_;
  SteadyTime last_imu_time_;
  SteadyTime last_body_omega_time_;
  SteadyTime last_body_omega_manual_time_;
  SteadyTime last_target_rate_time_;
  SteadyTime last_joint_states_time_;
  SteadyTime last_control_time_;

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void timer_callback();
  double gimbal_control(
    double dt, bool cmd_vel_is_fresh, bool target_rate_is_fresh, bool body_omega_is_fresh);
  void center_waist();
  void publish_command(double omega);
  bool is_fresh(const SteadyTime & last_time, bool has_received, double timeout_sec) const;
};

TargetAngleNode::TargetAngleNode()
: Node("target_angle_node"), last_control_time_(std::chrono::steady_clock::now())
{
  const double yaw_rotation_velocity =
    this->declare_parameter<double>("yaw_rotation_velocity", 6.28);
  yaw_rotation_velocity_ = yaw_rotation_velocity;
  const double yaw_rotation_acceleration =
    this->declare_parameter<double>("yaw_rotation_acceleration", M_PI * 3.0);
  cmd_vel_timeout_sec_ =
    this->declare_parameter<double>("cmd_vel_timeout_sec", cmd_vel_timeout_sec_);
  imu_timeout_sec_ = this->declare_parameter<double>("imu_timeout_sec", imu_timeout_sec_);
  body_omega_timeout_sec_ =
    this->declare_parameter<double>("body_omega_timeout_sec", body_omega_timeout_sec_);

  if (
    !std::isfinite(yaw_rotation_velocity) || !std::isfinite(yaw_rotation_acceleration) ||
    !std::isfinite(cmd_vel_timeout_sec_) || !std::isfinite(imu_timeout_sec_) ||
    !std::isfinite(body_omega_timeout_sec_) || yaw_rotation_velocity <= 0.0 ||
    yaw_rotation_acceleration <= 0.0 || cmd_vel_timeout_sec_ <= 0.0 || imu_timeout_sec_ <= 0.0 ||
    body_omega_timeout_sec_ <= 0.0)
  {
    throw std::invalid_argument("yaw limits and timeout parameters must be finite and positive");
  }
  waist_front_offset_ =
    this->declare_parameter<double>("waist_front_offset", waist_front_offset_);
  waist_joint_direction_ =
    this->declare_parameter<double>("waist_joint_direction", waist_joint_direction_);
  joint_states_timeout_sec_ =
    this->declare_parameter<double>("joint_states_timeout_sec", joint_states_timeout_sec_);
  if (!std::isfinite(waist_front_offset_)) {
    throw std::invalid_argument("waist_front_offset must be finite");
  }
  if (waist_joint_direction_ != 1.0 && waist_joint_direction_ != -1.0) {
    throw std::invalid_argument("waist_joint_direction must be 1.0 or -1.0");
  }
  if (!std::isfinite(joint_states_timeout_sec_) || joint_states_timeout_sec_ <= 0.0) {
    throw std::invalid_argument("joint_states_timeout_sec must be finite and positive");
  }
  yaw_center_velocity_ =
    this->declare_parameter<double>("yaw_center_velocity", yaw_center_velocity_);
  if (!std::isfinite(yaw_center_velocity_) || yaw_center_velocity_ <= 0.0) {
    throw std::invalid_argument("yaw_center_velocity must be finite and positive");
  }
  controller_ = std::make_unique<core_body_controller::TargetAngleController>(
    2.0, 0.0, 0.0, yaw_rotation_velocity, yaw_rotation_acceleration);

  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      if (!std::isfinite(msg->angular.z)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Ignoring cmd_vel containing a non-finite angular.z");
        return;
      }
      latest_twist_ = *msg;
      last_cmd_vel_time_ = std::chrono::steady_clock::now();
      has_received_cmd_vel_ = true;
    });
  can_pub_ = this->create_publisher<core_msgs::msg::CANArray>("can/tx", 10);
  rotation_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/rotation", 10,
    [this](const std_msgs::msg::Int32::SharedPtr msg) {rotation_mode_ = msg->data;});
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu", 10, std::bind(&TargetAngleNode::imu_callback, this, std::placeholders::_1));
  world_target_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "yaw_target_angle", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
      if (!std::isfinite(msg->data)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000, "Ignoring non-finite yaw_target_angle");
        return;
      }
      world_target_angle_ = core_body_controller::normalize_angle(msg->data);
    });
  body_omega_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "body_omega", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
      if (emergency_stop_flag_) {
        return;
      }
      if (!std::isfinite(msg->data)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000, "Ignoring non-finite body_omega");
        return;
      }
      latest_body_omega_ = msg->data;
      last_body_omega_time_ = std::chrono::steady_clock::now();
      has_received_body_omega_ = true;
    });
  body_omega_manual_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "body_omega_manual", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
      if (emergency_stop_flag_ || !std::isfinite(msg->data)) {
        return;
      }
      latest_body_omega_manual_ = msg->data;
      last_body_omega_manual_time_ = std::chrono::steady_clock::now();
      has_received_body_omega_manual_ = true;
    });
  emergency_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/system/emergency/hazard_status", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
      emergency_stop_flag_ = msg->data;
      if (emergency_stop_flag_) {
        latest_body_omega_ = 0.0;
        has_received_body_omega_ = false;
        latest_body_omega_manual_ = 0.0;
        has_received_body_omega_manual_ = false;
      }
    });
  yaw_mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "yaw_mode", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
      const auto mode = core_body_controller::to_yaw_mode(msg->data);
      if (!mode) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Ignoring unknown yaw_mode %d (0: front, 1: turret)", msg->data);
        return;
      }
      if (*mode != yaw_mode_) {
        RCLCPP_INFO(
          this->get_logger(), "yaw mode: %s",
          *mode == core_body_controller::YawMode::kTurret ? "turret" : "front");
      }
      yaw_mode_ = *mode;
    });
  yaw_target_rate_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "yaw_target_rate", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
      if (!std::isfinite(msg->data)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000, "Ignoring non-finite yaw_target_rate");
        return;
      }
      latest_target_rate_ = std::clamp(msg->data, -yaw_rotation_velocity_, yaw_rotation_velocity_);
      last_target_rate_time_ = std::chrono::steady_clock::now();
      has_received_target_rate_ = true;
    });
  yaw_center_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "yaw_center", 10, [this](const std_msgs::msg::Empty::SharedPtr) {center_waist();});
  joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      if (msg->position.size() <= WAIST_JOINT_INDEX ||
      !std::isfinite(msg->position[WAIST_JOINT_INDEX]))
      {
        return;
      }
      latest_waist_position_ = msg->position[WAIST_JOINT_INDEX];
      last_joint_states_time_ = std::chrono::steady_clock::now();
      has_received_joint_states_ = true;
    });
  target_omega_pub_ = this->create_publisher<std_msgs::msg::Float64>("target_omega", 10);
  timer_ = this->create_wall_timer(TIMER_PERIOD, std::bind(&TargetAngleNode::timer_callback, this));
}

void TargetAngleNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  double measured_yaw = 0.0;
  if (!core_body_controller::quaternion_to_yaw(
      msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w,
      measured_yaw))
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Ignoring IMU message containing an invalid orientation quaternion");
    return;
  }

  const auto arrival_time = std::chrono::steady_clock::now();
  const double arrival_dt =
    has_received_imu_ ? std::chrono::duration<double>(arrival_time - last_imu_time_).count() : 0.0;
  const bool imu_stream_is_continuous = has_received_imu_ && std::isfinite(arrival_dt) &&
    arrival_dt > 0.0 && arrival_dt <= imu_timeout_sec_;

  if (has_received_imu_ && !imu_stream_is_continuous) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "IMU stream resumed after a timeout; rebasing orientation without a yaw jump");
  }

  yaw_tracker_.update(measured_yaw, imu_stream_is_continuous);
  latest_world_angle_ = yaw_tracker_.yaw();
  has_received_imu_ = true;
  last_imu_time_ = arrival_time;
}

bool TargetAngleNode::is_fresh(
  const SteadyTime & last_time, bool has_received, double timeout_sec) const
{
  return has_received &&
         std::chrono::duration<double>(std::chrono::steady_clock::now() - last_time).count() <=
         timeout_sec;
}

void TargetAngleNode::center_waist()
{
  if (emergency_stop_flag_) {
    RCLCPP_WARN(this->get_logger(), "Ignoring yaw_center during emergency stop");
    return;
  }
  if (!is_fresh(last_imu_time_, has_received_imu_, imu_timeout_sec_)) {
    RCLCPP_WARN(this->get_logger(), "Ignoring yaw_center: no fresh IMU data");
    return;
  }
  if (!is_fresh(last_joint_states_time_, has_received_joint_states_, joint_states_timeout_sec_)) {
    RCLCPP_WARN(this->get_logger(), "Ignoring yaw_center: no fresh waist position in joint_states");
    return;
  }
  // 目標の向きを一気に変えず、gimbal_control で yaw_center_velocity の速さで正面に近づける
  centering_ = true;
  RCLCPP_INFO(
    this->get_logger(), "yaw center: start (waist=%.3f, velocity=%.2f rad/s)",
    latest_waist_position_, yaw_center_velocity_);
}

double TargetAngleNode::gimbal_control(
  double dt, bool cmd_vel_is_fresh, bool target_rate_is_fresh, bool body_omega_is_fresh)
{
  // TURRET で腰を回す操作が来たら、振り向きをやめて操作を優先する
  if (centering_ && yaw_mode_ == core_body_controller::YawMode::kTurret &&
    target_rate_is_fresh && std::abs(latest_target_rate_) > 1e-6)
  {
    centering_ = false;
    RCLCPP_INFO(this->get_logger(), "yaw center: canceled by yaw_target_rate");
  }
  if (centering_ &&
    !is_fresh(last_joint_states_time_, has_received_joint_states_, joint_states_timeout_sec_))
  {
    centering_ = false;
    RCLCPP_WARN(this->get_logger(), "yaw center: canceled (no fresh waist position)");
  }

  if (centering_) {
    // 正面の向きは毎回計算し直す (振り向く途中で車体が曲がっても正面に追いつく)
    const double front = core_body_controller::front_world_target(
      latest_world_angle_, latest_waist_position_, waist_front_offset_, waist_joint_direction_);
    world_target_angle_ = core_body_controller::step_toward_angle(
      world_target_angle_, front, yaw_center_velocity_ * dt);
    if (std::abs(core_body_controller::normalize_angle(front - world_target_angle_)) < 1e-3) {
      centering_ = false;
      RCLCPP_INFO(this->get_logger(), "yaw center: done (target=%.3f)", world_target_angle_);
    }
  } else {
    // FRONT は、車体が実際に回る分 (加速度の制限後、無限回転を除く) で目標を回し、上半身を車体と一体に動かす。
    // 届いていなければ従来どおり cmd_vel の angular.z を使う
    const bool body_omega_manual_is_fresh = is_fresh(
      last_body_omega_manual_time_, has_received_body_omega_manual_, body_omega_timeout_sec_);
    std::optional<double> front_rate;
    if (body_omega_manual_is_fresh) {
      front_rate = latest_body_omega_manual_;
    } else if (cmd_vel_is_fresh) {
      front_rate = latest_twist_.angular.z;
    }
    world_target_angle_ = core_body_controller::next_world_target(
      world_target_angle_, yaw_mode_, front_rate,
      target_rate_is_fresh ? std::optional<double>(latest_target_rate_) : std::nullopt, dt);
  }

  const double world_angle_error =
    core_body_controller::normalize_angle(world_target_angle_ - latest_world_angle_);
  // 車体の回転 (body_omega) は回転モードに関係なく打ち消す。モードの切り替えの瞬間に補正を切ると、
  // 加速・減速中 (body_control_node の加速度の制限で数秒かかる) に腰が車体と一緒に回されて前を向かなくなる
  const double feedforward = body_omega_is_fresh ? latest_body_omega_ : 0.0;
  return controller_->update(world_angle_error, feedforward, dt);
}

void TargetAngleNode::publish_command(double omega)
{
  core_msgs::msg::CANArray can_msg;
  can_msg.array.resize(1);
  can_msg.array[0].id = 4;
  can_msg.array[0].data = {3.0F, static_cast<float>(omega)};
  can_pub_->publish(can_msg);

  std_msgs::msg::Float64 omega_msg;
  omega_msg.data = omega;
  target_omega_pub_->publish(omega_msg);
}

void TargetAngleNode::timer_callback()
{
  const auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - last_control_time_).count();
  last_control_time_ = now;
  if (!std::isfinite(dt) || dt <= 0.0) {
    dt = NOMINAL_TIMER_DT;
  }

  if (emergency_stop_flag_) {
    controller_->reset();
    centering_ = false;
    publish_command(0.0);
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Emergency stop flag is set");
    return;
  }

  if (!is_fresh(last_imu_time_, has_received_imu_, imu_timeout_sec_)) {
    controller_->reset();
    publish_command(0.0);
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Waiting for fresh IMU data; publishing zero yaw command");
    return;
  }

  const bool cmd_vel_is_fresh =
    is_fresh(last_cmd_vel_time_, has_received_cmd_vel_, cmd_vel_timeout_sec_);
  if (!cmd_vel_is_fresh && has_received_cmd_vel_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "cmd_vel timed out; holding the current yaw target");
  }

  const bool body_omega_is_fresh =
    is_fresh(last_body_omega_time_, has_received_body_omega_, body_omega_timeout_sec_);
  if ((rotation_mode_ == 1 || rotation_mode_ == 2) && !body_omega_is_fresh) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "body_omega timed out; disabling rotation feedforward");
  }

  const bool target_rate_is_fresh =
    is_fresh(last_target_rate_time_, has_received_target_rate_, cmd_vel_timeout_sec_);

  const double omega =
    gimbal_control(dt, cmd_vel_is_fresh, target_rate_is_fresh, body_omega_is_fresh);
  publish_command(omega);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "yaw target=%.3f estimate=%.3f command=%.3f mode=%d yaw_mode=%d", world_target_angle_,
    latest_world_angle_, omega, rotation_mode_, static_cast<int>(yaw_mode_));
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetAngleNode>());
  rclcpp::shutdown();
  return 0;
}
