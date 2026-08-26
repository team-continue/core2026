#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>

#include <core_msgs/msg/can_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>

#include "core_body_controller/target_angle_controller.hpp"

class TargetAngleNode : public rclcpp::Node
{
public:
  TargetAngleNode();

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  static constexpr std::chrono::milliseconds TIMER_PERIOD{10};
  static constexpr double NOMINAL_TIMER_DT = 0.01;

  rclcpp::Publisher<core_msgs::msg::CANArray>::SharedPtr can_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr world_target_angle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr body_omega_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_omega_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rotation_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<core_body_controller::TargetAngleController> controller_;
  geometry_msgs::msg::Twist latest_twist_;
  double world_target_angle_ = 0.0;
  double latest_world_angle_ = 0.0;
  double latest_body_omega_ = 0.0;
  double cmd_vel_timeout_sec_ = 0.2;
  double imu_timeout_sec_ = 0.2;
  double body_omega_timeout_sec_ = 0.2;
  double imu_yaw_bias_ = M_PI * 0.01;
  bool emergency_stop_flag_ = true;
  bool has_received_cmd_vel_ = false;
  bool has_received_imu_ = false;
  bool has_received_body_omega_ = false;
  bool previous_imu_stamp_usable_ = false;
  int rotation_mode_ = 0;
  int64_t previous_imu_stamp_ns_ = 0;
  SteadyTime last_cmd_vel_time_;
  SteadyTime last_imu_time_;
  SteadyTime last_body_omega_time_;
  SteadyTime last_control_time_;

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void timer_callback();
  double gimbal_control(double dt, bool cmd_vel_is_fresh, bool body_omega_is_fresh);
  void publish_command(double omega);
  bool is_fresh(const SteadyTime & last_time, bool has_received, double timeout_sec) const;
};

TargetAngleNode::TargetAngleNode()
: Node("target_angle_node"), last_control_time_(std::chrono::steady_clock::now())
{
  const double yaw_rotation_velocity =
    this->declare_parameter<double>("yaw_rotation_velocity", 6.28);
  const double yaw_rotation_acceleration =
    this->declare_parameter<double>("yaw_rotation_acceleration", M_PI * 3.0);
  cmd_vel_timeout_sec_ =
    this->declare_parameter<double>("cmd_vel_timeout_sec", cmd_vel_timeout_sec_);
  imu_timeout_sec_ = this->declare_parameter<double>("imu_timeout_sec", imu_timeout_sec_);
  body_omega_timeout_sec_ =
    this->declare_parameter<double>("body_omega_timeout_sec", body_omega_timeout_sec_);
  imu_yaw_bias_ = this->declare_parameter<double>("imu_yaw_bias", imu_yaw_bias_);

  if (!std::isfinite(yaw_rotation_velocity) ||
    !std::isfinite(yaw_rotation_acceleration) ||
    !std::isfinite(cmd_vel_timeout_sec_) || !std::isfinite(imu_timeout_sec_) ||
    !std::isfinite(body_omega_timeout_sec_) ||
    yaw_rotation_velocity <= 0.0 || yaw_rotation_acceleration <= 0.0 ||
    cmd_vel_timeout_sec_ <= 0.0 || imu_timeout_sec_ <= 0.0 ||
    body_omega_timeout_sec_ <= 0.0 ||
    !std::isfinite(imu_yaw_bias_))
  {
    throw std::invalid_argument(
            "yaw limits and timeout parameters must be finite and positive; imu_yaw_bias must "
            "be finite");
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
    "/rotation", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
      rotation_mode_ = msg->data;
    });
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu", 10, std::bind(&TargetAngleNode::imu_callback, this, std::placeholders::_1));
  world_target_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "yaw_target_angle", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
      if (!std::isfinite(msg->data)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Ignoring non-finite yaw_target_angle");
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
          this->get_logger(), *this->get_clock(), 1000,
          "Ignoring non-finite body_omega");
        return;
      }
      latest_body_omega_ = msg->data;
      last_body_omega_time_ = std::chrono::steady_clock::now();
      has_received_body_omega_ = true;
    });
  emergency_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/system/emergency/hazard_status", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      emergency_stop_flag_ = msg->data;
      if (emergency_stop_flag_) {
        latest_body_omega_ = 0.0;
        has_received_body_omega_ = false;
      }
    });
  target_omega_pub_ = this->create_publisher<std_msgs::msg::Float64>("target_omega", 10);
  timer_ = this->create_wall_timer(
    TIMER_PERIOD, std::bind(&TargetAngleNode::timer_callback, this));
}

void TargetAngleNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (!std::isfinite(msg->angular_velocity.z)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Ignoring IMU message containing non-finite angular_velocity.z");
    return;
  }

  const auto arrival_time = std::chrono::steady_clock::now();
  const int64_t stamp_ns =
    static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL + msg->header.stamp.nanosec;

  const double arrival_dt = has_received_imu_ ?
    std::chrono::duration<double>(arrival_time - last_imu_time_).count() : 0.0;
  const bool imu_stream_is_continuous = has_received_imu_ && std::isfinite(arrival_dt) &&
    arrival_dt > 0.0 && arrival_dt <= imu_timeout_sec_;

  if (imu_stream_is_continuous) {
    double dt = arrival_dt;
    const bool current_stamp_usable = stamp_ns > 0;
    if (current_stamp_usable && previous_imu_stamp_usable_ &&
      stamp_ns > previous_imu_stamp_ns_)
    {
      const double stamp_dt = static_cast<double>(stamp_ns - previous_imu_stamp_ns_) * 1e-9;
      if (std::isfinite(stamp_dt) && stamp_dt <= imu_timeout_sec_) {
        dt = stamp_dt;
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "IMU timestamp interval is too large; using steady-clock arrival interval");
      }
    } else if (stamp_ns > 0 && previous_imu_stamp_ns_ > 0 &&
      stamp_ns <= previous_imu_stamp_ns_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "IMU timestamp is not monotonic; using steady-clock arrival interval");
    }

    if (std::isfinite(dt) && dt > 0.0) {
      latest_world_angle_ = core_body_controller::integrate_yaw(
        latest_world_angle_, msg->angular_velocity.z, imu_yaw_bias_, dt);
    }
    previous_imu_stamp_usable_ = current_stamp_usable &&
      (previous_imu_stamp_ns_ == 0 || stamp_ns > previous_imu_stamp_ns_);
  } else {
    if (has_received_imu_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "IMU stream resumed after a timeout; resetting the integration time baseline");
    }
    has_received_imu_ = true;
    previous_imu_stamp_usable_ = stamp_ns > 0;
  }

  previous_imu_stamp_ns_ = stamp_ns;
  last_imu_time_ = arrival_time;
}

bool TargetAngleNode::is_fresh(
  const SteadyTime & last_time, bool has_received, double timeout_sec) const
{
  return has_received &&
         std::chrono::duration<double>(std::chrono::steady_clock::now() - last_time).count() <=
         timeout_sec;
}

double TargetAngleNode::gimbal_control(
  double dt, bool cmd_vel_is_fresh, bool body_omega_is_fresh)
{
  if (cmd_vel_is_fresh) {
    world_target_angle_ = core_body_controller::normalize_angle(
      world_target_angle_ + latest_twist_.angular.z * dt);
  }

  const double world_angle_error = core_body_controller::normalize_angle(
    world_target_angle_ - latest_world_angle_);
  const bool rotation_compensation_enabled = rotation_mode_ == 1 || rotation_mode_ == 2;
  const double feedforward = rotation_compensation_enabled && body_omega_is_fresh ?
    latest_body_omega_ : 0.0;
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

  const double omega = gimbal_control(dt, cmd_vel_is_fresh, body_omega_is_fresh);
  publish_command(omega);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "yaw target=%.3f estimate=%.3f command=%.3f mode=%d",
    world_target_angle_, latest_world_angle_, omega, rotation_mode_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetAngleNode>());
  rclcpp::shutdown();
  return 0;
}
