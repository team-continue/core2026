// sim の足回りと腰のハードウェアの代わり（実機の core_hardware とマイコンの代わり）。launch の waist:=true で使う。
//
// - /hardware/can/tx（body_control_node の車輪 id 0〜3、target_angle_node の腰 id 4）を読み、
//   上半身（base_link）の座標の速度の指令（Gazebo の omni_force_drive）と、腰の関節の回る速さ（Gazebo の
//   JointController）を出す。換算と向きの約束は core_gazebo/sim_hardware_model.hpp
// - Gazebo の腰の角度（JointStatePublisher）を、実機と同じ形の /hardware/joint_states（名前なし、CAN の番号順、
//   5 番目が腰のモータの角度）にして出す。chassis_joint_state・body_control_node・target_angle_node がこれを読む。
//   モータの角度 = waist_front_offset + waist_joint_direction × 腰の角度（body_controller と同じ決め方）
// - 実機の core_hardware がマイコンから受けて出し続ける非常停止スイッチの状態（/hardware/hardware_emergency）を、
//   押されていない（false）として出し続ける。emergency_handler はこれを受けるたびに /system/emergency/hazard_status
//   を出し直す。target_angle_node は起動時に緊急停止の印を立てて始め、hazard_status の false を受けるまで腰を
//   動かさないので、出し続けないと腰が止まったままになる
//
// 指令が command_timeout_s より古くなったら、その分を 0 にする（body_control_node が止まったときに走り続けない）。

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

#include <core_msgs/msg/can_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include "core_gazebo/sim_hardware_model.hpp"

namespace core_gazebo
{
namespace
{
namespace sh = sim_hardware;
using SteadyTime = std::chrono::steady_clock::time_point;

double declarePositive(rclcpp::Node & node, const std::string & name, const double value)
{
  const double declared = node.declare_parameter<double>(name, value);
  if (!(declared > 0.0) || !std::isfinite(declared)) {
    throw std::invalid_argument(name + " は正の有限の値にしてください: " + std::to_string(declared));
  }
  return declared;
}

bool isFresh(const std::optional<SteadyTime> & received, const SteadyTime now, const double timeout_s)
{
  return received.has_value() && std::chrono::duration<double>(now - *received).count() <= timeout_s;
}
}  // namespace

class SimHardware : public rclcpp::Node
{
public:
  explicit SimHardware(const rclcpp::NodeOptions & options)
  : Node("sim_hardware", options),
    geometry_{declarePositive(*this, "wheel_radius_m", sh::WheelGeometry{}.wheel_radius_m),
      declarePositive(*this, "body_width_m", sh::WheelGeometry{}.body_width_m)},
    command_timeout_s_(declarePositive(*this, "command_timeout_s", 0.2)),
    // body_controller.launch.py の waist_front_offset・waist_joint_direction と同じ値にする
    waist_front_offset_(declare_parameter<double>("waist_front_offset", 3.7822790145874023)),
    waist_joint_direction_(declare_parameter<double>("waist_joint_direction", 1.0)),
    waist_joint_name_(declare_parameter<std::string>("waist_joint_name", "base_to_chassis")),
    joint_count_(declare_parameter<int>("joint_count", 15))
  {
    if (!std::isfinite(waist_front_offset_) || waist_joint_name_.empty() || joint_count_ <= sh::kWaistId ||
      (waist_joint_direction_ != 1.0 && waist_joint_direction_ != -1.0))
    {
      throw std::invalid_argument(
              "waist_front_offset は有限、waist_joint_direction は 1 か -1、waist_joint_name は空でない値、"
              "joint_count は " +
              std::to_string(sh::kWaistId + 1) + " 以上にしてください");
    }
    const double watchdog_hz = declarePositive(*this, "watchdog_rate_hz", 20.0);

    upper_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      declare_parameter<std::string>("upper_cmd_topic", "/sim/upper_cmd_vel"), rclcpp::QoS(10));
    waist_publisher_ = create_publisher<std_msgs::msg::Float64>(
      declare_parameter<std::string>("waist_cmd_topic", "/sim/waist_cmd"), rclcpp::QoS(10));
    joint_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      declare_parameter<std::string>("hardware_joint_states_topic", "/hardware/joint_states"),
      rclcpp::QoS(10));
    emergency_publisher_ = create_publisher<std_msgs::msg::Bool>(
      declare_parameter<std::string>("hardware_emergency_topic", "/hardware/hardware_emergency"),
      rclcpp::QoS(10));
    can_subscription_ = create_subscription<core_msgs::msg::CANArray>(
      declare_parameter<std::string>("can_topic", "/hardware/can/tx"), rclcpp::QoS(100),
      [this](const core_msgs::msg::CANArray::ConstSharedPtr & msg) {onCan(*msg);});
    joint_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      declare_parameter<std::string>("sim_joint_states_topic", "/sim/joint_states"),
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr & msg) {onSimJointStates(*msg);});
    // body_control_node と target_angle_node も実時間のタイマーと時計で動くので、それに合わせる
    watchdog_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / watchdog_hz), [this]() {
        publishCommands();
        std_msgs::msg::Bool released;
        released.data = false;
        emergency_publisher_->publish(released);
      });
    RCLCPP_INFO(
      get_logger(), "足回りと腰の sim のハードウェアを始めます（車輪の半径 %.4f m、幅 %.4f m、指令の期限 %.2f 秒）",
      geometry_.wheel_radius_m, geometry_.body_width_m, command_timeout_s_);
  }

private:
  void onCan(const core_msgs::msg::CANArray & msg)
  {
    const SteadyTime now = std::chrono::steady_clock::now();
    bool updated = false;
    for (const auto & frame : msg.array) {
      const auto command = sh::decode(static_cast<int>(frame.id), frame.data);
      if (!command.has_value()) {
        continue;
      }
      if (command->kind == sh::CommandKind::kWheel) {
        const auto index = static_cast<std::size_t>(command->wheel);
        wheels_[index] = command->value;
        wheel_received_[index] = now;
      } else {
        // 腰の指令はモータの回る速さ。腰の角度の回る速さはその waist_joint_direction 倍
        waist_rate_ = waist_joint_direction_ * command->value;
        waist_received_ = now;
      }
      updated = true;
    }
    if (updated) {
      publishCommands();
    }
  }

  // 今の指令を Gazebo へ出す。古い指令は 0 にする
  void publishCommands()
  {
    const SteadyTime now = std::chrono::steady_clock::now();
    std::array<double, sh::kWheelCount> wheels{};
    for (std::size_t i = 0; i < wheels.size(); ++i) {
      wheels[i] = isFresh(wheel_received_[i], now, command_timeout_s_) ? wheels_[i] : 0.0;
    }
    const double waist_rate = isFresh(waist_received_, now, command_timeout_s_) ? waist_rate_ : 0.0;
    const sh::PlanarTwist chassis = sh::chassisTwistFromWheels(wheels, geometry_);
    // 腰の角度がまだ分からない間は 0 とみなす（起動直後。Gazebo の腰は 0 から始まる）
    const sh::PlanarTwist upper = sh::upperTwist(chassis, waist_angle_.value_or(0.0), waist_rate);

    geometry_msgs::msg::Twist twist;
    twist.linear.x = upper.vx;
    twist.linear.y = upper.vy;
    twist.angular.z = upper.wz;
    upper_publisher_->publish(twist);
    std_msgs::msg::Float64 waist;
    waist.data = waist_rate;
    waist_publisher_->publish(waist);
    last_wheels_ = wheels;
  }

  void onSimJointStates(const sensor_msgs::msg::JointState & msg)
  {
    const auto found = std::find(msg.name.begin(), msg.name.end(), waist_joint_name_);
    if (found == msg.name.end()) {
      return;
    }
    const auto index = static_cast<std::size_t>(std::distance(msg.name.begin(), found));
    if (index >= msg.position.size() || !std::isfinite(msg.position[index])) {
      return;
    }
    waist_angle_ = msg.position[index];
    const double waist_velocity = index < msg.velocity.size() && std::isfinite(msg.velocity[index]) ?
      msg.velocity[index] : 0.0;

    // 実機の core_hardware と同じ形（名前なし、CAN の番号順）。車輪は今の指令の速さだけを入れる
    sensor_msgs::msg::JointState out;
    out.header.stamp = msg.header.stamp;
    const auto count = static_cast<std::size_t>(joint_count_);
    out.position.assign(count, 0.0);
    out.velocity.assign(count, 0.0);
    out.effort.assign(count, 0.0);
    for (std::size_t i = 0; i < last_wheels_.size(); ++i) {
      out.velocity[i] = last_wheels_[i];
    }
    out.position[sh::kWaistId] = waist_front_offset_ + waist_joint_direction_ * *waist_angle_;
    out.velocity[sh::kWaistId] = waist_joint_direction_ * waist_velocity;
    joint_publisher_->publish(out);
  }

  const sh::WheelGeometry geometry_;
  const double command_timeout_s_;
  const double waist_front_offset_;
  const double waist_joint_direction_;
  const std::string waist_joint_name_;
  const int joint_count_;

  std::array<double, sh::kWheelCount> wheels_{};
  std::array<std::optional<SteadyTime>, sh::kWheelCount> wheel_received_{};
  std::array<double, sh::kWheelCount> last_wheels_{};
  double waist_rate_{0.0};
  std::optional<SteadyTime> waist_received_;
  std::optional<double> waist_angle_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr upper_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr waist_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_publisher_;
  rclcpp::Subscription<core_msgs::msg::CANArray>::SharedPtr can_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};
}  // namespace core_gazebo

RCLCPP_COMPONENTS_REGISTER_NODE(core_gazebo::SimHardware)
