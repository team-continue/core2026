#include <chrono>
#include <cmath>
#include <optional>

#include "core_msgs/msg/can.hpp"
#include "core_msgs/msg/can_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

#include "core_body_controller/control_mode.hpp"

class BodyControlNode : public rclcpp::Node
{
public:
  BodyControlNode();

private:
  rclcpp::Publisher<core_msgs::msg::CANArray>::SharedPtr
    body_control_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr body_omega_;
  // 車体の回転のうち、無限回転 (rotation_mode) を除いた操作・自律移動の分 [rad/s]
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr body_omega_manual_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rotation_flag_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  // 運転モード（/control/mode の "manual" / "auto"）。自律移動中は回転モードの R2 を R1 にする
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_mode_sub_;
  core_body_controller::ControlMode control_mode_ = core_body_controller::ControlMode::kManual;

  void timer_callback();
  // 前の出力（上半身の座標）を、前の周期から上半身が回った分だけ今の上半身の座標へ直す
  void rotate_output_by_upper_turn(std::chrono::steady_clock::time_point now);
  std::vector<float> invert_kinematics_calc(
    const geometry_msgs::msg::Twist & cmd_vel, const float & body_angle = 0);
  core_msgs::msg::CANArray gen_body_control_command(
    const std::vector<float> & body_control_command);
  void emergency_stop();

  geometry_msgs::msg::Twist cmd_vel_;
  // 車体の回転を、無限回転の分と操作の分に分けて加速度を制限する (cmd_vel_.angular.z はその和)
  double spin_omega_ = 0.0;
  double manual_omega_ = 0.0;
  geometry_msgs::msg::Twist latest_twist_;
  bool emergency_stop_flag_ = true;
  bool has_received_cmd_vel_ = false;
  int rotation_mode_ = 0;
  double latest_body_angle_ = 0;
  // 腰 (joint_states の ID 4) が車体の正面を向くときの角度 [rad] と、回る向き (1 / -1)
  double waist_front_offset_ = 3.7822790145874023;
  double waist_joint_direction_ = 1.0;
  std::chrono::steady_clock::time_point last_cmd_vel_time_;
  // 加速度の制限を世界の座標で効かせる（上半身の IMU で、前の出力を上半身の回った分だけ直してから制限する）。
  // false なら上半身の座標で制限する（TURRET で上半身が回ると、曲がる・止まる指令に遅れる）
  bool acceleration_in_world_frame_ = false;
  double imu_timeout_sec_ = 0.2;
  // 上半身の IMU の向き [rad] と受けた時刻、前の周期で使った向き
  std::optional<double> latest_upper_yaw_;
  std::chrono::steady_clock::time_point last_imu_time_;
  std::optional<double> last_tick_upper_yaw_;
  // 1 周期（10ms）に前の出力を回す角度の上限 [rad]（10rad/s 相当。IMU の飛びで出力が回らないように）
  static constexpr double MAX_UPPER_TURN_PER_TICK = 0.1;

  double ACCELERATION = 2;  // m/s^2
  double ROTATION_ACCELERATION = 1.0;
  double AUTO_ROTATION_VELOCITY = 0.3 * M_PI;
  double HIGH_ROTATION_VELOCITY = 1.0 * M_PI;
  double CMD_VEL_TIMEOUT_SEC = 0.2;

  static constexpr double TIMER_PERIOD = 0.01;  // 10 ms
};
