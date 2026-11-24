// core_hardware の /hardware/joint_states（関節の名前がなく、CAN の番号順に並ぶ）から腰の角度を取り出し、
// URDF の関節名（base_to_chassis）を付けて出す。robot_state_publisher がこれで base_link → chassis_link の
// TF を出し、計画と安全停止が衝突の形を駆動部の向きに合わせて回す。
//
// 角度は body_control_node と同じ決め方（waist_joint_direction × (position[4] − waist_front_offset)）。
// body_control_node はこの角度で上部の座標の指令を駆動部の座標へ回すので、駆動部は base_link から見て
// この角度だけ回っている。2 つのパラメータは body_control_node・target_angle_node と同じ値にする
// （body_controller.launch.py が同じ引数を渡す）

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace core_body_controller
{
class ChassisJointStateNode : public rclcpp::Node
{
public:
  ChassisJointStateNode()
  : Node("chassis_joint_state")
  {
    joint_index_ = declare_parameter<int>("joint_index", 4);
    // 腰が車体の正面を向くときのモータの角度と、回る向き（body_control_node と同じ既定値）
    waist_front_offset_ = declare_parameter<double>("waist_front_offset", 3.084347);
    waist_joint_direction_ = declare_parameter<double>("waist_joint_direction", 1.0);
    joint_name_ = declare_parameter<std::string>("joint_name", "base_to_chassis");
    if (joint_index_ < 0 || !std::isfinite(waist_front_offset_) || joint_name_.empty() ||
      (waist_joint_direction_ != 1.0 && waist_joint_direction_ != -1.0))
    {
      throw std::invalid_argument(
              "joint_index は 0 以上、waist_front_offset は有限、waist_joint_direction は 1 か -1、"
              "joint_name は空でない値にしてください");
    }

    publisher_ =
      create_publisher<sensor_msgs::msg::JointState>("chassis/joint_states", rclcpp::QoS(10));
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/hardware/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr & msg) {onJointStates(*msg);});
  }

private:
  void onJointStates(const sensor_msgs::msg::JointState & msg)
  {
    const auto index = static_cast<std::size_t>(joint_index_);
    if (msg.position.size() <= index || !std::isfinite(msg.position[index])) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "腰の角度がありません（position の数 %zu、必要な番号 %zu）", msg.position.size(), index);
      return;
    }
    sensor_msgs::msg::JointState out;
    // core_hardware は時刻を入れないことがあるので、受けた時刻で出す
    out.header.stamp = msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0 ?
      now() : rclcpp::Time(msg.header.stamp);
    out.name.push_back(joint_name_);
    out.position.push_back(waist_joint_direction_ * (msg.position[index] - waist_front_offset_));
    if (msg.velocity.size() > index && std::isfinite(msg.velocity[index])) {
      out.velocity.push_back(waist_joint_direction_ * msg.velocity[index]);
    }
    publisher_->publish(out);
  }

  int joint_index_{4};
  double waist_front_offset_{0.0};
  double waist_joint_direction_{1.0};
  std::string joint_name_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};
}  // namespace core_body_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<core_body_controller::ChassisJointStateNode>());
  rclcpp::shutdown();
  return 0;
}
