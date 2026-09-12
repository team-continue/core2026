#ifndef CORE_SHOOTER__CAN_COMMAND_HPP_
#define CORE_SHOOTER__CAN_COMMAND_HPP_

#include "rclcpp/rclcpp.hpp"
#include "core_msgs/msg/can.hpp"
#include "core_msgs/msg/can_array.hpp"

namespace core_shooter
{

/// 単一モータ宛の CAN コマンドを 1 フレームだけ含む CANArray を生成する。
inline core_msgs::msg::CANArray makeMotorCommand(int id, float data)
{
  core_msgs::msg::CANArray can_array;
  core_msgs::msg::CAN can;
  can.id = id;
  can.data.push_back(data);
  can_array.array.push_back(can);
  return can_array;
}

/// 単一モータ宛の CAN コマンドを publish する。
inline void publishMotorCommand(
  const rclcpp::Publisher<core_msgs::msg::CANArray>::SharedPtr & publisher,
  int id,
  float data)
{
  publisher->publish(makeMotorCommand(id, data));
}

}  // namespace core_shooter

#endif  // CORE_SHOOTER__CAN_COMMAND_HPP_
