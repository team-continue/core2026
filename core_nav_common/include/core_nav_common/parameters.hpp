// 範囲検査付きで ROS 2 パラメータを宣言する共通処理。
//
// 範囲外の値と、起動時しか変えられない値の変更は、rclcpp が理由を付けて拒否する。

#ifndef CORE_NAV_COMMON__PARAMETERS_HPP_
#define CORE_NAV_COMMON__PARAMETERS_HPP_

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace core_nav_common
{
inline double declareDouble(
  rclcpp::Node & node, const std::string & name, const double default_value, const double min,
  const double max, const std::string & description, const bool read_only = false)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = read_only;
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = min;
  range.to_value = max;
  range.step = 0.0;
  descriptor.floating_point_range.push_back(range);
  return node.declare_parameter<double>(name, default_value, descriptor);
}

inline std::int64_t declareInt(
  rclcpp::Node & node, const std::string & name, const std::int64_t default_value,
  const std::int64_t min, const std::int64_t max, const std::string & description,
  const bool read_only = false)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = read_only;
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = min;
  range.to_value = max;
  range.step = 1;
  descriptor.integer_range.push_back(range);
  return node.declare_parameter<std::int64_t>(name, default_value, descriptor);
}

inline bool declareBool(
  rclcpp::Node & node, const std::string & name, const bool default_value,
  const std::string & description, const bool read_only = false)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = read_only;
  return node.declare_parameter<bool>(name, default_value, descriptor);
}

inline std::string declareString(
  rclcpp::Node & node, const std::string & name, const std::string & default_value,
  const std::string & description, const bool read_only = false)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = read_only;
  return node.declare_parameter<std::string>(name, default_value, descriptor);
}
}  // namespace core_nav_common

#endif  // CORE_NAV_COMMON__PARAMETERS_HPP_
