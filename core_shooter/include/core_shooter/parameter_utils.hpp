#ifndef CORE_SHOOTER__PARAMETER_UTILS_HPP_
#define CORE_SHOOTER__PARAMETER_UTILS_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace core_shooter
{

/// パラメータを宣言し、その値（override があればそちら）を返す。
///
/// `declare_parameter` と `get_parameter` の定型 2 行をまとめるためのヘルパ。
template<typename T>
T declareAndGet(rclcpp::Node & node, const std::string & name, const T & default_value)
{
  node.declare_parameter<T>(name, default_value);
  T value = default_value;
  node.get_parameter(name, value);
  return value;
}

}  // namespace core_shooter

#endif  // CORE_SHOOTER__PARAMETER_UTILS_HPP_
