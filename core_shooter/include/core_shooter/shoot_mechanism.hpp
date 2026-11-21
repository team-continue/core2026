#ifndef CORE_SHOOTER__SHOOT_MECHANISM_HPP_
#define CORE_SHOOTER__SHOOT_MECHANISM_HPP_

#include <optional>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "core_shooter/parameter_utils.hpp"

namespace core_shooter
{

/// 発射機構の種類。yaml の shoot_mechanism（砲塔ごとに 1 か所）で選び、
/// 同じ砲塔の shooter_controller と magazine_manager が同じ値を参照する。
enum class ShootMechanism
{
  RotaryLoader,  ///< 装填モータを 1 発ごとに π rad 回す方式
  BeltFlick      ///< サーボでベルトを揺動させ（角度1→角度2→角度1）ディスクを飛ばす方式。リグリップなし
};

inline std::optional<ShootMechanism> parseShootMechanism(const std::string & name)
{
  if (name == "rotary_loader") {
    return ShootMechanism::RotaryLoader;
  }
  if (name == "belt_flick") {
    return ShootMechanism::BeltFlick;
  }
  return std::nullopt;
}

inline const char * shootMechanismName(ShootMechanism mechanism)
{
  switch (mechanism) {
    case ShootMechanism::RotaryLoader: return "rotary_loader";
    case ShootMechanism::BeltFlick: return "belt_flick";
  }
  return "unknown";
}

/// shoot_mechanism パラメータを宣言して読み、不正な値なら FATAL を出して例外を投げる。
inline ShootMechanism declareShootMechanism(rclcpp::Node & node)
{
  const std::string name =
    declareAndGet<std::string>(node, "shoot_mechanism", "rotary_loader");
  const auto mechanism = parseShootMechanism(name);
  if (!mechanism) {
    RCLCPP_FATAL(
      node.get_logger(),
      "Invalid shoot_mechanism=\"%s\" (expected \"rotary_loader\" or \"belt_flick\")",
      name.c_str());
    throw std::runtime_error("invalid shoot_mechanism");
  }
  return *mechanism;
}

}  // namespace core_shooter

#endif  // CORE_SHOOTER__SHOOT_MECHANISM_HPP_
