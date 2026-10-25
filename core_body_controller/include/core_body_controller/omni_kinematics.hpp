#pragma once

#include <array>
#include <cmath>
#include <optional>

namespace core_body_controller
{

struct BodyVelocity
{
  double linear_x;
  double linear_y;
  double angular_z;
};

// body_control_node::invert_kinematics_calc の逆変換。
// wheel_velocities は車輪 0〜3 の角速度 [rad/s]、body_angle は腰の角度 [rad]。
// 戻り値は cmd_vel と同じ座標系 (腰の角度で回す前) の並進速度と、足回りの角速度。
// 車輪の並びは body_control_node と同じ:
//        x
//   1 [/]  [\] 0
//   2 [\]  [/] 3
inline std::optional<BodyVelocity> forward_kinematics(
  const std::array<double, 4> & wheel_velocities, double body_angle, double wheel_radius,
  double body_width)
{
  for (const double w : wheel_velocities) {
    if (!std::isfinite(w)) {
      return std::nullopt;
    }
  }
  if (!std::isfinite(body_angle) || !std::isfinite(wheel_radius) || !std::isfinite(body_width) ||
    wheel_radius <= 0.0 || body_width <= 0.0)
  {
    return std::nullopt;
  }

  // 逆運動学は w_i = -u_i / r なので u_i = -r * w_i に戻してから解く
  std::array<double, 4> u{};
  for (std::size_t i = 0; i < u.size(); ++i) {
    u[i] = -wheel_radius * wheel_velocities[i];
  }

  const double c = std::cos(M_PI / 4);
  const double s = std::sin(M_PI / 4);
  const double l = std::sqrt(2.0) * body_width / 2;

  // 4 輪の最小二乗解 (4 輪の速度に矛盾があっても平均化される)
  const double vx_body = (u[0] + u[1] - u[2] - u[3]) / (4 * c);
  const double vy_body = (-u[0] + u[1] + u[2] - u[3]) / (4 * s);
  const double omega = -(u[0] + u[1] + u[2] + u[3]) / (4 * l);

  // 逆運動学で -body_angle 回したのを戻す
  const double cos_a = std::cos(body_angle);
  const double sin_a = std::sin(body_angle);
  return BodyVelocity{
    vx_body * cos_a - vy_body * sin_a,
    vx_body * sin_a + vy_body * cos_a,
    omega,
  };
}

}  // namespace core_body_controller
