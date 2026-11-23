// 足回りの速度指令の加速度の制限を、上半身が回っても世界の座標で効かせるための計算。ROS に依存しない。
//
// body_control_node は /control/cmd_vel（上半身の座標）を、前の出力から軸ごとに加速度の上限までしか
// 変えない。TURRET で上半身が回ると、世界の座標で同じ速度でも上半身の座標の指令は回るので、その変化
// （速さ × 上半身の角速度。2.5m/s・0.9rad/s で約 2.3m/s²）が上限を使い切り、曲がる・止まる指令に
// 遅れて進む向きが上半身の回る向きへずれる。前の出力を、上半身の回った分だけ今の座標へ直してから
// 制限すれば、制限は世界の座標での速度の変化だけにかかる。

#ifndef CORE_BODY_CONTROLLER__VELOCITY_FRAME_HPP_
#define CORE_BODY_CONTROLLER__VELOCITY_FRAME_HPP_

#include <algorithm>
#include <cmath>

namespace core_body_controller
{
struct PlanarVelocity
{
  double x{0.0};
  double y{0.0};
};

// 上半身の座標の速度 velocity を、上半身が turned [rad] 回った後の上半身の座標で表す（世界の座標での
// 速度は同じ）
inline PlanarVelocity in_turned_frame(const PlanarVelocity & velocity, const double turned)
{
  const double c = std::cos(turned);
  const double s = std::sin(turned);
  return PlanarVelocity{c * velocity.x + s * velocity.y, -s * velocity.x + c * velocity.y};
}

// 向き previous_yaw から current_yaw へ回った角度（±π に巻き直す）。IMU の飛びで出力が大きく回らない
// よう ±max_step に収める。非有限なら 0
inline double limited_turn(
  const double previous_yaw, const double current_yaw,
  const double max_step)
{
  const double turned = std::remainder(current_yaw - previous_yaw, 2.0 * M_PI);
  if (!std::isfinite(turned)) {
    return 0.0;
  }
  return std::clamp(turned, -max_step, max_step);
}
}  // namespace core_body_controller

#endif  // CORE_BODY_CONTROLLER__VELOCITY_FRAME_HPP_
