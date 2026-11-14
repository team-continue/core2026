// 床と車輪の滑りを含む、全方位移動の機体の水平の運動（omni_force_drive が加える加速度）の計算。
// ROS と Gazebo に依存しないので、プラグインと分けてテストする。
// core_planning の tracking_bench（Gazebo を使わない試験台）に同じ式があるので、変えるときは両方を直す。
//
// - 指令の機体座標の速度に、前後・横・回転の割合（床で出せる割合）をかけたものを目標にする
// - 目標の速度に、時間相関のある雑音（床のむら）を、目標の速さに比例する大きさで足す（止まっていれば 0）
// - 目標と今の速度の差を時定数で埋める加速度（時定数 0 なら 1 ステップで埋める）を、駆動の上限で抑える
// - 床が出せる水平の加速度（静止摩擦、摩擦円）を超えたら滑り、動摩擦の加速度しか出ない

#ifndef CORE_GAZEBO__SLIP_MODEL_HPP_
#define CORE_GAZEBO__SLIP_MODEL_HPP_

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

namespace core_gazebo::slip
{
// 既定は滑らない（前の動き: 指令との差を 1 ステップで埋め、駆動の上限だけで抑える）
struct SlipParams
{
  // 指令の前後・横の速度と角速度のうち、床で出せる割合
  double longitudinal_ratio{1.0};
  double lateral_ratio{1.0};
  double yaw_ratio{1.0};
  // 目標の速度との差を埋める時定数 [s]。0 なら 1 ステップで埋める
  double time_constant_s{0.0};
  // 床が出せる水平の加速度（静止摩擦）[m/s2]。0 なら滑らない
  double static_friction_accel{0.0};
  // 滑っている間に出る加速度（動摩擦）[m/s2]。0 なら静止摩擦と同じ
  double kinetic_friction_accel{0.0};
  // 床のむらの雑音の大きさ（目標の速さに対する割合、標準偏差）と相関時間 [s]
  double noise_ratio{0.0};
  double noise_time_s{0.5};
};

// 設定が正しくなければ理由を返す
inline std::optional<std::string> validate(const SlipParams & params)
{
  if (!(params.longitudinal_ratio >= 0.0) || !(params.lateral_ratio >= 0.0) ||
    !(params.yaw_ratio >= 0.0))
  {
    return "床で出せる割合（longitudinal_ratio、lateral_ratio、yaw_ratio）は 0 以上にしてください";
  }
  if (!(params.time_constant_s >= 0.0) || !(params.static_friction_accel >= 0.0) ||
    !(params.kinetic_friction_accel >= 0.0) || !(params.noise_ratio >= 0.0))
  {
    return "時定数、摩擦、雑音は 0 以上にしてください";
  }
  if (params.kinetic_friction_accel > params.static_friction_accel) {
    return "動摩擦（kinetic_friction_accel）は静止摩擦（static_friction_accel）以下にしてください";
  }
  if (!(params.noise_time_s > 0.0)) {
    return "雑音の相関時間（noise_time）は正にしてください";
  }
  return std::nullopt;
}

// 滑っている間に出る加速度。動摩擦を 0 にしていれば静止摩擦と同じ
inline double kineticAccel(const SlipParams & params)
{
  return params.kinetic_friction_accel > 0.0 ?
         params.kinetic_friction_accel : params.static_friction_accel;
}

struct PlanarAccel
{
  // 世界座標の水平の加速度 [m/s2] と、yaw の角加速度 [rad/s2]
  double ax{0.0};
  double ay{0.0};
  double yaw_accel{0.0};
  // 床が出せる加速度を超えて滑っているか
  bool sliding{false};
};

// 機体の向き yaw、世界座標の今の速度 (vx, vy) と角速度 wz、機体座標の指令 (command_vx, command_vy,
// command_wz)、床のむら（標準偏差 1 の雑音 noise_x, noise_y、updateNoise で進める）から、この
// ステップ dt [s] に出す加速度を求める。max_linear_accel と max_angular_accel は駆動の上限
inline PlanarAccel accelerationFor(
  const SlipParams & params, const double max_linear_accel, const double max_angular_accel,
  const double yaw, const double vx, const double vy, const double wz, const double command_vx,
  const double command_vy, const double command_wz, const double noise_x, const double noise_y,
  const double dt)
{
  const double bx = command_vx * params.longitudinal_ratio;
  const double by = command_vy * params.lateral_ratio;
  const double noise = params.noise_ratio * std::hypot(bx, by);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double tau = std::max(params.time_constant_s, dt);
  PlanarAccel result;
  result.ax = (c * bx - s * by + noise * noise_x - vx) / tau;
  result.ay = (s * bx + c * by + noise * noise_y - vy) / tau;
  const double demand = std::hypot(result.ax, result.ay);
  if (demand > max_linear_accel) {
    result.ax *= max_linear_accel / demand;
    result.ay *= max_linear_accel / demand;
  }
  const double accel = std::hypot(result.ax, result.ay);
  result.sliding = params.static_friction_accel > 0.0 && accel > params.static_friction_accel;
  if (result.sliding) {
    result.ax *= kineticAccel(params) / accel;
    result.ay *= kineticAccel(params) / accel;
  }
  result.yaw_accel = std::clamp(
    (command_wz * params.yaw_ratio - wz) / tau, -max_angular_accel, max_angular_accel);
  return result;
}

// 標準偏差 1、相関時間 noise_time_s の雑音（Ornstein-Uhlenbeck）を dt [s] 進める。normal は標準正規分布の乱数
inline double updateNoise(
  const double value, const SlipParams & params, const double dt, const double normal)
{
  if (!(params.noise_ratio > 0.0) || !(params.noise_time_s > 0.0)) {
    return value;
  }
  const double decay = dt / params.noise_time_s;
  return value - value * decay + std::sqrt(2.0 * decay) * normal;
}
}  // namespace core_gazebo::slip

#endif  // CORE_GAZEBO__SLIP_MODEL_HPP_
