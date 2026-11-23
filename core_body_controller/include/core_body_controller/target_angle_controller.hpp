#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

namespace core_body_controller
{

inline double normalize_angle(double angle) {return std::atan2(std::sin(angle), std::cos(angle));}

// 腰 (上半身) の目標の向きを何で回すか
enum class YawMode
{
  // cmd_vel の angular.z で回す。車体と一緒に回り、車体の正面に対する向きを保つ
  kFront = 0,
  // yaw_target_rate で回す。車体が曲がってもワールドでの向きを保つ
  kTurret = 1,
};

// /control/yaw_mode の値をモードにする。知らない値なら nullopt
inline std::optional<YawMode> to_yaw_mode(int value)
{
  switch (value) {
    case static_cast<int>(YawMode::kFront):
      return YawMode::kFront;
    case static_cast<int>(YawMode::kTurret):
      return YawMode::kTurret;
    default:
      return std::nullopt;
  }
}

// 次の目標の向き [rad]。モードに応じた回転の速さ [rad/s] で回す。
// その速さが届いていない (途絶えた) ときは目標をそのまま保つ
inline double next_world_target(
  double world_target, YawMode mode, std::optional<double> cmd_vel_angular_z,
  std::optional<double> target_rate, double dt)
{
  const std::optional<double> rate = mode == YawMode::kFront ? cmd_vel_angular_z : target_rate;
  if (!rate || !std::isfinite(*rate) || !std::isfinite(dt) || dt <= 0.0) {
    return world_target;
  }
  return normalize_angle(world_target + *rate * dt);
}

// current から goal へ、近いほうの向きに最大 max_step [rad] だけ進めた角度 (-π〜π)
inline double step_toward_angle(double current, double goal, double max_step)
{
  const double diff = normalize_angle(goal - current);
  if (!std::isfinite(diff) || !std::isfinite(max_step) || max_step <= 0.0) {
    return normalize_angle(current);
  }
  return normalize_angle(current + std::clamp(diff, -max_step, max_step));
}

// 腰を車体の正面に向ける目標の向き [rad]。IMU は上半身にあるので、
// 車体の正面の向き = 上半身の向き + 腰の角度。腰の角度は waist_direction × (waist_position −
// waist_front_offset) で、上半身から見た車体の向き（車体 − 上半身）。body_control_node が上半身の座標の
// 指令を車体の座標へ回す角度（invert_kinematics_calc）と同じ決め方
// waist_direction は腰のモータの角度の正の向きが、ワールドの yaw の正の向きと同じなら 1、逆なら -1
inline double front_world_target(
  double world_angle, double waist_position, double waist_front_offset, double waist_direction)
{
  return normalize_angle(world_angle + waist_direction * (waist_position - waist_front_offset));
}

// 腰のモータの回る速さに足す先回り [rad/s] = 車体の回転（body_omega）− 上半身を回したい速さ。
// 上半身の回る速さ = 車体の回る速さ − 腰の回る速さ なので、上半身は回したい速さで回り始め、帰還は残りの
// ずれだけを詰める。先回りが車体の回転だけだと、上半身は帰還（比例 2.0）だけで目標を追い、回したい速さの
// 約半分 [rad] 遅れる（0.5rad/s で約 0.25rad）。
// 上半身を回したい速さは、FRONT なら操作の回転（body_omega_manual。無限回転の分は打ち消す）、TURRET なら
// 操作の yaw_target_rate（車体が回ってもワールドでの向きを保つ）。届いていなければ nullopt を渡し、車体の
// 回転を全部打ち消す。body_omega がなければ 0
inline double waist_feedforward(std::optional<double> body_omega, std::optional<double> upper_rate)
{
  if (!body_omega || !std::isfinite(*body_omega)) {
    return 0.0;
  }
  if (upper_rate && std::isfinite(*upper_rate)) {
    return *body_omega - *upper_rate;
  }
  return *body_omega;
}

// 操作の yaw_target_rate が動いているか。0 でない速さを最後に受けてから hold_sec の間は動いているとみなす
// （操作を止めても、すぐには自律移動の向きに戻さない）
class OperatorYawActivity
{
public:
  explicit OperatorYawActivity(double hold_sec, double threshold = 1e-3)
  : hold_sec_(hold_sec), threshold_(threshold) {}

  void update(double rate, double now_sec)
  {
    if (std::isfinite(rate) && std::abs(rate) > threshold_) {
      last_active_sec_ = now_sec;
    }
  }

  bool active(double now_sec) const
  {
    return last_active_sec_.has_value() && now_sec - *last_active_sec_ <= hold_sec_;
  }

private:
  double hold_sec_;
  double threshold_;
  std::optional<double> last_active_sec_;
};

// TURRET で上半身の目標を回す速さ。操作が動いていれば操作、そうでなければ自律移動（auto/yaw_target_rate）、
// どちらも届いていなければ操作の値（届いていなければ nullopt で、目標を保つ）
inline std::optional<double> turret_rate(
  std::optional<double> operator_rate, bool operator_active, std::optional<double> auto_rate)
{
  if (operator_active && operator_rate) {
    return operator_rate;
  }
  if (auto_rate) {
    return auto_rate;
  }
  return operator_rate;
}

// 腰のモードの切り替えを受けるか。車体が回り続けている（回転モード中）間は、TURRET から FRONT へは
// 切り替えない（FRONT は上半身を車体と一体に動かすモードなので、回転モードと組み合わせない）
inline bool accept_yaw_mode(YawMode current, YawMode requested, int rotation_mode)
{
  return !(current == YawMode::kTurret && requested == YawMode::kFront && rotation_mode != 0);
}

inline bool quaternion_to_yaw(double x, double y, double z, double w, double & yaw)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(w)) {
    return false;
  }

  const double norm_squared = x * x + y * y + z * z + w * w;
  constexpr double MIN_NORM_SQUARED = 1.0e-12;
  constexpr double MAX_NORM_SQUARED = 1.0e12;
  if (
    !std::isfinite(norm_squared) || norm_squared < MIN_NORM_SQUARED ||
    norm_squared > MAX_NORM_SQUARED)
  {
    return false;
  }

  const double inverse_norm = 1.0 / std::sqrt(norm_squared);
  x *= inverse_norm;
  y *= inverse_norm;
  z *= inverse_norm;
  w *= inverse_norm;
  yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return std::isfinite(yaw);
}

class YawTracker
{
public:
  bool update(double measured_yaw, bool stream_is_continuous)
  {
    if (!std::isfinite(measured_yaw)) {
      return false;
    }

    if (!has_measurement_) {
      previous_measured_yaw_ = measured_yaw;
      has_measurement_ = true;
      return true;
    }

    if (stream_is_continuous) {
      continuous_yaw_ += normalize_angle(measured_yaw - previous_measured_yaw_);
    }
    previous_measured_yaw_ = measured_yaw;
    return true;
  }

  double yaw() const {return continuous_yaw_;}

private:
  double continuous_yaw_ = 0.0;
  double previous_measured_yaw_ = 0.0;
  bool has_measurement_ = false;
};

inline double apply_rate_limit(double current, double target, double max_rate, double dt)
{
  const double max_step = max_rate * dt;
  return current + std::clamp(target - current, -max_step, max_step);
}

class TargetAngleController
{
public:
  TargetAngleController(
    double kp, double ki, double kd, double output_limit, double output_acceleration_limit,
    double deadband = 0.1)
  : kp_(kp),
    ki_(ki),
    kd_(kd),
    output_limit_(output_limit),
    output_acceleration_limit_(output_acceleration_limit),
    deadband_(deadband)
  {
  }

  double update(double angle_error, double feedforward, double dt)
  {
    if (
      !std::isfinite(angle_error) || !std::isfinite(feedforward) || !std::isfinite(dt) ||
      dt <= 0.0)
    {
      return output_;
    }

    integral_ += angle_error * dt;
    const double derivative = has_previous_error_ ? (angle_error - previous_error_) / dt : 0.0;
    previous_error_ = angle_error;
    has_previous_error_ = true;

    const double feedback = kp_ * angle_error + ki_ * integral_ + kd_ * derivative;
    double target = std::clamp(-feedback + feedforward, -output_limit_, output_limit_);
    if (std::abs(target) < deadband_) {
      target = 0.0;
    }
    output_ = apply_rate_limit(output_, target, output_acceleration_limit_, dt);
    output_ = std::clamp(output_, -output_limit_, output_limit_);
    return output_;
  }

  void reset()
  {
    integral_ = 0.0;
    previous_error_ = 0.0;
    output_ = 0.0;
    has_previous_error_ = false;
  }

  double output() const {return output_;}

private:
  double kp_;
  double ki_;
  double kd_;
  double output_limit_;
  double output_acceleration_limit_;
  double deadband_;
  double integral_ = 0.0;
  double previous_error_ = 0.0;
  double output_ = 0.0;
  bool has_previous_error_ = false;
};

}  // namespace core_body_controller
