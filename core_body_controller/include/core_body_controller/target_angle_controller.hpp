#pragma once

#include <algorithm>
#include <cmath>

namespace core_body_controller
{

inline double normalize_angle(double angle) {return std::atan2(std::sin(angle), std::cos(angle));}

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
