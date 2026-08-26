#pragma once

#include <algorithm>
#include <cmath>

namespace core_body_controller
{

inline double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline double integrate_yaw(
  double current_angle, double angular_velocity_z, double yaw_bias, double dt)
{
  return normalize_angle(current_angle + (-angular_velocity_z + yaw_bias) * dt);
}

inline double apply_rate_limit(
  double current, double target, double max_rate, double dt)
{
  const double max_step = max_rate * dt;
  return current + std::clamp(target - current, -max_step, max_step);
}

class TargetAngleController
{
public:
  TargetAngleController(
    double kp, double ki, double kd, double output_limit,
    double output_acceleration_limit, double deadband = 0.1)
  : kp_(kp), ki_(ki), kd_(kd), output_limit_(output_limit),
    output_acceleration_limit_(output_acceleration_limit), deadband_(deadband)
  {
  }

  double update(double angle_error, double feedforward, double dt)
  {
    if (!std::isfinite(angle_error) || !std::isfinite(feedforward) ||
      !std::isfinite(dt) || dt <= 0.0)
    {
      return output_;
    }

    integral_ += angle_error * dt;
    const double derivative = has_previous_error_ ? (angle_error - previous_error_) / dt : 0.0;
    previous_error_ = angle_error;
    has_previous_error_ = true;

    const double feedback = kp_ * angle_error + ki_ * integral_ + kd_ * derivative;
    double target = std::clamp(
      -feedback + feedforward, -output_limit_, output_limit_);
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

  double output() const
  {
    return output_;
  }

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
