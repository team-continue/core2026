// Copyright 2026 team-continue
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cmath>

namespace core_path_follower
{

class PID
{
public:
  explicit PID(double p = 0.0, double i = 0.0, double d = 0.0)
  : kp_(p), ki_(i), kd_(d), prev_(0.0), integral_(0.0)
  {
  }

  double update(double error, double dt)
  {
    if (dt <= 0.0) {
      return 0.0;
    }
    integral_ += error * dt;
    const double derivative = (error - prev_) / dt;
    prev_ = error;
    return kp_ * error + ki_ * integral_ + kd_ * derivative;
  }

  void reset()
  {
    prev_ = 0.0;
    integral_ = 0.0;
  }

  double kp_, ki_, kd_;

private:
  double prev_;
  double integral_;
};

}  // namespace core_path_follower
