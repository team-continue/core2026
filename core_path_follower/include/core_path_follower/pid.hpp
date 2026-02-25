// Copyright 2026 team-continue
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
