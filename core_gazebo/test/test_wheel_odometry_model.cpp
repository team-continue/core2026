#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "core_gazebo/wheel_odometry_model.hpp"

namespace
{
using core_gazebo::wheel_odometry::bodyTwistBetween;
using core_gazebo::wheel_odometry::BodyTwist;
using core_gazebo::wheel_odometry::ErrorParams;
using core_gazebo::wheel_odometry::PlanarPose;
using core_gazebo::wheel_odometry::WheelErrorModel;

constexpr double kDt = 0.02;

ErrorParams noErrors()
{
  ErrorParams params;
  params.scale_std_x = 0.0;
  params.scale_std_y = 0.0;
  params.scale_std_yaw = 0.0;
  params.drift_std_linear_mps = 0.0;
  params.drift_std_angular_radps = 0.0;
  params.accel_slip_s = 0.0;
  params.burst_rate_hz = 0.0;
  params.noise_std_linear_mps = 0.0;
  params.noise_std_angular_radps = 0.0;
  return params;
}

TEST(BodyTwistBetween, UsesBodyFrame)
{
  // 左（+y）を向いて、地図の +y に 1m/s、-x に 0.5m/s 動いた: 前に 1m/s、左に 0.5m/s
  const BodyTwist twist = bodyTwistBetween(
    PlanarPose{0.0, 0.0, 0.0, M_PI / 2.0}, PlanarPose{0.02, -0.01, 0.02, M_PI / 2.0});
  EXPECT_NEAR(twist.vx, 1.0, 1e-9);
  EXPECT_NEAR(twist.vy, 0.5, 1e-9);
  EXPECT_NEAR(twist.wz, 0.0, 1e-9);
}

TEST(BodyTwistBetween, WrapsYawRate)
{
  const BodyTwist twist = bodyTwistBetween(
    PlanarPose{0.0, 0.0, 0.0, M_PI - 0.01}, PlanarPose{0.02, 0.0, 0.0, -M_PI + 0.01});
  EXPECT_NEAR(twist.wz, 1.0, 1e-9);
}

TEST(BodyTwistBetween, RejectsSameStamp)
{
  EXPECT_THROW(
    bodyTwistBetween(PlanarPose{1.0, 0.0, 0.0, 0.0}, PlanarPose{1.0, 1.0, 0.0, 0.0}),
    std::invalid_argument);
}

TEST(WheelErrorModel, PassesTruthWithoutErrors)
{
  WheelErrorModel model(noErrors(), 7);
  const BodyTwist out = model.apply(BodyTwist{1.0, -0.5, 0.3}, BodyTwist{4.0, 0.0, 1.0}, kDt);
  EXPECT_DOUBLE_EQ(out.vx, 1.0);
  EXPECT_DOUBLE_EQ(out.vy, -0.5);
  EXPECT_DOUBLE_EQ(out.wz, 0.3);
}

TEST(WheelErrorModel, ScalesEachAxisOnceAndRepeatsWithSeed)
{
  ErrorParams params = noErrors();
  params.scale_std_x = 0.05;
  params.scale_std_y = 0.05;
  WheelErrorModel first(params, 3);
  WheelErrorModel second(params, 3);
  const BodyTwist truth{2.0, 1.0, 0.0};
  const BodyTwist a = first.apply(truth, BodyTwist{}, kDt);
  const BodyTwist b = first.apply(truth, BodyTwist{}, kDt);
  EXPECT_DOUBLE_EQ(a.vx, first.scale().vx * 2.0);
  EXPECT_DOUBLE_EQ(a.vy, first.scale().vy * 1.0);
  EXPECT_DOUBLE_EQ(a.vx, b.vx);
  EXPECT_NE(first.scale().vx, 1.0);
  EXPECT_DOUBLE_EQ(second.apply(truth, BodyTwist{}, kDt).vx, a.vx);
}

TEST(WheelErrorModel, SlipsWithAcceleration)
{
  ErrorParams params = noErrors();
  params.accel_slip_s = 0.01;
  WheelErrorModel model(params, 1);
  EXPECT_NEAR(model.apply(BodyTwist{1.0, 0.0, 0.0}, BodyTwist{5.0, 0.0, 0.0}, kDt).vx, 1.05, 1e-12);
  EXPECT_NEAR(model.apply(BodyTwist{1.0, 0.0, 0.0}, BodyTwist{-5.0, 0.0, 0.0}, kDt).vx, 0.95, 1e-12);
}

TEST(WheelErrorModel, DriftKeepsStationarySpread)
{
  ErrorParams params = noErrors();
  params.drift_std_linear_mps = 0.01;
  params.drift_time_constant_s = 1.0;
  WheelErrorModel model(params, 11);
  double sum_squares = 0.0;
  constexpr int kSteps = 200000;
  for (int i = 0; i < kSteps; ++i) {
    const double error = model.apply(BodyTwist{}, BodyTwist{}, kDt).vx;
    sum_squares += error * error;
  }
  EXPECT_NEAR(std::sqrt(sum_squares / kSteps), 0.01, 0.002);
}

TEST(WheelErrorModel, BurstsHaveConfiguredSpeedAndEnd)
{
  ErrorParams params = noErrors();
  params.burst_rate_hz = 1.0e6;
  params.burst_speed_min_mps = 0.3;
  params.burst_speed_max_mps = 0.5;
  params.burst_duration_min_s = 0.1;
  params.burst_duration_max_s = 0.1;
  WheelErrorModel model(params, 5);
  // 頻度が非常に高いので、すぐに始まる。長さ 0.1 秒（5 周期）のあいだ同じ大きさ
  for (int i = 0; i < 5; ++i) {
    const BodyTwist out = model.apply(BodyTwist{}, BodyTwist{}, kDt);
    const double speed = std::hypot(out.vx, out.vy);
    EXPECT_GE(speed, 0.3 - 1e-12);
    EXPECT_LE(speed, 0.5 + 1e-12);
    EXPECT_DOUBLE_EQ(out.wz, 0.0);
  }
  // 長さが過ぎると、いったん 0 に戻る
  const BodyTwist after = model.apply(BodyTwist{}, BodyTwist{}, kDt);
  EXPECT_DOUBLE_EQ(std::hypot(after.vx, after.vy), 0.0);
}

TEST(WheelErrorModel, VarianceGrowsWithSpeed)
{
  const ErrorParams params;
  const WheelErrorModel model(params, 1);
  const BodyTwist slow = model.variance(BodyTwist{0.0, 0.0, 0.0});
  const BodyTwist fast = model.variance(BodyTwist{3.0, 2.0, 1.0});
  EXPECT_GT(slow.vx, 0.0);
  EXPECT_GT(fast.vx, slow.vx);
  EXPECT_GT(fast.vy, slow.vy);
  EXPECT_GT(fast.wz, slow.wz);
}

TEST(WheelErrorModel, RejectsInvalidParams)
{
  ErrorParams negative;
  negative.noise_std_linear_mps = -0.1;
  EXPECT_THROW(WheelErrorModel(negative, 1), std::invalid_argument);
  ErrorParams reversed;
  reversed.burst_speed_min_mps = 1.0;
  reversed.burst_speed_max_mps = 0.5;
  EXPECT_THROW(WheelErrorModel(reversed, 1), std::invalid_argument);
}
}  // namespace
