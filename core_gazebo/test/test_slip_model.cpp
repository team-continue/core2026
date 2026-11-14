#include <cmath>

#include <gtest/gtest.h>

#include "core_gazebo/slip_model.hpp"

namespace
{
using core_gazebo::slip::accelerationFor;
using core_gazebo::slip::SlipParams;
using core_gazebo::slip::updateNoise;
using core_gazebo::slip::validate;

constexpr double kMaxLinear = 5.0;
constexpr double kMaxAngular = 10.0;
constexpr double kDt = 0.001;

TEST(SlipModel, DefaultFillsGapInOneStepWithinDriveLimit)
{
  // 既定は前の動き: 指令との差を 1 ステップで埋めようとし、駆動の上限で抑える
  const SlipParams params;
  const auto small = accelerationFor(
    params, kMaxLinear, kMaxAngular, 0.0, 1.0, 0.0, 0.0, 1.002, 0.0, 0.0, 0.0, 0.0, kDt);
  EXPECT_NEAR(small.ax, 2.0, 1e-9);
  EXPECT_NEAR(small.ay, 0.0, 1e-9);
  EXPECT_FALSE(small.sliding);
  const auto large = accelerationFor(
    params, kMaxLinear, kMaxAngular, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, kDt);
  EXPECT_NEAR(std::hypot(large.ax, large.ay), kMaxLinear, 1e-9);
  EXPECT_NEAR(large.ax, large.ay, 1e-9);
}

TEST(SlipModel, CommandIsRotatedByYawAndScaledByRatios)
{
  // 機体が北（yaw 90°）を向き、機体座標で前へ 1m/s、左へ 1m/s。横は 80% しか出ない
  SlipParams params;
  params.lateral_ratio = 0.8;
  params.time_constant_s = 1.0;
  const auto accel = accelerationFor(
    params, kMaxLinear, kMaxAngular, 0.5 * M_PI, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, kDt);
  // 目標の世界座標の速度は (-0.8, 1.0)。時定数 1 秒なので加速度も同じ値
  EXPECT_NEAR(accel.ax, -0.8, 1e-9);
  EXPECT_NEAR(accel.ay, 1.0, 1e-9);
}

TEST(SlipModel, SlidesWithKineticAccelBeyondStaticFriction)
{
  SlipParams params;
  params.time_constant_s = 0.1;
  params.static_friction_accel = 3.0;
  params.kinetic_friction_accel = 2.0;
  ASSERT_FALSE(validate(params).has_value());
  // 0.2m/s の差を 0.1 秒で埋める 2m/s2 は床が出せる
  const auto grip = accelerationFor(
    params, kMaxLinear, kMaxAngular, 0.0, 1.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.0, 0.0, kDt);
  EXPECT_FALSE(grip.sliding);
  EXPECT_NEAR(grip.ax, 2.0, 1e-9);
  // 0.4m/s の差（4m/s2）は床が出せないので滑り、向きを保って 2m/s2 しか出ない
  const auto slide = accelerationFor(
    params, kMaxLinear, kMaxAngular, 0.0, 1.0, 0.0, 0.0, 1.0, 0.4, 0.0, 0.0, 0.0, kDt);
  EXPECT_TRUE(slide.sliding);
  EXPECT_NEAR(std::hypot(slide.ax, slide.ay), 2.0, 1e-9);
  EXPECT_NEAR(slide.ax, 0.0, 1e-9);
  // 動摩擦を省けば静止摩擦と同じ
  params.kinetic_friction_accel = 0.0;
  const auto same = accelerationFor(
    params, kMaxLinear, kMaxAngular, 0.0, 1.0, 0.0, 0.0, 1.0, 0.4, 0.0, 0.0, 0.0, kDt);
  EXPECT_NEAR(std::hypot(same.ax, same.ay), 3.0, 1e-9);
}

TEST(SlipModel, YawFollowsRatioAndAngularLimit)
{
  SlipParams params;
  params.yaw_ratio = 0.5;
  params.time_constant_s = 0.1;
  const auto accel = accelerationFor(
    params, kMaxLinear, kMaxAngular, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, kDt);
  EXPECT_NEAR(accel.yaw_accel, 5.0, 1e-9);
  const auto limited = accelerationFor(
    params, kMaxLinear, kMaxAngular, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, kDt);
  EXPECT_NEAR(limited.yaw_accel, kMaxAngular, 1e-9);
}

TEST(SlipModel, NoiseScalesWithSpeedAndVanishesWhenStopped)
{
  SlipParams params;
  params.noise_ratio = 0.1;
  params.time_constant_s = 1.0;
  // 目標 2m/s、雑音 1（標準偏差 1 の値）なら、目標に 0.2m/s が足される
  const auto moving = accelerationFor(
    params, kMaxLinear, kMaxAngular, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 1.0, 0.0, kDt);
  EXPECT_NEAR(moving.ax, 2.2, 1e-9);
  const auto stopped = accelerationFor(
    params, kMaxLinear, kMaxAngular, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, kDt);
  EXPECT_NEAR(stopped.ax, 0.0, 1e-9);
  EXPECT_NEAR(stopped.ay, 0.0, 1e-9);
  // 雑音を使わない設定では値を進めない。使うときは平均へ戻りながら乱数を足す
  EXPECT_EQ(updateNoise(0.7, SlipParams{}, 0.01, 1.0), 0.7);
  EXPECT_NEAR(updateNoise(1.0, params, 0.01, 0.0), 1.0 - 0.01 / 0.5, 1e-12);
}

TEST(SlipModel, RejectsInvalidSettings)
{
  SlipParams params;
  EXPECT_FALSE(validate(params).has_value());
  params.kinetic_friction_accel = 2.0;
  params.static_friction_accel = 1.0;
  EXPECT_TRUE(validate(params).has_value());
  params = SlipParams{};
  params.lateral_ratio = -0.1;
  EXPECT_TRUE(validate(params).has_value());
  params = SlipParams{};
  params.noise_time_s = 0.0;
  EXPECT_TRUE(validate(params).has_value());
}
}  // namespace
