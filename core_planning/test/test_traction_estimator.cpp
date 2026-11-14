#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "core_planning/drive_delay.hpp"
#include "core_planning/mppi.hpp"
#include "core_planning/traction_estimator.hpp"

namespace
{
using core_planning::applyTraction;
using core_planning::DriveParams;
using core_planning::estimateTraction;
using core_planning::FloorTraction;
using core_planning::floorFor;
using core_planning::MppiParams;
using core_planning::MppiState;
using core_planning::OdometrySample;
using core_planning::predictDelayedState;
using core_planning::SentCommand;
using core_planning::TractionEstimate;
using core_planning::TractionParams;
using core_planning::tractionParamsError;
using core_planning::TractionState;

constexpr double kInf = std::numeric_limits<double>::infinity();

DriveParams robotDrive()
{
  DriveParams drive;
  drive.dead_time_s = 0.02;
  drive.time_constant_s = 0.05;
  return drive;
}

// 10Hz の指令（機体座標の向き angle へ step ずつ速くして top で hold 回走り、step ずつ遅くして止まる、を 2 回）
std::vector<SentCommand> rampCommands(
  const double step, const double top, const double angle = 0.0, const int hold = 5)
{
  std::vector<SentCommand> sent;
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  double t = 0.0;
  const auto push = [&](const double v) {
      sent.push_back(SentCommand{t, v * c, v * s, 0.0});
      t += 0.1;
    };
  for (int repeat = 0; repeat < 2; ++repeat) {
    for (double v = step; v <= top + 1e-9; v += step) {
      push(v);
    }
    for (int i = 0; i < hold; ++i) {
      push(top);
    }
    for (double v = top - step; v >= -1e-9; v -= step) {
      push(std::max(v, 0.0));
    }
  }
  return sent;
}

// 本当の床で、指令から 10Hz の速度のサンプルを作る（推定と同じ運動）
std::vector<OdometrySample> simulate(
  const std::vector<SentCommand> & sent, const DriveParams & drive, const FloorTraction & floor)
{
  std::vector<OdometrySample> samples{OdometrySample{}};
  const double end_s = sent.back().time_s + 0.5;
  while (samples.back().time_s < end_s) {
    const OdometrySample & last = samples.back();
    const double next_s = last.time_s + 0.1;
    samples.push_back(
      OdometrySample{next_s, predictDelayedState(last.state, last.time_s, next_s, sent, drive, floor)});
  }
  return samples;
}

TEST(Traction, ParamsValidation)
{
  TractionParams params;
  EXPECT_FALSE(tractionParamsError(params).has_value());
  params.horizon_s = params.window_s;
  EXPECT_TRUE(tractionParamsError(params).has_value());
  params = TractionParams{};
  params.min_accel_mps2 = 4.0;
  EXPECT_TRUE(tractionParamsError(params).has_value());
}

TEST(Traction, FloorSlowsAccelerationAboveStaticFriction)
{
  DriveParams drive;
  drive.max_accel_mps2 = 1e6;
  const std::vector<SentCommand> sent{{0.0, 1.0, 0.0, 0.0}};
  FloorTraction floor;
  floor.static_accel_mps2 = 3.0;
  floor.kinetic_ratio = 0.7;
  const MppiState predicted = predictDelayedState(MppiState{}, 0.0, 0.1, sent, drive, floor);
  EXPECT_NEAR(predicted.vx, 0.1 * 2.1, 1e-9);
  // 上限なしなら 1 ステップで指令に届く
  EXPECT_NEAR(predictDelayedState(MppiState{}, 0.0, 0.1, sent, drive).vx, 1.0, 1e-9);
}

TEST(Traction, MapsFrictionToPlanningLimits)
{
  MppiParams base;
  base.curve.max_lateral_accel_mps2 = 2.0;
  const TractionParams params;
  // 床 3.0 で今の既定、滑らない床で元の加減速度（曲がりは 2.5）
  const MppiParams heavy = applyTraction(base, 3.0, params);
  EXPECT_NEAR(heavy.max_accel_mps2, 2.5, 1e-9);
  EXPECT_NEAR(heavy.reference_decel_mps2, 1.8, 1e-9);
  EXPECT_NEAR(heavy.curve.max_lateral_accel_mps2, 2.0, 1e-9);
  const MppiParams good = applyTraction(base, kInf, params);
  EXPECT_DOUBLE_EQ(good.max_accel_mps2, 3.0);
  EXPECT_DOUBLE_EQ(good.reference_decel_mps2, 2.0);
  EXPECT_DOUBLE_EQ(good.curve.max_lateral_accel_mps2, 2.5);
  const MppiParams ice = applyTraction(base, 1.0, params);
  EXPECT_DOUBLE_EQ(ice.max_accel_mps2, 1.0);
  EXPECT_DOUBLE_EQ(ice.reference_decel_mps2, 0.8);
  // 曲がりの上限を使わない設定は変えない
  base.curve.max_lateral_accel_mps2 = 0.0;
  EXPECT_DOUBLE_EQ(applyTraction(base, 3.0, params).curve.max_lateral_accel_mps2, 0.0);
  EXPECT_DOUBLE_EQ(floorFor(kInf).static_accel_mps2, 0.0);
  EXPECT_DOUBLE_EQ(floorFor(2.5).static_accel_mps2, 2.5);
}

TEST(Traction, StateLowersAtOnceAndRaisesAfterRepeatedEstimates)
{
  const TractionParams params;
  TractionState state(3.0);
  EXPECT_DOUBLE_EQ(state.update(std::nullopt, params), 3.0);
  // 滑りが見えたらすぐ下げる
  TractionEstimate slip;
  slip.slipping = true;
  slip.friction_mps2 = 2.0;
  EXPECT_DOUBLE_EQ(state.update(slip, params), 2.0);
  // 滑りが見えない窓が 2 回続いたら、下限が低くても 1 段上（2.5）を試す
  TractionEstimate bound;
  bound.friction_mps2 = 1.0;
  EXPECT_DOUBLE_EQ(state.update(bound, params), 2.0);
  EXPECT_DOUBLE_EQ(state.update(bound, params), 2.5);
  // 下限が高ければそこまで。続いた中の低い方へ上げる
  bound.friction_mps2 = 4.0;
  EXPECT_DOUBLE_EQ(state.update(bound, params), 2.5);
  bound.friction_mps2 = kInf;
  EXPECT_DOUBLE_EQ(state.update(bound, params), 4.0);
  // 同じ値の滑りが見えた窓が間に入ったら数え直す
  EXPECT_DOUBLE_EQ(state.update(bound, params), 4.0);
  slip.friction_mps2 = 4.0;
  EXPECT_DOUBLE_EQ(state.update(slip, params), 4.0);
  EXPECT_DOUBLE_EQ(state.update(bound, params), 4.0);
  EXPECT_DOUBLE_EQ(state.update(bound, params), kInf);
  // 滑らない床（無限大）より上はない
  EXPECT_DOUBLE_EQ(state.update(bound, params), kInf);
}

TEST(Traction, EstimatesFloorFromCommandsAndVelocity)
{
  const DriveParams drive = robotDrive();
  // 0.1 秒ごとに 0.25m/s の段差（時定数 0.05 秒なので、段差の直後に約 5m/s2 を求める）
  const auto sent = rampCommands(0.25, 3.0);
  TractionParams params;
  params.window_s = 100.0;
  for (const double truth : {2.0, 3.0}) {
    FloorTraction floor;
    floor.static_accel_mps2 = truth;
    const auto samples = simulate(sent, drive, floor);
    const auto estimate = estimateTraction(samples, sent, drive, params, samples.back().time_s);
    ASSERT_TRUE(estimate.has_value());
    EXPECT_TRUE(estimate->slipping);
    EXPECT_DOUBLE_EQ(estimate->friction_mps2, truth);
    EXPECT_LT(estimate->error_mps, 1e-6);
  }
  // 滑らない床では滑りを見ず、段差で求めた加速度（約 5m/s2）に近い下限を返す
  const auto samples = simulate(sent, drive, FloorTraction{});
  const auto estimate = estimateTraction(samples, sent, drive, params, samples.back().time_s);
  ASSERT_TRUE(estimate.has_value());
  EXPECT_FALSE(estimate->slipping);
  EXPECT_GE(estimate->friction_mps2, 3.0);
}

TEST(Traction, SeparatesVelocityRatioFromFriction)
{
  // 床で指令の前後 95%・横 90% しか出ない床（静止摩擦 3.0）。斜めに走ると両方が見え、一定の指令で走り
  // 続ける区間（2 秒）で割合が分かる
  const DriveParams drive = robotDrive();
  const auto sent = rampCommands(0.25, 2.5, 0.6, 20);
  TractionParams params;
  params.window_s = 100.0;
  FloorTraction floor;
  floor.static_accel_mps2 = 3.0;
  floor.longitudinal_ratio = 0.95;
  floor.lateral_ratio = 0.9;
  const auto samples = simulate(sent, drive, floor);
  const auto estimate = estimateTraction(samples, sent, drive, params, samples.back().time_s);
  ASSERT_TRUE(estimate.has_value());
  EXPECT_TRUE(estimate->slipping);
  EXPECT_DOUBLE_EQ(estimate->friction_mps2, 3.0);
  EXPECT_NEAR(estimate->longitudinal_ratio, 0.95, 1e-3);
  EXPECT_NEAR(estimate->lateral_ratio, 0.9, 1e-3);
  // 滑っている窓の割合は覚えない
  EXPECT_FALSE(estimate->ratio_observed);
}

TEST(Traction, GentleOrSteadyDrivingDoesNotLowerFloor)
{
  const DriveParams drive = robotDrive();
  TractionParams params;
  params.window_s = 100.0;
  // 0.3 秒で 0.15m/s しか変わらないので、見分ける下限を下げて比べる
  params.min_excitation_mps = 0.1;
  // 小さい段差（求める加速度 約 1.2m/s2）では、本当の床（2.0）で滑らないので、滑りは見えない
  const auto gentle = rampCommands(0.05, 1.0);
  FloorTraction floor;
  floor.static_accel_mps2 = 2.0;
  const auto samples = simulate(gentle, drive, floor);
  const auto estimate = estimateTraction(samples, gentle, drive, params, samples.back().time_s);
  ASSERT_TRUE(estimate.has_value());
  EXPECT_FALSE(estimate->slipping);
  // 下限なので、使う床の値は下がらない（1 回では上げもしない）
  TractionState state(3.0);
  EXPECT_DOUBLE_EQ(state.update(estimate, params), 3.0);
  // 速度が変わらなければ見分けられない
  std::vector<OdometrySample> steady;
  for (int i = 0; i < 40; ++i) {
    OdometrySample sample;
    sample.time_s = 0.1 * i;
    sample.state.vx = 1.0;
    steady.push_back(sample);
  }
  const std::vector<SentCommand> hold{{0.0, 1.0, 0.0, 0.0}};
  EXPECT_FALSE(estimateTraction(steady, hold, drive, params, 3.9).has_value());
}
}  // namespace

namespace
{
// 加速の間だけの記録では、速度の割合で遅れを説明せず、滑りとして見る（走り始めの滑りを見逃さない）
TEST(Traction, AccelerationOnlyWindowSeesSlipNotRatio)
{
  const DriveParams drive = robotDrive();
  std::vector<SentCommand> sent;
  for (int i = 0; i < 20; ++i) {
    sent.push_back(SentCommand{0.1 * i, 0.25 * (i + 1), 0.0, 0.0});
  }
  FloorTraction floor;
  floor.static_accel_mps2 = 2.0;
  floor.longitudinal_ratio = 0.95;
  const auto samples = simulate(sent, drive, floor);
  TractionParams params;
  const auto estimate = estimateTraction(samples, sent, drive, params, 2.0);
  ASSERT_TRUE(estimate.has_value());
  EXPECT_TRUE(estimate->slipping);
  EXPECT_LE(estimate->friction_mps2, 2.0);
  // 割合は前の値（1）のまわりしか試さない
  EXPECT_GE(estimate->longitudinal_ratio, 0.96);
}
}  // namespace

namespace
{
// 滑らない窓で割合を少しずつ覚え、割合のある床でも静止摩擦を低く見ない
TEST(Traction, LearnsRatioFromSlipFreeWindows)
{
  const DriveParams drive = robotDrive();
  FloorTraction floor;
  floor.static_accel_mps2 = 3.0;
  floor.longitudinal_ratio = 0.92;
  // 小さい段差（滑らない）で走る窓
  const auto gentle = rampCommands(0.05, 1.5, 0.0, 3);
  const auto gentle_samples = simulate(gentle, drive, floor);
  TractionParams params;
  params.window_s = 100.0;
  params.min_excitation_mps = 0.1;
  TractionState state(3.0);
  for (int i = 0; i < 4; ++i) {
    const auto estimate = estimateTraction(
      gentle_samples, gentle, drive, params, gentle_samples.back().time_s, state.ratio());
    ASSERT_TRUE(estimate.has_value());
    EXPECT_FALSE(estimate->slipping);
    state.update(estimate, params);
  }
  EXPECT_NEAR(state.ratio()[0], 0.92, 0.021);
  // 覚えた割合で、段差の大きい（滑る）窓の静止摩擦を当てる
  const auto steps = rampCommands(0.25, 2.5);
  const auto samples = simulate(steps, drive, floor);
  const auto estimate =
    estimateTraction(samples, steps, drive, params, samples.back().time_s, state.ratio());
  ASSERT_TRUE(estimate.has_value());
  EXPECT_TRUE(estimate->slipping);
  EXPECT_GE(estimate->friction_mps2, 2.5);
}
}  // namespace
