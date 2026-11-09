// aim_bot の追従制御（関節角履歴・的の角速度推定・速度FF+PD）を ROS 起動なしで検証する。

#include <gtest/gtest.h>

#include <cmath>

#include "core_shooter/joint_state_history.hpp"
#include "core_shooter/tracking_controller.hpp"

using core_shooter::AxisPair;
using core_shooter::AxisTrackingGains;
using core_shooter::AxisTrackingInput;
using core_shooter::AxisTrackingState;
using core_shooter::JointStateHistory;
using core_shooter::TargetTrack;
using core_shooter::TargetVelocityFilter;
using core_shooter::TimedAngles;

namespace
{
constexpr double kEps = 1e-9;

TimedAngles sampleAt(double t, double yaw, double pitch)
{
  return TimedAngles{t, AxisPair{yaw, pitch}};
}
}  // namespace

// ---------------------------------------------------------------------------
// JointStateHistory
// ---------------------------------------------------------------------------

TEST(JointStateHistory, EmptyReturnsNullopt)
{
  const JointStateHistory history(1.0);
  EXPECT_FALSE(history.at(0.0).has_value());
}

TEST(JointStateHistory, InterpolatesBetweenSamples)
{
  JointStateHistory history(1.0);
  history.push(sampleAt(10.0, 0.0, 1.0));
  history.push(sampleAt(10.1, 1.0, 0.0));

  const auto lookup = history.at(10.025);
  ASSERT_TRUE(lookup.has_value());
  EXPECT_DOUBLE_EQ(lookup->outside_sec, 0.0);
  EXPECT_NEAR(lookup->angles.yaw, 0.25, kEps);
  EXPECT_NEAR(lookup->angles.pitch, 0.75, kEps);
}

TEST(JointStateHistory, ExactSampleTimeIsNotClamped)
{
  JointStateHistory history(1.0);
  history.push(sampleAt(1.0, 0.1, 0.2));
  history.push(sampleAt(2.0, 0.3, 0.4));

  const auto first = history.at(1.0);
  ASSERT_TRUE(first.has_value());
  EXPECT_DOUBLE_EQ(first->outside_sec, 0.0);
  EXPECT_NEAR(first->angles.yaw, 0.1, kEps);
}

TEST(JointStateHistory, OutOfRangeClampsToNearestEnd)
{
  JointStateHistory history(1.0);
  history.push(sampleAt(1.0, 0.1, 0.2));
  history.push(sampleAt(2.0, 0.3, 0.4));

  const auto before = history.at(0.5);
  ASSERT_TRUE(before.has_value());
  EXPECT_NEAR(before->outside_sec, -0.5, kEps);
  EXPECT_NEAR(before->angles.yaw, 0.1, kEps);

  const auto after = history.at(3.0);
  ASSERT_TRUE(after.has_value());
  EXPECT_NEAR(after->outside_sec, 1.0, kEps);
  EXPECT_NEAR(after->angles.pitch, 0.4, kEps);
}

TEST(JointStateHistory, IgnoresNonIncreasingTime)
{
  JointStateHistory history(1.0);
  history.push(sampleAt(2.0, 0.5, 0.0));
  history.push(sampleAt(1.0, 9.0, 9.0));
  history.push(sampleAt(2.0, 9.0, 9.0));

  const auto lookup = history.at(2.0);
  ASSERT_TRUE(lookup.has_value());
  EXPECT_NEAR(lookup->angles.yaw, 0.5, kEps);
  EXPECT_NEAR(history.at(1.5)->outside_sec, -0.5, kEps);
}

TEST(JointStateHistory, KeepsAtLeastMaxAgeOfSamples)
{
  JointStateHistory history(0.5);
  for (int i = 0; i <= 100; ++i) {
    history.push(sampleAt(i * 0.01, i * 0.01, 0.0));
  }
  // 最新 1.00s から約 0.5s 前までは補間で引ける
  const auto lookup = history.at(0.505);
  ASSERT_TRUE(lookup.has_value());
  EXPECT_DOUBLE_EQ(lookup->outside_sec, 0.0);
  EXPECT_NEAR(lookup->angles.yaw, 0.505, 1e-6);
  // それより十分古い時刻は捨てられている
  EXPECT_LT(history.at(0.1)->outside_sec, 0.0);
}

// ---------------------------------------------------------------------------
// 的の角速度推定と予測
// ---------------------------------------------------------------------------

TEST(TargetTrack, FirstSampleHasNoVelocity)
{
  const TargetTrack track = core_shooter::startTargetTrack(sampleAt(0.0, 1.0, 2.0));
  EXPECT_FALSE(track.has_velocity);
  const AxisPair predicted = core_shooter::predictTargetAngles(track, 1.0, 1.0);
  EXPECT_NEAR(predicted.yaw, 1.0, kEps);
  EXPECT_NEAR(predicted.pitch, 2.0, kEps);
}

TEST(TargetTrack, EstimatesVelocityAndPredicts)
{
  const TargetVelocityFilter filter{0.01, 10.0, 10.0, 1.0};
  TargetTrack track = core_shooter::startTargetTrack(sampleAt(0.0, 0.0, 0.0));
  track = core_shooter::updateTargetTrack(track, sampleAt(0.1, 0.1, -0.05), filter);

  ASSERT_TRUE(track.has_velocity);
  EXPECT_NEAR(track.velocity.yaw, 1.0, kEps);
  EXPECT_NEAR(track.velocity.pitch, -0.5, kEps);

  const AxisPair predicted = core_shooter::predictTargetAngles(track, 0.3, 1.0);
  EXPECT_NEAR(predicted.yaw, 0.3, kEps);
  EXPECT_NEAR(predicted.pitch, -0.15, kEps);

  const AxisPair half_ff = core_shooter::predictTargetAngles(track, 0.3, 0.5);
  EXPECT_NEAR(half_ff.yaw, 0.2, kEps);

  const AxisPair no_ff = core_shooter::predictTargetAngles(track, 0.3, 0.0);
  EXPECT_NEAR(no_ff.yaw, 0.1, kEps);
}

TEST(TargetTrack, AppliesEmaAfterFirstVelocity)
{
  const TargetVelocityFilter filter{0.01, 10.0, 10.0, 0.5};
  TargetTrack track = core_shooter::startTargetTrack(sampleAt(0.0, 0.0, 0.0));
  track = core_shooter::updateTargetTrack(track, sampleAt(0.1, 0.1, 0.0), filter);  // 1.0 rad/s
  track = core_shooter::updateTargetTrack(track, sampleAt(0.2, 0.4, 0.0), filter);  // 3.0 rad/s
  EXPECT_NEAR(track.velocity.yaw, 2.0, kEps);
}

TEST(TargetTrack, ClampsVelocity)
{
  const TargetVelocityFilter filter{0.01, 2.0, 1.0, 1.0};
  TargetTrack track = core_shooter::startTargetTrack(sampleAt(0.0, 0.0, 0.0));
  track = core_shooter::updateTargetTrack(track, sampleAt(0.1, 1.0, -1.0), filter);
  EXPECT_NEAR(track.velocity.yaw, 2.0, kEps);
  EXPECT_NEAR(track.velocity.pitch, -1.0, kEps);
}

TEST(TargetTrack, ShortIntervalUpdatesPositionButKeepsVelocityReference)
{
  const TargetVelocityFilter filter{0.05, 10.0, 10.0, 1.0};
  TargetTrack track = core_shooter::startTargetTrack(sampleAt(0.0, 0.0, 0.0));
  track = core_shooter::updateTargetTrack(track, sampleAt(0.01, 0.5, 0.0), filter);
  EXPECT_FALSE(track.has_velocity);
  EXPECT_NEAR(track.latest.angles.yaw, 0.5, kEps);

  // 速度は間隔の足りた基準（t=0）から測る
  track = core_shooter::updateTargetTrack(track, sampleAt(0.1, 0.1, 0.0), filter);
  ASSERT_TRUE(track.has_velocity);
  EXPECT_NEAR(track.velocity.yaw, 1.0, kEps);
}

TEST(TargetTrack, TimeRewindRestartsEstimation)
{
  const TargetVelocityFilter filter{0.01, 10.0, 10.0, 1.0};
  TargetTrack track = core_shooter::startTargetTrack(sampleAt(1.0, 0.0, 0.0));
  track = core_shooter::updateTargetTrack(track, sampleAt(1.1, 0.1, 0.0), filter);
  ASSERT_TRUE(track.has_velocity);
  track = core_shooter::updateTargetTrack(track, sampleAt(0.5, 0.2, 0.0), filter);
  EXPECT_FALSE(track.has_velocity);
  EXPECT_NEAR(track.latest.angles.yaw, 0.2, kEps);
}

// ---------------------------------------------------------------------------
// 速度FF + PD
// ---------------------------------------------------------------------------

TEST(AxisTracking, ZeroGainsCommandTargetDirectly)
{
  const auto result = core_shooter::stepAxisTracking(
    AxisTrackingGains{}, AxisTrackingState{},
    AxisTrackingInput{1.0, 0.5, 0.2, -0.3, 0.01});
  EXPECT_NEAR(result.command, 1.0, kEps);
}

TEST(AxisTracking, ProportionalLeadsBeyondTarget)
{
  const AxisTrackingGains gains{0.5, 0.0, 0.0, 0.0};
  const auto result = core_shooter::stepAxisTracking(
    gains, AxisTrackingState{}, AxisTrackingInput{1.0, 0.0, 0.6, 0.0, 0.01});
  EXPECT_NEAR(result.command, 1.0 + 0.5 * 0.4, kEps);
}

TEST(AxisTracking, DerivativeUsesVelocityError)
{
  const AxisTrackingGains gains{0.0, 0.0, 0.1, 0.0};
  // 目標より速く回っている -> 指令を手前に引く
  const auto result = core_shooter::stepAxisTracking(
    gains, AxisTrackingState{}, AxisTrackingInput{1.0, 1.0, 1.0, 3.0, 0.01});
  EXPECT_NEAR(result.command, 1.0 + 0.1 * (1.0 - 3.0), kEps);
}

TEST(AxisTracking, IntegralDisabledWithoutLimit)
{
  const AxisTrackingGains gains{0.0, 1.0, 0.0, 0.0};
  const auto result = core_shooter::stepAxisTracking(
    gains, AxisTrackingState{}, AxisTrackingInput{1.0, 0.0, 0.0, 0.0, 0.1});
  EXPECT_NEAR(result.state.integral, 0.0, kEps);
  EXPECT_NEAR(result.command, 1.0, kEps);
}

TEST(AxisTracking, IntegralAccumulatesAndSaturates)
{
  const AxisTrackingGains gains{0.0, 2.0, 0.0, 0.1};
  AxisTrackingState state;
  const auto first = core_shooter::stepAxisTracking(
    gains, state, AxisTrackingInput{1.0, 0.0, 0.0, 0.0, 0.01});
  EXPECT_NEAR(first.state.integral, 0.01, kEps);
  EXPECT_NEAR(first.command, 1.0 + 2.0 * 0.01, kEps);

  state = first.state;
  for (int i = 0; i < 1000; ++i) {
    state = core_shooter::stepAxisTracking(
      gains, state, AxisTrackingInput{1.0, 0.0, 0.0, 0.0, 0.01}).state;
  }
  // ki * integral は integral_limit で頭打ち
  EXPECT_NEAR(gains.ki * state.integral, 0.1, kEps);
}

TEST(AxisTracking, IsStateless)
{
  const AxisTrackingGains gains{0.3, 0.5, 0.1, 1.0};
  const AxisTrackingState state{0.2};
  const AxisTrackingInput input{0.4, 0.1, 0.1, 0.0, 0.02};
  const auto a = core_shooter::stepAxisTracking(gains, state, input);
  const auto b = core_shooter::stepAxisTracking(gains, state, input);
  EXPECT_DOUBLE_EQ(a.command, b.command);
  EXPECT_DOUBLE_EQ(a.state.integral, b.state.integral);
}

TEST(AxisTracking, ValidateRejectsBadGains)
{
  EXPECT_FALSE(core_shooter::validateAxisTrackingGains(AxisTrackingGains{}).has_value());
  EXPECT_TRUE(
    core_shooter::validateAxisTrackingGains(AxisTrackingGains{-0.1, 0.0, 0.0, 0.0}).has_value());
  EXPECT_TRUE(
    core_shooter::validateAxisTrackingGains(AxisTrackingGains{0.0, 0.0, 0.0, -1.0}).has_value());
  EXPECT_TRUE(
    core_shooter::validateAxisTrackingGains(
      AxisTrackingGains{std::nan(""), 0.0, 0.0, 0.0}).has_value());
}
