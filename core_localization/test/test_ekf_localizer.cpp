#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "core_localization/ekf_localizer.hpp"

namespace
{
using core_localization::EkfLocalizer;
using core_localization::EkfParams;
using core_localization::normalizeAngle;
using core_localization::PoseMeasurement;
using core_localization::TwistMeasurement;
using core_localization::UpdateResult;

// 50Hz
constexpr double kTwistPeriodS = 0.02;

PoseMeasurement pose(const double x, const double y, const double yaw, const double std_xy = 0.05)
{
  PoseMeasurement measurement;
  measurement.x = x;
  measurement.y = y;
  measurement.yaw = yaw;
  measurement.covariance = Eigen::Vector3d(std_xy * std_xy, std_xy * std_xy, 1.0e-4).asDiagonal();
  return measurement;
}

TwistMeasurement twist(const double vx, const double vy, const double wz)
{
  TwistMeasurement measurement;
  measurement.vx = vx;
  measurement.vy = vy;
  measurement.wz = wz;
  measurement.covariance = Eigen::Vector3d(1.0e-4, 1.0e-4, 1.0e-6).asDiagonal();
  return measurement;
}

// (start_s, end_s] に 50Hz で同じ速度を入れる
void feedTwist(
  EkfLocalizer & ekf, const double start_s, const double end_s, const TwistMeasurement & measurement)
{
  for (int step = 1; start_s + step * kTwistPeriodS <= end_s + 1e-9; ++step) {
    ASSERT_EQ(ekf.updateTwist(start_s + step * kTwistPeriodS, measurement), UpdateResult::kAccepted);
  }
}

TEST(EkfLocalizer, IgnoresTwistBeforeFirstPose)
{
  EkfLocalizer ekf{EkfParams{}};
  EXPECT_EQ(ekf.updateTwist(0.0, twist(1.0, 0.0, 0.0)), UpdateResult::kNotInitialized);
  EXPECT_FALSE(ekf.initialized());
}

TEST(EkfLocalizer, StartsFromFirstPoseAtRest)
{
  EkfLocalizer ekf{EkfParams{}};
  EXPECT_EQ(ekf.updatePose(5.0, pose(1.0, 2.0, 0.5)), UpdateResult::kAccepted);
  ASSERT_TRUE(ekf.initialized());
  EXPECT_DOUBLE_EQ(ekf.stamp(), 5.0);
  EXPECT_DOUBLE_EQ(ekf.state()(0), 1.0);
  EXPECT_DOUBLE_EQ(ekf.state()(1), 2.0);
  EXPECT_DOUBLE_EQ(ekf.state()(2), 0.5);
  EXPECT_DOUBLE_EQ(ekf.state().tail<3>().norm(), 0.0);
}

TEST(EkfLocalizer, MovesInBodyFrame)
{
  // 左（+y）を向いて前に 1m/s、横に 0.5m/s（右へ）で 1 秒
  EkfLocalizer ekf{EkfParams{}};
  ekf.updatePose(0.0, pose(0.0, 0.0, M_PI / 2.0, 0.01));
  feedTwist(ekf, 0.0, 1.0, twist(1.0, -0.5, 0.0));
  EXPECT_NEAR(ekf.state()(0), 0.5, 0.02);
  EXPECT_NEAR(ekf.state()(1), 1.0, 0.02);
  EXPECT_NEAR(ekf.state()(2), M_PI / 2.0, 1e-3);
  EXPECT_NEAR(ekf.state()(3), 1.0, 1e-3);
  EXPECT_NEAR(ekf.state()(4), -0.5, 1e-3);
}

TEST(EkfLocalizer, FollowsArcWhileTurning)
{
  constexpr double kSpeed = 1.0;
  constexpr double kYawRate = 0.5;
  constexpr double kDuration = 2.0;
  EkfLocalizer ekf{EkfParams{}};
  ekf.updatePose(0.0, pose(0.0, 0.0, 0.0, 0.01));
  feedTwist(ekf, 0.0, kDuration, twist(kSpeed, 0.0, kYawRate));
  const double yaw = kYawRate * kDuration;
  EXPECT_NEAR(ekf.state()(0), kSpeed * std::sin(yaw) / kYawRate, 0.03);
  EXPECT_NEAR(ekf.state()(1), kSpeed * (1.0 - std::cos(yaw)) / kYawRate, 0.03);
  EXPECT_NEAR(ekf.state()(2), yaw, 0.01);
}

TEST(EkfLocalizer, PredictedDoesNotChangeState)
{
  EkfLocalizer ekf{EkfParams{}};
  ekf.updatePose(0.0, pose(0.0, 0.0, 0.0, 0.01));
  feedTwist(ekf, 0.0, 0.5, twist(2.0, 0.0, 0.0));
  const EkfLocalizer::State before = ekf.state();
  const EkfLocalizer::Belief ahead = ekf.predicted(0.6);
  EXPECT_DOUBLE_EQ(ahead.stamp_s, 0.6);
  EXPECT_NEAR(ahead.x(0) - before(0), 0.2, 1e-3);
  // 先へ進めた分だけ、位置の不確かさは増える
  EXPECT_GT(ahead.p(0, 0), ekf.covariance()(0, 0));
  EXPECT_EQ(ekf.state(), before);
  EXPECT_DOUBLE_EQ(ekf.stamp(), 0.5);
  // 今より前なら、今の状態のまま
  EXPECT_EQ(ekf.predicted(0.4).x, before);
}

TEST(EkfLocalizer, PoseUpdatePullsTowardMeasurement)
{
  EkfLocalizer ekf{EkfParams{}};
  ekf.updatePose(0.0, pose(0.0, 0.0, 0.0));
  feedTwist(ekf, 0.0, 0.1, twist(0.0, 0.0, 0.0));
  const double variance_before = ekf.covariance()(0, 0);
  ASSERT_EQ(ekf.updatePose(0.1, pose(0.05, 0.0, 0.0)), UpdateResult::kAccepted);
  EXPECT_GT(ekf.state()(0), 0.0);
  EXPECT_LT(ekf.state()(0), 0.05);
  EXPECT_LT(ekf.covariance()(0, 0), variance_before);
}

TEST(EkfLocalizer, RejectsOutlierPose)
{
  EkfLocalizer ekf{EkfParams{}};
  ekf.updatePose(0.0, pose(0.0, 0.0, 0.0, 0.01));
  feedTwist(ekf, 0.0, 0.1, twist(0.0, 0.0, 0.0));
  EXPECT_EQ(ekf.updatePose(0.1, pose(3.0, 0.0, 0.0, 0.01)), UpdateResult::kRejectedByGate);
  EXPECT_NEAR(ekf.state()(0), 0.0, 1e-6);
}

TEST(EkfLocalizer, ResetsPoseAfterConsecutiveRejections)
{
  EkfParams params;
  params.pose_rejections_before_reset = 3;
  EkfLocalizer ekf{params};
  ekf.updatePose(0.0, pose(0.0, 0.0, 0.0, 0.01));
  feedTwist(ekf, 0.0, 0.1, twist(0.0, 0.0, 0.0));
  EXPECT_EQ(ekf.updatePose(0.1, pose(3.0, 1.0, 0.2, 0.01)), UpdateResult::kRejectedByGate);
  // 採用した測定があいだに入ると、数え直す
  EXPECT_EQ(ekf.updatePose(0.1, pose(0.0, 0.0, 0.0, 0.01)), UpdateResult::kAccepted);
  EXPECT_EQ(ekf.updatePose(0.1, pose(3.0, 1.0, 0.2, 0.01)), UpdateResult::kRejectedByGate);
  EXPECT_EQ(ekf.updatePose(0.1, pose(3.0, 1.0, 0.2, 0.01)), UpdateResult::kRejectedByGate);
  EXPECT_EQ(ekf.updatePose(0.1, pose(3.0, 1.0, 0.2, 0.01)), UpdateResult::kReset);
  EXPECT_DOUBLE_EQ(ekf.state()(0), 3.0);
  EXPECT_DOUBLE_EQ(ekf.state()(1), 1.0);
  // 角度は [-π, π) に直すときに末尾の桁が変わる
  EXPECT_NEAR(ekf.state()(2), 0.2, 1e-12);
  EXPECT_DOUBLE_EQ(ekf.covariance()(0, 3), 0.0);
  // 置き換えた後は、同じ所の測定を採る
  feedTwist(ekf, 0.1, 0.2, twist(0.0, 0.0, 0.0));
  EXPECT_EQ(ekf.updatePose(0.2, pose(3.0, 1.0, 0.2, 0.01)), UpdateResult::kAccepted);
}

TEST(EkfLocalizer, NeverResetsWhenDisabled)
{
  EkfParams params;
  params.pose_rejections_before_reset = 0;
  EkfLocalizer ekf{params};
  ekf.updatePose(0.0, pose(0.0, 0.0, 0.0, 0.01));
  for (int i = 1; i <= 30; ++i) {
    EXPECT_EQ(ekf.updatePose(0.01 * i, pose(10.0, 0.0, 0.0, 0.01)), UpdateResult::kRejectedByGate);
  }
}

TEST(EkfLocalizer, DelayedPoseGivesSameResultAsInOrderPose)
{
  // 位置が 0.1m ずれたまま走り、0.8 秒の姿勢（真値）で直す。遅れて届いても、時刻どおりに届いたときと同じになる
  const TwistMeasurement moving = twist(1.0, 0.0, 0.0);
  const PoseMeasurement truth_at_0_8 = pose(0.8, 0.0, 0.0);

  EkfLocalizer in_order{EkfParams{}};
  in_order.updatePose(0.0, pose(0.1, 0.0, 0.0));
  feedTwist(in_order, 0.0, 0.8, moving);
  ASSERT_EQ(in_order.updatePose(0.8, truth_at_0_8), UpdateResult::kAccepted);
  feedTwist(in_order, 0.8, 1.0, moving);

  EkfLocalizer delayed{EkfParams{}};
  delayed.updatePose(0.0, pose(0.1, 0.0, 0.0));
  feedTwist(delayed, 0.0, 1.0, moving);
  const double x_before = delayed.state()(0);
  ASSERT_EQ(delayed.updatePose(0.8, truth_at_0_8), UpdateResult::kAccepted);

  EXPECT_DOUBLE_EQ(delayed.stamp(), 1.0);
  EXPECT_LT(delayed.state()(0), x_before);
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(delayed.state()(i), in_order.state()(i), 1e-9) << i;
  }
  EXPECT_TRUE(delayed.covariance().isApprox(in_order.covariance(), 1e-9));
}

TEST(EkfLocalizer, DropsPoseOlderThanHistory)
{
  EkfParams params;
  params.history_s = 0.2;
  EkfLocalizer ekf{params};
  ekf.updatePose(0.0, pose(0.0, 0.0, 0.0));
  feedTwist(ekf, 0.0, 1.0, twist(1.0, 0.0, 0.0));
  EXPECT_EQ(ekf.updatePose(0.5, pose(0.5, 0.0, 0.0)), UpdateResult::kTooOld);
  // 残している範囲（0.8 秒より後）なら当てる
  EXPECT_EQ(ekf.updatePose(0.9, pose(0.9, 0.0, 0.0)), UpdateResult::kAccepted);
}

TEST(EkfLocalizer, WrapsYawAcrossPi)
{
  EkfLocalizer ekf{EkfParams{}};
  ekf.updatePose(0.0, pose(0.0, 0.0, M_PI - 0.01));
  feedTwist(ekf, 0.0, 0.1, twist(0.0, 0.0, 0.0));
  // -π + 0.01 は、π - 0.01 から +0.02 rad の向き。2π 回った差として捨てない
  ASSERT_EQ(ekf.updatePose(0.1, pose(0.0, 0.0, -M_PI + 0.01)), UpdateResult::kAccepted);
  const double yaw = ekf.state()(2);
  EXPECT_GE(yaw, -M_PI);
  EXPECT_LT(yaw, M_PI);
  EXPECT_LT(std::abs(normalizeAngle(yaw - M_PI)), 0.011);
}

TEST(EkfLocalizer, NormalizesAngles)
{
  EXPECT_NEAR(normalizeAngle(3.0 * M_PI / 2.0), -M_PI / 2.0, 1e-12);
  EXPECT_NEAR(normalizeAngle(-3.0 * M_PI / 2.0), M_PI / 2.0, 1e-12);
  EXPECT_NEAR(normalizeAngle(0.25), 0.25, 1e-12);
  EXPECT_NEAR(normalizeAngle(M_PI), -M_PI, 1e-12);
}
}  // namespace
