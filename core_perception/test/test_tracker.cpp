#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "core_perception/hungarian.hpp"
#include "core_perception/tracker.hpp"

namespace
{
using core_perception::classify;
using core_perception::ClassificationParams;
using core_perception::Detection;
using core_perception::MultiObjectTracker;
using core_perception::ObjectClass;
using core_perception::Track;
using core_perception::TrackingParams;
using core_perception::uncertainVelocitySigma;
using core_perception::velocitySigma;

Detection makeDetection(const float x, const float y)
{
  Detection detection;
  detection.center = Eigen::Vector2f(x, y);
  detection.depth = 0.2F;
  detection.width = 0.8F;
  detection.height = 0.8F;
  detection.num_points = 20U;
  return detection;
}

TEST(SolveAssignment, FindsMinimumTotal)
{
  const std::vector<std::vector<double>> cost{{4.0, 1.0, 3.0}, {2.0, 0.0, 5.0}, {3.0, 2.0, 2.0}};
  const auto assignment = core_perception::solveAssignment(cost);
  ASSERT_EQ(assignment.size(), 3U);
  EXPECT_EQ(assignment[0], 1U);
  EXPECT_EQ(assignment[1], 0U);
  EXPECT_EQ(assignment[2], 2U);
}

TEST(Tracker, EstimatesConstantVelocity)
{
  MultiObjectTracker tracker;
  const TrackingParams params;
  for (int i = 0; i < 30; ++i) {
    tracker.update({makeDetection(0.3F * i, 1.0F)}, 0.1, params);
  }
  ASSERT_EQ(tracker.tracks().size(), 1U);
  EXPECT_NEAR(tracker.tracks()[0].state(2), 3.0, 0.1);
  EXPECT_NEAR(tracker.tracks()[0].state(3), 0.0, 0.1);
  EXPECT_TRUE(MultiObjectTracker::isConfirmed(tracker.tracks()[0], params));
}

TEST(Tracker, KeepsIdWhileCrossing)
{
  MultiObjectTracker tracker;
  const TrackingParams params;
  for (int i = 0; i < 40; ++i) {
    tracker.update(
      {makeDetection(-6.0F + 0.3F * i, 0.0F), makeDetection(6.0F - 0.3F * i, 0.5F)}, 0.1, params);
  }
  ASSERT_EQ(tracker.tracks().size(), 2U);
  EXPECT_GT(tracker.tracks()[0].state(2), 2.0);
  EXPECT_LT(tracker.tracks()[1].state(2), -2.0);
}

TEST(Tracker, DropsTrackAfterMaxCoast)
{
  MultiObjectTracker tracker;
  TrackingParams params;
  params.max_coast_s = 0.5;
  for (int i = 0; i < 5; ++i) {
    tracker.update({makeDetection(1.0F, 1.0F)}, 0.1, params);
  }
  for (int i = 0; i < 4; ++i) {
    tracker.update({}, 0.1, params);
  }
  EXPECT_EQ(tracker.tracks().size(), 1U);
  for (int i = 0; i < 3; ++i) {
    tracker.update({}, 0.1, params);
  }
  EXPECT_TRUE(tracker.tracks().empty());
}

TEST(Tracker, LimitsUnrealisticSpeed)
{
  MultiObjectTracker tracker;
  TrackingParams params;
  params.association_gate_m = 5.0;
  tracker.update({makeDetection(0.0F, 0.0F)}, 0.1, params);
  tracker.update({makeDetection(3.0F, 0.0F)}, 0.1, params);
  ASSERT_EQ(tracker.tracks().size(), 1U);
  EXPECT_LE(tracker.tracks()[0].state.tail<2>().norm(), params.max_speed_mps + 1e-6);
}

TEST(Tracker, ReportsUncertainVelocityRightAfterStart)
{
  // 追跡を始めた直後（確定の 2 回目）は速度が定まっていない。落ち着けば不確かさは 0
  MultiObjectTracker tracker;
  TrackingParams params;
  params.accel_sigma_mps2 = 2.0;
  tracker.update({makeDetection(0.0F, 0.0F)}, 0.1, params);
  tracker.update({makeDetection(0.2F, 0.0F)}, 0.1, params);
  ASSERT_EQ(tracker.tracks().size(), 1U);
  ASSERT_TRUE(MultiObjectTracker::isConfirmed(tracker.tracks()[0], params));
  EXPECT_GT(uncertainVelocitySigma(tracker.tracks()[0], params), 1.0);
  for (int i = 2; i < 12; ++i) {
    tracker.update({makeDetection(0.2F * i, 0.0F)}, 0.1, params);
  }
  EXPECT_LT(velocitySigma(tracker.tracks()[0]), params.velocity_settled_sigma_mps);
  EXPECT_DOUBLE_EQ(uncertainVelocitySigma(tracker.tracks()[0], params), 0.0);
}

// x 方向に 2m/s で進む物体を frames 回観測し、hidden 回見えなくした後の追跡器
MultiObjectTracker trackThenHide(const TrackingParams & params, const int frames, const int hidden)
{
  MultiObjectTracker tracker;
  for (int i = 0; i < frames; ++i) {
    tracker.update({makeDetection(0.2F * i, 0.0F)}, 0.1, params);
  }
  for (int i = 0; i < hidden; ++i) {
    tracker.update({}, 0.1, params);
  }
  return tracker;
}

TrackingParams shortCoastParams()
{
  TrackingParams params;
  params.accel_sigma_mps2 = 2.0;
  params.max_coast_s = 0.3;
  return params;
}

TEST(Tracker, HandsOverVelocityAndIdAfterLoss)
{
  // 静止物の陰に入って消えた追跡の近くで、すぐ後に生まれた追跡は、速度と ID を引き継ぐ
  const TrackingParams params = shortCoastParams();
  MultiObjectTracker tracker = trackThenHide(params, 20, 5);
  ASSERT_TRUE(tracker.tracks().empty());
  const std::uint32_t lost_id = 1U;
  // 消してから 0.2 秒後（最後の観測から 0.6 秒後）。等速で進めた位置は x = 3.8 + 2.0 * 0.6 = 5.0。
  // 陰から出た直後で、観測の中心は少しずれている
  tracker.update({makeDetection(4.8F, 0.1F)}, 0.1, params);
  ASSERT_EQ(tracker.tracks().size(), 1U);
  const Track & child = tracker.tracks()[0];
  EXPECT_EQ(child.id, lost_id);
  EXPECT_TRUE(child.inherited);
  EXPECT_TRUE(MultiObjectTracker::isConfirmed(child, params));
  EXPECT_NEAR(child.state(2), 2.0, 0.2);
  EXPECT_NEAR(child.state(3), 0.0, 0.2);
  // 引き継いだ速度は、速度 0 から始めるよりずっと確かとする
  EXPECT_LT(velocitySigma(child), 1.0);
}

TEST(Tracker, DoesNotHandOverToDistantTrack)
{
  const TrackingParams params = shortCoastParams();
  MultiObjectTracker tracker = trackThenHide(params, 20, 5);
  tracker.update({makeDetection(5.0F, 3.0F)}, 0.1, params);
  ASSERT_EQ(tracker.tracks().size(), 1U);
  EXPECT_NE(tracker.tracks()[0].id, 1U);
  EXPECT_FALSE(tracker.tracks()[0].inherited);
  EXPECT_DOUBLE_EQ(tracker.tracks()[0].state(2), 0.0);
}

TEST(Tracker, DoesNotHandOverAfterTimeGate)
{
  // 消えてから max_gap_s（0.5 秒）より後に生まれた追跡には引き継がない
  const TrackingParams params = shortCoastParams();
  MultiObjectTracker tracker = trackThenHide(params, 20, 11);
  tracker.update({makeDetection(6.2F, 0.0F)}, 0.1, params);
  ASSERT_EQ(tracker.tracks().size(), 1U);
  EXPECT_FALSE(tracker.tracks()[0].inherited);
  EXPECT_NE(tracker.tracks()[0].id, 1U);
}

TEST(Tracker, HandoverCanBeDisabled)
{
  TrackingParams params = shortCoastParams();
  params.handover.enabled = false;
  MultiObjectTracker tracker = trackThenHide(params, 20, 5);
  tracker.update({makeDetection(4.8F, 0.1F)}, 0.1, params);
  ASSERT_EQ(tracker.tracks().size(), 1U);
  EXPECT_FALSE(tracker.tracks()[0].inherited);
  EXPECT_FALSE(MultiObjectTracker::isConfirmed(tracker.tracks()[0], params));
}

TEST(Tracker, DoesNotHandOverUnconfirmedTrack)
{
  // 1 回しか観測していない追跡（誤検出の可能性が高い）は、消えても引き継がない
  const TrackingParams params = shortCoastParams();
  MultiObjectTracker tracker = trackThenHide(params, 1, 3);
  ASSERT_TRUE(tracker.tracks().empty());
  tracker.update({makeDetection(0.1F, 0.0F)}, 0.1, params);
  ASSERT_EQ(tracker.tracks().size(), 1U);
  EXPECT_FALSE(tracker.tracks()[0].inherited);
}

TEST(Classify, IgnoresBriefMergeWithTallerObject)
{
  const ClassificationParams params;
  Track robot;
  for (int i = 0; i < 20; ++i) {
    robot.height.add(0.8F);
    robot.width.add(0.8F);
  }
  robot.height.add(1.7F);
  robot.width.add(1.2F);
  EXPECT_EQ(classify(robot, params), ObjectClass::kRobot);
}

TEST(Classify, SeparatesRobotAndHuman)
{
  const ClassificationParams params;
  Track robot;
  robot.height.add(0.8F);
  robot.width.add(0.8F);
  EXPECT_EQ(classify(robot, params), ObjectClass::kRobot);
  Track human;
  human.height.add(1.7F);
  human.width.add(0.5F);
  EXPECT_EQ(classify(human, params), ObjectClass::kHuman);
  Track other;
  other.height.add(1.7F);
  other.width.add(2.0F);
  EXPECT_EQ(classify(other, params), ObjectClass::kUnknown);
}
}  // namespace
