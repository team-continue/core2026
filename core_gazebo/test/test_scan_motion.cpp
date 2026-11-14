#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "core_gazebo/scan_motion.hpp"

namespace
{
using core_gazebo::BodyBox;
using core_gazebo::insideBody;
using core_gazebo::interpolatePose;
using core_gazebo::pointSeenAt;
using core_gazebo::StampedPose;
using core_gazebo::sweepTime;

Eigen::Isometry3d pose(const double x, const double yaw)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(x, 0.0, 0.0);
  result.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return result;
}

TEST(SweepTime, CoversScanInTurnsWithAzimuthIncreasing)
{
  // 1 走査 100ms で 18 周。-π は周の始め、0 は半分
  EXPECT_NEAR(sweepTime(-M_PI, 0, 18, 0.1), 0.0, 1e-12);
  EXPECT_NEAR(sweepTime(0.0, 0, 18, 0.1), 0.1 / 36.0, 1e-12);
  EXPECT_NEAR(sweepTime(0.0, 17, 18, 0.1), 0.1 * 17.5 / 18.0, 1e-12);
  // 同じ周では方位角が大きいほど遅い
  EXPECT_LT(sweepTime(-1.0, 3, 18, 0.1), sweepTime(1.0, 3, 18, 0.1));
  // π は次の周の始めと同じ位置なので、この周の始めに戻す（結果は 1 走査の中）
  EXPECT_LT(sweepTime(M_PI, 17, 18, 0.1), 0.1);
}

TEST(InterpolatePose, InterpolatesBetweenSamplesAndRejectsOutside)
{
  const std::vector<StampedPose> poses{
    {1000U, pose(0.0, 0.0)}, {2000U, pose(1.0, 0.2)}, {3000U, pose(3.0, 0.6)}};
  const auto middle = interpolatePose(poses, 1500U);
  ASSERT_TRUE(middle.has_value());
  EXPECT_NEAR(middle->translation().x(), 0.5, 1e-12);
  EXPECT_NEAR(Eigen::AngleAxisd(middle->rotation()).angle(), 0.1, 1e-9);
  // 2 つ目の区間（加速している）
  const auto later = interpolatePose(poses, 2500U);
  ASSERT_TRUE(later.has_value());
  EXPECT_NEAR(later->translation().x(), 2.0, 1e-12);
  EXPECT_NEAR(Eigen::AngleAxisd(later->rotation()).angle(), 0.4, 1e-9);
  // ちょうどの時刻と端
  EXPECT_NEAR(interpolatePose(poses, 1000U)->translation().x(), 0.0, 1e-12);
  EXPECT_NEAR(interpolatePose(poses, 3000U)->translation().x(), 3.0, 1e-12);
  EXPECT_FALSE(interpolatePose(poses, 999U).has_value());
  EXPECT_FALSE(interpolatePose(poses, 3001U).has_value());
  EXPECT_FALSE(interpolatePose({}, 1000U).has_value());
}

TEST(PointSeenAt, ShowsStaticPointFromEarlierAndLaterSensorPoses)
{
  // 撮った時刻の LiDAR は原点で正面（+x）を向き、点は 2m 先
  const Eigen::Isometry3d capture = pose(0.0, 0.0);
  const Eigen::Vector3d point(2.0, 0.0, 0.0);
  // 撮る前（まだ右を向いていた）の LiDAR から見ると、点は左にある
  const Eigen::Vector3d before = pointSeenAt(point, capture, pose(0.0, -0.1));
  EXPECT_NEAR(std::atan2(before.y(), before.x()), 0.1, 1e-12);
  // 撮った後に 0.1m 進んだ LiDAR から見ると、点は 0.1m 近い
  EXPECT_NEAR(pointSeenAt(point, capture, pose(0.1, 0.0)).x(), 1.9, 1e-12);
}

TEST(InsideBody, KeepsRobotBodyPointsWithMargin)
{
  // 0.8m 角・高さ 0.8m の車体と余裕 5cm
  const BodyBox box;
  EXPECT_TRUE(insideBody(Eigen::Vector3d(0.40, 0.0, 0.3), box));    // 前の面
  EXPECT_TRUE(insideBody(Eigen::Vector3d(-0.44, 0.44, 0.84), box));  // 後ろの角の上（余裕の中）
  EXPECT_FALSE(insideBody(Eigen::Vector3d(0.46, 0.0, 0.3), box));    // 前の面の 6cm 外
  EXPECT_FALSE(insideBody(Eigen::Vector3d(0.0, 0.0, 0.9), box));     // 上面より上
  EXPECT_FALSE(insideBody(Eigen::Vector3d(0.0, 0.0, -0.1), box));    // 床より下
}

TEST(PointSeenAt, IsUndoneByCorrectingWithSensorPoses)
{
  // FAST-LIO のように、点の時刻の LiDAR の姿勢で世界へ戻すと、撮った時刻の点と同じ所になる
  const Eigen::Isometry3d capture = pose(1.0, 0.3);
  const Eigen::Isometry3d at = pose(1.2, 0.9);
  const Eigen::Vector3d point(3.0, -1.0, 0.5);
  const Eigen::Vector3d seen = pointSeenAt(point, capture, at);
  EXPECT_NEAR(((at * seen) - (capture * point)).norm(), 0.0, 1e-12);
}
}  // namespace
