#include <gtest/gtest.h>

#include <cmath>

#include "core_body_controller/dead_reckoning.hpp"

namespace
{

using core_body_controller::integrate_pose;
using core_body_controller::Pose2D;

TEST(DeadReckoningTest, StraightMotionFollowsYaw)
{
  const auto pose = integrate_pose(Pose2D{1.0, 2.0, M_PI / 2}, 1.0, 0.0, M_PI / 2, 0.5);
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->x, 1.0, 1e-12);
  EXPECT_NEAR(pose->y, 2.5, 1e-12);
  EXPECT_NEAR(pose->yaw, M_PI / 2, 1e-12);
}

TEST(DeadReckoningTest, LateralVelocityIsRotatedIntoFrame)
{
  const auto pose = integrate_pose(Pose2D{0.0, 0.0, 0.0}, 0.0, 1.0, 0.0, 1.0);
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->x, 0.0, 1e-12);
  EXPECT_NEAR(pose->y, 1.0, 1e-12);
}

TEST(DeadReckoningTest, HeadingUsesMidpointYaw)
{
  const auto pose = integrate_pose(Pose2D{0.0, 0.0, 0.0}, 1.0, 0.0, M_PI / 2, 1.0);
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->x, std::cos(M_PI / 4), 1e-12);
  EXPECT_NEAR(pose->y, std::sin(M_PI / 4), 1e-12);
  EXPECT_NEAR(pose->yaw, M_PI / 2, 1e-12);
}

TEST(DeadReckoningTest, MidpointHandlesUnwrappedYaw)
{
  // 連続化された yaw (2π を超える) でも中間の向きは差分から決まる
  const auto pose = integrate_pose(Pose2D{0.0, 0.0, 4 * M_PI}, 1.0, 0.0, 4 * M_PI + 0.2, 1.0);
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->x, std::cos(0.1), 1e-12);
  EXPECT_NEAR(pose->y, std::sin(0.1), 1e-12);
  EXPECT_NEAR(pose->yaw, 4 * M_PI + 0.2, 1e-12);
}

TEST(DeadReckoningTest, CircleReturnsToStart)
{
  // 半径 1 m の円を 1000 ステップで 1 周すると始点に戻る
  constexpr int STEPS = 1000;
  constexpr double DT = 2 * M_PI / STEPS;
  Pose2D pose{0.0, 0.0, 0.0};
  for (int i = 1; i <= STEPS; ++i) {
    const auto next = integrate_pose(pose, 1.0, 0.0, i * DT, DT);
    ASSERT_TRUE(next.has_value());
    pose = *next;
  }
  EXPECT_NEAR(pose.x, 0.0, 1e-5);
  EXPECT_NEAR(pose.y, 0.0, 1e-5);
}

TEST(DeadReckoningTest, InvalidInputsAreRejected)
{
  const Pose2D origin{0.0, 0.0, 0.0};
  EXPECT_FALSE(integrate_pose(origin, 1.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(integrate_pose(origin, 1.0, 0.0, 0.0, -0.1));
  EXPECT_FALSE(integrate_pose(origin, std::nan(""), 0.0, 0.0, 0.1));
  EXPECT_FALSE(integrate_pose(origin, 1.0, 0.0, std::nan(""), 0.1));
  EXPECT_FALSE(integrate_pose(Pose2D{std::nan(""), 0.0, 0.0}, 1.0, 0.0, 0.0, 0.1));
}

}  // namespace
