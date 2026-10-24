#include <gtest/gtest.h>

#include <cmath>

#include "core_body_controller/target_angle_controller.hpp"

namespace
{

TEST(TargetAngleControllerTest, QuaternionIsNormalizedBeforeExtractingYaw)
{
  double yaw = 0.0;
  constexpr double HALF_YAW = M_PI / 4.0;

  EXPECT_TRUE(
    core_body_controller::quaternion_to_yaw(
      0.0, 0.0, 2.0 * std::sin(HALF_YAW), 2.0 * std::cos(HALF_YAW), yaw));
  EXPECT_NEAR(yaw, M_PI / 2.0, 1e-12);
}

TEST(TargetAngleControllerTest, InvalidQuaternionsAreRejected)
{
  double yaw = 0.0;

  EXPECT_FALSE(core_body_controller::quaternion_to_yaw(0.0, 0.0, 0.0, 0.0, yaw));
  EXPECT_FALSE(core_body_controller::quaternion_to_yaw(0.0, 0.0, std::nan(""), 1.0, yaw));
  EXPECT_FALSE(core_body_controller::quaternion_to_yaw(0.0, 0.0, 0.0, 1.0e7, yaw));
}

TEST(TargetAngleControllerTest, FirstYawSampleDefinesZero)
{
  core_body_controller::YawTracker tracker;

  EXPECT_TRUE(tracker.update(1.2, false));
  EXPECT_DOUBLE_EQ(tracker.yaw(), 0.0);
  EXPECT_TRUE(tracker.update(1.4, true));
  EXPECT_NEAR(tracker.yaw(), 0.2, 1e-12);
}

TEST(TargetAngleControllerTest, YawIsUnwrappedAcrossPiBoundary)
{
  core_body_controller::YawTracker tracker;
  const double first = 179.0 * M_PI / 180.0;
  const double second = -179.0 * M_PI / 180.0;

  tracker.update(first, false);
  tracker.update(second, true);

  EXPECT_NEAR(tracker.yaw(), 2.0 * M_PI / 180.0, 1e-12);
}

TEST(TargetAngleControllerTest, ResumeSampleRebasesWithoutChangingTrackedYaw)
{
  core_body_controller::YawTracker tracker;

  tracker.update(0.2, false);
  tracker.update(0.7, true);
  tracker.update(-2.5, false);

  EXPECT_NEAR(tracker.yaw(), 0.5, 1e-12);
  tracker.update(-2.4, true);
  EXPECT_NEAR(tracker.yaw(), 0.6, 1e-12);
}

TEST(TargetAngleControllerTest, NonFiniteYawDoesNotChangeState)
{
  core_body_controller::YawTracker tracker;

  tracker.update(0.0, false);
  EXPECT_FALSE(tracker.update(std::nan(""), true));
  EXPECT_DOUBLE_EQ(tracker.yaw(), 0.0);
}

TEST(TargetAngleControllerTest, FinalOutputHonorsVelocityAndAccelerationLimits)
{
  core_body_controller::TargetAngleController controller(2.0, 0.0, 0.0, 1.0, 2.0, 0.0);

  EXPECT_NEAR(controller.update(-10.0, 100.0, 0.1), 0.2, 1e-12);
  EXPECT_NEAR(controller.update(-10.0, 100.0, 0.1), 0.4, 1e-12);
  for (int i = 0; i < 10; ++i) {
    controller.update(-10.0, 100.0, 0.1);
  }
  EXPECT_LE(std::abs(controller.output()), 1.0);
}

TEST(TargetAngleControllerTest, FeedforwardCanBeEnabledWithoutBypassingLimits)
{
  core_body_controller::TargetAngleController controller(2.0, 0.0, 0.0, 1.0, 1000.0, 0.0);

  EXPECT_DOUBLE_EQ(controller.update(0.0, 0.0, 0.01), 0.0);
  EXPECT_NEAR(controller.update(0.0, 0.5, 0.01), 0.5, 1e-12);
  EXPECT_NEAR(controller.update(0.0, 5.0, 0.01), 1.0, 1e-12);
}

TEST(TargetAngleControllerTest, DeadbandDoesNotBypassAccelerationLimit)
{
  constexpr double acceleration_limit = 2.0;
  constexpr double dt = 0.01;
  core_body_controller::TargetAngleController controller(
    0.0, 0.0, 0.0, 1.0, acceleration_limit, 0.1);

  EXPECT_DOUBLE_EQ(controller.update(0.0, 0.05, dt), 0.0);

  double previous = 0.0;
  for (int i = 0; i < 10; ++i) {
    const double command = controller.update(0.0, 1.0, dt);
    EXPECT_LE(std::abs(command - previous), acceleration_limit * dt + 1e-12);
    previous = command;
  }

  for (int i = 0; i < 10; ++i) {
    const double command = controller.update(0.0, 0.0, dt);
    EXPECT_LE(std::abs(command - previous), acceleration_limit * dt + 1e-12);
    previous = command;
  }
}

}  // namespace
