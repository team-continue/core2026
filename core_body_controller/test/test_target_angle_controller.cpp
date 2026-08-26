#include <cmath>

#include <gtest/gtest.h>

#include "core_body_controller/target_angle_controller.hpp"

namespace
{

TEST(TargetAngleControllerTest, ImuIntegrationUsesElapsedTime)
{
  double angle_at_100_hz = 0.0;
  double angle_at_50_hz = 0.0;

  for (int i = 0; i < 100; ++i) {
    angle_at_100_hz = core_body_controller::integrate_yaw(
      angle_at_100_hz, 1.0, 0.0, 0.01);
  }
  for (int i = 0; i < 50; ++i) {
    angle_at_50_hz = core_body_controller::integrate_yaw(
      angle_at_50_hz, 1.0, 0.0, 0.02);
  }

  EXPECT_NEAR(angle_at_100_hz, -1.0, 1e-12);
  EXPECT_NEAR(angle_at_50_hz, angle_at_100_hz, 1e-12);
}

TEST(TargetAngleControllerTest, BiasCompensationIsIndependentOfSampleCount)
{
  double angle_at_100_hz = 0.0;
  double angle_at_50_hz = 0.0;

  for (int i = 0; i < 100; ++i) {
    angle_at_100_hz = core_body_controller::integrate_yaw(
      angle_at_100_hz, 0.0, 0.03, 0.01);
  }
  for (int i = 0; i < 50; ++i) {
    angle_at_50_hz = core_body_controller::integrate_yaw(
      angle_at_50_hz, 0.0, 0.03, 0.02);
  }

  EXPECT_NEAR(angle_at_100_hz, 0.03, 1e-12);
  EXPECT_NEAR(angle_at_50_hz, angle_at_100_hz, 1e-12);
}

TEST(TargetAngleControllerTest, FinalOutputHonorsVelocityAndAccelerationLimits)
{
  core_body_controller::TargetAngleController controller(
    2.0, 0.0, 0.0, 1.0, 2.0, 0.0);

  EXPECT_NEAR(controller.update(-10.0, 100.0, 0.1), 0.2, 1e-12);
  EXPECT_NEAR(controller.update(-10.0, 100.0, 0.1), 0.4, 1e-12);
  for (int i = 0; i < 10; ++i) {
    controller.update(-10.0, 100.0, 0.1);
  }
  EXPECT_LE(std::abs(controller.output()), 1.0);
}

TEST(TargetAngleControllerTest, FeedforwardCanBeEnabledWithoutBypassingLimits)
{
  core_body_controller::TargetAngleController controller(
    2.0, 0.0, 0.0, 1.0, 1000.0, 0.0);

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
