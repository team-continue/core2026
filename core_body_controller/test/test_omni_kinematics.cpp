#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "core_body_controller/omni_kinematics.hpp"

namespace
{

constexpr double WHEEL_RADIUS = 0.13 / 2;
constexpr double BODY_WIDTH = 0.5304;

// body_control_node::invert_kinematics_calc と同じ式
std::array<double, 4> inverse_kinematics(double vx, double vy, double omega, double body_angle)
{
  const double vx_body = vx * std::cos(body_angle) + vy * std::sin(body_angle);
  const double vy_body = -vx * std::sin(body_angle) + vy * std::cos(body_angle);
  const double c = std::cos(M_PI / 4);
  const double s = std::sin(M_PI / 4);
  const double l = std::sqrt(2.0) * BODY_WIDTH / 2;
  return {
    -(vx_body * c - vy_body * s - l * omega) / WHEEL_RADIUS,
    -(vx_body * c + vy_body * s - l * omega) / WHEEL_RADIUS,
    -(-vx_body * c + vy_body * s - l * omega) / WHEEL_RADIUS,
    -(-vx_body * c - vy_body * s - l * omega) / WHEEL_RADIUS,
  };
}

void expect_round_trip(double vx, double vy, double omega, double body_angle)
{
  const auto velocity = core_body_controller::forward_kinematics(
    inverse_kinematics(vx, vy, omega, body_angle), body_angle, WHEEL_RADIUS, BODY_WIDTH);
  ASSERT_TRUE(velocity.has_value());
  EXPECT_NEAR(velocity->linear_x, vx, 1e-9);
  EXPECT_NEAR(velocity->linear_y, vy, 1e-9);
  EXPECT_NEAR(velocity->angular_z, omega, 1e-9);
}

TEST(OmniKinematicsTest, RecoversCommandWithoutBodyAngle)
{
  expect_round_trip(1.0, 0.0, 0.0, 0.0);
  expect_round_trip(0.0, -0.7, 0.0, 0.0);
  expect_round_trip(0.0, 0.0, 3.14, 0.0);
  expect_round_trip(0.5, 0.3, -6.28, 0.0);
}

TEST(OmniKinematicsTest, RecoversCommandWithBodyAngle)
{
  expect_round_trip(1.0, 0.0, 0.0, M_PI / 2);
  expect_round_trip(0.4, -0.2, 2.0, -2.5);
  expect_round_trip(-0.8, 0.6, -3.14, 7.0);
}

TEST(OmniKinematicsTest, InconsistentWheelsAreAveraged)
{
  // 前進の車輪速度に 1 輪だけ誤差を足しても、全体は前進のまま
  auto wheels = inverse_kinematics(1.0, 0.0, 0.0, 0.0);
  const auto clean = core_body_controller::forward_kinematics(
    wheels, 0.0, WHEEL_RADIUS, BODY_WIDTH);
  wheels[0] += 1.0;
  const auto noisy = core_body_controller::forward_kinematics(
    wheels, 0.0, WHEEL_RADIUS, BODY_WIDTH);
  ASSERT_TRUE(clean.has_value());
  ASSERT_TRUE(noisy.has_value());
  EXPECT_NEAR(noisy->linear_x, clean->linear_x, 0.05);
}

TEST(OmniKinematicsTest, InvalidInputsAreRejected)
{
  const std::array<double, 4> zeros{0.0, 0.0, 0.0, 0.0};
  const std::array<double, 4> with_nan{0.0, std::nan(""), 0.0, 0.0};

  EXPECT_FALSE(
    core_body_controller::forward_kinematics(with_nan, 0.0, WHEEL_RADIUS, BODY_WIDTH));
  EXPECT_FALSE(
    core_body_controller::forward_kinematics(zeros, std::nan(""), WHEEL_RADIUS, BODY_WIDTH));
  EXPECT_FALSE(core_body_controller::forward_kinematics(zeros, 0.0, 0.0, BODY_WIDTH));
  EXPECT_FALSE(core_body_controller::forward_kinematics(zeros, 0.0, WHEEL_RADIUS, -1.0));
}

}  // namespace
