#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <random>
#include <vector>

#include "core_gazebo/sim_hardware_model.hpp"

namespace sh = core_gazebo::sim_hardware;

namespace
{
constexpr double kTolerance = 1e-9;
}  // namespace

// 車輪の速さから足回りの速度に戻すと、body_control_node の式で作った車輪の速さの元の速度になる
TEST(SimHardwareModel, ChassisTwistInvertsBodyControlNodeKinematics)
{
  const sh::WheelGeometry geometry;
  std::mt19937 random(7);
  std::uniform_real_distribution<double> speed(-3.0, 3.0);
  for (int i = 0; i < 200; ++i) {
    const sh::PlanarTwist chassis{speed(random), speed(random), speed(random)};
    const auto twist = sh::chassisTwistFromWheels(sh::wheelsFromChassisTwist(chassis, geometry), geometry);
    EXPECT_NEAR(twist.vx, chassis.vx, kTolerance);
    EXPECT_NEAR(twist.vy, chassis.vy, kTolerance);
    EXPECT_NEAR(twist.wz, chassis.wz, kTolerance);
  }
}

// body_control_node の invert_kinematics_calc を手で計算した値（半径 0.065m、幅 0.5304m）と同じ
TEST(SimHardwareModel, WheelsMatchBodyControlNodeForForwardAndTurn)
{
  const sh::WheelGeometry geometry;
  // 前へ 1m/s: 各車輪 ∓cos(45°) / 0.065
  const auto forward = sh::wheelsFromChassisTwist({1.0, 0.0, 0.0}, geometry);
  const double w = std::cos(M_PI / 4.0) / 0.065;
  EXPECT_NEAR(forward[0], -w, kTolerance);
  EXPECT_NEAR(forward[1], -w, kTolerance);
  EXPECT_NEAR(forward[2], w, kTolerance);
  EXPECT_NEAR(forward[3], w, kTolerance);
  // 反時計回りに 1rad/s: 各車輪 sqrt(2) * 0.5304 / 2 / 0.065
  const auto turn = sh::wheelsFromChassisTwist({0.0, 0.0, 1.0}, geometry);
  for (const double wheel : turn) {
    EXPECT_NEAR(wheel, std::sqrt(2.0) * 0.5304 / 2.0 / 0.065, kTolerance);
  }
}

// 腰の角度だけ並進を回す。body_control_node が上半身の指令を −q 回して足回りの指令にするのと逆
TEST(SimHardwareModel, UpperTwistUndoesBodyControlNodeRotation)
{
  // 足回りが上半身から見て 90° 左を向いているとき、足回りの前は上半身の左
  const auto upper = sh::upperTwist({1.0, 0.0, 0.0}, M_PI / 2.0, 0.0);
  EXPECT_NEAR(upper.vx, 0.0, kTolerance);
  EXPECT_NEAR(upper.vy, 1.0, kTolerance);

  // body_control_node: vx_body = vx cos q + vy sin q、vy_body = −vx sin q + vy cos q
  const double q = 0.7;
  const double vx = 0.4;
  const double vy = -1.1;
  const sh::PlanarTwist chassis{
    vx * std::cos(q) + vy * std::sin(q), -vx * std::sin(q) + vy * std::cos(q), 0.3};
  const auto back = sh::upperTwist(chassis, q, 0.0);
  EXPECT_NEAR(back.vx, vx, kTolerance);
  EXPECT_NEAR(back.vy, vy, kTolerance);
  EXPECT_NEAR(back.wz, 0.3, kTolerance);
}

// 回転モードで、target_angle_node が足回りの回る速さを先回りで腰に足すと、上半身は回らない
TEST(SimHardwareModel, WaistFeedforwardKeepsUpperStillWhileChassisSpins)
{
  const double spin = -M_PI;  // body_controller.launch.py の auto_rotation_velocity
  const auto upper = sh::upperTwist({0.0, 0.0, spin}, 1.2, spin);
  EXPECT_NEAR(upper.wz, 0.0, kTolerance);
  // 腰が回らなければ、上半身は足回りと一緒に回る（回転モード 0 の普段の走り）
  EXPECT_NEAR(sh::upperTwist({0.0, 0.0, 0.5}, 0.0, 0.0).wz, 0.5, kTolerance);
}

TEST(SimHardwareModel, DecodeReadsOnlyWheelAndWaistVelocityCommands)
{
  const auto wheel = sh::decode(2, {20.0F, -4.5F});
  ASSERT_TRUE(wheel.has_value());
  EXPECT_EQ(wheel->kind, sh::CommandKind::kWheel);
  EXPECT_EQ(wheel->wheel, 2);
  EXPECT_DOUBLE_EQ(wheel->value, -4.5);

  const auto waist = sh::decode(4, {3.0F, 1.25F});
  ASSERT_TRUE(waist.has_value());
  EXPECT_EQ(waist->kind, sh::CommandKind::kWaist);
  EXPECT_DOUBLE_EQ(waist->value, 1.25);

  // 車輪の番号でもモードが違う、腰の番号でもモードが違う、ほかの番号、値が足りない・有限でない
  EXPECT_FALSE(sh::decode(1, {5.0F, 1.0F}).has_value());
  EXPECT_FALSE(sh::decode(4, {20.0F, 1.0F}).has_value());
  EXPECT_FALSE(sh::decode(5, {20.0F, 1.0F}).has_value());
  EXPECT_FALSE(sh::decode(0, {20.0F}).has_value());
  EXPECT_FALSE(sh::decode(0, {20.0F, std::nanf("")}).has_value());
}
