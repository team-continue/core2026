#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "core_body_controller/velocity_frame.hpp"

namespace
{
using core_body_controller::in_turned_frame;
using core_body_controller::limited_turn;
using core_body_controller::PlanarVelocity;

PlanarVelocity to_world(const PlanarVelocity & velocity, const double yaw)
{
  return PlanarVelocity{
    std::cos(yaw) * velocity.x - std::sin(yaw) * velocity.y,
    std::sin(yaw) * velocity.x + std::cos(yaw) * velocity.y};
}

// body_control_node と同じ、軸ごとに 1 周期の変化を step までにする制限
PlanarVelocity rate_limit(
  const PlanarVelocity & current, const PlanarVelocity & target,
  const double step)
{
  return PlanarVelocity{
    current.x + std::clamp(target.x - current.x, -step, step),
    current.y + std::clamp(target.y - current.y, -step, step)};
}

TEST(VelocityFrame, TurnedFrameKeepsWorldVelocity)
{
  const PlanarVelocity velocity{1.0, 0.5};
  const auto turned = in_turned_frame(velocity, 1.1 - 0.4);
  EXPECT_NEAR(to_world(turned, 1.1).x, to_world(velocity, 0.4).x, 1e-12);
  EXPECT_NEAR(to_world(turned, 1.1).y, to_world(velocity, 0.4).y, 1e-12);
}

TEST(VelocityFrame, LimitedTurnWrapsAndClamps)
{
  EXPECT_NEAR(limited_turn(3.1, -3.1, 1.0), 2.0 * M_PI - 6.2, 1e-12);
  EXPECT_DOUBLE_EQ(limited_turn(0.0, 0.5, 0.1), 0.1);
  EXPECT_DOUBLE_EQ(limited_turn(0.0, -0.5, 0.1), -0.1);
  EXPECT_DOUBLE_EQ(limited_turn(0.0, std::nan(""), 0.1), 0.0);
}

TEST(VelocityFrame, RotatingUpperDoesNotUseAccelerationLimit)
{
  // 世界の座標で 2.5m/s のまま 0.6rad/s で曲がる（横の加速度 1.5m/s²。制限 2.0m/s² の内）間に、上半身が
  // 逆向きに 0.9rad/s で回る。指令は上半身の座標で、制限は軸ごとに 2.0m/s²、10ms 周期
  const double speed = 2.5;
  const double turn_rate = 0.6;
  const double upper_rate = -0.9;
  const double dt = 0.01;
  const double step = 2.0 * dt;
  PlanarVelocity compensated{speed, 0.0};
  PlanarVelocity plain{speed, 0.0};
  double upper_yaw = 0.0;
  double direction = 0.0;
  for (int i = 0; i < 50; ++i) {
    const double next_upper_yaw = upper_yaw + upper_rate * dt;
    direction += turn_rate * dt;
    const PlanarVelocity world_target{speed * std::cos(direction), speed * std::sin(direction)};
    const PlanarVelocity target = in_turned_frame(world_target, next_upper_yaw);
    compensated = rate_limit(
      in_turned_frame(compensated, limited_turn(upper_yaw, next_upper_yaw, 0.1)), target, step);
    plain = rate_limit(plain, target, step);
    upper_yaw = next_upper_yaw;
  }
  // 前の出力を上半身の回った分だけ直せば、世界の座標の変化（1.5m/s²）だけが制限にかかり、指令どおり曲がる
  const auto world = to_world(compensated, upper_yaw);
  EXPECT_NEAR(world.x, speed * std::cos(direction), 1e-9);
  EXPECT_NEAR(world.y, speed * std::sin(direction), 1e-9);
  // 直さないと、上半身の座標での変化（2.5m/s × 1.5rad/s）が制限を超え、曲がり遅れる
  const auto plain_world = to_world(plain, upper_yaw);
  EXPECT_LT(std::atan2(plain_world.y, plain_world.x), direction - 0.05);
}
}  // namespace
