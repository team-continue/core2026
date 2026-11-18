#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "core_planning/yaw_hold.hpp"

namespace
{
using core_planning::BodyVelocity;
using core_planning::holdWorldDirection;
using core_planning::YawHold;
using core_planning::YawHoldParams;

// 機体の座標の並進を、向き yaw の機体から見た世界の座標に直した向き
double worldDirection(const BodyVelocity & body, const double yaw)
{
  return std::atan2(
    std::sin(yaw) * body.vx + std::cos(yaw) * body.vy,
    std::cos(yaw) * body.vx - std::sin(yaw) * body.vy);
}

TEST(HoldWorldDirection, KeepsWorldDirectionWhileTurning)
{
  const BodyVelocity command{1.5, 0.5, 0.8};
  const double start_yaw = 0.3;
  const double expected = worldDirection(command, start_yaw);
  for (const double turned : {-0.4, -0.1, 0.0, 0.05, 0.2, 0.45}) {
    const BodyVelocity held = holdWorldDirection(command, start_yaw, start_yaw + turned, 0.5);
    EXPECT_NEAR(worldDirection(held, start_yaw + turned), expected, 1e-9) << "turned=" << turned;
    EXPECT_NEAR(std::hypot(held.vx, held.vy), std::hypot(command.vx, command.vy), 1e-12);
    EXPECT_EQ(held.wz, command.wz);
  }
}

TEST(HoldWorldDirection, LimitsCorrection)
{
  const BodyVelocity command{1.0, 0.0, 0.0};
  const BodyVelocity held = holdWorldDirection(command, 0.0, 1.2, 0.5);
  // 1.2rad 回っても 0.5rad だけ逆に回す
  EXPECT_NEAR(std::atan2(held.vy, held.vx), -0.5, 1e-12);
}

TEST(YawHold, IntegratesImuAndHoldsDirectionUntilTimeout)
{
  YawHold hold(YawHoldParams{});
  EXPECT_FALSE(hold.output(0.0).has_value());
  EXPECT_FALSE(hold.hasYaw());
  // 1rad/s で回る IMU（200Hz）
  double t = 0.0;
  for (int i = 0; i <= 20; ++i) {
    t = 0.005 * i;
    hold.onYawRate(t, 1.0);
  }
  ASSERT_TRUE(hold.hasYaw());
  const BodyVelocity command{2.0, 0.0, 1.0};
  hold.onCommand(command, t);
  // 受けた直後はそのまま
  const auto first = hold.output(t);
  ASSERT_TRUE(first.has_value());
  EXPECT_NEAR(first->vx, 2.0, 1e-9);
  EXPECT_NEAR(first->vy, 0.0, 1e-9);
  // 0.1 秒回ったら、並進を −0.1rad 回す
  const double command_time = t;
  for (int i = 0; i < 20; ++i) {
    t += 0.005;
    hold.onYawRate(t, 1.0);
  }
  const auto later = hold.output(t);
  ASSERT_TRUE(later.has_value());
  EXPECT_NEAR(std::atan2(later->vy, later->vx), -0.1, 1e-6);
  EXPECT_EQ(later->wz, 1.0);
  // timeout_s（0.2 秒）を過ぎたら出さない
  EXPECT_FALSE(hold.output(command_time + 0.21).has_value());
}

TEST(YawHold, SkipsImuGapsAndTimeGoingBack)
{
  YawHoldParams params;
  params.max_imu_gap_s = 0.05;
  YawHold hold(params);
  hold.onYawRate(0.0, 1.0);
  hold.onCommand(BodyVelocity{1.0, 0.0, 0.0}, 0.0);
  // 0.5 秒の抜けは積まない
  hold.onYawRate(0.5, 1.0);
  // 時計が戻った分も積まない
  hold.onYawRate(0.1, 1.0);
  const auto out = hold.output(0.1);
  ASSERT_TRUE(out.has_value());
  EXPECT_NEAR(out->vy, 0.0, 1e-12);
  // 戻った後の続きは積む
  hold.onYawRate(0.12, 1.0);
  const auto after = hold.output(0.12);
  ASSERT_TRUE(after.has_value());
  EXPECT_NEAR(std::atan2(after->vy, after->vx), -0.02, 1e-9);
}

TEST(YawHold, PassesStopCommandAsZero)
{
  YawHold hold(YawHoldParams{});
  hold.onYawRate(0.0, 2.0);
  hold.onCommand(BodyVelocity{}, 0.0);
  hold.onYawRate(0.01, 2.0);
  const auto out = hold.output(0.01);
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(out->vx, 0.0);
  EXPECT_EQ(out->vy, 0.0);
  EXPECT_EQ(out->wz, 0.0);
}

TEST(YawHold, RejectsInvalidParams)
{
  YawHoldParams params;
  params.timeout_s = 0.0;
  EXPECT_THROW(YawHold{params}, std::invalid_argument);
  params = YawHoldParams{};
  params.max_correction_rad = -0.1;
  EXPECT_THROW(YawHold{params}, std::invalid_argument);
  params = YawHoldParams{};
  params.max_imu_gap_s = std::nan("");
  EXPECT_THROW(YawHold{params}, std::invalid_argument);
}
}  // namespace

TEST(YawHold, UsesYawAtReferenceTimeOfCommand)
{
  // 指令を作った側が機体の座標に直した時刻を渡せば、受けた時ではなくその時の向きから回った分を戻す
  YawHold hold(YawHoldParams{});
  double t = 0.0;
  for (int i = 0; i <= 40; ++i) {
    t = 0.005 * i;
    hold.onYawRate(t, 1.0);
  }
  // 0.2 秒に受けた指令は、0.1 秒に作ったもの（その間に 0.1rad 回った）
  hold.onCommand(BodyVelocity{2.0, 0.0, 0.0}, t, 0.1);
  const auto now = hold.output(t);
  ASSERT_TRUE(now.has_value());
  EXPECT_NEAR(std::atan2(now->vy, now->vx), -0.1, 1e-6);
  ASSERT_TRUE(hold.yawAt(0.1).has_value());
  EXPECT_NEAR(*hold.yawAt(0.1), 0.1, 1e-9);
  // 覚えている範囲の外（先の時刻・古すぎる時刻）は使わず、受けた時の向きにする
  EXPECT_FALSE(hold.yawAt(t + 0.1).has_value());
  hold.onCommand(BodyVelocity{2.0, 0.0, 0.0}, t, t + 0.05);
  EXPECT_NEAR(std::atan2(hold.output(t)->vy, hold.output(t)->vx), 0.0, 1e-9);
  hold.onCommand(BodyVelocity{2.0, 0.0, 0.0}, t, t - 1.0);
  EXPECT_NEAR(std::atan2(hold.output(t)->vy, hold.output(t)->vx), 0.0, 1e-9);
}
