#include <gtest/gtest.h>

#include "core_localization/gyro_odometer.hpp"

namespace
{
using core_localization::GyroOdometer;
using core_localization::GyroOdometerParams;
using core_localization::GyroRate;
using core_localization::TwistEstimate;
using core_localization::WheelTwist;

WheelTwist wheel(const double stamp_s, const double vx, const double vy, const double wz)
{
  return WheelTwist{stamp_s, vx, vy, wz, 0.0, 0.0, 0.0};
}

TEST(GyroOdometer, TakesTranslationFromWheelAndYawRateFromGyro)
{
  GyroOdometer odometer{GyroOdometerParams{}};
  odometer.addGyro(GyroRate{0.005, 0.4, 0.0});
  odometer.addGyro(GyroRate{0.010, 0.6, 0.0});
  const TwistEstimate estimate = odometer.combine(wheel(0.010, 1.5, -0.3, 9.0));
  EXPECT_DOUBLE_EQ(estimate.vx, 1.5);
  EXPECT_DOUBLE_EQ(estimate.vy, -0.3);
  // 車輪の角速度（9.0）ではなく、ジャイロの平均
  EXPECT_DOUBLE_EQ(estimate.wz, 0.5);
  EXPECT_TRUE(estimate.used_gyro);
}

TEST(GyroOdometer, UsesDefaultVariancesWhenMessagesHaveNone)
{
  const GyroOdometerParams params;
  GyroOdometer odometer{params};
  odometer.addGyro(GyroRate{0.005, 0.0, 0.0});
  const TwistEstimate estimate = odometer.combine(wheel(0.010, 0.0, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(estimate.var_vx, params.wheel_linear_variance);
  EXPECT_DOUBLE_EQ(estimate.var_vy, params.wheel_linear_variance);
  EXPECT_DOUBLE_EQ(estimate.var_wz, params.gyro_variance);
}

TEST(GyroOdometer, KeepsGivenVariances)
{
  GyroOdometer odometer{GyroOdometerParams{}};
  odometer.addGyro(GyroRate{0.005, 0.0, 4.0e-6});
  odometer.addGyro(GyroRate{0.010, 0.0, 6.0e-6});
  const TwistEstimate estimate =
    odometer.combine(WheelTwist{0.010, 0.0, 0.0, 0.0, 0.02, 0.03, 0.5});
  EXPECT_DOUBLE_EQ(estimate.var_vx, 0.02);
  EXPECT_DOUBLE_EQ(estimate.var_vy, 0.03);
  // 平均しても偏りは減らないので、1 つ分の分散（の平均）のまま
  EXPECT_DOUBLE_EQ(estimate.var_wz, 5.0e-6);
}

TEST(GyroOdometer, KeepsGyroAfterWheelStampForNextWheel)
{
  GyroOdometer odometer{GyroOdometerParams{}};
  odometer.addGyro(GyroRate{0.005, 1.0, 0.0});
  odometer.addGyro(GyroRate{0.015, 3.0, 0.0});
  EXPECT_DOUBLE_EQ(odometer.combine(wheel(0.010, 0.0, 0.0, 0.0)).wz, 1.0);
  EXPECT_DOUBLE_EQ(odometer.combine(wheel(0.020, 0.0, 0.0, 0.0)).wz, 3.0);
}

TEST(GyroOdometer, UsesLastGyroWhileItIsFresh)
{
  GyroOdometer odometer{GyroOdometerParams{}};
  odometer.addGyro(GyroRate{0.005, 0.7, 0.0});
  odometer.combine(wheel(0.010, 0.0, 0.0, 0.0));
  // 新しいジャイロがなくても、最後の値から gyro_timeout_s（0.1 秒）以内なら、それを使う
  const TwistEstimate estimate = odometer.combine(wheel(0.090, 0.0, 0.0, -2.0));
  EXPECT_DOUBLE_EQ(estimate.wz, 0.7);
  EXPECT_TRUE(estimate.used_gyro);
}

TEST(GyroOdometer, FallsBackToWheelYawRateAfterTimeout)
{
  const GyroOdometerParams params;
  GyroOdometer odometer{params};
  odometer.addGyro(GyroRate{0.005, 0.7, 0.0});
  odometer.combine(wheel(0.010, 0.0, 0.0, 0.0));
  const TwistEstimate estimate = odometer.combine(wheel(0.200, 0.0, 0.0, -2.0));
  EXPECT_DOUBLE_EQ(estimate.wz, -2.0);
  EXPECT_DOUBLE_EQ(estimate.var_wz, params.wheel_angular_variance);
  EXPECT_FALSE(estimate.used_gyro);
}

TEST(GyroOdometer, FallsBackToWheelYawRateBeforeAnyGyro)
{
  GyroOdometer odometer{GyroOdometerParams{}};
  const TwistEstimate estimate = odometer.combine(wheel(0.010, 0.0, 0.0, 0.3));
  EXPECT_DOUBLE_EQ(estimate.wz, 0.3);
  EXPECT_FALSE(estimate.used_gyro);
}

TEST(GyroOdometer, UsesLateGyroInNextAverage)
{
  GyroOdometer odometer{GyroOdometerParams{}};
  odometer.addGyro(GyroRate{0.005, 1.0, 0.0});
  odometer.combine(wheel(0.010, 0.0, 0.0, 0.0));
  // 前回の車輪の時刻より古い値も、合わせた後に届いたものは次の平均に入れる（捨てない）
  odometer.addGyro(GyroRate{0.008, 1.0, 0.0});
  odometer.addGyro(GyroRate{0.015, 3.0, 0.0});
  EXPECT_DOUBLE_EQ(odometer.combine(wheel(0.020, 0.0, 0.0, 0.0)).wz, 2.0);
}
}  // namespace
