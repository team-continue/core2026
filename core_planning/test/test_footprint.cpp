#include <cmath>
#include <optional>

#include <gtest/gtest.h>

#include "core_planning/footprint.hpp"
#include "core_planning/mppi.hpp"
#include "core_planning/safety.hpp"

namespace
{
using core_planning::boundingHalfExtents;
using core_planning::ChassisShape;
using core_planning::chassisShape;
using core_planning::FootprintParams;
using core_planning::orientFootprint;
using core_planning::signedDistanceToChassis;
using core_planning::signedDistanceToFootprint;

constexpr double kHalf = 0.4;

TEST(ChassisFootprint, AlignedChassisIsSameAsRectangle)
{
  const ChassisShape shape = chassisShape(kHalf, kHalf, 0.0, false);
  EXPECT_FALSE(shape.circle);
  EXPECT_NEAR(signedDistanceToChassis(1.0, 0.0, shape), 0.6, 1e-12);
  EXPECT_NEAR(signedDistanceToChassis(0.0, 0.0, shape), -kHalf, 1e-12);
}

TEST(ChassisFootprint, RotatedChassisMovesCorners)
{
  // 駆動部が 45 度回ると、角が base_link の x 軸の上（0.566m）に来る
  const ChassisShape shape = chassisShape(kHalf, kHalf, M_PI / 4.0, false);
  const double corner = std::hypot(kHalf, kHalf);
  EXPECT_NEAR(signedDistanceToChassis(corner, 0.0, shape), 0.0, 1e-9);
  EXPECT_LT(signedDistanceToChassis(0.5, 0.0, shape), 0.0);
  // 回っていない長方形なら 0.5m は外
  EXPECT_GT(signedDistanceToChassis(0.5, 0.0, chassisShape(kHalf, kHalf, 0.0, false)), 0.0);
  const auto extents = boundingHalfExtents(shape);
  EXPECT_NEAR(extents.half_length, corner, 1e-9);
  EXPECT_NEAR(extents.half_width, corner, 1e-9);
}

TEST(ChassisFootprint, RotatingOrUnknownAngleUsesCircle)
{
  const double radius = std::hypot(kHalf, kHalf);
  for (const ChassisShape & shape :
    {chassisShape(kHalf, kHalf, 0.0, true), chassisShape(kHalf, kHalf, std::nullopt, false)})
  {
    EXPECT_TRUE(shape.circle);
    EXPECT_NEAR(signedDistanceToChassis(0.0, 1.0, shape), 1.0 - radius, 1e-12);
    EXPECT_NEAR(signedDistanceToChassis(0.7, 0.7, shape), std::hypot(0.7, 0.7) - radius, 1e-12);
    const auto extents = boundingHalfExtents(shape);
    EXPECT_NEAR(extents.half_length, radius, 1e-12);
    EXPECT_NEAR(extents.half_width, radius, 1e-12);
  }
}

TEST(ChassisFootprint, OrientFootprintKeepsChassisAndBoundsTheRest)
{
  FootprintParams footprint;
  // 駆動部の形が決まっていなければ、今までどおり base_link の向きの長方形
  EXPECT_NEAR(signedDistanceToFootprint(0.5, 0.0, footprint), 0.1, 1e-12);
  orientFootprint(footprint, chassisShape(0.4, 0.3, M_PI / 2.0, false));
  // 90 度回ると、base_link の x の半分が 0.3m、y の半分が 0.4m
  EXPECT_NEAR(footprint.half_length_m, 0.3, 1e-9);
  EXPECT_NEAR(footprint.half_width_m, 0.4, 1e-9);
  EXPECT_NEAR(signedDistanceToFootprint(0.5, 0.0, footprint), 0.2, 1e-9);
  EXPECT_NEAR(signedDistanceToFootprint(0.0, 0.5, footprint), 0.1, 1e-9);

  core_planning::safety::Footprint safety_footprint;
  orientFootprint(safety_footprint, chassisShape(0.4, 0.4, 0.0, true));
  EXPECT_NEAR(signedDistanceToFootprint(1.0, 0.0, safety_footprint), 1.0 - std::hypot(0.4, 0.4), 1e-12);
}
}  // namespace
