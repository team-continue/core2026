#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Geometry>

#include "core_perception/track_frame.hpp"

namespace
{
using core_perception::PlanarTrackState;
using core_perception::transformTrackState;

TEST(TrackFrame, MovesPositionAndRotatesVelocity)
{
  // map ← odom: odom の原点が地図の (-10.25, -5.5)、向きが +90°
  Eigen::Isometry3f map_from_odom = Eigen::Isometry3f::Identity();
  map_from_odom.translate(Eigen::Vector3f(-10.25F, -5.5F, 0.0F));
  map_from_odom.rotate(Eigen::AngleAxisf(static_cast<float>(M_PI / 2.0), Eigen::Vector3f::UnitZ()));
  const auto result = transformTrackState(PlanarTrackState{1.0, 0.0, 2.0, 0.0}, map_from_odom);
  EXPECT_NEAR(result.x, -10.25, 1e-5);
  EXPECT_NEAR(result.y, -4.5, 1e-5);
  // 速度は回すだけで、原点のずれは足さない
  EXPECT_NEAR(result.vx, 0.0, 1e-5);
  EXPECT_NEAR(result.vy, 2.0, 1e-5);
}
}  // namespace
