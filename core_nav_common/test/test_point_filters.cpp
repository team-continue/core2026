#include <array>
#include <random>
#include <set>
#include <vector>

#include <gtest/gtest.h>

#include "core_nav_common/point_filters.hpp"

namespace
{
using Point = std::array<float, 3>;

const auto kPositionOf = [](const Point & point) {return point;};

TEST(VoxelFilter, KeepsFirstPointOfEachVoxel)
{
  const std::vector<Point> points{
    {0.1F, 0.1F, 0.1F}, {0.2F, 0.2F, 0.2F}, {0.6F, 0.1F, 0.1F}, {-0.1F, 0.1F, 0.1F}};
  const auto output = core_nav_common::voxelFilter(points, 0.5, kPositionOf);
  ASSERT_EQ(output.size(), 3U);
  EXPECT_EQ(output[0], points[0]);
  EXPECT_EQ(output[1], points[2]);
  EXPECT_EQ(output[2], points[3]);
}

TEST(RandomSample, ReturnsTargetCountOfDistinctPoints)
{
  std::vector<Point> points;
  for (int i = 0; i < 1000; ++i) {
    points.push_back({static_cast<float>(i), 0.0F, 0.0F});
  }
  std::mt19937 random(0U);
  const auto output = core_nav_common::randomSample(points, 100U, random);
  ASSERT_EQ(output.size(), 100U);
  EXPECT_EQ(std::set<Point>(output.begin(), output.end()).size(), 100U);
}

TEST(RandomSample, KeepsAllPointsWhenTargetIsLarger)
{
  const std::vector<Point> points{{1.0F, 0.0F, 0.0F}, {2.0F, 0.0F, 0.0F}};
  std::mt19937 random(0U);
  EXPECT_EQ(core_nav_common::randomSample(points, 10U, random).size(), 2U);
}
}  // namespace
