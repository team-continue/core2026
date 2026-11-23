#include <cstdint>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "core_planning/grid_map.hpp"

namespace
{
using core_planning::addPolygons;
using core_planning::buildDistanceField;
using core_planning::FieldBounds;
using core_planning::GridCell;
using core_planning::makeFieldGrid;
using core_planning::polygonsFromArrays;
using core_planning::StaticMapOptions;

FieldBounds field()
{
  FieldBounds bounds;
  bounds.x_min = -5.0;
  bounds.x_max = 5.0;
  bounds.y_min = -5.0;
  bounds.y_max = 5.0;
  return bounds;
}

TEST(KeepOut, PolygonCellsBecomeOccupied)
{
  auto grid = makeFieldGrid(field(), 0.1);
  // 斜めの四角形（菱形）。中心は占有、外は空き
  addPolygons(grid, {{{0.0, -1.0}, {1.0, 0.0}, {0.0, 1.0}, {-1.0, 0.0}}});
  EXPECT_TRUE(grid.isOccupied(grid.info().cellAt(0.0, 0.0)));
  EXPECT_TRUE(grid.isOccupied(grid.info().cellAt(0.45, 0.45)));
  EXPECT_FALSE(grid.isOccupied(grid.info().cellAt(0.65, 0.65)));
  EXPECT_FALSE(grid.isOccupied(grid.info().cellAt(2.0, 0.0)));
}

TEST(KeepOut, DistanceFieldTreatsKeepOutAsObstacle)
{
  StaticMapOptions options;
  options.keep_out = polygonsFromArrays({-0.5, 0.5, 0.5, -0.5}, {-0.5, -0.5, 0.5, 0.5}, {4});
  const auto with_zone = buildDistanceField(field(), 0.1, "", options);
  const auto without = buildDistanceField(field(), 0.1, "");
  EXPECT_LE(with_zone.at(0.0, 0.0), 0.0F);
  EXPECT_NEAR(with_zone.at(1.5, 0.0), 1.0F, 0.11F);
  EXPECT_GT(without.at(1.5, 0.0), 3.0F);
}

TEST(KeepOut, ArraysMustMatch)
{
  EXPECT_TRUE(polygonsFromArrays({}, {}, {}).empty());
  const auto polygons = polygonsFromArrays(
    {0, 1, 1, 5, 6, 6, 5}, {0, 0, 1, 5, 5, 6, 6}, {3, 4});
  ASSERT_EQ(polygons.size(), 2U);
  EXPECT_EQ(polygons[1].size(), 4U);
  EXPECT_THROW(polygonsFromArrays({0, 1}, {0}, {2}), std::invalid_argument);
  EXPECT_THROW(polygonsFromArrays({0, 1}, {0, 1}, {2}), std::invalid_argument);
  EXPECT_THROW(polygonsFromArrays({0, 1, 2}, {0, 1, 2}, {4}), std::invalid_argument);
  EXPECT_THROW(polygonsFromArrays({0, 1, 2, 3}, {0, 1, 2, 3}, {3}), std::invalid_argument);
}
}  // namespace
