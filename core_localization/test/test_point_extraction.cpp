// 点群抽出（point_extraction.hpp）の計算の試験。

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "core_localization/point_extraction.hpp"

namespace
{
using core_localization::crowd::makeRegions;
using core_localization::point_extraction::Area;
using core_localization::point_extraction::Cylinder;
using core_localization::point_extraction::extract;
using core_localization::point_extraction::keepArea;
using core_localization::point_extraction::Mode;
using core_localization::point_extraction::modeName;
using core_localization::point_extraction::outsideBoundary;
using core_localization::point_extraction::Params;
using core_localization::point_extraction::parseMode;
using core_localization::point_extraction::predictCylinders;
using core_localization::point_extraction::Result;
using core_localization::point_extraction::Track;
using core_localization::point_extraction::TrackClass;
using core_localization::point_extraction::Transforms;
using core_localization::point_extraction::validate;

// field.yaml の範囲（柵の内側の面）
Params fieldObjects()
{
  Params params;
  params.mode = Mode::kFieldObjects;
  params.field_objects.field = Area{-12.0F, 12.0F, -7.5F, 7.5F};
  return params;
}

Params ceiling()
{
  Params params;
  params.mode = Mode::kCeiling;
  // 観客席の箱（exclude_regions.yaml の stand_north）
  params.ceiling.regions = makeRegions({"stand_north"}, {-21.0, 11.0, 13.0, 18.9, 5.1, 7.5});
  return params;
}

// 柵の外の帯（point_extraction.yaml の値。field.yaml の範囲に 1.0m を足した長方形の外で、上限 3.0m）
Params ceilingWithOutside()
{
  Params params = ceiling();
  params.ceiling.outside.enabled = true;
  params.ceiling.outside.field = Area{-12.0F, 12.0F, -7.5F, 7.5F};
  params.ceiling.outside.field_margin_m = 1.0F;
  params.ceiling.outside.band_max_m = 3.0F;
  return params;
}

// 点群の座標系が map・odom と一致し、床が z=0
Transforms identity()
{
  Transforms transforms;
  transforms.height_from_cloud = Eigen::Isometry3f::Identity();
  transforms.map_from_cloud = Eigen::Isometry3f::Identity();
  transforms.tracks_from_cloud = Eigen::Isometry3f::Identity();
  return transforms;
}

std::vector<std::uint8_t> keepOf(const Result & result)
{
  return result.keep;
}

TEST(PointExtractionMode, ParsesNamesAndRejectsUnknown)
{
  for (const Mode mode : {Mode::kOff, Mode::kCeiling, Mode::kFieldObjects}) {
    EXPECT_EQ(parseMode(modeName(mode)), mode);
  }
  EXPECT_EQ(modeName(Mode::kFieldObjects), "field_objects");
  EXPECT_THROW(parseMode("map"), std::invalid_argument);
  EXPECT_THROW(parseMode("lio_and_map"), std::invalid_argument);
  EXPECT_THROW(parseMode(""), std::invalid_argument);
}

TEST(PointExtraction, OffPassesEveryPoint)
{
  const std::vector<Eigen::Vector3f> points{{0.0F, 0.0F, 1.0F}, {30.0F, 0.0F, 5.0F}};
  const Result result = extract(points, Params{}, Transforms{}, {});
  EXPECT_EQ(keepOf(result), (std::vector<std::uint8_t>{1U, 1U}));
  EXPECT_EQ(result.kept, 2U);
  EXPECT_FALSE(result.fallback);
}

TEST(PointExtractionCeiling, KeepsFloorAndAboveHeadAndRemovesBandAndStand)
{
  const std::vector<Eigen::Vector3f> points{
    {3.0F, 0.0F, 0.05F},   // 床: 残す
    {3.0F, 0.0F, 1.0F},    // 人の帯: 除く
    {3.0F, 0.0F, 2.19F},   // 帯の上の端の中: 除く
    {3.0F, 0.0F, 2.3F},    // 頭より上: 残す
    {0.0F, 15.0F, 6.0F},   // 観客席の上の段の人: 除く
    {0.0F, 15.0F, 4.9F},   // 観客席の段（箱の下）: 残す
    {30.0F, 0.0F, 10.0F},  // 壁の上の方: 残す
  };
  const Result result = extract(points, ceiling(), identity(), {});
  EXPECT_EQ(keepOf(result), (std::vector<std::uint8_t>{1U, 0U, 0U, 1U, 0U, 1U, 1U}));
  EXPECT_EQ(result.removed_band, 2U);
  EXPECT_EQ(result.removed_region, 1U);
  EXPECT_EQ(result.kept, 4U);
  EXPECT_EQ(result.removed_outside + result.removed_above + result.removed_track, 0U);
}

TEST(PointExtractionCeiling, SkipsStandWithoutMapAndBandWithoutFloor)
{
  const std::vector<Eigen::Vector3f> points{{3.0F, 0.0F, 1.0F}, {0.0F, 15.0F, 6.0F}};
  Transforms no_map = identity();
  no_map.map_from_cloud.reset();
  const Result without_map = extract(points, ceiling(), no_map, {});
  EXPECT_TRUE(without_map.region_skipped);
  EXPECT_EQ(keepOf(without_map), (std::vector<std::uint8_t>{0U, 1U}));

  Transforms no_floor = identity();
  no_floor.height_from_cloud.reset();
  const Result without_floor = extract(points, ceiling(), no_floor, {});
  EXPECT_TRUE(without_floor.band_skipped);
  EXPECT_EQ(keepOf(without_floor), (std::vector<std::uint8_t>{1U, 0U}));
}

TEST(PointExtractionCeiling, PassesEveryPointWhenTooFewRemain)
{
  Params params = ceiling();
  params.ceiling.min_points = 3U;
  const std::vector<Eigen::Vector3f> points{
    {3.0F, 0.0F, 0.05F}, {3.0F, 0.0F, 1.0F}, {3.0F, 0.0F, 1.5F}, {3.0F, 0.0F, 3.0F}};
  const Result result = extract(points, params, identity(), {});
  EXPECT_TRUE(result.fallback);
  EXPECT_EQ(result.removed_band, 2U);
  EXPECT_EQ(result.kept, 4U);
  EXPECT_EQ(keepOf(result), (std::vector<std::uint8_t>{1U, 1U, 1U, 1U}));
}

TEST(PointExtractionCeiling, WidensBandOnlyOutsideFieldMargin)
{
  const std::vector<Eigen::Vector3f> points{
    {3.0F, 0.0F, 2.5F},    // フィールドの中の 2.5m: 残す
    {12.5F, 0.0F, 2.5F},   // 東の柵から 0.5m（余裕の中）: 残す
    {13.5F, 0.0F, 2.5F},   // 東の柵から 1.5m: 除く
    {0.0F, -9.0F, 2.9F},   // 南の柵から 1.5m: 除く
    {13.5F, 0.0F, 3.1F},   // 柵の外の帯より上: 残す
    {13.5F, 0.0F, 0.05F},  // 柵の外の床: 残す
    {13.5F, 0.0F, 1.0F},   // 柵の外のふつうの帯: 除く（帯として数える）
    {0.0F, 15.0F, 6.0F},   // 観客席の上の段の人: 除く（箱として数える）
  };
  const Result result = extract(points, ceilingWithOutside(), identity(), {});
  EXPECT_EQ(keepOf(result), (std::vector<std::uint8_t>{1U, 1U, 0U, 0U, 1U, 1U, 0U, 0U}));
  EXPECT_EQ(result.removed_band, 1U);
  EXPECT_EQ(result.removed_region, 1U);
  EXPECT_EQ(result.removed_outside_band, 2U);
  EXPECT_EQ(result.kept, 4U);
  EXPECT_FALSE(result.outside_band_skipped);
  EXPECT_FALSE(result.fallback);
}

TEST(PointExtractionCeiling, KeepsTallerPointsOutsideWhenOutsideBandIsOff)
{
  Params params = ceilingWithOutside();
  params.ceiling.outside.enabled = false;
  const std::vector<Eigen::Vector3f> points{{13.5F, 0.0F, 2.5F}, {13.5F, 0.0F, 1.0F}};
  const Result result = extract(points, params, identity(), {});
  EXPECT_EQ(keepOf(result), (std::vector<std::uint8_t>{1U, 0U}));
  EXPECT_EQ(result.removed_outside_band, 0U);
  EXPECT_FALSE(result.outside_band_skipped);
}

TEST(PointExtractionCeiling, SkipsOutsideBandWithoutMapOrFloor)
{
  const std::vector<Eigen::Vector3f> points{{13.5F, 0.0F, 2.5F}, {13.5F, 0.0F, 1.0F}};
  Transforms no_map = identity();
  no_map.map_from_cloud.reset();
  const Result without_map = extract(points, ceilingWithOutside(), no_map, {});
  EXPECT_TRUE(without_map.outside_band_skipped);
  EXPECT_EQ(keepOf(without_map), (std::vector<std::uint8_t>{1U, 0U}));

  Transforms no_floor = identity();
  no_floor.height_from_cloud.reset();
  const Result without_floor = extract(points, ceilingWithOutside(), no_floor, {});
  EXPECT_TRUE(without_floor.outside_band_skipped);
  EXPECT_TRUE(without_floor.band_skipped);
  EXPECT_EQ(keepOf(without_floor), (std::vector<std::uint8_t>{1U, 1U}));
}

TEST(PointExtractionCeiling, CountsOutsideBandTowardMinPoints)
{
  Params params = ceilingWithOutside();
  params.ceiling.min_points = 2U;
  // ふつうの帯と箱では 1 点も除かないが、柵の外の帯で 2 点を除くと 1 点しか残らない
  const std::vector<Eigen::Vector3f> points{
    {3.0F, 0.0F, 0.05F}, {13.5F, 0.0F, 2.5F}, {13.5F, 0.0F, 2.8F}};
  const Result result = extract(points, params, identity(), {});
  EXPECT_TRUE(result.fallback);
  EXPECT_EQ(result.removed_outside_band, 2U);
  EXPECT_EQ(result.kept, 3U);
  EXPECT_EQ(keepOf(result), (std::vector<std::uint8_t>{1U, 1U, 1U}));
}

TEST(PointExtractionFieldObjects, KeepsFieldAndMarginBelowHeightLimit)
{
  const std::vector<Eigen::Vector3f> points{
    {0.0F, 0.0F, 0.0F},     // フィールドの床: 残す
    {11.9F, 7.4F, 0.1F},    // 柵: 残す
    {12.45F, 0.0F, 0.5F},   // 余裕 0.5m の中: 残す
    {12.55F, 0.0F, 0.5F},   // 余裕の外（観客）: 除く
    {0.0F, -8.2F, 1.0F},    // 余裕の外（観客）: 除く
    {0.0F, 0.0F, 2.2F},     // 高さの上限ちょうど: 残す
    {0.0F, 0.0F, 2.3F},     // 上限より上（屋根）: 除く
    {-20.0F, 0.0F, 10.0F},  // 外で上限より上: 範囲の外として数える
  };
  const Result result = extract(points, fieldObjects(), identity(), {});
  EXPECT_EQ(keepOf(result), (std::vector<std::uint8_t>{1U, 1U, 1U, 0U, 0U, 1U, 0U, 0U}));
  EXPECT_EQ(result.removed_outside, 3U);
  EXPECT_EQ(result.removed_above, 1U);
  EXPECT_EQ(result.removed_track, 0U);
  EXPECT_EQ(result.kept, 4U);
}

TEST(PointExtractionFieldObjects, UsesMapForAreaAndFloorForHeight)
{
  // 点群の座標系は map で (10, 0) にあり、床から 0.25m（LiDAR の高さ）にある
  Transforms transforms = identity();
  transforms.map_from_cloud = Eigen::Isometry3f(Eigen::Translation3f(10.0F, 0.0F, 0.25F));
  transforms.height_from_cloud = Eigen::Isometry3f(Eigen::Translation3f(0.0F, 0.0F, 0.25F));
  const std::vector<Eigen::Vector3f> points{
    {2.4F, 0.0F, 0.0F},   // map x 12.4: 残す
    {2.6F, 0.0F, 0.0F},   // map x 12.6: 除く
    {0.0F, 0.0F, 1.9F},   // 床から 2.15m: 残す
    {0.0F, 0.0F, 2.0F},   // 床から 2.25m: 除く
  };
  const Result result = extract(points, fieldObjects(), transforms, {});
  EXPECT_EQ(keepOf(result), (std::vector<std::uint8_t>{1U, 0U, 1U, 0U}));
}

TEST(PointExtractionFieldObjects, RemovesPointsInsideTrackCylinders)
{
  const std::vector<Cylinder> cylinders{{Eigen::Vector2f(3.0F, 0.0F), 0.45F}};
  const std::vector<Eigen::Vector3f> points{
    {3.0F, 0.0F, 1.5F},   // 人の頭: 除く
    {3.4F, 0.0F, 0.2F},   // 柱の中: 除く
    {3.5F, 0.0F, 0.2F},   // 柱の外 5cm: 残す
    {-3.0F, 0.0F, 1.0F},  // 別の場所: 残す
  };
  const Result result = extract(points, fieldObjects(), identity(), cylinders);
  EXPECT_EQ(keepOf(result), (std::vector<std::uint8_t>{0U, 0U, 1U, 1U}));
  EXPECT_EQ(result.removed_track, 2U);

  // 追跡を使わない設定では柱をかけない
  Params disabled = fieldObjects();
  disabled.field_objects.tracks.enabled = false;
  EXPECT_EQ(extract(points, disabled, identity(), cylinders).kept, 4U);
}

TEST(PointExtractionFieldObjects, SkipsTracksWithoutTransform)
{
  Transforms transforms = identity();
  transforms.tracks_from_cloud.reset();
  const std::vector<Eigen::Vector3f> points{{3.0F, 0.0F, 1.0F}, {30.0F, 0.0F, 1.0F}};
  const Result result = extract(
    points, fieldObjects(), transforms, {{Eigen::Vector2f(3.0F, 0.0F), 0.45F}});
  EXPECT_TRUE(result.tracks_skipped);
  // 範囲は変わらずかける
  EXPECT_EQ(keepOf(result), (std::vector<std::uint8_t>{1U, 0U}));
}

TEST(PointExtractionFieldObjects, PassesEveryPointWithoutMapOrFloor)
{
  const std::vector<Eigen::Vector3f> points{{30.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 5.0F}};
  for (const bool drop_map : {true, false}) {
    Transforms transforms = identity();
    if (drop_map) {
      transforms.map_from_cloud.reset();
    } else {
      transforms.height_from_cloud.reset();
    }
    const Result result = extract(points, fieldObjects(), transforms, {});
    EXPECT_TRUE(result.passed_without_transform);
    EXPECT_EQ(result.kept, 2U);
    EXPECT_EQ(keepOf(result), (std::vector<std::uint8_t>{1U, 1U}));
  }
}

TEST(PointExtractionFieldObjects, PassesEveryPointWhenTooFewRemain)
{
  Params params = fieldObjects();
  params.field_objects.min_points = 2U;
  const std::vector<Eigen::Vector3f> points{
    {0.0F, 0.0F, 0.0F}, {30.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 9.0F}};
  const Result result = extract(points, params, identity(), {});
  EXPECT_TRUE(result.fallback);
  EXPECT_EQ(result.removed_outside, 1U);
  EXPECT_EQ(result.removed_above, 1U);
  EXPECT_EQ(result.kept, 3U);
  EXPECT_EQ(keepOf(result), (std::vector<std::uint8_t>{1U, 1U, 1U}));

  // 何も除かなければ、点が少なくても fallback にしない
  const Result nothing_removed = extract({{0.0F, 0.0F, 0.0F}}, params, identity(), {});
  EXPECT_FALSE(nothing_removed.fallback);
}

TEST(PointExtractionTracks, PredictsToCloudTimeWithRadiusByClass)
{
  Params params = fieldObjects();
  const auto & tracks = params.field_objects.tracks;
  const std::vector<Track> input{
    {Eigen::Vector2f(1.0F, 2.0F), Eigen::Vector2f(2.0F, 0.0F), TrackClass::kHuman},
    {Eigen::Vector2f(-1.0F, 0.0F), Eigen::Vector2f(0.0F, -1.0F), TrackClass::kRobot},
    {Eigen::Vector2f(0.0F, 0.0F), Eigen::Vector2f::Zero(), TrackClass::kUnknown},
  };
  const auto cylinders = predictCylinders(input, 0.1, tracks);
  ASSERT_EQ(cylinders.size(), 3U);
  EXPECT_TRUE(cylinders[0].center.isApprox(Eigen::Vector2f(1.2F, 2.0F)));
  EXPECT_FLOAT_EQ(cylinders[0].radius_m, tracks.human_radius_m + tracks.margin_m);
  EXPECT_TRUE(cylinders[1].center.isApprox(Eigen::Vector2f(-1.0F, -0.1F)));
  EXPECT_FLOAT_EQ(cylinders[1].radius_m, tracks.robot_radius_m + tracks.margin_m);
  EXPECT_FLOAT_EQ(cylinders[2].radius_m, tracks.unknown_radius_m + tracks.margin_m);
  // 追跡の方が新しいときは戻す
  const auto back = predictCylinders(input, -0.05, tracks);
  EXPECT_TRUE(back[0].center.isApprox(Eigen::Vector2f(0.9F, 2.0F)));
  // 既定値: 人 0.25 + 0.2、ロボット 0.57 + 0.2
  EXPECT_NEAR(cylinders[0].radius_m, 0.45F, 1e-6F);
  EXPECT_NEAR(cylinders[1].radius_m, 0.77F, 1e-6F);
}

TEST(PointExtractionParams, KeepAreaAddsMargin)
{
  Params params = fieldObjects();
  params.field_objects.field_margin_m = 0.5F;
  const Area area = keepArea(params.field_objects);
  EXPECT_FLOAT_EQ(area.x_min, -12.5F);
  EXPECT_FLOAT_EQ(area.x_max, 12.5F);
  EXPECT_FLOAT_EQ(area.y_min, -8.0F);
  EXPECT_FLOAT_EQ(area.y_max, 8.0F);
}

TEST(PointExtractionParams, RejectsInvalidCombinations)
{
  EXPECT_NO_THROW(validate(fieldObjects()));
  EXPECT_NO_THROW(validate(ceiling()));

  Params band = ceiling();
  band.ceiling.band_min_m = 2.5F;
  EXPECT_THROW(validate(band), std::invalid_argument);

  Params field = fieldObjects();
  field.field_objects.field = Area{12.0F, -12.0F, -7.5F, 7.5F};
  EXPECT_THROW(validate(field), std::invalid_argument);

  Params margin = fieldObjects();
  margin.field_objects.field_margin_m = -0.1F;
  EXPECT_THROW(validate(margin), std::invalid_argument);

  Params height = fieldObjects();
  height.field_objects.max_height_m = 0.0F;
  EXPECT_THROW(validate(height), std::invalid_argument);

  Params radius = fieldObjects();
  radius.field_objects.tracks.human_radius_m = std::nanf("");
  EXPECT_THROW(validate(radius), std::invalid_argument);

  Params age = fieldObjects();
  age.field_objects.tracks.max_age_s = 0.0;
  EXPECT_THROW(validate(age), std::invalid_argument);
}

TEST(PointExtractionParams, OutsideBoundaryAddsMargin)
{
  const Area boundary = outsideBoundary(ceilingWithOutside().ceiling.outside);
  EXPECT_FLOAT_EQ(boundary.x_min, -13.0F);
  EXPECT_FLOAT_EQ(boundary.x_max, 13.0F);
  EXPECT_FLOAT_EQ(boundary.y_min, -8.5F);
  EXPECT_FLOAT_EQ(boundary.y_max, 8.5F);
}

TEST(PointExtractionParams, RejectsInvalidOutsideBand)
{
  EXPECT_NO_THROW(validate(ceilingWithOutside()));

  Params lower = ceilingWithOutside();
  // ふつうの帯の上限 2.2m より低い
  lower.ceiling.outside.band_max_m = 2.0F;
  EXPECT_THROW(validate(lower), std::invalid_argument);
  // 使わないときは確かめない
  Params unused = lower;
  unused.ceiling.outside.enabled = false;
  EXPECT_NO_THROW(validate(unused));

  Params not_finite = ceilingWithOutside();
  not_finite.ceiling.outside.band_max_m = std::nanf("");
  EXPECT_THROW(validate(not_finite), std::invalid_argument);

  Params margin = ceilingWithOutside();
  margin.ceiling.outside.field_margin_m = -0.5F;
  EXPECT_THROW(validate(margin), std::invalid_argument);

  Params field = ceilingWithOutside();
  field.ceiling.outside.field = Area{-12.0F, 12.0F, 7.5F, -7.5F};
  EXPECT_THROW(validate(field), std::invalid_argument);
}
}  // namespace
