#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "core_localization/crowd_filter.hpp"

namespace
{
using core_localization::crowd::AxisBox;
using core_localization::crowd::classify;
using core_localization::crowd::contains;
using core_localization::crowd::makeBox;
using core_localization::crowd::makeRegions;
using core_localization::crowd::Params;
using core_localization::crowd::Region;
using core_localization::crowd::Result;
using core_localization::crowd::selectKept;
using core_localization::crowd::Transforms;
using core_localization::crowd::validate;

// LiDAR は床から 0.85m。点群の座標系は LiDAR の位置にあり、機体と向きが同じとする
constexpr float kLidarHeightM = 0.85F;

Eigen::Isometry3f lidarOnBase()
{
  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
  transform.translation() = Eigen::Vector3f(0.0F, 0.0F, kLidarHeightM);
  return transform;
}

// 機体が map の (x, y) にあり、yaw だけ向いているときの map←点群の変換
Eigen::Isometry3f lidarOnMap(const float x, const float y, const float yaw)
{
  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
  transform.translation() = Eigen::Vector3f(x, y, kLidarHeightM);
  transform.linear() = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  return transform;
}

// 点群の座標系での点。z は床からの高さで書き、LiDAR の高さを引く
Eigen::Vector3f atHeight(const float x, const float y, const float height)
{
  return Eigen::Vector3f(x, y, height - kLidarHeightM);
}

Params withoutFallback()
{
  Params params;
  params.min_points = 0U;
  return params;
}

std::vector<std::uint8_t> keepOf(const Result & result)
{
  return result.keep;
}

TEST(CrowdFilter, DisabledKeepsEverything)
{
  const std::vector<Eigen::Vector3f> points{atHeight(0.2F, 0.0F, 0.5F), atHeight(5.0F, 0.0F, 1.0F)};
  Transforms transforms;
  transforms.base_from_cloud = lidarOnBase();
  transforms.height_from_cloud = lidarOnBase();
  transforms.map_from_cloud = lidarOnMap(0.0F, 0.0F, 0.0F);
  const Result result = classify(points, Params{}, transforms);
  EXPECT_EQ(keepOf(result), (std::vector<std::uint8_t>{1U, 1U}));
  EXPECT_EQ(result.kept, 2U);
  EXPECT_EQ(result.removed_self + result.removed_height + result.removed_region, 0U);
  EXPECT_FALSE(result.fallback);
  EXPECT_FALSE(result.self_skipped || result.height_skipped || result.region_skipped);
}

TEST(CrowdFilter, SelfBoxRemovesOnlyBodyPoints)
{
  Params params = withoutFallback();
  params.self_enabled = true;
  Transforms transforms;
  transforms.base_from_cloud = lidarOnBase();
  const std::vector<Eigen::Vector3f> points{
    atHeight(0.3F, 0.0F, 0.55F),    // 機体の側面
    atHeight(-0.45F, 0.45F, 0.8F),  // 天板の角（余裕の内側）
    atHeight(0.6F, 0.0F, 0.55F),    // 余裕の外
    atHeight(0.0F, 0.0F, 1.0F),     // 機体の上面より上
  };
  const Result result = classify(points, params, transforms);
  EXPECT_EQ(keepOf(result), (std::vector<std::uint8_t>{0U, 0U, 1U, 1U}));
  EXPECT_EQ(result.removed_self, 2U);
  EXPECT_EQ(result.kept, 2U);
}

TEST(CrowdFilter, HeightBandRemovesPeopleAndKeepsFloorAndCeiling)
{
  Params params = withoutFallback();
  params.height_band_enabled = true;
  Transforms transforms;
  transforms.height_from_cloud = lidarOnBase();
  const std::vector<Eigen::Vector3f> points{
    atHeight(8.0F, 0.0F, 0.0F),     // 床
    atHeight(8.0F, 0.0F, 0.09F),    // 床の凹凸
    atHeight(8.0F, 0.0F, 0.11F),    // 下限の少し上
    atHeight(3.0F, 1.0F, 1.2F),     // 人の胴
    atHeight(3.0F, 1.0F, 2.19F),    // 上限の少し下
    atHeight(3.0F, 1.0F, 2.3F),     // 頭より上
    atHeight(20.0F, 5.0F, 15.0F),   // 天井
  };
  const Result result = classify(points, params, transforms);
  EXPECT_EQ(keepOf(result), (std::vector<std::uint8_t>{1U, 1U, 0U, 0U, 0U, 1U, 1U}));
  EXPECT_EQ(result.removed_height, 3U);
}

TEST(CrowdFilter, HeightBandIncludesBoundary)
{
  // 変換で丸めが入らないよう、点群の座標系を床の座標系と同じにする
  Params params = withoutFallback();
  params.height_band_enabled = true;
  Transforms transforms;
  transforms.height_from_cloud = Eigen::Isometry3f::Identity();
  const Result result = classify(
    {Eigen::Vector3f(5.0F, 0.0F, params.height_min_m), Eigen::Vector3f(5.0F, 0.0F, params.height_max_m)},
    params, transforms);
  EXPECT_EQ(keepOf(result), (std::vector<std::uint8_t>{0U, 0U}));
}

TEST(CrowdFilter, HeightUsesFloorFrameWhenRobotTilts)
{
  // 機体が前に 5 度傾くと、20m 先の床は点群の座標系で約 1.7m 下がって見える。
  // 床の座標系で高さを求めれば、床は帯に入らない
  Params params = withoutFallback();
  params.height_band_enabled = true;
  const float pitch = 5.0F * static_cast<float>(M_PI) / 180.0F;
  Eigen::Isometry3f floor_from_cloud = Eigen::Isometry3f::Identity();
  floor_from_cloud.translation() = Eigen::Vector3f(0.0F, 0.0F, kLidarHeightM);
  floor_from_cloud.linear() = Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()).toRotationMatrix();
  // 床の点と、1.0m の高さの点を床の座標系で作り、点群の座標系に直す
  const Eigen::Vector3f floor_point = floor_from_cloud.inverse() * Eigen::Vector3f(20.0F, 0.0F, 0.0F);
  const Eigen::Vector3f person_point = floor_from_cloud.inverse() * Eigen::Vector3f(20.0F, 0.0F, 1.0F);
  Transforms transforms;
  transforms.height_from_cloud = floor_from_cloud;
  const Result result = classify({floor_point, person_point}, params, transforms);
  EXPECT_EQ(keepOf(result), (std::vector<std::uint8_t>{1U, 0U}));
}

TEST(CrowdFilter, RegionUsesMapFrame)
{
  // 観客席の上（map の y: 14〜19、z: 5.1〜7.5）にいる人を除く。
  // 機体は map の (0, 0) で左（+y）を向くので、点群の座標系の +x が map の +y になる
  Params params = withoutFallback();
  params.region_enabled = true;
  params.regions = {Region{"stand", AxisBox{Eigen::Vector3f(-20.0F, 14.0F, 5.1F),
      Eigen::Vector3f(10.0F, 19.0F, 7.5F)}}};
  Transforms transforms;
  transforms.map_from_cloud = lidarOnMap(0.0F, 0.0F, static_cast<float>(M_PI) / 2.0F);
  const std::vector<Eigen::Vector3f> points{
    atHeight(16.0F, 0.0F, 6.0F),   // map (0, 16, 6): 観客席の上の人
    atHeight(16.0F, 0.0F, 5.0F),   // map (0, 16, 5): 観客席の天板（残す）
    atHeight(16.0F, -15.0F, 6.0F),  // map (15, 16, 6): 観客席の外
    atHeight(0.0F, 16.0F, 6.0F),   // map (-16, 0, 6): 向きを間違えると入る位置
  };
  const Result result = classify(points, params, transforms);
  EXPECT_EQ(keepOf(result), (std::vector<std::uint8_t>{0U, 1U, 1U, 1U}));
  EXPECT_EQ(result.removed_region, 1U);
}

TEST(CrowdFilter, CountsEachPointOnceInFilterOrder)
{
  Params params = withoutFallback();
  params.self_enabled = true;
  params.height_band_enabled = true;
  params.region_enabled = true;
  params.regions = {Region{"all", AxisBox{Eigen::Vector3f::Constant(-100.0F),
      Eigen::Vector3f::Constant(100.0F)}}};
  Transforms transforms;
  transforms.base_from_cloud = lidarOnBase();
  transforms.height_from_cloud = lidarOnBase();
  transforms.map_from_cloud = lidarOnMap(0.0F, 0.0F, 0.0F);
  // 機体の点は高さの帯にも領域にも入るが、自機の除去として数える
  const std::vector<Eigen::Vector3f> points{
    atHeight(0.3F, 0.0F, 0.5F), atHeight(5.0F, 0.0F, 1.0F), atHeight(5.0F, 0.0F, 5.0F)};
  const Result result = classify(points, params, transforms);
  EXPECT_EQ(result.removed_self, 1U);
  EXPECT_EQ(result.removed_height, 1U);
  EXPECT_EQ(result.removed_region, 1U);
  EXPECT_EQ(result.kept, 0U);
}

TEST(CrowdFilter, MissingTransformSkipsOnlyThatFilter)
{
  Params params = withoutFallback();
  params.self_enabled = true;
  params.height_band_enabled = true;
  params.region_enabled = true;
  params.regions = {Region{"all", AxisBox{Eigen::Vector3f::Constant(-100.0F),
      Eigen::Vector3f::Constant(100.0F)}}};
  Transforms transforms;
  transforms.base_from_cloud = lidarOnBase();
  transforms.height_from_cloud = lidarOnBase();
  const std::vector<Eigen::Vector3f> points{
    atHeight(0.3F, 0.0F, 0.5F), atHeight(5.0F, 0.0F, 1.0F), atHeight(5.0F, 0.0F, 5.0F)};
  const Result result = classify(points, params, transforms);
  EXPECT_TRUE(result.region_skipped);
  EXPECT_FALSE(result.self_skipped);
  EXPECT_FALSE(result.height_skipped);
  EXPECT_EQ(keepOf(result), (std::vector<std::uint8_t>{0U, 0U, 1U}));

  Transforms only_map;
  only_map.map_from_cloud = lidarOnMap(0.0F, 0.0F, 0.0F);
  const Result without_base = classify(points, params, only_map);
  EXPECT_TRUE(without_base.self_skipped);
  EXPECT_TRUE(without_base.height_skipped);
  EXPECT_FALSE(without_base.region_skipped);
  EXPECT_EQ(without_base.removed_region, 3U);
}

TEST(CrowdFilter, FallsBackToSelfOnlyWhenTooFewPointsRemain)
{
  Params params;
  params.self_enabled = true;
  params.height_band_enabled = true;
  params.min_points = 3U;
  Transforms transforms;
  transforms.base_from_cloud = lidarOnBase();
  transforms.height_from_cloud = lidarOnBase();
  const std::vector<Eigen::Vector3f> points{
    atHeight(0.3F, 0.0F, 0.5F),   // 機体
    atHeight(5.0F, 0.0F, 1.0F),   // 人
    atHeight(6.0F, 0.0F, 1.5F),   // 人
    atHeight(9.0F, 0.0F, 0.0F),   // 床
    atHeight(9.0F, 0.0F, 9.0F),   // 壁の上
  };
  // 帯をかけると 2 点しか残らないので、自機の除去だけをかけて 4 点を残す
  const Result result = classify(points, params, transforms);
  EXPECT_TRUE(result.fallback);
  EXPECT_EQ(keepOf(result), (std::vector<std::uint8_t>{0U, 1U, 1U, 1U, 1U}));
  EXPECT_EQ(result.kept, 4U);
  EXPECT_EQ(result.removed_height, 2U);

  params.min_points = 2U;
  const Result enough = classify(points, params, transforms);
  EXPECT_FALSE(enough.fallback);
  EXPECT_EQ(keepOf(enough), (std::vector<std::uint8_t>{0U, 0U, 0U, 1U, 1U}));
  EXPECT_EQ(enough.kept, 2U);
}

TEST(CrowdFilter, NoFallbackWhenCrowdFiltersRemovedNothing)
{
  // 入力がもともと少ないだけなら、外しても変わらないので fallback にしない
  Params params;
  params.height_band_enabled = true;
  params.min_points = 100U;
  Transforms transforms;
  transforms.height_from_cloud = lidarOnBase();
  const Result result = classify({atHeight(9.0F, 0.0F, 0.0F)}, params, transforms);
  EXPECT_FALSE(result.fallback);
  EXPECT_EQ(result.kept, 1U);
}

TEST(CrowdFilter, EmptyInput)
{
  Params params;
  params.self_enabled = true;
  params.height_band_enabled = true;
  Transforms transforms;
  transforms.base_from_cloud = lidarOnBase();
  transforms.height_from_cloud = lidarOnBase();
  const Result result = classify({}, params, transforms);
  EXPECT_TRUE(result.keep.empty());
  EXPECT_EQ(result.kept, 0U);
  EXPECT_FALSE(result.fallback);
}

TEST(CrowdFilter, ContainsIncludesBoundary)
{
  const AxisBox box{Eigen::Vector3f(0.0F, 0.0F, 0.0F), Eigen::Vector3f(1.0F, 2.0F, 3.0F)};
  EXPECT_TRUE(contains(box, Eigen::Vector3f(0.0F, 2.0F, 3.0F)));
  EXPECT_FALSE(contains(box, Eigen::Vector3f(-0.01F, 1.0F, 1.0F)));
  EXPECT_FALSE(contains(box, Eigen::Vector3f(0.5F, 1.0F, 3.01F)));
}

TEST(CrowdFilter, MakeBoxChecksValues)
{
  const AxisBox box = makeBox({-0.5, -0.5, -0.1}, {0.5, 0.5, 0.85}, "自機の箱");
  EXPECT_FLOAT_EQ(box.min.z(), -0.1F);
  EXPECT_FLOAT_EQ(box.max.x(), 0.5F);
  EXPECT_THROW(makeBox({0.0, 0.0}, {1.0, 1.0, 1.0}, "自機の箱"), std::invalid_argument);
  EXPECT_THROW(makeBox({1.0, 0.0, 0.0}, {0.0, 1.0, 1.0}, "自機の箱"), std::invalid_argument);
  EXPECT_THROW(makeBox({NAN, 0.0, 0.0}, {1.0, 1.0, 1.0}, "自機の箱"), std::invalid_argument);
}

TEST(CrowdFilter, MakeRegionsReadsSixValuesPerRegion)
{
  const auto regions = makeRegions(
    {"stand_north", "stand_east"},
    {-20.0, 10.0, 13.8, 18.9, 5.1, 7.5, 20.0, 25.0, -5.0, 5.0, 0.1, 6.0});
  ASSERT_EQ(regions.size(), 2U);
  EXPECT_EQ(regions[0].name, "stand_north");
  EXPECT_FLOAT_EQ(regions[0].box.min.y(), 13.8F);
  EXPECT_FLOAT_EQ(regions[0].box.max.z(), 7.5F);
  EXPECT_FLOAT_EQ(regions[1].box.min.x(), 20.0F);
  EXPECT_TRUE(makeRegions({}, {}).empty());
}

TEST(CrowdFilter, MakeRegionsRejectsBadValues)
{
  // 6 個で割り切れない
  EXPECT_THROW(makeRegions({"a"}, {0.0, 1.0, 0.0, 1.0, 0.0}), std::invalid_argument);
  // 名前の数が違う
  EXPECT_THROW(makeRegions({"a", "b"}, {0.0, 1.0, 0.0, 1.0, 0.0, 1.0}), std::invalid_argument);
  // x の下限が上限を超える
  EXPECT_THROW(makeRegions({"a"}, {1.0, 0.0, 0.0, 1.0, 0.0, 1.0}), std::invalid_argument);
}

TEST(CrowdFilter, ValidateChecksHeightBand)
{
  Params params;
  EXPECT_NO_THROW(validate(params));
  params.height_min_m = 2.5F;
  EXPECT_THROW(validate(params), std::invalid_argument);
  params.height_min_m = NAN;
  EXPECT_THROW(validate(params), std::invalid_argument);
}

TEST(CrowdFilter, SelectKeptKeepsOrderAndChecksSize)
{
  const std::vector<int> values{10, 20, 30, 40};
  EXPECT_EQ(selectKept(values, {1U, 0U, 1U, 1U}), (std::vector<int>{10, 30, 40}));
  EXPECT_THROW(selectKept(values, {1U, 0U}), std::invalid_argument);
}
}  // namespace
