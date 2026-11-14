// scan_filter と livox_filter で共通の、点群を減らす処理とパラメータ。
//
// 処理は every_n_frames、ボクセル、ランダムサンプルの順。実行中に値を変えられる。

#ifndef CORE_LOCALIZATION__FILTER_PARAMS_HPP_
#define CORE_LOCALIZATION__FILTER_PARAMS_HPP_

#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "core_nav_common/parameters.hpp"
#include "core_nav_common/point_filters.hpp"

namespace core_localization
{
struct FilterParams
{
  std::int64_t every_n_frames{1};
  bool voxel_enabled{false};
  double voxel_leaf_size_m{0.5};
  bool random_enabled{false};
  std::int64_t random_target_points{3000};
};

inline void declareFilterParams(rclcpp::Node & node)
{
  using core_nav_common::declareBool;
  using core_nav_common::declareDouble;
  using core_nav_common::declareInt;
  declareInt(node, "every_n_frames", 1, 1, 100, "N フレームに 1 回だけ処理する");
  declareBool(node, "voxel.enabled", false, "ボクセルフィルタを使う");
  declareDouble(node, "voxel.leaf_size_m", 0.5, 0.05, 5.0, "ボクセルの大きさ");
  declareBool(node, "random.enabled", false, "ランダムサンプルを使う");
  declareInt(node, "random.target_points", 3000, 100, 200000, "ランダムサンプル後の点数");
}

inline FilterParams readFilterParams(const rclcpp::Node & node)
{
  FilterParams params;
  params.every_n_frames = node.get_parameter("every_n_frames").as_int();
  params.voxel_enabled = node.get_parameter("voxel.enabled").as_bool();
  params.voxel_leaf_size_m = node.get_parameter("voxel.leaf_size_m").as_double();
  params.random_enabled = node.get_parameter("random.enabled").as_bool();
  params.random_target_points = node.get_parameter("random.target_points").as_int();
  return params;
}

template<typename Point, typename PositionOf>
std::vector<Point> applyFilters(
  std::vector<Point> points, const FilterParams & params, std::mt19937 & random,
  PositionOf position_of)
{
  if (params.voxel_enabled) {
    points = core_nav_common::voxelFilter(points, params.voxel_leaf_size_m, position_of);
  }
  if (params.random_enabled) {
    points = core_nav_common::randomSample(
      points, static_cast<std::size_t>(params.random_target_points), random);
  }
  return points;
}
}  // namespace core_localization

#endif  // CORE_LOCALIZATION__FILTER_PARAMS_HPP_
