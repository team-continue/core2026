// 静止物の地図の作り方（StaticMapOptions）のパラメータ。大域計画・局所計画・安全停止が同じ値を読む。
//
// - static_map.min_height_m / max_height_m: 地図の PCD のうち静止物として使う点の高さ [m]
// - keep_out.x / keep_out.y / keep_out.counts: 入らない範囲の多角形（map 座標）。x・y は全多角形の頂点を
//   続けて並べ、counts に多角形ごとの頂点の数を書く。どれも空なら入らない範囲はない
// フィールドごとの設定（launch の field_file）で変える。起動の後は変えない（地図を作り直さない）

#ifndef CORE_PLANNING__STATIC_MAP_PARAMS_HPP_
#define CORE_PLANNING__STATIC_MAP_PARAMS_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "core_nav_common/parameters.hpp"
#include "core_planning/grid_map.hpp"

namespace core_planning
{
inline void declareStaticMapParams(rclcpp::Node & node)
{
  core_nav_common::declareDouble(
    node, "static_map.min_height_m", 0.15, 0.0, 5.0, "地図の PCD のうち静止物として使う点の高さの下限", true);
  core_nav_common::declareDouble(
    node, "static_map.max_height_m", 2.0, 0.0, 10.0, "地図の PCD のうち静止物として使う点の高さの上限", true);
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  descriptor.description = "入らない範囲の多角形の頂点の x（map 座標、全多角形を続けて並べる）";
  node.declare_parameter<std::vector<double>>("keep_out.x", std::vector<double>{}, descriptor);
  descriptor.description = "入らない範囲の多角形の頂点の y（map 座標、全多角形を続けて並べる）";
  node.declare_parameter<std::vector<double>>("keep_out.y", std::vector<double>{}, descriptor);
  descriptor.description = "入らない範囲の多角形ごとの頂点の数";
  node.declare_parameter<std::vector<std::int64_t>>(
    "keep_out.counts", std::vector<std::int64_t>{}, descriptor);
}

// 宣言したパラメータから読む。値が合わなければ std::invalid_argument
inline StaticMapOptions staticMapOptions(const rclcpp::Node & node)
{
  StaticMapOptions options;
  options.min_height_m = node.get_parameter("static_map.min_height_m").as_double();
  options.max_height_m = node.get_parameter("static_map.max_height_m").as_double();
  if (options.min_height_m >= options.max_height_m) {
    throw std::invalid_argument("static_map.min_height_m は max_height_m より小さくしてください");
  }
  options.keep_out = polygonsFromArrays(
    node.get_parameter("keep_out.x").as_double_array(),
    node.get_parameter("keep_out.y").as_double_array(),
    node.get_parameter("keep_out.counts").as_integer_array());
  return options;
}
}  // namespace core_planning

#endif  // CORE_PLANNING__STATIC_MAP_PARAMS_HPP_
