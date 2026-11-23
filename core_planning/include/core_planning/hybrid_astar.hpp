// 曲率に上限のある経路の探索（Hybrid A*）。ROS に依存しない。
//
// 状態は（位置、進む向き）。全方位に動ける機体なので、向きは機体の向きではなく経路の進む向き。
// 1 歩は長さ step_m の円弧で、曲率は 0、±1/(2 min_radius_m)、±1/min_radius_m の 5 通り。
// 費用は「長さ + curvature_weight × |曲率| × 長さ + curvature_change_weight × |曲率の変化|」。
// 見積もりは、ゴールから膨らませた格子の上の 8 方向の最短距離（Dijkstra）で、長さの下限なので過大にならない。
// 状態は位置 bin_m・向き 360/heading_bins 度の箱で 1 つにまとめる。
// ゴールが見通せて、ゴールへの向きと進む向きの差が shot_angle_deg 以下なら、直線でつないで終える。
// 始めの向きは決めない（どの向きからでも始められる）。

#ifndef CORE_PLANNING__HYBRID_ASTAR_HPP_
#define CORE_PLANNING__HYBRID_ASTAR_HPP_

#include <optional>
#include <utility>
#include <vector>

#include "core_planning/grid_map.hpp"

namespace core_planning
{
struct HybridAStarParams
{
  // 1 歩の円弧の長さ [m]
  double step_m{0.3};
  // 最小の曲がりの半径 [m]（曲率の上限 1/min_radius_m）
  double min_radius_m{1.0};
  // 状態をまとめる箱の大きさ [m] と、向きの分け方
  double bin_m{0.2};
  int heading_bins{72};
  // 費用の重み
  double curvature_weight{0.5};
  double curvature_change_weight{0.5};
  // 直線でつないで終える向きの差 [deg]
  double shot_angle_deg{10.0};
  // 調べる状態の数の上限。超えたら見つからないとする
  int max_expansions{100000};
};

// 値が範囲外なら std::invalid_argument を投げる
void validateHybridAStarParams(const HybridAStarParams & params);

// inflated（膨らませた格子）の空きセルだけを通る、start から goal までの点の列（0.05m ほどの間隔）。
// start と goal のセルが占有なら、または見つからなければ nullopt
std::optional<std::vector<std::pair<double, double>>> findPathHybridAStar(
  const OccupancyGrid & inflated, const std::pair<double, double> & start,
  const std::pair<double, double> & goal, const HybridAStarParams & params);
}  // namespace core_planning

#endif  // CORE_PLANNING__HYBRID_ASTAR_HPP_
