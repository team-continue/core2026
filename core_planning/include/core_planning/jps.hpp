// Jump Point Search（角を横切らない 8 方向移動）と経路の単純化。ROS に依存しない。

#ifndef CORE_PLANNING__JPS_HPP_
#define CORE_PLANNING__JPS_HPP_

#include <optional>
#include <vector>

#include "core_planning/grid_map.hpp"

namespace core_planning
{
// 始点から終点までの最短経路を、ジャンプポイントの列で返す。経路がなければ nullopt
std::optional<std::vector<GridCell>> findPath(
  const OccupancyGrid & grid, const GridCell & start, const GridCell & goal);

// 2 点間の見通し（占有セルを通らない）を調べる
bool hasLineOfSight(const OccupancyGrid & grid, const GridCell & from, const GridCell & to);

// 見通しのある点を飛ばして経路を短くする
std::vector<GridCell> simplifyPath(const OccupancyGrid & grid, const std::vector<GridCell> & path);

// ジャンプポイントの列を、間のセルも含めた列にする。区間は 8 方向の直線（JPS の出力）でなければならず、
// そうでなければ std::invalid_argument を投げる
std::vector<GridCell> expandPath(const std::vector<GridCell> & path);

// 見通しのある最も遠いセルへ飛んで経路を短くする。simplifyPath と違い、ジャンプポイントの間のセルにも
// 飛ぶので、8 方向の格子の経路が角で折れる所を斜めの直線にできる（直線で行ける所に折れ線を残さない）
std::vector<GridCell> shortcutPath(const OccupancyGrid & grid, const std::vector<GridCell> & path);

// 占有されていれば、半径 max_radius_cells 以内で最も近い空きセルを返す
std::optional<GridCell> nearestFreeCell(
  const OccupancyGrid & grid, const GridCell & cell, int max_radius_cells);

// 経路の長さ（セル単位）
double pathLength(const std::vector<GridCell> & path);
}  // namespace core_planning

#endif  // CORE_PLANNING__JPS_HPP_
