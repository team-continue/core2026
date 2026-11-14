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

// 占有されていれば、半径 max_radius_cells 以内で最も近い空きセルを返す
std::optional<GridCell> nearestFreeCell(
  const OccupancyGrid & grid, const GridCell & cell, int max_radius_cells);

// 経路の長さ（セル単位）
double pathLength(const std::vector<GridCell> & path);
}  // namespace core_planning

#endif  // CORE_PLANNING__JPS_HPP_
