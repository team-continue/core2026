// フィールドの占有格子と、障害物までの距離の場。ROS に依存しない。

#ifndef CORE_PLANNING__GRID_MAP_HPP_
#define CORE_PLANNING__GRID_MAP_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "core_nav_common/field_bounds.hpp"

namespace core_planning
{
using core_nav_common::FieldBounds;

struct GridCell
{
  int x{0};
  int y{0};

  bool operator==(const GridCell & other) const {return x == other.x && y == other.y;}
};

struct GridInfo
{
  double origin_x{0.0};
  double origin_y{0.0};
  double resolution{0.1};
  int width{0};
  int height{0};

  bool contains(const GridCell & cell) const
  {
    return cell.x >= 0 && cell.y >= 0 && cell.x < width && cell.y < height;
  }

  int index(const GridCell & cell) const {return cell.y * width + cell.x;}

  GridCell cellAt(double x, double y) const;
  double centerX(const int cell_x) const {return origin_x + (cell_x + 0.5) * resolution;}
  double centerY(const int cell_y) const {return origin_y + (cell_y + 0.5) * resolution;}
};

class OccupancyGrid
{
public:
  OccupancyGrid() = default;
  explicit OccupancyGrid(const GridInfo & info);

  const GridInfo & info() const {return info_;}

  bool isOccupied(const GridCell & cell) const
  {
    return !info_.contains(cell) || cells_[static_cast<std::size_t>(info_.index(cell))] != 0U;
  }

  bool isFree(const GridCell & cell) const {return !isOccupied(cell);}

  void setOccupied(const GridCell & cell)
  {
    if (info_.contains(cell)) {
      cells_[static_cast<std::size_t>(info_.index(cell))] = 1U;
    }
  }

  // (x, y) を中心とする半径 radius の円を占有にする
  void fillCircle(double x, double y, double radius);

private:
  GridInfo info_;
  std::vector<std::uint8_t> cells_;
};

// 最も近い占有セルまでの距離 [m]
class DistanceField
{
public:
  DistanceField() = default;
  DistanceField(const GridInfo & info, std::vector<float> distances);

  const GridInfo & info() const {return info_;}

  // 格子の外は距離 0
  float at(double x, double y) const;

  float at(const GridCell & cell) const
  {
    return info_.contains(cell) ? distances_[static_cast<std::size_t>(info_.index(cell))] : 0.0F;
  }

private:
  GridInfo info_;
  std::vector<float> distances_;
};

// フィールドを覆う格子を作り、外周 1 セルを柵として占有にする
OccupancyGrid makeFieldGrid(const FieldBounds & field, double resolution);

// PCD のうち、フィールド内の高さが範囲内の点を占有にする。読めなければ std::runtime_error
void addStaticObstacles(
  OccupancyGrid & grid, const std::string & pcd_path, const FieldBounds & field,
  double field_inset_m, double min_height_m, double max_height_m);

DistanceField computeDistanceField(const OccupancyGrid & grid);

// map 座標の多角形（頂点の並び。閉じた形として扱う）
using Polygon = std::vector<std::array<double, 2>>;

// 静止物の地図の作り方。フィールドごとの設定（field_file）で変える
struct StaticMapOptions
{
  // 地図の PCD のうち、静止物として使う点の高さ [m]。CoRE-1 のフィールドは低い物（バンカー 0.15m、
  // 高台 0.2m）が多いので、下限を下げる
  double min_height_m{0.15};
  double max_height_m{2.0};
  // 床に物はないが入らない範囲（CoRE-1 の進入禁止ゾーンなど）。占有にする
  std::vector<Polygon> keep_out;
};

// 多角形の中（セルの中心が内側）を占有にする
void addPolygons(OccupancyGrid & grid, const std::vector<Polygon> & polygons);

// keep_out.x・keep_out.y（全多角形の頂点を続けて並べたもの）と keep_out.counts（多角形ごとの頂点の数）から
// 多角形を作る。数が合わなければ std::invalid_argument
std::vector<Polygon> polygonsFromArrays(
  const std::vector<double> & xs, const std::vector<double> & ys, const std::vector<std::int64_t> & counts);

// 距離の場から、半径 radius 未満を占有にした格子を作る
OccupancyGrid inflate(const DistanceField & field, double radius);

// 大域計画の格子で静止物を膨らませる半径。機体の外接円の半径に余裕を足した値。狭い通路を通すとき
// （narrow_enabled）は、機体の半幅に通路の片側の余裕のしきい値を足した値のほうが小さければ、そちらにする
// （機体は進む向きへ回るので、通路の中では幅の分だけ要る）
double planningInflationRadius(
  double half_length, double half_width, double margin, bool narrow_enabled,
  double min_side_clearance);

// 局所計画（mppi_lite）と安全停止（safety_gate）が使う、静止物の距離の場を作る。フィールドの外周 1 セルを
// 柵とし、map_path が空でなければ、地図の PCD のうちフィールドの内側 0.1m より内で、高さ 0.15〜2.0m の
// 点を占有にする。地図を読めなければ std::runtime_error
DistanceField buildDistanceField(
  const FieldBounds & field, double resolution, const std::string & map_path,
  const StaticMapOptions & options = StaticMapOptions{});
}  // namespace core_planning

#endif  // CORE_PLANNING__GRID_MAP_HPP_
