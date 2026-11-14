#include "core_planning/grid_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace core_planning
{
namespace
{
// 地図のうち、固定物として扱う範囲。柵は別に格子の外周として持つ
constexpr double kStaticFieldInsetM = 0.1;
constexpr double kStaticMinHeightM = 0.15;
constexpr double kStaticMaxHeightM = 2.0;

// 1 次元の距離変換（Felzenszwalb）。input は各セルのコスト、output は変換後
void distanceTransform1d(
  const std::vector<float> & input, std::vector<float> & output, std::vector<int> & parabola,
  std::vector<float> & boundary)
{
  const int n = static_cast<int>(input.size());
  int k = 0;
  parabola[0] = 0;
  boundary[0] = -std::numeric_limits<float>::infinity();
  boundary[1] = std::numeric_limits<float>::infinity();
  for (int q = 1; q < n; ++q) {
    float s = 0.0F;
    while (true) {
      const int v = parabola[k];
      s = ((input[q] + static_cast<float>(q * q)) - (input[v] + static_cast<float>(v * v))) /
        (2.0F * static_cast<float>(q - v));
      if (s > boundary[k] || k == 0) {
        break;
      }
      --k;
    }
    ++k;
    parabola[k] = q;
    boundary[k] = s;
    boundary[k + 1] = std::numeric_limits<float>::infinity();
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (boundary[k + 1] < static_cast<float>(q)) {
      ++k;
    }
    const int v = parabola[k];
    output[q] = static_cast<float>((q - v) * (q - v)) + input[v];
  }
}
}  // namespace

GridCell GridInfo::cellAt(const double x, const double y) const
{
  return GridCell{
    static_cast<int>(std::floor((x - origin_x) / resolution)),
    static_cast<int>(std::floor((y - origin_y) / resolution))};
}

OccupancyGrid::OccupancyGrid(const GridInfo & info)
: info_(info), cells_(static_cast<std::size_t>(info.width * info.height), 0U)
{
}

void OccupancyGrid::fillCircle(const double x, const double y, const double radius)
{
  const int cells = static_cast<int>(std::ceil(radius / info_.resolution));
  const GridCell center = info_.cellAt(x, y);
  for (int dy = -cells; dy <= cells; ++dy) {
    for (int dx = -cells; dx <= cells; ++dx) {
      const GridCell cell{center.x + dx, center.y + dy};
      if (std::hypot(info_.centerX(cell.x) - x, info_.centerY(cell.y) - y) <= radius) {
        setOccupied(cell);
      }
    }
  }
}

DistanceField::DistanceField(const GridInfo & info, std::vector<float> distances)
: info_(info), distances_(std::move(distances))
{
}

float DistanceField::at(const double x, const double y) const
{
  return at(info_.cellAt(x, y));
}

OccupancyGrid makeFieldGrid(const FieldBounds & field, const double resolution)
{
  GridInfo info;
  info.resolution = resolution;
  info.origin_x = field.x_min - resolution;
  info.origin_y = field.y_min - resolution;
  info.width = static_cast<int>(std::ceil((field.x_max - field.x_min) / resolution)) + 2;
  info.height = static_cast<int>(std::ceil((field.y_max - field.y_min) / resolution)) + 2;
  OccupancyGrid grid(info);
  for (int x = 0; x < info.width; ++x) {
    grid.setOccupied(GridCell{x, 0});
    grid.setOccupied(GridCell{x, info.height - 1});
  }
  for (int y = 0; y < info.height; ++y) {
    grid.setOccupied(GridCell{0, y});
    grid.setOccupied(GridCell{info.width - 1, y});
  }
  return grid;
}

void addStaticObstacles(
  OccupancyGrid & grid, const std::string & pcd_path, const FieldBounds & field,
  const double field_inset_m, const double min_height_m, const double max_height_m)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile(pcd_path, cloud) != 0) {
    throw std::runtime_error("地図を読めません: " + pcd_path);
  }
  // LiDAR の物体の縁に出る孤立した点で、通れる場所を塞がないようにする。
  // 地図は 0.2m 間隔なので、本物の固定物なら 0.4m 四方に複数の点が入る
  constexpr double kBlockSizeM = 0.4;
  constexpr int kMinPointsInBlock = 3;
  const auto block_key = [](const double x, const double y) {
      const auto ix = static_cast<std::int64_t>(std::floor(x / kBlockSizeM));
      const auto iy = static_cast<std::int64_t>(std::floor(y / kBlockSizeM));
      return (ix << 32) ^ static_cast<std::uint32_t>(iy);
    };
  const auto selected = [&](const pcl::PointXYZ & point) {
      return point.z >= min_height_m && point.z <= max_height_m &&
             field.contains(point.x, point.y, field_inset_m);
    };
  std::unordered_map<std::int64_t, int> block_counts;
  for (const auto & point : cloud.points) {
    if (selected(point)) {
      ++block_counts[block_key(point.x, point.y)];
    }
  }
  for (const auto & point : cloud.points) {
    if (selected(point) && block_counts[block_key(point.x, point.y)] >= kMinPointsInBlock) {
      grid.setOccupied(grid.info().cellAt(point.x, point.y));
    }
  }
}

DistanceField computeDistanceField(const OccupancyGrid & grid)
{
  const GridInfo & info = grid.info();
  constexpr float kFar = 1e10F;
  std::vector<float> squared(static_cast<std::size_t>(info.width * info.height));
  for (int y = 0; y < info.height; ++y) {
    for (int x = 0; x < info.width; ++x) {
      const GridCell cell{x, y};
      squared[static_cast<std::size_t>(info.index(cell))] = grid.isOccupied(cell) ? 0.0F : kFar;
    }
  }
  const int longest = std::max(info.width, info.height);
  std::vector<float> line(static_cast<std::size_t>(longest));
  std::vector<float> transformed(static_cast<std::size_t>(longest));
  std::vector<int> parabola(static_cast<std::size_t>(longest));
  std::vector<float> boundary(static_cast<std::size_t>(longest) + 1U);

  // 列方向と行方向に順に変換する
  line.resize(static_cast<std::size_t>(info.height));
  transformed.resize(static_cast<std::size_t>(info.height));
  for (int x = 0; x < info.width; ++x) {
    for (int y = 0; y < info.height; ++y) {
      line[static_cast<std::size_t>(y)] = squared[static_cast<std::size_t>(info.index(GridCell{x, y}))];
    }
    distanceTransform1d(line, transformed, parabola, boundary);
    for (int y = 0; y < info.height; ++y) {
      squared[static_cast<std::size_t>(info.index(GridCell{x, y}))] = transformed[static_cast<std::size_t>(y)];
    }
  }
  line.resize(static_cast<std::size_t>(info.width));
  transformed.resize(static_cast<std::size_t>(info.width));
  for (int y = 0; y < info.height; ++y) {
    for (int x = 0; x < info.width; ++x) {
      line[static_cast<std::size_t>(x)] = squared[static_cast<std::size_t>(info.index(GridCell{x, y}))];
    }
    distanceTransform1d(line, transformed, parabola, boundary);
    for (int x = 0; x < info.width; ++x) {
      squared[static_cast<std::size_t>(info.index(GridCell{x, y}))] = transformed[static_cast<std::size_t>(x)];
    }
  }

  std::vector<float> distances(squared.size());
  std::transform(
    squared.begin(), squared.end(), distances.begin(),
    [&info](const float value) {
      return std::sqrt(value) * static_cast<float>(info.resolution);
    });
  return DistanceField(info, std::move(distances));
}

DistanceField buildDistanceField(
  const FieldBounds & field, const double resolution, const std::string & map_path)
{
  auto grid = makeFieldGrid(field, resolution);
  if (!map_path.empty()) {
    addStaticObstacles(
      grid, map_path, field, kStaticFieldInsetM, kStaticMinHeightM, kStaticMaxHeightM);
  }
  return computeDistanceField(grid);
}

double planningInflationRadius(
  const double half_length, const double half_width, const double margin, const bool narrow_enabled,
  const double min_side_clearance)
{
  const double circumscribed = std::hypot(half_length, half_width) + margin;
  if (!narrow_enabled) {
    return circumscribed;
  }
  // 距離の場（float）の丸めで、中心の距離がちょうどしきい値の通路を塞がない
  constexpr double kRoundingToleranceM = 1e-3;
  return std::min(circumscribed, half_width + min_side_clearance - kRoundingToleranceM);
}

OccupancyGrid inflate(const DistanceField & field, const double radius)
{
  OccupancyGrid grid(field.info());
  const GridInfo & info = field.info();
  for (int y = 0; y < info.height; ++y) {
    for (int x = 0; x < info.width; ++x) {
      const GridCell cell{x, y};
      if (field.at(cell) < radius) {
        grid.setOccupied(cell);
      }
    }
  }
  return grid;
}
}  // namespace core_planning
