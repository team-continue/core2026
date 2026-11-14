#include "core_planning/path_smoother.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <string>

namespace core_planning
{
namespace
{
using Point = std::pair<double, double>;

// これより短い区間は、重なった点として除く
constexpr double kDegenerateSegmentM = 1e-9;
// 1 回の移動がこれより小さければ動かさない
constexpr double kMinMoveM = 1e-9;
// 1 周で動いた最大の距離がこれより小さければ、収束したとして終える
constexpr double kConvergedMoveM = 1e-4;
// 増やした点の数の上限。範囲の値なら届かない（60m 四方のフィールドを 0.02m 間隔でも数万点）
constexpr double kMaxPoints = 1e6;
// 端点からこの道のりの間は、取りたい距離を端点の距離から少しずつ上げる。
// 端点は動かないので、目標の距離のまま隣の点を押し出すと、端点で鉤形に曲がる（機体が障害物の近く
// にいるとき、ゴールが障害物の近くにあるとき）。壁と平行に 0.7m の位置から始まる経路は、上げないと
// 最初の 0.1m で 0.19m 横へ出る（約 60°）。1m で上げると最初の区間の向きの変化は約 15° になる
constexpr double kEndpointRampM = 1.0;

void requireRange(const char * name, const double value, const double min, const double max)
{
  if (!std::isfinite(value) || value < min || value > max) {
    std::ostringstream message;
    message << "経路の曲線化の " << name << " は " << min << "〜" << max << " にしてください: " <<
      value;
    throw std::invalid_argument(message.str());
  }
}

void validate(const double hard_radius_m, const SmoothingParams & params)
{
  requireRange("hard_radius_m", hard_radius_m, 0.0, 1e3);
  requireRange("spacing_m", params.spacing_m, 1e-6, 1e3);
  requireRange("extra_clearance_m", params.extra_clearance_m, 0.0, 1e3);
  requireRange("iterations", static_cast<double>(params.iterations), 0.0, 1e6);
  requireRange("smooth_weight", params.smooth_weight, 0.0, 1.0);
  requireRange("clearance_weight", params.clearance_weight, 0.0, 1.0);
}

double cellDistance(const DistanceField & field, const Point & point)
{
  return static_cast<double>(field.at(point.first, point.second));
}

Point midpoint(const Point & a, const Point & b)
{
  return {0.5 * (a.first + b.first), 0.5 * (a.second + b.second)};
}

// 距離場をセルの中心の間で双一次補間した距離と、その勾配
struct DistanceSample
{
  double distance{0.0};
  double gradient_x{0.0};
  double gradient_y{0.0};
};

DistanceSample sampleDistance(const DistanceField & field, const Point & point)
{
  const GridInfo & info = field.info();
  const double u = (point.first - info.origin_x) / info.resolution - 0.5;
  const double v = (point.second - info.origin_y) / info.resolution - 0.5;
  const int x = static_cast<int>(std::floor(u));
  const int y = static_cast<int>(std::floor(v));
  const double fx = u - x;
  const double fy = v - y;
  const double d00 = field.at(GridCell{x, y});
  const double d10 = field.at(GridCell{x + 1, y});
  const double d01 = field.at(GridCell{x, y + 1});
  const double d11 = field.at(GridCell{x + 1, y + 1});
  return DistanceSample{
    (1.0 - fx) * (1.0 - fy) * d00 + fx * (1.0 - fy) * d10 + (1.0 - fx) * fy * d01 + fx * fy * d11,
    ((1.0 - fy) * (d10 - d00) + fy * (d11 - d01)) / info.resolution,
    ((1.0 - fx) * (d01 - d00) + fx * (d11 - d10)) / info.resolution};
}

struct Resampled
{
  std::vector<Point> points;
  // 最初の区間の終わりの点と、最後の区間の始まりの点の番号
  std::size_t first_segment_end{0U};
  std::size_t last_segment_begin{0U};
};

// 各区間を spacing 以下の等しい間隔に分ける。元の頂点はそのまま残す
Resampled resample(const std::vector<Point> & polyline, const double spacing)
{
  double total_length = 0.0;
  for (std::size_t i = 1U; i < polyline.size(); ++i) {
    total_length += std::hypot(
      polyline[i].first - polyline[i - 1U].first, polyline[i].second - polyline[i - 1U].second);
  }
  if (total_length / spacing > kMaxPoints) {
    std::ostringstream message;
    message << "経路が長すぎて曲線にできません: 長さ " << total_length << "m、間隔 " << spacing << "m";
    throw std::invalid_argument(message.str());
  }

  Resampled resampled;
  resampled.points.reserve(static_cast<std::size_t>(total_length / spacing) + polyline.size());
  resampled.points.push_back(polyline.front());
  bool first_segment = true;
  for (std::size_t i = 1U; i < polyline.size(); ++i) {
    const auto [x0, y0] = resampled.points.back();
    const auto [x1, y1] = polyline[i];
    const double length = std::hypot(x1 - x0, y1 - y0);
    if (length <= kDegenerateSegmentM) {
      continue;
    }
    const std::size_t segment_begin = resampled.points.size() - 1U;
    const auto count = static_cast<std::size_t>(std::max(1.0, std::ceil(length / spacing - 1e-9)));
    for (std::size_t k = 1U; k < count; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(count);
      resampled.points.emplace_back(x0 + t * (x1 - x0), y0 + t * (y1 - y0));
    }
    resampled.points.push_back(polyline[i]);
    if (first_segment) {
      resampled.first_segment_end = resampled.points.size() - 1U;
      first_segment = false;
    }
    resampled.last_segment_begin = segment_begin;
  }
  // 重なった点を除いても、終点は元の終点と同じにする
  resampled.points.back() = polyline.back();
  return resampled;
}

// 点と、隣り合う点の中点で守る距離
struct Floors
{
  std::vector<double> points;
  // midpoints[i] は points[i] と points[i + 1] の中点
  std::vector<double> midpoints;
};

// 最初と最後の区間は、元の折れ線の距離が hard_radius_m より近ければ、その距離を守る（ヘッダの説明）
Floors makeFloors(const Resampled & resampled, const DistanceField & field, const double hard_radius)
{
  const auto & points = resampled.points;
  const auto in_end_segment = [&resampled](const std::size_t first, const std::size_t last) {
      return last <= resampled.first_segment_end || first >= resampled.last_segment_begin;
    };
  Floors floors;
  floors.points.reserve(points.size());
  floors.midpoints.reserve(points.size());
  for (std::size_t i = 0U; i < points.size(); ++i) {
    floors.points.push_back(
      in_end_segment(i, i) ? std::min(hard_radius, cellDistance(field, points[i])) : hard_radius);
  }
  for (std::size_t i = 0U; i + 1U < points.size(); ++i) {
    floors.midpoints.push_back(
      in_end_segment(i, i + 1U) ?
      std::min(hard_radius, cellDistance(field, midpoint(points[i], points[i + 1U]))) :
      hard_radius);
  }
  return floors;
}

bool satisfiesFloors(
  const std::vector<Point> & points, const Floors & floors, const DistanceField & field)
{
  for (std::size_t i = 0U; i < points.size(); ++i) {
    if (cellDistance(field, points[i]) < floors.points[i]) {
      return false;
    }
  }
  for (std::size_t i = 0U; i + 1U < points.size(); ++i) {
    if (cellDistance(field, midpoint(points[i], points[i + 1U])) < floors.midpoints[i]) {
      return false;
    }
  }
  return true;
}

// 点ごとに取りたい距離。端点の近くでは端点の距離から目標の距離まで道のりに比例して上げる
std::vector<double> makeTargets(
  const std::vector<Point> & points, const DistanceField & field, const double full_target)
{
  std::vector<double> along(points.size(), 0.0);
  for (std::size_t i = 1U; i < points.size(); ++i) {
    along[i] = along[i - 1U] + std::hypot(
      points[i].first - points[i - 1U].first, points[i].second - points[i - 1U].second);
  }
  const double total = along.back();
  const double start_distance = sampleDistance(field, points.front()).distance;
  const double goal_distance = sampleDistance(field, points.back()).distance;
  std::vector<double> targets(points.size());
  std::transform(
    along.begin(), along.end(), targets.begin(),
    [total, start_distance, goal_distance, full_target](const double s) {
      const bool near_start = s <= total - s;
      const double from_end = near_start ? s : total - s;
      const double base = near_start ? start_distance : goal_distance;
      if (base >= full_target || from_end >= kEndpointRampM) {
        return full_target;
      }
      return base + (full_target - base) * from_end / kEndpointRampM;
    });
  return targets;
}

// 1 点を動かした先。なめらかさと余裕の 2 つの力を足す
Point movedPoint(
  const Point & previous, const Point & current, const Point & next, const DistanceField & field,
  const double target_distance, const double push_range, const SmoothingParams & params)
{
  const Point center = midpoint(previous, next);
  double x = current.first + params.smooth_weight * (center.first - current.first);
  double y = current.second + params.smooth_weight * (center.second - current.second);
  // セルの距離が十分遠ければ、補間した距離も目標より遠いので調べない
  if (cellDistance(field, current) < push_range) {
    const DistanceSample sample = sampleDistance(field, current);
    const double deficit = target_distance - sample.distance;
    if (deficit > 0.0) {
      // 勾配の大きさは 1 を超えないようにする。2 つの障害物から等距離の尾根の近くでは小さくなり、
      // 両側から押されて行き来しにくい
      const double norm = std::hypot(sample.gradient_x, sample.gradient_y);
      const double scale = params.clearance_weight * deficit / std::max(norm, 1.0);
      x += scale * sample.gradient_x;
      y += scale * sample.gradient_y;
    }
  }
  return {x, y};
}

// 端点以外の点を動かす。動かした点で、その点と両隣との中点が守る距離を満たす移動だけを採るので、
// 初めに満たしていれば最後まで満たす
std::vector<Point> relax(
  std::vector<Point> points, const Floors & floors, const DistanceField & field,
  const double target_distance, const SmoothingParams & params)
{
  const std::size_t count = points.size();
  if (count < 3U) {
    return points;
  }
  const std::vector<double> targets = makeTargets(points, field, target_distance);
  // 双一次補間の値は、点のあるセルの値と最大で対角 1 つ分しか違わない
  const double push_range = target_distance + 2.0 * field.info().resolution;
  for (int iteration = 0; iteration < params.iterations; ++iteration) {
    // 動かした点をすぐ次の点に使うので、向きを交互に変えて片側へ寄る偏りをなくす
    const bool forward = iteration % 2 == 0;
    double largest_move = 0.0;
    for (std::size_t k = 1U; k + 1U < count; ++k) {
      const std::size_t i = forward ? k : count - 1U - k;
      const Point candidate = movedPoint(
        points[i - 1U], points[i], points[i + 1U], field, targets[i], push_range, params);
      const double move =
        std::hypot(candidate.first - points[i].first, candidate.second - points[i].second);
      if (move < kMinMoveM ||
        cellDistance(field, candidate) < floors.points[i] ||
        cellDistance(field, midpoint(points[i - 1U], candidate)) < floors.midpoints[i - 1U] ||
        cellDistance(field, midpoint(candidate, points[i + 1U])) < floors.midpoints[i])
      {
        continue;
      }
      points[i] = candidate;
      largest_move = std::max(largest_move, move);
    }
    if (largest_move < kConvergedMoveM) {
      break;
    }
  }
  return points;
}
}  // namespace

std::optional<std::vector<std::pair<double, double>>> smoothPath(
  const std::vector<std::pair<double, double>> & polyline, const DistanceField & field,
  const double hard_radius_m, const SmoothingParams & params)
{
  validate(hard_radius_m, params);
  if (!params.enabled || polyline.size() < 2U) {
    return polyline;
  }
  const bool finite = std::all_of(
    polyline.begin(), polyline.end(),
    [](const Point & point) {return std::isfinite(point.first) && std::isfinite(point.second);});
  if (!finite) {
    throw std::invalid_argument("曲線にする経路に有限でない座標があります");
  }

  const Resampled resampled = resample(polyline, params.spacing_m);
  if (resampled.points.size() < 2U) {
    // すべての点が重なっている
    return polyline;
  }
  const Floors floors = makeFloors(resampled, field, hard_radius_m);
  if (!satisfiesFloors(resampled.points, floors, field)) {
    // 元の折れ線が途中で膨らみに入っている
    return std::nullopt;
  }
  auto points = relax(
    resampled.points, floors, field, hard_radius_m + params.extra_clearance_m, params);
  // 移動ごとに確かめているので通るはずだが、返す前に全点を確かめる
  if (!satisfiesFloors(points, floors, field)) {
    return std::nullopt;
  }
  return points;
}
}  // namespace core_planning
