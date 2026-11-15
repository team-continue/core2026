#include "core_perception/detection.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace core_perception
{
namespace
{
// フィールドの端から少し内側までを使う。柵とその周りの距離ノイズを除く
constexpr double kFieldInsetM = 0.1;

using Cell = std::pair<std::int32_t, std::int32_t>;

std::int64_t cellKey(const std::int32_t x, const std::int32_t y)
{
  return (static_cast<std::int64_t>(x) << 32) ^ static_cast<std::uint32_t>(y);
}

Cell cellOf(const Eigen::Vector2f & position, const double cell_size)
{
  return {
    static_cast<std::int32_t>(std::floor(position.x() / cell_size)),
    static_cast<std::int32_t>(std::floor(position.y() / cell_size))};
}

using CellIndex = std::unordered_map<std::int64_t, std::vector<std::size_t>>;
using Sensors = std::vector<Eigen::Vector2f>;

// position に最も近いセンサの位置。複数の LiDAR の点を合わせたとき、その位置を見ている LiDAR とみなす
const Eigen::Vector2f & nearestSensor(const Sensors & sensors, const Eigen::Vector2f & position)
{
  return *std::min_element(
    sensors.begin(), sensors.end(),
    [&position](const Eigen::Vector2f & a, const Eigen::Vector2f & b) {
      return (a - position).squaredNorm() < (b - position).squaredNorm();
    });
}

CellIndex indexPoints(const std::vector<ObstaclePoint> & points, const double cell_size)
{
  CellIndex index;
  index.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto [x, y] = cellOf(points[i].position_odom, cell_size);
    index[cellKey(x, y)].push_back(i);
  }
  return index;
}

// 遠くほど点がまばらなので、つなげるセルの範囲を広げる
std::int32_t connectRadius(
  const Cell & cell, const Sensors & sensors, const double cell_size,
  const double wide_range_m)
{
  const float x = static_cast<float>((cell.first + 0.5) * cell_size);
  const float y = static_cast<float>((cell.second + 0.5) * cell_size);
  const Eigen::Vector2f center(x, y);
  return (center - nearestSensor(sensors, center)).norm() > wide_range_m ? 2 : 1;
}

// 近傍のセルでつながった集まりごとに、点の番号を返す
std::vector<std::vector<std::size_t>> connectedComponents(
  const std::vector<ObstaclePoint> & points, const Sensors & sensors,
  const double cell_size, const double wide_range_m)
{
  CellIndex index = indexPoints(points, cell_size);
  std::vector<std::vector<std::size_t>> components;
  std::vector<Cell> stack;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto start = cellOf(points[i].position_odom, cell_size);
    const auto found = index.find(cellKey(start.first, start.second));
    if (found == index.end()) {
      continue;
    }
    std::vector<std::size_t> component = std::move(found->second);
    index.erase(found);
    stack.assign(1U, start);
    while (!stack.empty()) {
      const auto [x, y] = stack.back();
      stack.pop_back();
      const std::int32_t radius = connectRadius(Cell{x, y}, sensors, cell_size, wide_range_m);
      for (std::int32_t dx = -radius; dx <= radius; ++dx) {
        for (std::int32_t dy = -radius; dy <= radius; ++dy) {
          const auto neighbor = index.find(cellKey(x + dx, y + dy));
          if (neighbor == index.end()) {
            continue;
          }
          component.insert(component.end(), neighbor->second.begin(), neighbor->second.end());
          index.erase(neighbor);
          stack.emplace_back(x + dx, y + dy);
        }
      }
    }
    components.push_back(std::move(component));
  }
  return components;
}

struct Extent
{
  Eigen::Vector2f min{
    Eigen::Vector2f::Constant(std::numeric_limits<float>::max())};
  Eigen::Vector2f max{
    Eigen::Vector2f::Constant(std::numeric_limits<float>::lowest())};
};

Extent extentOf(
  const std::vector<ObstaclePoint> & points, const std::vector<std::size_t> & indices,
  const Eigen::Vector2f & axis, const Eigen::Vector2f & origin)
{
  const Eigen::Vector2f normal(-axis.y(), axis.x());
  Extent extent;
  for (const std::size_t i : indices) {
    const Eigen::Vector2f relative = points[i].position_odom - origin;
    const Eigen::Vector2f projected(relative.dot(axis), relative.dot(normal));
    extent.min = extent.min.cwiseMin(projected);
    extent.max = extent.max.cwiseMax(projected);
  }
  return extent;
}

// 手前の面と、同じ物体の天板の奥の端かどうかを調べる。低い天板は点がまばらで、離れて見える。
// センサから見て横方向に重なり、奥行きの隙間が小さく、合わせた奥行きが物体の大きさに収まること
bool isBehindSameObject(
  const std::vector<ObstaclePoint> & points, const std::vector<std::size_t> & near_part,
  const std::vector<std::size_t> & far_part, const Sensors & sensors,
  const double max_gap, const double max_depth)
{
  Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
  for (const std::size_t i : near_part) {
    centroid += points[i].position_odom;
  }
  centroid /= static_cast<float>(near_part.size());
  const Eigen::Vector2f & sensor = nearestSensor(sensors, centroid);
  const Eigen::Vector2f to_centroid = centroid - sensor;
  const float distance = to_centroid.norm();
  if (distance < 1e-3F) {
    return false;
  }
  const Eigen::Vector2f axis = to_centroid / distance;
  const Extent a = extentOf(points, near_part, axis, sensor);
  const Extent b = extentOf(points, far_part, axis, sensor);

  // 手前と奥の順序は問わない
  const float depth_gap = std::max(b.min.x() - a.max.x(), a.min.x() - b.max.x());
  const float total_depth = std::max(a.max.x(), b.max.x()) - std::min(a.min.x(), b.min.x());
  const float overlap = std::min(a.max.y(), b.max.y()) - std::max(a.min.y(), b.min.y());
  const float narrower = std::min(a.max.y() - a.min.y(), b.max.y() - b.min.y());
  return depth_gap <= max_gap && total_depth <= max_depth && overlap >= 0.5F * narrower;
}

// 隙間が小さく、合わせても max_size に収まる集まりを 1 つにまとめる
std::vector<std::vector<std::size_t>> mergeNearbyComponents(
  const std::vector<ObstaclePoint> & points, std::vector<std::vector<std::size_t>> components,
  const Sensors & sensors, const double max_gap, const double max_size,
  const double max_depth)
{
  const auto bounds = [&points](const std::vector<std::size_t> & indices) {
      Eigen::Vector2f low = Eigen::Vector2f::Constant(std::numeric_limits<float>::max());
      Eigen::Vector2f high = Eigen::Vector2f::Constant(std::numeric_limits<float>::lowest());
      for (const std::size_t i : indices) {
        low = low.cwiseMin(points[i].position_odom);
        high = high.cwiseMax(points[i].position_odom);
      }
      return std::make_pair(low, high);
    };
  bool merged = true;
  while (merged) {
    merged = false;
    for (std::size_t a = 0; a < components.size() && !merged; ++a) {
      const auto [low_a, high_a] = bounds(components[a]);
      for (std::size_t b = a + 1; b < components.size(); ++b) {
        const auto [low_b, high_b] = bounds(components[b]);
        const Eigen::Vector2f gap =
          (low_b - high_a).cwiseMax(low_a - high_b).cwiseMax(Eigen::Vector2f::Zero());
        const float size = (high_a.cwiseMax(high_b) - low_a.cwiseMin(low_b)).norm();
        const bool same_object = gap.norm() <= max_gap && size <= max_size;
        if (same_object ||
          isBehindSameObject(points, components[a], components[b], sensors, max_gap, max_depth))
        {
          components[a].insert(components[a].end(), components[b].begin(), components[b].end());
          components.erase(components.begin() + static_cast<std::ptrdiff_t>(b));
          merged = true;
          break;
        }
      }
    }
  }
  return components;
}

// 最も広がった方向に沿って k 個に分ける（k-means）
std::vector<std::vector<std::size_t>> splitCluster(
  const std::vector<ObstaclePoint> & points, const std::vector<std::size_t> & indices,
  const std::size_t k)
{
  Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
  for (const std::size_t i : indices) {
    centroid += points[i].position_odom;
  }
  centroid /= static_cast<float>(indices.size());

  Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
  for (const std::size_t i : indices) {
    const Eigen::Vector2f relative = points[i].position_odom - centroid;
    covariance += relative * relative.transpose();
  }
  // 2x2 の対称行列の主軸
  const float angle = 0.5F * std::atan2(
    2.0F * covariance(0, 1), covariance(0, 0) - covariance(1, 1));
  const Eigen::Vector2f axis(std::cos(angle), std::sin(angle));

  std::vector<float> projections;
  projections.reserve(indices.size());
  for (const std::size_t i : indices) {
    projections.push_back((points[i].position_odom - centroid).dot(axis));
  }
  const auto [low, high] = std::minmax_element(projections.begin(), projections.end());
  std::vector<float> centers(k);
  for (std::size_t c = 0; c < k; ++c) {
    centers[c] = *low + (*high - *low) * (static_cast<float>(c) + 0.5F) / static_cast<float>(k);
  }

  std::vector<std::size_t> labels(indices.size(), 0U);
  constexpr int kIterations = 5;
  for (int iteration = 0; iteration < kIterations; ++iteration) {
    std::vector<float> sums(k, 0.0F);
    std::vector<std::size_t> counts(k, 0U);
    for (std::size_t p = 0; p < projections.size(); ++p) {
      std::size_t best = 0U;
      for (std::size_t c = 1; c < k; ++c) {
        if (std::abs(projections[p] - centers[c]) < std::abs(projections[p] - centers[best])) {
          best = c;
        }
      }
      labels[p] = best;
      sums[best] += projections[p];
      ++counts[best];
    }
    for (std::size_t c = 0; c < k; ++c) {
      if (counts[c] > 0U) {
        centers[c] = sums[c] / static_cast<float>(counts[c]);
      }
    }
  }

  std::vector<std::vector<std::size_t>> parts(k);
  for (std::size_t p = 0; p < labels.size(); ++p) {
    parts[labels[p]].push_back(indices[p]);
  }
  parts.erase(
    std::remove_if(parts.begin(), parts.end(), [](const auto & part) {return part.empty();}),
    parts.end());
  return parts;
}

Detection makeDetection(
  const std::vector<ObstaclePoint> & points, const std::vector<std::size_t> & indices,
  const Sensors & sensors, const DetectionParams & params)
{
  Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
  float height = 0.0F;
  for (const std::size_t i : indices) {
    centroid += points[i].position_odom;
    height = std::max(height, points[i].height);
  }
  centroid /= static_cast<float>(indices.size());

  const Eigen::Vector2f & sensor = nearestSensor(sensors, centroid);
  const Eigen::Vector2f to_centroid = centroid - sensor;
  const float distance = to_centroid.norm();
  const Eigen::Vector2f axis =
    distance > 1e-3F ? Eigen::Vector2f(to_centroid / distance) : Eigen::Vector2f::UnitX();
  const Eigen::Vector2f normal(-axis.y(), axis.x());
  const Extent extent = extentOf(points, indices, axis, sensor);

  Detection detection;
  detection.depth = extent.max.x() - extent.min.x();
  detection.width = extent.max.y() - extent.min.y();
  detection.height = height;
  detection.num_points = indices.size();

  // 手前の面しか見えないので、大きさを仮定して奥へ中心をずらす
  const float expected_size = static_cast<float>(
    height >= params.human_min_height_m ? params.human_size_m : params.split.expected_robot_size_m);
  const float center_depth = extent.min.x() + 0.5F * std::max(detection.depth, expected_size);
  const float center_lateral = 0.5F * (extent.min.y() + extent.max.y());
  detection.center = sensor + axis * center_depth + normal * center_lateral;
  detection.box_center = sensor + axis * (0.5F * (extent.min.x() + extent.max.x())) +
    normal * center_lateral;
  detection.box_yaw = std::atan2(axis.y(), axis.x());
  return detection;
}
}  // namespace

std::vector<ObstaclePoint> removeOutliers(
  const std::vector<ObstaclePoint> & points, const OutlierFilterParams & params)
{
  if (!params.enabled) {
    return points;
  }
  const CellIndex index = indexPoints(points, params.radius_m);
  const float radius_squared = static_cast<float>(params.radius_m * params.radius_m);
  std::vector<ObstaclePoint> output;
  output.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto [x, y] = cellOf(points[i].position_odom, params.radius_m);
    std::size_t neighbors = 0U;
    for (std::int32_t dx = -1; dx <= 1 && neighbors < params.min_neighbors; ++dx) {
      for (std::int32_t dy = -1; dy <= 1 && neighbors < params.min_neighbors; ++dy) {
        const auto found = index.find(cellKey(x + dx, y + dy));
        if (found == index.end()) {
          continue;
        }
        for (const std::size_t j : found->second) {
          if (j != i &&
            (points[j].position_odom - points[i].position_odom).squaredNorm() <= radius_squared)
          {
            ++neighbors;
          }
        }
      }
    }
    if (neighbors >= params.min_neighbors) {
      output.push_back(points[i]);
    }
  }
  return output;
}

SelfFilterParams selfFilterFromFootprint(
  const double length_m, const double width_m, const double height_m, const double margin_m)
{
  SelfFilterParams params;
  params.half_length_m = 0.5 * length_m + margin_m;
  params.half_width_m = 0.5 * width_m + margin_m;
  params.top_height_m = height_m + margin_m;
  return params;
}

SelfFilterParams cylinderSelfFilterFromFootprint(
  const double length_m, const double width_m, const double height_m, const double margin_m)
{
  SelfFilterParams params = selfFilterFromFootprint(length_m, width_m, height_m, margin_m);
  params.radius_m = std::hypot(0.5 * length_m, 0.5 * width_m) + margin_m;
  return params;
}

bool insideSelf(const Eigen::Vector3f & base, const SelfFilterParams & params)
{
  if (base.z() > params.top_height_m) {
    return false;
  }
  if (params.radius_m > 0.0) {
    return static_cast<double>(base.x()) * base.x() + static_cast<double>(base.y()) * base.y() <=
           params.radius_m * params.radius_m;
  }
  return std::abs(base.x()) <= params.half_length_m && std::abs(base.y()) <= params.half_width_m;
}

std::vector<ObstaclePoint> selectObstaclePoints(
  const std::vector<FramePoint> & points, const FrameTransforms & transforms,
  const DetectionParams & params, const StaticMap & static_map, SelectionCounts & counts)
{
  counts = SelectionCounts{};
  counts.input = points.size();
  std::vector<ObstaclePoint> selected;
  selected.reserve(points.size() / 4U);
  const float min_range_squared =
    static_cast<float>(params.self.min_range_m * params.self.min_range_m);
  for (const auto & point : points) {
    // 返りのない (0,0,0) の点と、LiDAR に近すぎる点は自機の点と同じに数える
    if (point.lidar.squaredNorm() < min_range_squared) {
      continue;
    }
    const Eigen::Vector3f base = transforms.base_to_lidar * point.lidar;
    if (insideSelf(base, params.self)) {
      continue;
    }
    ++counts.after_self;

    // 機体の傾きを除いた床からの高さ。LIO の高さのずれの影響を受けない
    const float height = (point.odom_to_base.rotation() * base).z();
    if (height <= params.height.ground_height_m || height > params.height.max_height_m) {
      continue;
    }
    ++counts.after_height;

    const Eigen::Vector3f odom = point.odom_to_base * base;
    const Eigen::Vector3f map = transforms.map_to_odom * odom;
    if (!static_map.field().contains(map.x(), map.y(), kFieldInsetM)) {
      continue;
    }
    ++counts.after_field;

    if (static_map.contains(map)) {
      continue;
    }
    ++counts.after_map;
    selected.push_back(ObstaclePoint{odom.head<2>(), height});
  }
  auto output = removeOutliers(selected, params.outlier);
  counts.after_outlier = output.size();
  return output;
}

std::vector<Detection> clusterObstaclePoints(
  const std::vector<ObstaclePoint> & points, const Eigen::Vector2f & sensor,
  const DetectionParams & params)
{
  return clusterObstaclePoints(points, Sensors{sensor}, params);
}

std::vector<Detection> clusterObstaclePoints(
  const std::vector<ObstaclePoint> & points, const std::vector<Eigen::Vector2f> & sensors,
  const DetectionParams & params)
{
  if (sensors.empty()) {
    throw std::invalid_argument("clusterObstaclePoints: センサの位置が 1 つもありません");
  }
  std::vector<Detection> detections;
  const auto components = mergeNearbyComponents(
    points,
    connectedComponents(
      points, sensors, params.cluster.cell_size_m, params.cluster.wide_connect_range_m),
    sensors, params.cluster.merge_gap_m, params.split.max_cluster_size_m,
    params.split.expected_robot_size_m * 1.3);
  for (const auto & component : components) {
    if (component.size() < params.cluster.min_points) {
      continue;
    }
    // 接近したクラスタは、想定の大きさから個数を見積もって分ける
    std::vector<std::vector<std::size_t>> parts{component};
    if (params.split.enabled) {
      // 軸に沿った広がりの大きさだけを使うので、原点はどのセンサでもよい
      const Extent extent = extentOf(points, component, Eigen::Vector2f::UnitX(), sensors.front());
      const float size = (extent.max - extent.min).norm();
      if (size > params.split.max_cluster_size_m) {
        const auto k = static_cast<std::size_t>(
          std::ceil(size / params.split.expected_robot_size_m / std::sqrt(2.0F)));
        parts = splitCluster(points, component, std::max<std::size_t>(k, 2U));
      }
    }
    for (const auto & part : parts) {
      if (part.size() < params.cluster.min_points) {
        continue;
      }
      Detection detection = makeDetection(points, part, sensors, params);
      if (detection.height >= params.cluster.min_height_m) {
        detections.push_back(detection);
      }
    }
  }
  return detections;
}
}  // namespace core_perception
