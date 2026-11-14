#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "core_perception/detection.hpp"

namespace
{
using core_perception::clusterObstaclePoints;
using core_perception::DetectionParams;
using core_perception::ObstaclePoint;
using core_perception::OutlierFilterParams;
using core_perception::removeOutliers;
using core_perception::FramePoint;
using core_perception::FrameTransforms;
using core_perception::selectObstaclePoints;
using core_perception::SelectionCounts;
using core_perception::selfFilterFromFootprint;
using core_perception::StaticMap;

std::vector<ObstaclePoint> squareFace(const float x, const float y, const float height)
{
  // センサに向いた幅 0.8m の面
  std::vector<ObstaclePoint> points;
  for (int i = 0; i <= 16; ++i) {
    points.push_back(ObstaclePoint{Eigen::Vector2f(x, y - 0.4F + 0.05F * i), height});
  }
  return points;
}

TEST(RemoveOutliers, RemovesIsolatedPoint)
{
  auto points = squareFace(3.0F, 0.0F, 0.7F);
  const auto before = points.size();
  points.push_back(ObstaclePoint{Eigen::Vector2f(10.0F, 10.0F), 0.5F});
  OutlierFilterParams params;
  EXPECT_EQ(removeOutliers(points, params).size(), before);
  params.enabled = false;
  EXPECT_EQ(removeOutliers(points, params).size(), before + 1U);
}

TEST(Cluster, EstimatesRobotCenterBehindVisibleFace)
{
  const auto detections = clusterObstaclePoints(
    squareFace(3.0F, 0.0F, 0.7F), Eigen::Vector2f::Zero(), DetectionParams{});
  ASSERT_EQ(detections.size(), 1U);
  EXPECT_NEAR(detections[0].center.x(), 3.4F, 0.05F);
  EXPECT_NEAR(detections[0].center.y(), 0.0F, 0.05F);
  EXPECT_NEAR(detections[0].width, 0.8F, 0.05F);
}

// 点がすべて計測した箱（box_center、box_yaw、depth、width）の中にあるか
bool boxEncloses(
  const core_perception::Detection & detection, const std::vector<ObstaclePoint> & points)
{
  const Eigen::Vector2f axis(std::cos(detection.box_yaw), std::sin(detection.box_yaw));
  const Eigen::Vector2f normal(-axis.y(), axis.x());
  constexpr float kTolerance = 1e-3F;
  return std::all_of(
    points.begin(), points.end(), [&](const ObstaclePoint & point) {
      const Eigen::Vector2f offset = point.position_odom - detection.box_center;
      return std::abs(offset.dot(axis)) <= 0.5F * detection.depth + kTolerance &&
             std::abs(offset.dot(normal)) <= 0.5F * detection.width + kTolerance;
    });
}

TEST(Cluster, MeasuredBoxEnclosesVisiblePoints)
{
  // 正面の面と、奥へ伸びる側面（センサから見て L 字）
  auto points = squareFace(3.0F, 0.0F, 0.7F);
  for (int i = 1; i <= 8; ++i) {
    points.push_back(ObstaclePoint{Eigen::Vector2f(3.0F + 0.08F * i, -0.4F), 0.7F});
  }
  const auto detections = clusterObstaclePoints(points, Eigen::Vector2f::Zero(), DetectionParams{});
  ASSERT_EQ(detections.size(), 1U);
  EXPECT_TRUE(boxEncloses(detections[0], points));
  EXPECT_NEAR(detections[0].depth, 0.64F, 0.05F);
  // 推定した中心は奥へずらすが、箱の中心は見えている点の範囲の中央
  EXPECT_LT(detections[0].box_center.x(), detections[0].center.x());

  // センサから見て真横（+y 方向）にある面は、箱も +y に向く
  std::vector<ObstaclePoint> side;
  for (int i = 0; i <= 16; ++i) {
    side.push_back(ObstaclePoint{Eigen::Vector2f(-0.4F + 0.05F * i, 3.0F), 0.7F});
  }
  const auto side_detections =
    clusterObstaclePoints(side, Eigen::Vector2f::Zero(), DetectionParams{});
  ASSERT_EQ(side_detections.size(), 1U);
  EXPECT_NEAR(side_detections[0].box_yaw, static_cast<float>(M_PI / 2.0), 0.05F);
  EXPECT_TRUE(boxEncloses(side_detections[0], side));
}

TEST(Cluster, SplitsTwoAdjacentRobots)
{
  // 隙間なく横に並んだ 2 台の面
  auto points = squareFace(3.0F, -0.4F, 0.7F);
  const auto other = squareFace(3.0F, 0.4F, 0.7F);
  points.insert(points.end(), other.begin(), other.end());

  DetectionParams params;
  EXPECT_EQ(clusterObstaclePoints(points, Eigen::Vector2f::Zero(), params).size(), 2U);
  params.split.enabled = false;
  EXPECT_EQ(clusterObstaclePoints(points, Eigen::Vector2f::Zero(), params).size(), 1U);
}

TEST(Cluster, MergesFaceAndTopOfSameRobot)
{
  // 手前の面と、離れて見える天板の奥の端
  auto points = squareFace(3.0F, 0.0F, 0.7F);
  const auto far_edge = squareFace(3.6F, 0.0F, 0.8F);
  points.insert(points.end(), far_edge.begin(), far_edge.end());
  const auto detections = clusterObstaclePoints(points, Eigen::Vector2f::Zero(), DetectionParams{});
  ASSERT_EQ(detections.size(), 1U);
  EXPECT_NEAR(detections[0].center.x(), 3.4F, 0.1F);
}

TEST(Cluster, MergesFarEdgesOfAdjacentRobotsBeforeSplitting)
{
  // 並んだ 2 台の手前の面と、それぞれの天板の奥の端
  auto points = squareFace(3.0F, -0.4F, 0.7F);
  for (const auto & part : {squareFace(3.0F, 0.4F, 0.7F), squareFace(3.65F, -0.4F, 0.8F),
      squareFace(3.65F, 0.4F, 0.8F)})
  {
    points.insert(points.end(), part.begin(), part.end());
  }
  EXPECT_EQ(
    clusterObstaclePoints(points, Eigen::Vector2f::Zero(), DetectionParams{}).size(), 2U);
}

TEST(SelfFilter, FootprintGivesSameBoxAsBefore)
{
  // footprint.yaml の 0.8m 角（高さ 0.8m）と余裕 0.05m。移す前の perception.yaml の self_filter_margin_m と
  // 同じ箱。上面は機体の高さに余裕を足す
  const auto self = selfFilterFromFootprint(0.8, 0.8, 0.8, 0.05);
  EXPECT_DOUBLE_EQ(self.half_length_m, 0.45);
  EXPECT_DOUBLE_EQ(self.half_width_m, 0.45);
  EXPECT_DOUBLE_EQ(self.top_height_m, 0.85);
  const auto rectangle = selfFilterFromFootprint(1.0, 0.6, 0.5, 0.1);
  EXPECT_DOUBLE_EQ(rectangle.half_length_m, 0.6);
  EXPECT_DOUBLE_EQ(rectangle.half_width_m, 0.4);
  EXPECT_DOUBLE_EQ(rectangle.top_height_m, 0.6);
}

TEST(SelfFilter, RemovesPointsInsideBoxBelowLidar)
{
  // 以前の取り付け（機体の中心の上 0.85m）。箱の上面 0.85m は LiDAR の高さと同じ
  DetectionParams params;
  params.self = selfFilterFromFootprint(0.8, 0.8, 0.8, 0.05);
  FrameTransforms transforms;
  transforms.base_to_lidar.translation() = Eigen::Vector3f(0.0F, 0.0F, 0.85F);
  const auto at = [](const float x, const float y, const float z) {
      return FramePoint{Eigen::Vector3f(x, y, z), Eigen::Isometry3f::Identity()};
    };
  // LiDAR 座標の点。base_link では z に 0.85 を足す
  const std::vector<FramePoint> points{
    at(0.44F, 0.0F, -0.1F),   // 箱の中（天板）: 除く
    at(-0.3F, 0.44F, -0.5F),  // 箱の中（側面）: 除く
    at(0.46F, 0.0F, -0.1F),   // 箱の前 1cm: 残す
    at(0.0F, -0.46F, -0.1F),  // 箱の横 1cm: 残す
    at(0.2F, 0.2F, 0.2F),     // 箱の上で LiDAR より高い: 残す
  };
  SelectionCounts counts;
  selectObstaclePoints(points, transforms, params, StaticMap{}, counts);
  EXPECT_EQ(counts.input, 5U);
  EXPECT_EQ(counts.after_self, 3U);
}

TEST(SelfFilter, RemovesBodyAboveLowFrontLidar)
{
  // 今の取り付け: 機体の前の端（前の面 0.4m の 4cm 外）、床から 0.25m。LiDAR より上の機体の面も除く
  DetectionParams params;
  params.self = selfFilterFromFootprint(0.8, 0.8, 0.8, 0.05);
  FrameTransforms transforms;
  transforms.base_to_lidar.translation() = Eigen::Vector3f(0.44F, 0.0F, 0.25F);
  const auto at = [](const float x, const float y, const float z) {
      return FramePoint{Eigen::Vector3f(x, y, z), Eigen::Isometry3f::Identity()};
    };
  // LiDAR 座標の点。base_link では x に 0.44、z に 0.25 を足す
  const std::vector<FramePoint> points{
    at(-0.04F, 0.3F, 0.35F),   // 前の面（base z 0.6、LiDAR より上）: 除く
    at(-0.04F, -0.2F, -0.1F),  // 前の面（base z 0.15、LiDAR より下）: 除く
    at(-0.84F, 0.0F, 0.59F),   // 機体の後ろの端の上面（base z 0.84）: 除く
    at(0.03F, 0.0F, 0.3F),     // 箱の前 2cm（base x 0.47）: 残す
    at(-0.44F, 0.0F, 0.65F),   // 機体の上面より上（base z 0.9）: 残す
    at(-0.44F, 0.48F, 0.3F),   // 箱の横（base y 0.48）: 残す
  };
  SelectionCounts counts;
  selectObstaclePoints(points, transforms, params, StaticMap{}, counts);
  EXPECT_EQ(counts.input, 6U);
  EXPECT_EQ(counts.after_self, 3U);
}

TEST(SelfFilter, RemovesNoReturnPointsOutsideBox)
{
  // LiDAR を箱の外（前の面 0.4m の 10cm 外）に付けたとき。実機のドライバが返りのない向きに出す (0,0,0) の点は、
  // 箱では除けないので距離で除く
  DetectionParams params;
  params.self = selfFilterFromFootprint(0.8, 0.8, 0.8, 0.05);
  FrameTransforms transforms;
  transforms.base_to_lidar.translation() = Eigen::Vector3f(0.5F, 0.0F, 0.25F);
  const auto at = [](const float x, const float y, const float z) {
      return FramePoint{Eigen::Vector3f(x, y, z), Eigen::Isometry3f::Identity()};
    };
  const std::vector<FramePoint> points{
    at(0.0F, 0.0F, 0.0F),     // 返りのない点: 除く
    at(0.05F, 0.03F, 0.0F),   // LiDAR から 6cm: 除く
    at(0.15F, 0.0F, 0.1F),    // LiDAR から 18cm、箱の外: 残す
    at(1.0F, 0.0F, 0.2F),     // 遠い点: 残す
  };
  SelectionCounts counts;
  selectObstaclePoints(points, transforms, params, StaticMap{}, counts);
  EXPECT_EQ(counts.after_self, 2U);

  // 0 にすると距離では除かない（箱の外の (0,0,0) は残る）
  params.self.min_range_m = 0.0;
  selectObstaclePoints(points, transforms, params, StaticMap{}, counts);
  EXPECT_EQ(counts.after_self, 4U);
}

TEST(Cluster, DropsLowClusters)
{
  EXPECT_TRUE(
    clusterObstaclePoints(squareFace(3.0F, 0.0F, 0.1F), Eigen::Vector2f::Zero(), DetectionParams{})
    .empty());
}

// 2 台の LiDAR（前の端と後ろの端）の点を合わせたとき: クラスタごとに近いほうの LiDAR から見た向きを使う
TEST(Cluster, UsesNearestSensorForMergedLidars)
{
  const Eigen::Vector2f front(0.44F, 0.0F);
  const Eigen::Vector2f rear(-0.44F, 0.0F);
  // センサが 1 つなら、1 つの位置を渡したときと同じ
  const auto face = squareFace(3.0F, 1.0F, 0.7F);
  const auto single = clusterObstaclePoints(face, front, DetectionParams{});
  const auto as_list = clusterObstaclePoints(
    face, std::vector<Eigen::Vector2f>{front}, DetectionParams{});
  ASSERT_EQ(single.size(), 1U);
  ASSERT_EQ(as_list.size(), 1U);
  EXPECT_TRUE(single[0].center.isApprox(as_list[0].center));

  // 前の相手は前の LiDAR、後ろ横の相手は後ろの LiDAR から見たときと同じ推定になる
  const std::vector<Eigen::Vector2f> both{front, rear};
  const auto front_object = clusterObstaclePoints(face, both, DetectionParams{});
  ASSERT_EQ(front_object.size(), 1U);
  EXPECT_TRUE(front_object[0].center.isApprox(single[0].center));

  // 後ろ横（x=-1.5）に、後ろの LiDAR へ向いた面
  std::vector<ObstaclePoint> rear_face;
  for (int i = 0; i <= 16; ++i) {
    const float along = 0.05F * static_cast<float>(i);
    rear_face.push_back(
      ObstaclePoint{Eigen::Vector2f(-1.5F + 0.6F * along, 1.2F + 0.8F * along), 0.7F});
  }
  const auto from_rear = clusterObstaclePoints(rear_face, rear, DetectionParams{});
  const auto from_front = clusterObstaclePoints(rear_face, front, DetectionParams{});
  const auto merged = clusterObstaclePoints(rear_face, both, DetectionParams{});
  ASSERT_EQ(from_rear.size(), 1U);
  ASSERT_EQ(from_front.size(), 1U);
  ASSERT_EQ(merged.size(), 1U);
  EXPECT_TRUE(merged[0].center.isApprox(from_rear[0].center));
  // 前の LiDAR から見た向きでは、中心を奥へずらす向きが違う
  EXPECT_GT((from_front[0].center - from_rear[0].center).norm(), 0.05F);
}

TEST(Cluster, RejectsEmptySensorList)
{
  EXPECT_THROW(
    clusterObstaclePoints(
      squareFace(3.0F, 0.0F, 0.7F), std::vector<Eigen::Vector2f>{}, DetectionParams{}),
    std::invalid_argument);
}
}  // namespace
