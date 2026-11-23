// 生点群から障害物の候補を取り出す処理。ROS に依存しない。
//
// 流れ: 自機の除去、高さでの地面と天井の除去、フィールド外の除去、地図との差分、
//       外れ値の除去、2D グリッドでのクラスタリング、接近したクラスタの分離。

#ifndef CORE_PERCEPTION__DETECTION_HPP_
#define CORE_PERCEPTION__DETECTION_HPP_

#include <cstddef>
#include <vector>

#include <Eigen/Geometry>

#include "core_perception/static_map.hpp"

namespace core_perception
{
struct SelfFilterParams
{
  double half_length_m{0.45};
  double half_width_m{0.45};
  // base_link（床に投影した機体中心）での箱の上面の高さ（機体の上面 0.8m と余裕 0.05m）
  double top_height_m{0.85};
  // 0 より大きければ、xy は箱ではなくこの半径 [m] の円柱で判定する（上部の下で駆動部が回る機体）
  double radius_m{0.0};
  // LiDAR からこの距離 [m] 未満の点を、箱とは別に除く。実機のドライバは返りのない向きの点を (0,0,0) で出す
  // （2026-03 の実機の bag では 1 回の約 35%）。箱だけに頼ると、LiDAR を箱の外に付けたとき、その点が LiDAR の
  // 位置の障害物になる。Mid-360 の最短距離 0.1m。0 で使わない
  double min_range_m{0.1};
};

struct HeightFilterParams
{
  // 床からの高さでこの値以下の点を地面として除く。柵（0.14m）はフィールド範囲で除く
  double ground_height_m{0.12};
  double max_height_m{2.2};
};

struct OutlierFilterParams
{
  bool enabled{true};
  double radius_m{0.2};
  std::size_t min_neighbors{2};
};

struct ClusterParams
{
  double cell_size_m{0.2};
  std::size_t min_points{3};
  double min_height_m{0.25};
  // これより遠いセルは、2 セル離れていてもつなげる
  double wide_connect_range_m{10.0};
  // この隙間以内で、合わせてもロボットの大きさに収まるクラスタは 1 つにまとめる。
  // 低い天板は点がまばらで、手前の面と離れて別のクラスタになることがある
  double merge_gap_m{0.7};
};

struct ClusterSplitParams
{
  bool enabled{true};
  double max_cluster_size_m{1.2};
  double expected_robot_size_m{0.8};
};

struct DetectionParams
{
  SelfFilterParams self;
  HeightFilterParams height;
  OutlierFilterParams outlier;
  ClusterParams cluster;
  ClusterSplitParams split;
  // 種類ごとの大きさ。見えている面から中心を推定するのに使う
  double human_size_m{0.5};
  double human_min_height_m{1.2};
};

// 1 点ごとの位置。center は odom 座標系
struct ObstaclePoint
{
  Eigen::Vector2f position_odom;
  float height;
};

// 1 クラスタの観測
struct Detection
{
  Eigen::Vector2f center;
  // センサから見た奥行きと幅
  float depth;
  float width;
  float height;
  std::size_t num_points;
  // 見えている点を囲む箱の中心（odom 座標系）と、奥行きの向き（センサから見た向き）。大きさは
  // depth と width。center は大きさを仮定して奥へずらした推定なので、箱の中心とは一致しない
  Eigen::Vector2f box_center{Eigen::Vector2f::Zero()};
  float box_yaw{0.0F};
};

// 1 点ごとの LiDAR 座標と、その時刻の姿勢
struct FramePoint
{
  Eigen::Vector3f lidar;
  Eigen::Isometry3f odom_to_base;
};

struct FrameTransforms
{
  Eigen::Isometry3f base_to_lidar{Eigen::Isometry3f::Identity()};
  Eigen::Isometry3f map_to_odom{Eigen::Isometry3f::Identity()};
};

// 処理ごとの残り点数。調整と性能の確認に使う
struct SelectionCounts
{
  std::size_t input{0};
  std::size_t after_self{0};
  std::size_t after_height{0};
  std::size_t after_field{0};
  std::size_t after_map{0};
  std::size_t after_outlier{0};
};

// 機体の長さと幅（footprint.length_m・width_m）の半分に余裕（footprint.self_filter_margin_m）を足し、
// 上面を機体の高さ（footprint.height_m）に余裕を足した値にした箱。FAST-LIO の前処理も launch で同じ値を受け取る
SelfFilterParams selfFilterFromFootprint(
  double length_m, double width_m, double height_m, double margin_m);

// 上の箱の xy を、長さと幅の半分の外接円に余裕を足した半径の円柱にする（footprint.self_filter_shape: cylinder）
SelfFilterParams cylinderSelfFilterFromFootprint(
  double length_m, double width_m, double height_m, double margin_m);

// base_link の点が自機の形（箱か円柱）の中にあるか
bool insideSelf(const Eigen::Vector3f & base, const SelfFilterParams & params);

// 自機、地面、フィールド外、固定物、外れ値を除いた点を返す。自機は base_link で xy が箱の中にあり、
// 高さが箱の上面以下の点（LiDAR は機体の前の端の低い所にあるので、LiDAR より上の機体の面も除く）
std::vector<ObstaclePoint> selectObstaclePoints(
  const std::vector<FramePoint> & points, const FrameTransforms & transforms,
  const DetectionParams & params, const StaticMap & static_map, SelectionCounts & counts);

// 外れ値の除去。半径内の近傍が少ない点を除く
std::vector<ObstaclePoint> removeOutliers(
  const std::vector<ObstaclePoint> & points, const OutlierFilterParams & params);

// 点をクラスタにまとめる。sensor はセンサの位置で、奥行きと幅の計算に使う
std::vector<Detection> clusterObstaclePoints(
  const std::vector<ObstaclePoint> & points, const Eigen::Vector2f & sensor,
  const DetectionParams & params);

// 複数の LiDAR の点を合わせた点群のとき。sensors はそれぞれのセンサの位置（空でないこと）で、クラスタ
// （と、つなぐ・まとめる判定の相手の点）ごとに最も近いセンサから見た向きで、奥行きと幅、見えている面の
// 奥の中心を求める。センサが 1 つなら上と同じ結果になる
std::vector<Detection> clusterObstaclePoints(
  const std::vector<ObstaclePoint> & points, const std::vector<Eigen::Vector2f> & sensors,
  const DetectionParams & params);
}  // namespace core_perception

#endif  // CORE_PERCEPTION__DETECTION_HPP_
