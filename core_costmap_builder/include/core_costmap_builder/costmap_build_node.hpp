// Copyright 2026 CoRE2026 Team
//
// ローカルコストマップ生成ノードのヘッダファイル
// LiDAR点群からローリングウィンドウ方式のコストマップを構築する
//
#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// ボクセルダウンサンプリング用の格子キー
// 3D空間を voxel_leaf_m 刻みの整数格子に分割し、
// 同一ボクセル内の重複点を排除するために使用する
struct VoxelKey
{
  int32_t ix;
  int32_t iy;
  int32_t iz;

  bool operator==(const VoxelKey & o) const {return ix == o.ix && iy == o.iy && iz == o.iz;}
};

// VoxelKey のハッシュ関数（unordered_set で使用）
// boost::hash_combine と同等のビット混合を行う
struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & k) const noexcept;
};

// インフレーション（膨張）カーネルの1要素
// 占有セルからの相対オフセット (dx, dy) と、そのセルに書き込むコスト値を保持する
// 起動時に一度だけ計算し、毎サイクルはテーブル参照のみで済ませる
struct InflationCell
{
  int dx, dy;
  int8_t cost;
};

namespace core_costmap_builder
{

/// @brief ローカルコストマップ生成ノード
///
/// 処理フロー:
///   1. LiDAR点群を受信してバッファに保持
///   2. タイマー（update_hz）ごとに onUpdate() を実行
///   3. 点群を odom フレームに変換
///   4. クロップ / 高さ / 距離 / ボクセル でフィルタリング
///   5. 通過した点を occupied としてグリッドにマーキング
///   6. 事前計算済みカーネルでインフレーション
///   7. OccupancyGrid をパブリッシュ
class CostmapBuildNode : public rclcpp::Node
{
public:
  explicit CostmapBuildNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // --- ROSコールバック ---

  /// @brief 点群受信コールバック。最新の点群をバッファに保存する
  void onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /// @brief タイマーコールバック。コストマップの更新処理を実行する
  void onUpdate();

  // --- TFユーティリティ ---

  /// @brief 2フレーム間の2D並進（x, y）を取得する
  bool get2DTranslation(
    const std::string & target, const std::string & source, double & x,
    double & y);

  /// @brief 点群を指定フレームに座標変換する
  bool transformPointCloudToFrame(
    const sensor_msgs::msg::PointCloud2 & in,
    const std::string & target_frame,
    sensor_msgs::msg::PointCloud2 & out);

  // --- 点群ユーティリティ ---

  /// @brief PointCloud2 に指定名のフィールドが存在するか確認する
  bool hasField(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name) const;

  // --- グリッド操作 ---

  /// @brief ワールド座標 (wx, wy) をローカルグリッドのセル座標に変換する
  bool worldToLocalCell(double wx, double wy, int & ix, int & iy) const;

  /// @brief セル座標 (ix, iy) を1次元インデックスに変換する（行優先）
  inline int lidx(int ix, int iy) const {return iy * size_x_ + ix;}

  /// @brief 指定ワールド座標のセルを LETHAL にし、occ_cells_ に追加する
  void setOccupied(double wx, double wy);

  /// @brief 事前計算カーネルを使って占有セル周囲にコストを膨張させる
  void applyInflation();

  // --- パブリッシュ ---

  /// @brief ローカルコストマップ (OccupancyGrid) をパブリッシュする
  void publishLocal();

  /// @brief フィルタ後の点群をデバッグ用にパブリッシュする
  void publishFilteredPoints(
    const std::vector<float> & xs, const std::vector<float> & ys,
    const std::vector<float> & zs);

private:
  // --- OccupancyGrid のセル値定数 ---
  static constexpr int8_t UNKNOWN = -1;  // 未知
  static constexpr int8_t FREE = 0;      // 通行可能
  static constexpr int8_t LETHAL = 100;  // 致命的障害物

  // --- TF ---
  tf2_ros::Buffer tf_buffer_;               // TF変換のバッファ
  tf2_ros::TransformListener tf_listener_;  // TFリスナー

  // --- サブスクライバ / パブリッシャ ---
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;    // 入力点群
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_local_;         // コストマップ
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_filt_;  // デバッグ点群
  rclcpp::TimerBase::SharedPtr timer_;  // 更新タイマー

  // --- トピック名 / フレーム名 ---
  std::string points_in_topic_;                        // 入力点群トピック
  std::string points_filt_topic_, local_topic_;        // 出力トピック
  std::string odom_frame_, base_frame_, lidar_frame_;  // 座標フレーム

  // --- パラメータ ---
  double local_width_m_{10.0};       // ローリングウィンドウの幅 [m]
  double local_height_m_{10.0};      // ローリングウィンドウの高さ [m]
  double resolution_m_{0.05};        // セル解像度 [m/cell]
  double update_hz_{15.0};           // 更新周波数 [Hz]

  double crop_xy_m_{6.0};            // ロボット中心からのXYクロップ範囲 [m]
  double min_z_m_{0.10};             // 有効な点の最小Z高さ [m]（odomフレーム基準）
  double max_z_m_{1.60};             // 有効な点の最大Z高さ [m]
  double voxel_leaf_m_{0.05};        // ボクセルダウンサンプリングの格子サイズ [m]
  double min_range_m_{0.30};         // センサからの最小有効距離 [m]
  double max_range_m_{6.0};          // センサからの最大有効距離 [m]

  double robot_radius_m_{0.71};      // ロボット半径 [m]（この内側は LETHAL）
  double inflation_radius_m_{0.90};  // インフレーション半径 [m]

  double points_timeout_sec_{0.2};   // 点群タイムアウト [秒]
  int tf_timeout_ms_{50};            // TF待ちタイムアウト [ms]

  bool publish_filtered_points_{true};  // デバッグ用フィルタ点群を出すか
  int max_debug_points_{20000};         // デバッグ点群の最大点数

  double min_range_sq_{0.09};   // min_range_m_ の二乗（高速比較用）
  double max_range_sq_{36.0};   // max_range_m_ の二乗（高速比較用）

  // --- ローカルグリッド ---
  int size_x_{0}, size_y_{0};                          // グリッドのセル数
  double local_origin_x_{0.0}, local_origin_y_{0.0};   // グリッド原点のワールド座標
  std::vector<int8_t> local_grid_;                      // コストマップ本体

  // --- 事前計算済みバッファ（毎サイクル再利用） ---
  std::vector<InflationCell> inflation_kernel_;            // インフレーションカーネル
  std::unordered_set<VoxelKey, VoxelKeyHash> voxel_used_;  // ボクセル重複排除セット
  std::vector<std::pair<int, int>> occ_cells_;             // 今サイクルの占有セル一覧

  // --- 最新データ ---
  sensor_msgs::msg::PointCloud2::SharedPtr last_points_;  // 最新の受信点群
  builtin_interfaces::msg::Time last_points_stamp_{};     // 受信時刻
};

}  // namespace core_costmap_builder
