// Copyright 2026 CoRE2026 Team
//
// ローカルコストマップ生成ノードの実装
// LiDAR点群を受信し、ボクセルフィルタ → マーキング → インフレーション の
// パイプラインでローリングウィンドウ型コストマップを構築・配信する
//

#include "core_costmap_builder/costmap_build_node.hpp"

#include <algorithm>
#include <cmath>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

// ---------------------------------------------------------------------------
// VoxelKey のハッシュ関数
// boost::hash_combine 同等のビット混合で ix, iy, iz を結合する
// ---------------------------------------------------------------------------
std::size_t VoxelKeyHash::operator()(const VoxelKey & k) const noexcept
{
  std::size_t h = std::hash<int32_t>{}(k.ix);
  h ^= std::hash<int32_t>{}(k.iy) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
  h ^= std::hash<int32_t>{}(k.iz) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
  return h;
}

namespace core_costmap_builder
{

// ===========================================================================
// コンストラクタ
// パラメータ読み込み → グリッド初期化 → カーネル事前計算 → Sub/Pub 生成
// ===========================================================================
CostmapBuildNode::CostmapBuildNode(const rclcpp::NodeOptions & options)
: Node("costmap_build_node", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  // ---------- トピック名 ----------
  points_in_topic_ = declare_parameter<std::string>("points_in_topic", "/lidar/points");
  points_filt_topic_ =
    declare_parameter<std::string>("points_filt_topic", "/lidar/points_filtered");
  local_topic_ = declare_parameter<std::string>("local_topic", "/costmap/local");

  // ---------- フレーム名 ----------
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  lidar_frame_ = declare_parameter<std::string>("lidar_frame", "");

  // ---------- ローリングウィンドウ設定 ----------
  local_width_m_ = declare_parameter<double>("local_width_m", 10.0);
  local_height_m_ = declare_parameter<double>("local_height_m", 10.0);
  resolution_m_ = declare_parameter<double>("resolution_m", 0.05);
  update_hz_ = declare_parameter<double>("update_hz", 15.0);

  // ---------- 点群フィルタリングパラメータ ----------
  crop_xy_m_ = declare_parameter<double>("crop_xy_m", 6.0);
  min_z_m_ = declare_parameter<double>("min_z_m", 0.10);
  max_z_m_ = declare_parameter<double>("max_z_m", 1.60);
  voxel_leaf_m_ = declare_parameter<double>("voxel_leaf_m", 0.05);
  min_range_m_ = declare_parameter<double>("min_range_m", 0.30);
  max_range_m_ = declare_parameter<double>("max_range_m", 6.0);

  // ---------- ロボット形状 / インフレーション ----------
  robot_radius_m_ = declare_parameter<double>("robot_radius_m", 0.71);
  inflation_radius_m_ = declare_parameter<double>("inflation_radius_m", 0.90);

  // inflation_radius_m_ が robot_radius_m_ 以下だと range_diff が 0 以下になり
  // ゼロ除算が発生するため、最低でも 1 セル分の余裕を持たせる
  if (inflation_radius_m_ <= robot_radius_m_) {
    inflation_radius_m_ = robot_radius_m_ + resolution_m_;
    RCLCPP_WARN(get_logger(),
      "inflation_radius_m (%.3f) <= robot_radius_m (%.3f): "
      "auto-corrected to %.3f",
      inflation_radius_m_ - resolution_m_, robot_radius_m_, inflation_radius_m_);
  }

  // ---------- 動作パラメータ ----------
  points_timeout_sec_ = declare_parameter<double>("points_timeout_sec", 0.2);
  tf_timeout_ms_ = declare_parameter<int>("tf_timeout_ms", 50);
  publish_filtered_points_ = declare_parameter<bool>("publish_filtered_points", true);
  max_debug_points_ = declare_parameter<int>("max_debug_points", 20000);

  // ---------- ローカルグリッド初期化 ----------
  // 幅・高さをセル数に変換し、全セルを FREE で埋める
  size_x_ = static_cast<int>(std::round(local_width_m_ / resolution_m_));
  size_y_ = static_cast<int>(std::round(local_height_m_ / resolution_m_));
  if (size_x_ <= 0 || size_y_ <= 0) {
    throw std::runtime_error("Invalid local grid size. Check local_width/height/resolution.");
  }
  local_grid_.assign(size_x_ * size_y_, FREE);

  // ---------- インフレーションカーネルの事前計算 ----------
  // ロボットから inflation_radius_m_ 以内のセルに対してコスト値を計算し、
  // テーブルに保存しておく。毎サイクルはこのテーブルを走査するだけで済む
  min_range_sq_ = min_range_m_ * min_range_m_;
  max_range_sq_ = max_range_m_ * max_range_m_;
  {
    const int r_infl = static_cast<int>(std::ceil(inflation_radius_m_ / resolution_m_));
    const double infl_sq = inflation_radius_m_ * inflation_radius_m_;
    const double robot_sq = robot_radius_m_ * robot_radius_m_;
    const double range_diff = inflation_radius_m_ - robot_radius_m_;

    for (int dy = -r_infl; dy <= r_infl; ++dy) {
      for (int dx = -r_infl; dx <= r_infl; ++dx) {
        // 自分自身のセルはスキップ（occupied で既に LETHAL）
        if (dx == 0 && dy == 0) {continue;}

        const double dist_sq = (dx * dx + dy * dy) * resolution_m_ * resolution_m_;
        if (dist_sq > infl_sq) {continue;}

        int8_t cost;
        if (dist_sq <= robot_sq) {
          // ロボット半径以内 → 致命的コスト
          cost = LETHAL;
        } else {
          // ロボット半径〜インフレーション半径 → 線形減衰コスト
          const double dist = std::sqrt(dist_sq);
          const double t = (inflation_radius_m_ - dist) / range_diff;
          cost = static_cast<int8_t>(std::round(t * (LETHAL - 1)));
        }
        inflation_kernel_.push_back({dx, dy, cost});
      }
    }
  }

  // 再利用バッファの初期容量を確保（ヒープ再確保を抑制）
  voxel_used_.reserve(50000);
  occ_cells_.reserve(2048);

  // ---------- サブスクライバ / パブリッシャ ----------
  sub_points_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    points_in_topic_, rclcpp::SensorDataQoS(),
    std::bind(&CostmapBuildNode::onPoints, this, std::placeholders::_1));

  pub_local_ = create_publisher<nav_msgs::msg::OccupancyGrid>(local_topic_, 10);
  pub_points_filt_ =
    create_publisher<sensor_msgs::msg::PointCloud2>(points_filt_topic_, rclcpp::SensorDataQoS());

  // ---------- 更新タイマー ----------
  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, update_hz_));
  timer_ = create_wall_timer(period, std::bind(&CostmapBuildNode::onUpdate, this));

  RCLCPP_INFO(
    get_logger(), "costmap_build_node started: local=%dx%d res=%.3f window=%.1fx%.1fm",
    size_x_, size_y_, resolution_m_, local_width_m_, local_height_m_);
}

// ===========================================================================
// 点群受信コールバック
// 受信した点群を保持し、受信時刻を記録する
//
// 注意: last_points_stamp_ には this->now() を使用する
// 理由: Livox Mid-360 ドライバは msg->header.stamp にデバイス起動時刻起算の
//       タイムスタンプ（例: sec: 4550 ≈ 75分）を設定するため、システム時刻
//       this->now()（例: sec: 1.77×10⁹ ≈ 2026年）との差分が約56年になる。
//       msg->header.stamp を使用すると、鮮度判定で常にタイムアウト扱いとなり
//       点群が利用できなくなる。this->now() で「データ受信時刻」を記録する
//       ことで正しく動作する。
// ===========================================================================
void CostmapBuildNode::onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_points_ = msg;
  last_points_stamp_ = this->now();
}

// ===========================================================================
// コストマップ更新メインループ（タイマーコールバック）
// 毎サイクル:
//   ロボ位置取得 → 点群鮮度確認 → odom変換 → フィルタ → マーク → 膨張 → 配信
// ===========================================================================
void CostmapBuildNode::onUpdate()
{
  // --- ロボット位置を取得してローリングウィンドウの原点を更新 ---
  double robot_x = 0.0, robot_y = 0.0;
  if (!get2DTranslation(odom_frame_, base_frame_, robot_x, robot_y)) {
    return;  // TFが取れなければ何もしない
  }
  local_origin_x_ = robot_x - local_width_m_ * 0.5;
  local_origin_y_ = robot_y - local_height_m_ * 0.5;

  // --- 点群の鮮度チェック ---
  // 点群がまだ来ていない、またはタイムアウトしている場合は
  // グリッドを更新せずにそのまま配信する（安全側に倒す）
  if (!last_points_) {
    publishLocal();
    return;
  }
  const double age = (this->now() - last_points_stamp_).seconds();
  if (age > points_timeout_sec_) {
    publishLocal();
    return;
  }

  // --- 点群を odom フレームに座標変換 ---
  sensor_msgs::msg::PointCloud2 points_odom;
  if (!transformPointCloudToFrame(*last_points_, odom_frame_, points_odom)) {
    publishLocal();
    return;
  }

  // --- センサ原点の取得（距離フィルタの基準点） ---
  double sensor_x = robot_x, sensor_y = robot_y;
  if (!lidar_frame_.empty()) {
    double lx, ly;
    if (get2DTranslation(odom_frame_, lidar_frame_, lx, ly)) {
      sensor_x = lx;
      sensor_y = ly;
    }
  }

  // --- グリッドを全セルFREEにリセット ---
  std::fill(local_grid_.begin(), local_grid_.end(), FREE);
  occ_cells_.clear();

  // --- ボクセル重複排除セットをクリア（バケット配列は再利用） ---
  voxel_used_.clear();

  // --- デバッグ用フィルタ後点群バッファ ---
  std::vector<float> dbg_x, dbg_y, dbg_z;
  if (publish_filtered_points_) {
    dbg_x.reserve(10000);
    dbg_y.reserve(10000);
    dbg_z.reserve(10000);
  }

  // --- PointCloud2 のフィールド確認 ---
  if (!hasField(points_odom, "x") || !hasField(points_odom, "y") || !hasField(points_odom, "z")) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "PointCloud2 has no x/y/z fields.");
    publishLocal();
    return;
  }

  // --- 点群イテレータ ---
  sensor_msgs::PointCloud2ConstIterator<float> it_x(points_odom, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(points_odom, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(points_odom, "z");

  // --- 1点ずつフィルタしてコストマップに反映 ---
  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    const float x = *it_x;
    const float y = *it_y;
    const float z = *it_z;

    // NaN / Inf を除去
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    // XYクロップ: ロボット中心から ±crop_xy_m_ の範囲外は捨てる
    if (std::fabs(static_cast<double>(x) - robot_x) > crop_xy_m_) {
      continue;
    }
    if (std::fabs(static_cast<double>(y) - robot_y) > crop_xy_m_) {
      continue;
    }

    // 高さフィルタ: odom フレームでの Z 範囲外は捨てる
    if (z < min_z_m_ || z > max_z_m_) {
      continue;
    }

    // 距離フィルタ: センサからの2D距離の二乗で比較（sqrt回避）
    const double dx = static_cast<double>(x) - sensor_x;
    const double dy = static_cast<double>(y) - sensor_y;
    const double rsq = dx * dx + dy * dy;
    if (rsq < min_range_sq_ || rsq > max_range_sq_) {
      continue;
    }

    // ボクセルダウンサンプリング: 同一ボクセル内で最初の1点のみ通す
    const int32_t vx = static_cast<int32_t>(std::floor(static_cast<double>(x) / voxel_leaf_m_));
    const int32_t vy = static_cast<int32_t>(std::floor(static_cast<double>(y) / voxel_leaf_m_));
    const int32_t vz = static_cast<int32_t>(std::floor(static_cast<double>(z) / voxel_leaf_m_));
    if (!voxel_used_.emplace(VoxelKey{vx, vy, vz}).second) {
      continue;
    }

    // デバッグ点群に追加（上限付き）
    if (publish_filtered_points_ && static_cast<int>(dbg_x.size()) < max_debug_points_) {
      dbg_x.push_back(x);
      dbg_y.push_back(y);
      dbg_z.push_back(z);
    }

    // セルを LETHAL にマーキング（occ_cells_ にも登録）
    setOccupied(x, y);
  }

  // --- 事前計算カーネルによるインフレーション ---
  applyInflation();

  // --- コストマップ配信 ---
  publishLocal();
  if (publish_filtered_points_) {
    publishFilteredPoints(dbg_x, dbg_y, dbg_z);
  }
}

// ===========================================================================
// PointCloud2 に指定名フィールドが含まれるか確認する
// ===========================================================================
bool CostmapBuildNode::hasField(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & name) const
{
  for (const auto & f : cloud.fields) {
    if (f.name == name) {
      return true;
    }
  }
  return false;
}

// ===========================================================================
// 2フレーム間の2D並進ベクトル (x, y) をTFから取得する
// 取得できなかった場合は false を返す
// ===========================================================================
bool CostmapBuildNode::get2DTranslation(
  const std::string & target, const std::string & source,
  double & x, double & y)
{
  try {
    const auto tf = tf_buffer_.lookupTransform(
      target, source, tf2::TimePointZero,
      std::chrono::milliseconds(tf_timeout_ms_));
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "TF failed (%s <- %s): %s",
      target.c_str(), source.c_str(), ex.what());
    return false;
  }
}

// ===========================================================================
// 点群全体を指定フレームに座標変換する
// tf2::doTransform を使用し、失敗した場合は false を返す
// ===========================================================================
bool CostmapBuildNode::transformPointCloudToFrame(
  const sensor_msgs::msg::PointCloud2 & in,
  const std::string & target_frame,
  sensor_msgs::msg::PointCloud2 & out)
{
  try {
    const auto tf = tf_buffer_.lookupTransform(
      target_frame, in.header.frame_id, tf2::TimePointZero,
      std::chrono::milliseconds(tf_timeout_ms_));
    tf2::doTransform(in, out, tf);
    out.header.frame_id = target_frame;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "TF cloud failed (%s -> %s): %s",
      in.header.frame_id.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

// ===========================================================================
// ワールド座標 → ローカルグリッドのセル座標変換
// グリッド外の場合は false を返す
// ===========================================================================
bool CostmapBuildNode::worldToLocalCell(double wx, double wy, int & ix, int & iy) const
{
  const double gx = (wx - local_origin_x_) / resolution_m_;
  const double gy = (wy - local_origin_y_) / resolution_m_;
  ix = static_cast<int>(std::floor(gx));
  iy = static_cast<int>(std::floor(gy));
  return 0 <= ix && ix < size_x_ && 0 <= iy && iy < size_y_;
}

// ===========================================================================
// 指定ワールド座標のセルを致命的障害物 (LETHAL) にマーキングし、
// 占有セル一覧 (occ_cells_) にインデックスを登録する
// ===========================================================================
void CostmapBuildNode::setOccupied(double wx, double wy)
{
  int ix, iy;
  if (!worldToLocalCell(wx, wy, ix, iy)) {
    return;
  }
  local_grid_[lidx(ix, iy)] = LETHAL;
  occ_cells_.emplace_back(ix, iy);
}

// ===========================================================================
// インフレーション処理
// occ_cells_ に登録された各占有セルに対し、事前計算済みカーネル
// (inflation_kernel_) を適用して周囲のセルにコスト値を書き込む
// 同一セルに複数の影響がある場合は最大値を採用する
// ===========================================================================
void CostmapBuildNode::applyInflation()
{
  for (const auto & [cx, cy] : occ_cells_) {
    for (const auto & k : inflation_kernel_) {
      const int nx = cx + k.dx;
      const int ny = cy + k.dy;
      if (nx < 0 || nx >= size_x_ || ny < 0 || ny >= size_y_) {continue;}
      auto & cell = local_grid_[lidx(nx, ny)];
      cell = std::max(cell, k.cost);
    }
  }
}

// ===========================================================================
// ローカルコストマップを OccupancyGrid メッセージとしてパブリッシュする
// ===========================================================================
void CostmapBuildNode::publishLocal()
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = now();
  msg.header.frame_id = odom_frame_;

  msg.info.resolution = static_cast<float>(resolution_m_);
  msg.info.width = static_cast<uint32_t>(size_x_);
  msg.info.height = static_cast<uint32_t>(size_y_);

  msg.info.origin.position.x = local_origin_x_;
  msg.info.origin.position.y = local_origin_y_;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.data = local_grid_;
  pub_local_->publish(msg);
}

// ===========================================================================
// フィルタ後の点群をデバッグ用にパブリッシュする
// xyz のみの PointCloud2 を構築して配信する
// ===========================================================================
void CostmapBuildNode::publishFilteredPoints(
  const std::vector<float> & xs,
  const std::vector<float> & ys,
  const std::vector<float> & zs)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = now();
  cloud.header.frame_id = odom_frame_;

  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(xs.size());

  sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");

  for (size_t i = 0; i < xs.size(); ++i, ++it_x, ++it_y, ++it_z) {
    *it_x = xs[i];
    *it_y = ys[i];
    *it_z = zs[i];
  }
  pub_points_filt_->publish(cloud);
}

}  // namespace core_costmap_builder

// ===========================================================================
// エントリポイント
// ===========================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<core_costmap_builder::CostmapBuildNode>());
  rclcpp::shutdown();
  return 0;
}
