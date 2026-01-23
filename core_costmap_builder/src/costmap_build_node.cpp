#include "core_costmap_builder/costmap_build_node.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <cmath>
#include <algorithm>

std::size_t VoxelKeyHash::operator()(const VoxelKey &k) const noexcept
{
  std::size_t h1 = std::hash<int32_t>{}(k.ix);
  std::size_t h2 = std::hash<int32_t>{}(k.iy);
  std::size_t h3 = std::hash<int32_t>{}(k.iz);

  std::size_t h = h1;
  h ^= h2 + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
  h ^= h3 + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
  return h;
}

namespace core_costmap_builder
{

  CostmapBuildNode::CostmapBuildNode(const rclcpp::NodeOptions &options)
      : Node("costmap_build_node", options),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
  {
    // =========================
    // Topics
    // =========================
    points_in_topic_ = declare_parameter<std::string>("points_in_topic", "/lidar/points");
    global_topic_ = declare_parameter<std::string>("global_topic", "/costmap/global");

    points_filt_topic_ = declare_parameter<std::string>("points_filt_topic", "/lidar/points_filtered");
    local_topic_ = declare_parameter<std::string>("local_topic", "/costmap/local");
    fused_topic_ = declare_parameter<std::string>("fused_topic", "/costmap/fused");
    local_in_map_topic_ = declare_parameter<std::string>("local_in_map_topic", "/costmap/local_in_map");

    // =========================
    // Frames
    // =========================
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "");

    // =========================
    // Local rolling window
    // =========================
    local_width_m_ = declare_parameter<double>("local_width_m", 10.0);
    local_height_m_ = declare_parameter<double>("local_height_m", 10.0);
    resolution_m_ = declare_parameter<double>("resolution_m", 0.05);
    update_hz_ = declare_parameter<double>("update_hz", 15.0);

    // =========================
    // Filtering（PCLなし、ifだけ）
    // =========================
    crop_xy_m_ = declare_parameter<double>("crop_xy_m", 6.0);
    min_z_m_ = declare_parameter<double>("min_z_m", 0.10);
    max_z_m_ = declare_parameter<double>("max_z_m", 1.60);
    voxel_leaf_m_ = declare_parameter<double>("voxel_leaf_m", 0.05);

    min_range_m_ = declare_parameter<double>("min_range_m", 0.30);
    max_range_m_ = declare_parameter<double>("max_range_m", 6.0);

    // =========================
    // Robot / inflation
    // =========================
    robot_radius_m_ = declare_parameter<double>("robot_radius_m", 0.71);
    inflation_radius_m_ = declare_parameter<double>("inflation_radius_m", 0.90);

    // =========================
    // Behavior
    // =========================
    enable_clearing_ = declare_parameter<bool>("enable_clearing", true);
    points_timeout_sec_ = declare_parameter<double>("points_timeout_sec", 0.2);
    tf_timeout_ms_ = declare_parameter<int>("tf_timeout_ms", 50);

    publish_filtered_points_ = declare_parameter<bool>("publish_filtered_points", true);
    publish_local_in_map_ = declare_parameter<bool>("publish_local_in_map", true);
    max_debug_points_ = declare_parameter<int>("max_debug_points", 20000);

    // =========================
    // Local grid init
    // =========================
    size_x_ = static_cast<int>(std::round(local_width_m_ / resolution_m_));
    size_y_ = static_cast<int>(std::round(local_height_m_ / resolution_m_));
    if (size_x_ <= 0 || size_y_ <= 0)
    {
      throw std::runtime_error("Invalid local grid size. Check local_width/height/resolution.");
    }
    local_grid_.assign(size_x_ * size_y_, UNKNOWN);

    // =========================
    // Sub/Pub
    // =========================
    sub_points_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        points_in_topic_, rclcpp::SensorDataQoS(),
        std::bind(&CostmapBuildNode::onPoints, this, std::placeholders::_1));

    // globalは “最後の1枚を保持して欲しい” ことが多いので transient_local を推奨
    // 送信側QoSと合わない場合はここを変更
    auto global_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    sub_global_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        global_topic_, global_qos,
        std::bind(&CostmapBuildNode::onGlobal, this, std::placeholders::_1));

    pub_local_ = create_publisher<nav_msgs::msg::OccupancyGrid>(local_topic_, 10);
    pub_fused_ = create_publisher<nav_msgs::msg::OccupancyGrid>(fused_topic_, 10);

    pub_points_filt_ = create_publisher<sensor_msgs::msg::PointCloud2>(points_filt_topic_, rclcpp::SensorDataQoS());
    pub_local_in_map_ = create_publisher<nav_msgs::msg::OccupancyGrid>(local_in_map_topic_, 10);

    // update timer（ここで点群処理→local→fusion をまとめて回す）
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, update_hz_));
    timer_ = create_wall_timer(period, std::bind(&CostmapBuildNode::onUpdate, this));

    RCLCPP_INFO(get_logger(),
                "costmap_build_node started: local=%dx%d res=%.3f window=%.1fx%.1fm",
                size_x_, size_y_, resolution_m_, local_width_m_, local_height_m_);
  }

  void CostmapBuildNode::onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    last_points_ = msg;
    //last_points_stamp_ = msg->header.stamp;
    last_points_stamp_ = this->now();
  }

  void CostmapBuildNode::onGlobal(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    global_map_ = msg;
  }

  void CostmapBuildNode::onUpdate()
  {
    // ---------- 0) ロボ位置（odom<-base_link）を取得して rolling window 原点更新 ----------
    double robot_x = 0.0, robot_y = 0.0;
    if (!get2DTranslation(odom_frame_, base_frame_, robot_x, robot_y))
      return;

    local_origin_x_ = robot_x - local_width_m_ * 0.5;
    local_origin_y_ = robot_y - local_height_m_ * 0.5;

    // ---------- 1) 点群鮮度チェック ----------
    if (!last_points_)
    {
      publishLocal();
      publishFused();
      return;
    }
    const double age = (this->now() - last_points_stamp_).seconds();
    if (age > points_timeout_sec_)
    {
      // 点群が止まった時に地図を消すと危険なので「更新しない」方が安全
      publishLocal();
      publishFused();
      return;
    }

    // ---------- 2) 点群を odom に変換 ----------
    sensor_msgs::msg::PointCloud2 points_odom;
    if (!transformPointCloudToFrame(*last_points_, odom_frame_, points_odom))
    {
      publishLocal();
      publishFused();
      return;
    }

    // ---------- 3) センサ原点（clearingの始点） ----------
    double sensor_x = robot_x, sensor_y = robot_y;
    if (!lidar_frame_.empty())
    {
      double lx, ly;
      if (get2DTranslation(odom_frame_, lidar_frame_, lx, ly))
      {
        sensor_x = lx;
        sensor_y = ly;
      }
    }

    // ---------- 4) local grid を毎回 unknown にリセット（最初はこれが一番デバッグ容易） ----------
    std::fill(local_grid_.begin(), local_grid_.end(), UNKNOWN);

    // ---------- 5) voxel set（leafごとに1点だけ採用） ----------
    std::unordered_set<VoxelKey, VoxelKeyHash> voxel_used;
    voxel_used.reserve(50000);

    // デバッグ用フィルタ後点群（上限付き）
    std::vector<float> dbg_x, dbg_y, dbg_z;
    if (publish_filtered_points_)
    {
      dbg_x.reserve(10000);
      dbg_y.reserve(10000);
      dbg_z.reserve(10000);
    }

    // fieldsチェック（x/y/zが無い点群は扱えない）
    if (!hasField(points_odom, "x") || !hasField(points_odom, "y") || !hasField(points_odom, "z"))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "PointCloud2 has no x/y/z fields.");
      publishLocal();
      publishFused();
      return;
    }

    sensor_msgs::PointCloud2ConstIterator<float> it_x(points_odom, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(points_odom, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(points_odom, "z");

    // ---------- 6) 1点ずつフィルタ→costmap反映 ----------
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
    {
      const float x = *it_x;
      const float y = *it_y;
      const float z = *it_z;

      // NaN/Inf除去
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        continue;

      // Crop（ロボ近傍±crop_xy）
      if (std::fabs(static_cast<double>(x) - robot_x) > crop_xy_m_)
        continue;
      if (std::fabs(static_cast<double>(y) - robot_y) > crop_xy_m_)
        continue;

      // Z
      if (z < min_z_m_ || z > max_z_m_)
        continue;

      // Range
      const double dx = static_cast<double>(x) - sensor_x;
      const double dy = static_cast<double>(y) - sensor_y;
      const double r = std::hypot(dx, dy);
      if (r < min_range_m_ || r > max_range_m_)
        continue;

      // Voxel（leafで整数格子化して重複排除）
      const int32_t ix = static_cast<int32_t>(std::floor(static_cast<double>(x) / voxel_leaf_m_));
      const int32_t iy = static_cast<int32_t>(std::floor(static_cast<double>(y) / voxel_leaf_m_));
      const int32_t iz = static_cast<int32_t>(std::floor(static_cast<double>(z) / voxel_leaf_m_));
      VoxelKey key{ix, iy, iz};
      if (voxel_used.find(key) != voxel_used.end())
        continue;
      voxel_used.insert(key);

      // debug点群に保存（上限あり）
      if (publish_filtered_points_ && static_cast<int>(dbg_x.size()) < max_debug_points_)
      {
        dbg_x.push_back(x);
        dbg_y.push_back(y);
        dbg_z.push_back(z);
      }

      // clearing（射線上のunknownをfree化。occupiedは消さない）
      if (enable_clearing_)
      {
        raytraceFree(sensor_x, sensor_y, x, y);
      }

      // marking（終点をoccupied）
      setOccupied(x, y);
    }

    // ---------- 7) inflation ----------
    applyInflation();

    // ---------- 8) publish ----------
    publishLocal();
    if (publish_filtered_points_)
      publishFilteredPoints(dbg_x, dbg_y, dbg_z);
    publishFused();
  }

  bool CostmapBuildNode::hasField(const sensor_msgs::msg::PointCloud2 &cloud, const std::string &name) const
  {
    for (const auto &f : cloud.fields)
    {
      if (f.name == name)
        return true;
    }
    return false;
  }

  bool CostmapBuildNode::get2DTranslation(const std::string &target, const std::string &source, double &x, double &y)
  {
    try
    {
      const auto tf = tf_buffer_.lookupTransform(
          target, source, tf2::TimePointZero,
          std::chrono::milliseconds(tf_timeout_ms_));
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TF failed (%s <- %s): %s", target.c_str(), source.c_str(), ex.what());
      return false;
    }
  }

  bool CostmapBuildNode::transformPointCloudToFrame(const sensor_msgs::msg::PointCloud2 &in,
                                                    const std::string &target_frame,
                                                    sensor_msgs::msg::PointCloud2 &out)
  {
    try
    {
      const auto tf = tf_buffer_.lookupTransform(
          target_frame, in.header.frame_id, tf2::TimePointZero,
          std::chrono::milliseconds(tf_timeout_ms_));
      tf2::doTransform(in, out, tf);
      out.header.frame_id = target_frame;
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TF cloud failed (%s -> %s): %s", in.header.frame_id.c_str(), target_frame.c_str(), ex.what());
      return false;
    }
  }

  bool CostmapBuildNode::worldToLocalCell(double wx, double wy, int &ix, int &iy) const
  {
    const double gx = (wx - local_origin_x_) / resolution_m_;
    const double gy = (wy - local_origin_y_) / resolution_m_;
    ix = static_cast<int>(std::floor(gx));
    iy = static_cast<int>(std::floor(gy));
    return (0 <= ix && ix < size_x_ && 0 <= iy && iy < size_y_);
  }

  void CostmapBuildNode::setOccupied(double wx, double wy)
  {
    int ix, iy;
    if (!worldToLocalCell(wx, wy, ix, iy))
      return;
    local_grid_[lidx(ix, iy)] = LETHAL;
  }

  void CostmapBuildNode::raytraceFree(double sx, double sy, double ex, double ey)
  {
    int x0, y0, x1, y1;
    if (!worldToLocalCell(sx, sy, x0, y0))
      return;

    // NOTE: 簡略化：終点が窓外ならスキップ（後で線分クリッピング入れると強い）
    if (!worldToLocalCell(ex, ey, x1, y1))
      return;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx_step = (x0 < x1) ? 1 : -1;
    int sy_step = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int x = x0, y = y0;
    while (!(x == x1 && y == y1))
    {
      auto &cell = local_grid_[lidx(x, y)];
      if (cell == UNKNOWN)
        cell = FREE; // occupiedは消さない

      int e2 = 2 * err;
      if (e2 > -dy)
      {
        err -= dy;
        x += sx_step;
      }
      if (e2 < dx)
      {
        err += dx;
        y += sy_step;
      }

      if (x < 0 || y < 0 || x >= size_x_ || y >= size_y_)
        break;
    }
  }

  void CostmapBuildNode::applyInflation()
  {
    const int r_infl = static_cast<int>(std::ceil(inflation_radius_m_ / resolution_m_));
    if (r_infl <= 0)
      return;

    // occupiedセルを列挙
    std::vector<std::pair<int, int>> occ;
    occ.reserve(1024);
    for (int y = 0; y < size_y_; ++y)
    {
      for (int x = 0; x < size_x_; ++x)
      {
        if (local_grid_[lidx(x, y)] == LETHAL)
          occ.emplace_back(x, y);
      }
    }

    // 各occupied周りを塗る（ナイーブ版）
    for (const auto &c : occ)
    {
      const int cx = c.first, cy = c.second;

      const int x_min = std::max(0, cx - r_infl);
      const int x_max = std::min(size_x_ - 1, cx + r_infl);
      const int y_min = std::max(0, cy - r_infl);
      const int y_max = std::min(size_y_ - 1, cy + r_infl);

      for (int y = y_min; y <= y_max; ++y)
      {
        for (int x = x_min; x <= x_max; ++x)
        {
          const double dist = std::hypot(x - cx, y - cy) * resolution_m_;
          if (dist > inflation_radius_m_)
            continue;

          if (dist <= robot_radius_m_)
          {
            local_grid_[lidx(x, y)] = LETHAL;
            continue;
          }

          // 線形減衰（指数にしたければここを変更）
          const double t = (inflation_radius_m_ - dist) / (inflation_radius_m_ - robot_radius_m_);
          const int cost = static_cast<int>(std::round(t * (LETHAL - 1))); // 1..99

          auto &cell = local_grid_[lidx(x, y)];
          if (cell == UNKNOWN)
            cell = static_cast<int8_t>(cost);
          else
            cell = std::max<int8_t>(cell, static_cast<int8_t>(cost));
        }
      }
    }
  }

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

  void CostmapBuildNode::publishFilteredPoints(const std::vector<float> &xs,
                                               const std::vector<float> &ys,
                                               const std::vector<float> &zs)
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

    for (size_t i = 0; i < xs.size(); ++i, ++it_x, ++it_y, ++it_z)
    {
      *it_x = xs[i];
      *it_y = ys[i];
      *it_z = zs[i];
    }
    pub_points_filt_->publish(cloud);
  }

  void CostmapBuildNode::publishFused()
  {
    if (!global_map_)
      return;

    const auto &G = *global_map_;

    // map<-odom TF
    geometry_msgs::msg::TransformStamped tf_map_odom;
    try
    {
      tf_map_odom = tf_buffer_.lookupTransform(
          map_frame_, odom_frame_, tf2::TimePointZero,
          std::chrono::milliseconds(tf_timeout_ms_));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TF failed (%s <- %s): %s", map_frame_.c_str(), odom_frame_.c_str(), ex.what());
      return;
    }

    // 2D変換：yaw + translation だけ使う（local->map投影には十分）
    const auto &q = tf_map_odom.transform.rotation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const auto &t = tf_map_odom.transform.translation;

    // fused = global copy
    nav_msgs::msg::OccupancyGrid fused = G;
    fused.header.stamp = now();

    nav_msgs::msg::OccupancyGrid local_in_map;
    if (publish_local_in_map_)
    {
      local_in_map = G;
      local_in_map.header.stamp = now();
      std::fill(local_in_map.data.begin(), local_in_map.data.end(), UNKNOWN);
    }

    const double gres = G.info.resolution;
    const double g_origin_x = G.info.origin.position.x;
    const double g_origin_y = G.info.origin.position.y;
    const int gW = static_cast<int>(G.info.width);
    const int gH = static_cast<int>(G.info.height);

    for (int ly = 0; ly < size_y_; ++ly)
    {
      for (int lx = 0; lx < size_x_; ++lx)
      {
        const int8_t L = local_grid_[lidx(lx, ly)];
        if (L == UNKNOWN)
          continue;

        // localセル中心（odom）
        const double x_odom = local_origin_x_ + (lx + 0.5) * resolution_m_;
        const double y_odom = local_origin_y_ + (ly + 0.5) * resolution_m_;

        // mapへ投影
        const double x_map = c * x_odom - s * y_odom + t.x;
        const double y_map = s * x_odom + c * y_odom + t.y;

        // global index
        const int gx = static_cast<int>(std::floor((x_map - g_origin_x) / gres));
        const int gy = static_cast<int>(std::floor((y_map - g_origin_y) / gres));
        if (gx < 0 || gy < 0 || gx >= gW || gy >= gH)
          continue;

        const int gidx = gy * gW + gx;

        // 合成（unknownルール + max）
        const int8_t Gv = fused.data[gidx];
        fused.data[gidx] = (Gv == UNKNOWN) ? L : std::max<int8_t>(Gv, L);

        if (publish_local_in_map_)
          local_in_map.data[gidx] = L;
      }
    }

    pub_fused_->publish(fused);
    if (publish_local_in_map_)
      pub_local_in_map_->publish(local_in_map);
  }

} // namespace core_costmap_builder

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<core_costmap_builder::CostmapBuildNode>());
  rclcpp::shutdown();
  return 0;
}