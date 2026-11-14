// Gazebo の点群を、実機のドライバと同じ Livox の CustomMsg に変換するノード。
//
// FAST-LIO の設定を実機と揃えるために、入力の形式を合わせる。Gazebo の gpu_lidar は 1 フレームの
// 全点を同じ時刻に取得する。scan_motion が false なら点ごとの時刻 offset_time を 0 にする（時刻だけを
// ばらつかせると、FAST-LIO が存在しない歪みを補正してしまう）。scan_motion が true（シミュレーションの launch の
// 既定、引数 lidar_scan_motion）なら、実機と同じく点の時刻を
// 1 走査（scan_period_s）に散らし（方位角を azimuth_turns_per_scan 周する並び）、その時刻の LiDAR の位置から
// 見えたはずの位置に点を移す。LiDAR の位置は真値（/ground_truth/odom の姿勢、200Hz）を補間して求めるので、
// 加速している間の動きも入る（core_gazebo/scan_motion.hpp）。真値が走査の終わりまで届いてから出すので、
// 実機と同じく、届くのは走査が終わった後になる。自機の車体（body.*）に当たった点は LiDAR と一緒に動くので
// ずらさない（ずらすと自機の除去の箱の外に出て、自機が障害物に見える）。
// 撮った時刻を走査の真ん中に置き、ヘッダの時刻は実機と同じく走査の始め（撮った時刻の scan_period_s / 2 前）に
// する。Gazebo の動く相手は撮った一瞬の位置にいるので、走査の始めに置くと、ヘッダの時刻から見て 1 走査分
// 遅れた位置に見える。実機の相手の点は走査の間に散らばり、平均は真ん中なので、それに合わせる。
//
// 実機は非反復スキャンで毎フレーム違う場所に点が当たるので、密な格子の点群から毎回ランダムに
// 間引いて、実機と同じ点数にする。include_no_return を true にすると、返りのない向きも含めて間引き、
// その点を実機のドライバと同じく (0,0,0) で出す（実機の 1 回の点数には返りのない点も入る）。

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <iterator>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "core_gazebo/scan_motion.hpp"

namespace core_gazebo
{
namespace
{
using livox_ros_driver2::msg::CustomMsg;
using livox_ros_driver2::msg::CustomPoint;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;

// 実機の Mid-360 は line が 0〜3 の 4 本
constexpr std::uint16_t kLineCount = 4;
// Livox の tag で通常点（信頼度が高い点）を表す値
constexpr std::uint8_t kTagNormal = 0x00;
// 最後の点の時刻を、次の走査の始め（次のヘッダの時刻）より少し前にする [s]。検出は姿勢の時刻と点群の
// ヘッダの時刻を 1µs の余裕で比べる（core_perception/pose_timing.hpp）
constexpr double kScanEndGapS = 10e-6;
// 真値の姿勢の記録を残す数（200Hz で 2 秒）
constexpr std::size_t kPoseHistory = 400;
// 真値を待つ点群の数の上限。これを超えたら古い点群を捨てる
constexpr std::size_t kMaxPendingClouds = 5;

bool hasField(const PointCloud2 & cloud, const std::string & name)
{
  return std::any_of(
    cloud.fields.begin(), cloud.fields.end(),
    [&name](const sensor_msgs::msg::PointField & field) {return field.name == name;});
}

std::uint8_t toReflectivity(const float intensity)
{
  return std::isfinite(intensity) ?
         static_cast<std::uint8_t>(std::clamp(intensity, 0.0F, 255.0F)) : std::uint8_t{0};
}

std::uint64_t toNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL + stamp.nanosec;
}

// 他の LiDAR の干渉を模して、周囲に孤立した点を足す
void appendInterference(
  std::vector<CustomPoint> & points, const std::size_t count, const double scan_period_s,
  std::mt19937 & random)
{
  std::uniform_real_distribution<float> horizontal(-15.0F, 15.0F);
  std::uniform_real_distribution<float> vertical(-0.5F, 1.0F);
  std::uniform_real_distribution<double> time(0.0, std::max(0.0, scan_period_s - kScanEndGapS));
  for (std::size_t i = 0; i < count; ++i) {
    CustomPoint point;
    point.x = horizontal(random);
    point.y = horizontal(random);
    point.z = vertical(random);
    point.tag = kTagNormal;
    point.line = 0;
    point.offset_time = static_cast<std::uint32_t>(std::llround(time(random) * 1e9));
    points.push_back(point);
  }
}

// Gazebo の 1 本の光線
struct Ray
{
  float x;
  float y;
  float z;
  float intensity;
  std::uint16_t ring;
  bool valid;
};

struct ConvertOptions
{
  float min_range{0.1F};
  float max_range{40.0F};
  std::size_t target_points{20000};
  std::size_t interference_points{0};
  bool include_no_return{false};
  // 走査の間の機体の姿勢（世界から見た真値、時刻の順）と取り付け。あれば点の時刻を散らし、その時刻の
  // LiDAR から見えたはずの位置に点を移す。走査の始めから終わりまでを含んでいること
  std::optional<std::vector<StampedPose>> base_poses;
  Eigen::Isometry3d base_to_sensor{Eigen::Isometry3d::Identity()};
  // 自機の車体。この中の点は LiDAR と一緒に動くので、ずらさない
  BodyBox body;
  double scan_period_s{0.1};
  int turns_per_scan{18};
};

// 光線を target_points 本に間引いて CustomMsg にする。必要なフィールドがなければ nullopt を返す
std::optional<CustomMsg> toCustomMsg(
  const PointCloud2 & cloud, const ConvertOptions & options, std::mt19937 & random)
{
  if (!hasField(cloud, "x") || !hasField(cloud, "y") || !hasField(cloud, "z") ||
    !hasField(cloud, "intensity") || !hasField(cloud, "ring"))
  {
    return std::nullopt;
  }

  std::vector<Ray> rays;
  rays.reserve(static_cast<std::size_t>(cloud.width) * cloud.height);
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2ConstIterator<float> intensity(cloud, "intensity");
  sensor_msgs::PointCloud2ConstIterator<std::uint16_t> ring(cloud, "ring");
  for (; x != x.end(); ++x, ++y, ++z, ++intensity, ++ring) {
    const float range = std::sqrt((*x) * (*x) + (*y) * (*y) + (*z) * (*z));
    const bool valid =
      std::isfinite(range) && range >= options.min_range && range <= options.max_range;
    if (valid || options.include_no_return) {
      rays.push_back(Ray{*x, *y, *z, *intensity, *ring, valid});
    }
  }
  std::vector<Ray> chosen;
  chosen.reserve(std::min(options.target_points, rays.size()));
  std::sample(rays.begin(), rays.end(), std::back_inserter(chosen), options.target_points, random);

  const bool moving = options.base_poses.has_value();
  const std::uint64_t capture_ns = toNanoseconds(cloud.header.stamp);
  const std::uint64_t period_ns =
    static_cast<std::uint64_t>(std::llround(options.scan_period_s * 1e9));
  CustomMsg output;
  output.header = cloud.header;
  // ヘッダの時刻は走査の始め。撮った時刻は走査の真ん中
  const std::uint64_t half_period_ns = period_ns / 2U;
  output.timebase =
    moving && capture_ns > half_period_ns ? capture_ns - half_period_ns : capture_ns;
  output.header.stamp = rclcpp::Time(static_cast<std::int64_t>(output.timebase), RCL_ROS_TIME);
  output.lidar_id = 0;
  // 世界から見た、時刻 stamp_ns の LiDAR の姿勢
  const auto sensor_at =
    [&options](const std::uint64_t stamp_ns) -> std::optional<Eigen::Isometry3d> {
      const auto base = interpolatePose(*options.base_poses, stamp_ns);
      return base.has_value() ? std::optional<Eigen::Isometry3d>(*base * options.base_to_sensor) :
             std::nullopt;
    };
  const auto sensor_capture = moving ? sensor_at(capture_ns) : std::nullopt;

  std::uniform_int_distribution<int> turn(0, std::max(1, options.turns_per_scan) - 1);
  std::uniform_real_distribution<double> azimuth(-M_PI, M_PI);
  const double last_time = std::max(0.0, options.scan_period_s - kScanEndGapS);
  output.points.reserve(chosen.size() + options.interference_points);
  for (const auto & ray : chosen) {
    CustomPoint point;
    point.tag = kTagNormal;
    point.line = static_cast<std::uint8_t>(ray.ring % kLineCount);
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    if (ray.valid) {
      position = Eigen::Vector3d(ray.x, ray.y, ray.z);
      point.reflectivity = toReflectivity(ray.intensity);
    }
    if (moving) {
      // 返りのない点は向きがないので、時刻だけを振る
      const double direction = ray.valid ? std::atan2(position.y(), position.x()) : azimuth(random);
      const int turns = std::max(1, options.turns_per_scan);
      const double time = std::min(
        sweepTime(direction, turn(random), turns, options.scan_period_s), last_time);
      point.offset_time = static_cast<std::uint32_t>(std::llround(time * 1e9));
      const auto sensor = sensor_at(output.timebase + point.offset_time);
      const bool on_body = insideBody(options.base_to_sensor * position, options.body);
      if (ray.valid && !on_body && sensor.has_value() && sensor_capture.has_value()) {
        position = pointSeenAt(position, *sensor_capture, *sensor);
      }
    }
    point.x = static_cast<float>(position.x());
    point.y = static_cast<float>(position.y());
    point.z = static_cast<float>(position.z());
    output.points.push_back(point);
  }
  appendInterference(
    output.points, options.interference_points, moving ? options.scan_period_s : 0.0, random);
  if (moving) {
    // 実機と同じく時刻の順に並べる
    std::stable_sort(
      output.points.begin(), output.points.end(),
      [](const CustomPoint & a, const CustomPoint & b) {return a.offset_time < b.offset_time;});
  }
  output.point_num = static_cast<std::uint32_t>(output.points.size());
  return output;
}
}  // namespace

class PointsToLivox final : public rclcpp::Node
{
public:
  explicit PointsToLivox(const rclcpp::NodeOptions & options)
  : Node("points_to_livox", options),
    min_range_(static_cast<float>(declare_parameter<double>("min_range_m", 0.1))),
    max_range_(static_cast<float>(declare_parameter<double>("max_range_m", 40.0))),
    // 実機の 200,000点/秒 @ 10Hz
    target_points_(static_cast<std::size_t>(declare_parameter<int>("target_points", 20000))),
    // 検出の外れ値除去を確かめるときだけ使う
    interference_points_(static_cast<std::size_t>(declare_parameter<int>("interference_points", 0))),
    include_no_return_(declare_parameter<bool>("include_no_return", false)),
    scan_motion_(declare_parameter<bool>("scan_motion", false)),
    scan_period_s_(declare_parameter<double>("scan_period_s", 0.1)),
    // 実機（2026-03 の bag）は 1 走査で方位角を 18.0〜18.2 周する
    turns_per_scan_(static_cast<int>(declare_parameter<int>("azimuth_turns_per_scan", 18))),
    base_frame_(declare_parameter<std::string>("base_frame", "base_link")),
    // 自機の車体（URDF の body_length・body_width・body_height）
    body_{
      declare_parameter<double>("body.half_length_m", 0.4),
      declare_parameter<double>("body.half_width_m", 0.4),
      declare_parameter<double>("body.height_m", 0.8),
      declare_parameter<double>("body.margin_m", 0.05)},
    odom_topic_(declare_parameter<std::string>("odom_topic", "/ground_truth/odom")),
    parameter_callback_(add_post_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &) {
          interference_points_ =
          static_cast<std::size_t>(get_parameter("interference_points").as_int());
        })),
    publisher_(create_publisher<CustomMsg>("/sensing/livox/lidar", rclcpp::QoS(10))),
    // 1 メッセージが大きく、Best Effort だと欠落するので Reliable で受ける
    subscription_(create_subscription<PointCloud2>(
        "/sensing/sim/points", rclcpp::QoS(5).reliable(),
        std::bind(&PointsToLivox::onCloud, this, std::placeholders::_1)))
  {
    if (scan_period_s_ <= kScanEndGapS || turns_per_scan_ < 1) {
      throw std::invalid_argument(
              "scan_period_s は 10µs より長く、azimuth_turns_per_scan は 1 以上にしてください");
    }
    if (scan_motion_) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
      odometry_subscription_ = create_subscription<Odometry>(
        odom_topic_, rclcpp::QoS(50),
        [this](const Odometry::ConstSharedPtr & odometry) {onOdometry(*odometry);});
    }
    RCLCPP_INFO(
      get_logger(), "点の時刻と走査の間の動き: %s、返りのない点: %s",
      scan_motion_ ? "付ける" : "付けない（全点 0）", include_no_return_ ? "(0,0,0) で出す" : "出さない");
  }

private:
  void onOdometry(const Odometry & odometry)
  {
    const auto & position = odometry.pose.pose.position;
    const auto & orientation = odometry.pose.pose.orientation;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(position.x, position.y, position.z);
    pose.linear() = Eigen::Quaterniond(orientation.w, orientation.x, orientation.y, orientation.z)
      .normalized().toRotationMatrix();
    const std::uint64_t stamp_ns = toNanoseconds(odometry.header.stamp);
    if (!poses_.empty() && stamp_ns <= poses_.back().stamp_ns) {
      // 時刻が戻ったら（シミュレーションのやり直しなど）記録を捨てる
      if (stamp_ns < poses_.back().stamp_ns) {
        poses_.clear();
      } else {
        return;
      }
    }
    poses_.push_back(StampedPose{stamp_ns, pose});
    while (poses_.size() > kPoseHistory) {
      poses_.pop_front();
    }
    publishReady();
  }

  // 真値が走査の終わりまで届いた点群を、点ごとの時刻と動きを付けて出す
  void publishReady()
  {
    const std::uint64_t half_period_ns =
      static_cast<std::uint64_t>(std::llround(0.5 * scan_period_s_ * 1e9));
    while (!pending_.empty() && !poses_.empty()) {
      const auto cloud = pending_.front();
      const std::uint64_t capture_ns = toNanoseconds(cloud->header.stamp);
      const std::uint64_t start_ns = capture_ns > half_period_ns ? capture_ns - half_period_ns : 0U;
      const std::uint64_t end_ns = capture_ns + half_period_ns;
      if (poses_.back().stamp_ns < end_ns) {
        return;
      }
      pending_.pop_front();
      if (poses_.front().stamp_ns > start_ns) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "走査の始めの真値がないので、この点群は出しません");
        continue;
      }
      if (!base_to_sensor_.has_value()) {
        try {
          base_to_sensor_ = tf2::transformToEigen(
            tf_buffer_->lookupTransform(base_frame_, cloud->header.frame_id, tf2::TimePointZero));
        } catch (const tf2::TransformException & error) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "%s → %s の TF がまだないので、この点群は出しません: %s",
            base_frame_.c_str(), cloud->header.frame_id.c_str(), error.what());
          continue;
        }
      }
      // 走査の始めと終わりを挟む真値だけを渡す
      std::vector<StampedPose> window;
      for (std::size_t i = 0; i < poses_.size(); ++i) {
        const bool next_reaches_start =
          i + 1U < poses_.size() && poses_[i + 1U].stamp_ns >= start_ns;
        const bool previous_before_end = i == 0U || poses_[i - 1U].stamp_ns <= end_ns;
        if ((poses_[i].stamp_ns >= start_ns || next_reaches_start) && previous_before_end) {
          window.push_back(poses_[i]);
        }
      }
      ConvertOptions options = baseOptions();
      options.base_poses = std::move(window);
      options.base_to_sensor = *base_to_sensor_;
      options.body = body_;
      publish(*cloud, options);
    }
  }

  ConvertOptions baseOptions() const
  {
    ConvertOptions options;
    options.min_range = min_range_;
    options.max_range = max_range_;
    options.target_points = target_points_;
    options.interference_points = interference_points_;
    options.include_no_return = include_no_return_;
    options.scan_period_s = scan_period_s_;
    options.turns_per_scan = turns_per_scan_;
    return options;
  }

  void publish(const PointCloud2 & cloud, const ConvertOptions & options)
  {
    auto output = toCustomMsg(cloud, options, random_);
    if (!output.has_value()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "/sim/points に x, y, z, intensity, ring のいずれかがないので変換できません");
      return;
    }
    RCLCPP_INFO_ONCE(
      get_logger(), "1 回の光線 %u 本から %u 点を出します", cloud.width * cloud.height, output->point_num);
    // 同じプロセスの検出（composition:=true）へは、コピーせずにポインタで渡す
    publisher_->publish(std::make_unique<CustomMsg>(std::move(*output)));
  }

  void onCloud(const PointCloud2::ConstSharedPtr & cloud)
  {
    if (!scan_motion_) {
      publish(*cloud, baseOptions());
      return;
    }
    pending_.push_back(cloud);
    while (pending_.size() > kMaxPendingClouds) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "真値（odom_topic）が届かないので、古い点群を捨てます");
      pending_.pop_front();
    }
    publishReady();
  }

  const float min_range_;
  const float max_range_;
  const std::size_t target_points_;
  std::size_t interference_points_;
  const bool include_no_return_;
  const bool scan_motion_;
  const double scan_period_s_;
  const int turns_per_scan_;
  const std::string base_frame_;
  const BodyBox body_;
  const std::string odom_topic_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr parameter_callback_;
  // 同じ入力なら同じ結果になるよう、シードは固定する
  std::mt19937 random_{0U};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::optional<Eigen::Isometry3d> base_to_sensor_;
  std::deque<StampedPose> poses_;
  std::deque<PointCloud2::ConstSharedPtr> pending_;
  rclcpp::Publisher<CustomMsg>::SharedPtr publisher_;
  rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription_;
};
}  // namespace core_gazebo

RCLCPP_COMPONENTS_REGISTER_NODE(core_gazebo::PointsToLivox)
