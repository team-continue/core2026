// 自動の速度指令の並進の向きを、IMU の速い周期で世界の座標に保って出し直すノード（core_planning/yaw_hold.hpp）。
//
// 入出力:
//   input_topic（既定 /control/auto/cmd_vel_raw）   安全停止（safety_gate）の後の指令（機体の座標、約 10Hz）
//   imu_topic（既定 /sensing/livox/imu）            上部の IMU。角速度を base_link の z 軸まわりに直して積む
//   output_topic（既定 /control/auto/cmd_vel）      出し直した指令（指令を受けたときと、IMU を受けるたび）
//   frame_topic（既定 /planning/cmd_vel_nav/frame）  局所計画の指令と、それを機体の座標に直した時刻（TwistStamped）。
//                                                    受けた指令と向きが同じものがあれば、その時刻の向きから回った分を
//                                                    戻す（局所計画から届くまでの遅れの間に回った分も戻す）。なければ
//                                                    （安全停止が作った指令など）受けた時の向きから戻す
// IMU の座標から base_link への回転は TF（静的）から取る。取れるまでと、IMU が来ないときは、指令をそのまま出す。
// 指令が timeout_s 来なければ出し直さない（足回りが自分のタイムアウトで止まる）。

#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "core_nav_common/parameters.hpp"
#include "core_planning/yaw_hold.hpp"

namespace core_planning
{
namespace
{
// 回転 q（base_link ← IMU）の行列の 3 行目。IMU の角速度との内積が base_link の z 軸まわりの角速度
struct ZRow
{
  double x{0.0};
  double y{0.0};
  double z{1.0};
};

ZRow zRowOf(const geometry_msgs::msg::Quaternion & q)
{
  return ZRow{
    2.0 * (q.x * q.z - q.w * q.y), 2.0 * (q.y * q.z + q.w * q.x),
    1.0 - 2.0 * (q.x * q.x + q.y * q.y)};
}

geometry_msgs::msg::Twist toTwist(const BodyVelocity & velocity)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = velocity.vx;
  twist.linear.y = velocity.vy;
  twist.angular.z = velocity.wz;
  return twist;
}

YawHoldParams readParams(rclcpp::Node & node)
{
  using core_nav_common::declareDouble;
  YawHoldParams params;
  params.timeout_s = declareDouble(
    node, "timeout_s", params.timeout_s, 1e-3, 10.0, "指令を受けてから出し直す時間の上限 [s]", true);
  params.max_correction_rad = declareDouble(
    node, "max_correction_deg", 30.0, 0.0, 180.0, "逆に回す角度の上限 [deg]", true) * M_PI / 180.0;
  params.max_imu_gap_s = declareDouble(
    node, "max_imu_gap_s", params.max_imu_gap_s, 1e-4, 1.0, "これより間の空いた IMU は積まない [s]",
    true);
  params.max_reference_age_s = declareDouble(
    node, "max_reference_age_s", params.max_reference_age_s, 0.0, 5.0,
    "局所計画が指令を作った時刻として使う、受けた時からの古さの上限 [s]", true);
  return params;
}
}  // namespace

class CmdYawHoldNode : public rclcpp::Node
{
public:
  explicit CmdYawHoldNode(const rclcpp::NodeOptions & options)
  : Node("cmd_yaw_hold", options),
    hold_(readParams(*this)),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    using core_nav_common::declareBool;
    using core_nav_common::declareString;
    enabled_ = declareBool(*this, "enabled", true, "false なら指令をそのまま出す", true);
    base_frame_ = declareString(*this, "base_frame", "base_link", "機体のフレーム", true);
    const auto input = declareString(
      *this, "input_topic", "/control/auto/cmd_vel_raw", "安全停止の後の指令", true);
    const auto output = declareString(
      *this, "output_topic", "/control/auto/cmd_vel", "出し直した指令", true);
    const auto imu = declareString(*this, "imu_topic", "/sensing/livox/imu", "上部の IMU", true);
    const auto frame = declareString(
      *this, "frame_topic", "/planning/cmd_vel_nav/frame", "局所計画の指令と、機体の座標に直した時刻", true);

    publisher_ = create_publisher<geometry_msgs::msg::Twist>(output, rclcpp::QoS(10));
    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      input, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::ConstSharedPtr msg) {onCommand(*msg);});
    if (enabled_) {
      frame_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        frame, rclcpp::QoS(10),
        [this](const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {onFrame(*msg);});
      imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        imu, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {onImu(*msg);});
    }
    RCLCPP_INFO(
      get_logger(), "指令の向きの保持: %s（%s → %s、IMU %s）", enabled_ ? "有効" : "無効",
      input.c_str(), output.c_str(), imu.c_str());
  }

private:
  void onCommand(const geometry_msgs::msg::Twist & twist)
  {
    const BodyVelocity command{twist.linear.x, twist.linear.y, twist.angular.z};
    const bool finite = std::isfinite(command.vx) && std::isfinite(command.vy) &&
      std::isfinite(command.wz);
    if (!enabled_ || !finite) {
      // 有限でない指令は足回りが捨てるので、判断をそのまま任せる
      publisher_->publish(twist);
      return;
    }
    const double now_s = get_clock()->now().seconds();
    const auto reference = referenceTime(command, now_s);
    hold_.onCommand(command, now_s, reference);
    ++commands_;
    matched_ += reference.has_value() ? 1U : 0U;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000, "局所計画の指令を作った時刻の向きから戻した指令: %zu / %zu",
      matched_, commands_);
    if (!hold_.hasYaw()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "IMU（または %s への TF）がまだないので、指令をそのまま出します",
        base_frame_.c_str());
      publisher_->publish(twist);
      return;
    }
    publishHeld();
  }

  // 局所計画の指令と、機体の座標に直した時刻を覚える（新しい順に数個）
  void onFrame(const geometry_msgs::msg::TwistStamped & frame)
  {
    frames_.push_front(
      PlannedFrame{rclcpp::Time(frame.header.stamp).seconds(), frame.twist.linear.x, frame.twist.linear.y});
    while (frames_.size() > kMaxFrames) {
      frames_.pop_back();
    }
  }

  // 受けた指令と並進の向きが同じ、いちばん新しい局所計画の指令の時刻。安全停止は速さを落としても向きは変えない
  // ので、向きで見分ける。止まる指令（並進なし）と、見つからないときは nullopt
  std::optional<double> referenceTime(const BodyVelocity & command, const double now_s) const
  {
    if (std::hypot(command.vx, command.vy) < kMinSpeedMps) {
      return std::nullopt;
    }
    const double heading = std::atan2(command.vy, command.vx);
    for (const auto & frame : frames_) {
      if (std::hypot(frame.vx, frame.vy) < kMinSpeedMps || frame.stamp_s > now_s) {
        continue;
      }
      const double difference = std::remainder(std::atan2(frame.vy, frame.vx) - heading, 2.0 * M_PI);
      if (std::abs(difference) < kSameDirectionRad) {
        return frame.stamp_s;
      }
    }
    return std::nullopt;
  }

  void onImu(const sensor_msgs::msg::Imu & imu)
  {
    if (!z_row_.has_value()) {
      try {
        z_row_ = zRowOf(
          tf_buffer_.lookupTransform(base_frame_, imu.header.frame_id, tf2::TimePointZero)
          .transform.rotation);
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "IMU のフレーム %s から %s への TF がありません: %s",
          imu.header.frame_id.c_str(), base_frame_.c_str(), error.what());
        return;
      }
    }
    const auto & w = imu.angular_velocity;
    hold_.onYawRate(
      rclcpp::Time(imu.header.stamp).seconds(), z_row_->x * w.x + z_row_->y * w.y + z_row_->z * w.z);
    publishHeld();
  }

  void publishHeld()
  {
    if (const auto held = hold_.output(get_clock()->now().seconds())) {
      publisher_->publish(toTwist(*held));
    }
  }

  YawHold hold_;
  bool enabled_{true};
  std::string base_frame_;
  std::optional<ZRow> z_row_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr frame_subscription_;
  // 局所計画の指令と、機体の座標に直した時刻
  struct PlannedFrame
  {
    double stamp_s{0.0};
    double vx{0.0};
    double vy{0.0};
  };
  std::deque<PlannedFrame> frames_;
  // 受けた指令の数と、局所計画の指令を作った時刻を使えた数（ログ用）
  std::size_t commands_{0U};
  std::size_t matched_{0U};
  static constexpr std::size_t kMaxFrames = 10U;
  static constexpr double kMinSpeedMps = 1e-3;
  static constexpr double kSameDirectionRad = 1e-3;
};
}  // namespace core_planning

RCLCPP_COMPONENTS_REGISTER_NODE(core_planning::CmdYawHoldNode)
