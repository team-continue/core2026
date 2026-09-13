#include <memory>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "core_msgs/msg/can.hpp"
#include "core_msgs/msg/can_array.hpp"

#include "core_shooter/ballistics.hpp"
#include "core_shooter/can_command.hpp"
#include "core_shooter/parameter_utils.hpp"
#include "core_shooter/test_mode_gate.hpp"
#include "core_shooter/turret_envelope.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kDegToRad = M_PI / 180.0;

using core_shooter::BallisticTable;
using core_shooter::MotorLimits;
using core_shooter::TurretEnvelope;
using core_shooter::DirectionAngles;
using core_shooter::Vector3;
using core_shooter::isFinite;
using core_shooter::toDirectionAngles;
using core_shooter::vectorNorm;

/// 追尾に使うターゲット入力の種類。
enum class TargetInputMode
{
  Image,    ///< 画像座標（中心原点ピクセル）で追尾する方式
  Point3D,  ///< 砲塔座標系の3次元座標で追尾する方式
};

/// 3次元座標入力を解釈する基準座標系。
enum class Point3DFrame
{
  Turret,  ///< 砲身の現在向きを基準とする相対座標。方向角をそのまま角度誤差として扱う
  Base,    ///< 砲塔基部に固定された座標。方向角をそのまま目標角として扱う
};

std::optional<TargetInputMode> parseTargetInputMode(const std::string & name)
{
  if (name == "image") {
    return TargetInputMode::Image;
  }
  if (name == "point3d") {
    return TargetInputMode::Point3D;
  }
  return std::nullopt;
}

std::optional<Point3DFrame> parsePoint3DFrame(const std::string & name)
{
  if (name == "turret") {
    return Point3DFrame::Turret;
  }
  if (name == "base") {
    return Point3DFrame::Base;
  }
  return std::nullopt;
}

}  // namespace

class AimBot : public rclcpp::Node
{
public:
  AimBot()
  : Node("aim_bot")
  {
    // ----------------------------
    // パラメータ宣言と取得
    // ----------------------------
    rate_ = core_shooter::declareAndGet<double>(*this, "rate", 30.0);
    pitch_motor_id_ = core_shooter::declareAndGet<int>(*this, "pitch_motor_id", 10);
    yaw_motor_id_ = core_shooter::declareAndGet<int>(*this, "yaw_motor_id", 7);
    pitch_offset_ = core_shooter::declareAndGet<double>(*this, "pitch_offset", 0.0);
    yaw_min_angle_ = core_shooter::declareAndGet<double>(*this, "yaw_min_angle", -3.14159265359);
    yaw_max_angle_ = core_shooter::declareAndGet<double>(*this, "yaw_max_angle", 3.14159265359);
    pitch_min_angle_ = core_shooter::declareAndGet<double>(
      *this, "pitch_min_angle", -3.14159265359);
    pitch_max_angle_ = core_shooter::declareAndGet<double>(*this, "pitch_max_angle", 3.14159265359);
    const std::string target_input_mode_name =
      core_shooter::declareAndGet<std::string>(*this, "target_input_mode", "image");
    const std::string point3d_frame_name =
      core_shooter::declareAndGet<std::string>(*this, "point3d.frame", "turret");
    point3d_tolerance_yaw_rad_ = core_shooter::declareAndGet<double>(
      *this, "point3d.tolerance_yaw_rad", 0.01);
    point3d_tolerance_pitch_rad_ = core_shooter::declareAndGet<double>(
      *this, "point3d.tolerance_pitch_rad", 0.01);
    point3d_min_range_m_ = core_shooter::declareAndGet<double>(*this, "point3d.min_range_m", 0.05);
    point3d_max_range_m_ = core_shooter::declareAndGet<double>(*this, "point3d.max_range_m", 0.0);
    point3d_limit_command_rate_ = core_shooter::declareAndGet<bool>(
      *this, "point3d.limit_command_rate", true);
    point3d_target_velocity_max_m_per_sec_ = core_shooter::declareAndGet<double>(
      *this, "point3d.target_velocity_max_m_per_sec", 10.0);
    const std::string ballistic_table_path = core_shooter::declareAndGet<std::string>(
      *this, "point3d.ballistic.table_path", "");
    ballistic_max_shoot_range_m_ = core_shooter::declareAndGet<double>(
      *this, "point3d.ballistic.max_shoot_range_m", 5.0);
    camera_offset_y_m_ = core_shooter::declareAndGet<double>(
      *this, "point3d.camera_offset.y_m", 0.0);
    camera_offset_z_m_ = core_shooter::declareAndGet<double>(
      *this, "point3d.camera_offset.z_m", 0.0);
    shoot_status_rate_ = core_shooter::declareAndGet<double>(*this, "shoot_status_rate", 2.0);
    image_shoot_tolerance_x_px_ = core_shooter::declareAndGet<double>(
      *this, "image_shoot_tolerance_x_px", 50.0);
    image_shoot_tolerance_y_px_ = core_shooter::declareAndGet<double>(
      *this, "image_shoot_tolerance_y_px", 50.0);
    aim_tolerance_x_m_ = core_shooter::declareAndGet<double>(
      *this, "point3d.aim_tolerance.x_m", 0.5);
    aim_tolerance_y_m_ = core_shooter::declareAndGet<double>(
      *this, "point3d.aim_tolerance.y_m", 0.08);
    aim_tolerance_z_m_ = core_shooter::declareAndGet<double>(
      *this, "point3d.aim_tolerance.z_m", 0.08);
    image_center_x_ = core_shooter::declareAndGet<double>(*this, "image_center_x", 0.5);
    image_center_y_ = core_shooter::declareAndGet<double>(*this, "image_center_y", 0.5);
    image_width_ = core_shooter::declareAndGet<double>(*this, "image_width", 1280.0);
    image_height_ = core_shooter::declareAndGet<double>(*this, "image_height", 720.0);
    horizontal_fov_deg_ = core_shooter::declareAndGet<double>(*this, "horizontal_fov_deg", 100.0);
    use_fov_image_tracking_ = core_shooter::declareAndGet<bool>(
      *this, "use_fov_image_tracking", true);
    image_tolerance_x_ = core_shooter::declareAndGet<double>(*this, "image_tolerance_x", 8.0);
    image_tolerance_y_ = core_shooter::declareAndGet<double>(*this, "image_tolerance_y", 8.0);
    target_lead_time_sec_ = core_shooter::declareAndGet<double>(*this, "target_lead_time_sec", 0.0);
    target_velocity_min_dt_sec_ = core_shooter::declareAndGet<double>(
      *this, "target_velocity_min_dt_sec", 0.01);
    target_velocity_max_px_per_sec_ = core_shooter::declareAndGet<double>(
      *this, "target_velocity_max_px_per_sec", 1500.0);
    target_velocity_ema_alpha_ = core_shooter::declareAndGet<double>(
      *this, "target_velocity_ema_alpha", 0.25);
    max_yaw_rate_ = core_shooter::declareAndGet<double>(*this, "max_yaw_rate", 0.5);
    max_pitch_rate_ = core_shooter::declareAndGet<double>(*this, "max_pitch_rate", 0.5);
    yaw_image_gain_ = core_shooter::declareAndGet<double>(*this, "yaw_image_gain", 0.5);
    pitch_image_gain_ = core_shooter::declareAndGet<double>(*this, "pitch_image_gain", 0.5);
    yaw_direction_ = core_shooter::declareAndGet<double>(*this, "yaw_direction", 1.0);
    pitch_direction_ = core_shooter::declareAndGet<double>(*this, "pitch_direction", 1.0);
    target_timeout_sec_ = core_shooter::declareAndGet<double>(*this, "target_timeout_sec", 0.2);
    target_lost_return_to_startup_delay_sec_ = core_shooter::declareAndGet<double>(
      *this, "target_lost_return_to_startup_delay_sec", 2.0);
    enable_test_mode_ = core_shooter::declareAndGet<bool>(*this, "enable_test_mode", false);
    test_yaw_gain_ = core_shooter::declareAndGet<double>(*this, "test_yaw_gain", 0.05);
    test_pitch_gain_ = core_shooter::declareAndGet<double>(*this, "test_pitch_gain", 0.05);
    manual_mode_yaw_fixed_angle_ = core_shooter::declareAndGet<double>(
      *this, "manual_mode_yaw_fixed_angle", 0.0);
    manual_mode_pitch_initial_angle_ = core_shooter::declareAndGet<double>(
      *this, "manual_mode_pitch_initial_angle", 0.0);
    startup_release_yaw_angle_ = core_shooter::declareAndGet<double>(
      *this, "startup_release_yaw_angle", 0.0);
    startup_release_pitch_angle_ = core_shooter::declareAndGet<double>(
      *this, "startup_release_pitch_angle", 0.0);
    // 既定は無効。config で包絡線と合わせて有効化する。
    // 単体起動やテストで包絡線なしに動かせるようにしておく。
    enable_angle_limit_ = core_shooter::declareAndGet<bool>(
      *this, "enable_angle_limit", false);
    const std::string envelope_path = core_shooter::declareAndGet<std::string>(
      *this, "envelope.table_path", "");
    const double envelope_pitch_margin = core_shooter::declareAndGet<double>(
      *this, "envelope.pitch_margin_rad", 0.01);

    MotorLimits motor_limits;
    motor_limits.yaw_min = yaw_min_angle_;
    motor_limits.yaw_max = yaw_max_angle_;
    motor_limits.pitch_min = pitch_min_angle_;
    motor_limits.pitch_max = pitch_max_angle_;
    envelope_.setMotorLimits(motor_limits);
    envelope_.setPitchMargin(envelope_pitch_margin);

    test_mode_.setDefault(enable_test_mode_);

    const auto parsed_target_input_mode = parseTargetInputMode(target_input_mode_name);
    if (!parsed_target_input_mode) {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid target_input_mode='%s' (must be \"image\" or \"point3d\")",
        target_input_mode_name.c_str());
      throw std::runtime_error("invalid aimbot target input mode");
    }
    target_input_mode_ = *parsed_target_input_mode;

    const auto parsed_point3d_frame = parsePoint3DFrame(point3d_frame_name);
    if (!parsed_point3d_frame) {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid point3d.frame='%s' (must be \"turret\" or \"base\")",
        point3d_frame_name.c_str());
      throw std::runtime_error("invalid aimbot point3d frame");
    }
    point3d_frame_ = *parsed_point3d_frame;

    if (rate_ <= 0.0) {
      RCLCPP_FATAL(get_logger(), "Invalid rate=%f (must be > 0)", rate_);
      throw std::runtime_error("invalid aimbot rate");
    }
    if (pitch_motor_id_ < 0 || yaw_motor_id_ < 0) {
      RCLCPP_FATAL(
        get_logger(), "Invalid motor ids: pitch_motor_id=%d, yaw_motor_id=%d",
        pitch_motor_id_, yaw_motor_id_);
      throw std::runtime_error("invalid aimbot motor ids");
    }
    if (yaw_min_angle_ > yaw_max_angle_) {
      RCLCPP_FATAL(
        get_logger(), "Invalid yaw angle caps: yaw_min_angle=%f > yaw_max_angle=%f",
        yaw_min_angle_, yaw_max_angle_);
      throw std::runtime_error("invalid yaw angle caps");
    }
    if (pitch_min_angle_ > pitch_max_angle_) {
      RCLCPP_FATAL(
        get_logger(), "Invalid pitch angle caps: pitch_min_angle=%f > pitch_max_angle=%f",
        pitch_min_angle_, pitch_max_angle_);
      throw std::runtime_error("invalid pitch angle caps");
    }
    if (image_width_ <= 0.0 || image_height_ <= 0.0 ||
      test_yaw_gain_ < 0.0 || test_pitch_gain_ < 0.0 ||
      image_tolerance_x_ < 0.0 || image_tolerance_y_ < 0.0 || target_timeout_sec_ < 0.0 ||
      target_lost_return_to_startup_delay_sec_ < 0.0 || target_lead_time_sec_ < 0.0 ||
      target_velocity_min_dt_sec_ < 0.0 || target_velocity_max_px_per_sec_ < 0.0 ||
      max_yaw_rate_ < 0.0 || max_pitch_rate_ < 0.0)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid image params: image_width=%f, image_height=%f, horizontal_fov_deg=%f, test_yaw_gain=%f, test_pitch_gain=%f, image_tolerance_x=%f, image_tolerance_y=%f, target_timeout_sec=%f, target_lost_return_to_startup_delay_sec=%f, target_lead_time_sec=%f, target_velocity_min_dt_sec=%f, target_velocity_max_px_per_sec=%f, max_yaw_rate=%f, max_pitch_rate=%f",
        image_width_, image_height_, horizontal_fov_deg_, test_yaw_gain_, test_pitch_gain_,
        image_tolerance_x_, image_tolerance_y_, target_timeout_sec_,
        target_lost_return_to_startup_delay_sec_, target_lead_time_sec_,
        target_velocity_min_dt_sec_, target_velocity_max_px_per_sec_,
        max_yaw_rate_, max_pitch_rate_);
      throw std::runtime_error("invalid aimbot image parameters");
    }
    if (target_velocity_ema_alpha_ < 0.0 || target_velocity_ema_alpha_ > 1.0) {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid target_velocity_ema_alpha=%f (must satisfy 0 <= alpha <= 1)",
        target_velocity_ema_alpha_);
      throw std::runtime_error("invalid aimbot target velocity ema alpha");
    }
    if (target_input_mode_ == TargetInputMode::Image && use_fov_image_tracking_ &&
      (horizontal_fov_deg_ <= 0.0 || horizontal_fov_deg_ >= 180.0))
    {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid horizontal_fov_deg=%f for FOV image tracking (must be 0 < hfov < 180)",
        horizontal_fov_deg_);
      throw std::runtime_error("invalid aimbot horizontal fov");
    }
    if (point3d_tolerance_yaw_rad_ < 0.0 || point3d_tolerance_pitch_rad_ < 0.0 ||
      point3d_min_range_m_ < 0.0 || point3d_max_range_m_ < 0.0 ||
      point3d_target_velocity_max_m_per_sec_ < 0.0)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid point3d params: tolerance_yaw_rad=%f, tolerance_pitch_rad=%f, min_range_m=%f, max_range_m=%f, target_velocity_max_m_per_sec=%f (all must be >= 0)",
        point3d_tolerance_yaw_rad_, point3d_tolerance_pitch_rad_, point3d_min_range_m_,
        point3d_max_range_m_, point3d_target_velocity_max_m_per_sec_);
      throw std::runtime_error("invalid aimbot point3d parameters");
    }
    if (!std::isfinite(camera_offset_y_m_) || !std::isfinite(camera_offset_z_m_)) {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid point3d.camera_offset: y_m=%f, z_m=%f (must be finite)",
        camera_offset_y_m_, camera_offset_z_m_);
      throw std::runtime_error("invalid aimbot camera offset");
    }
    if (!std::isfinite(shoot_status_rate_)) {
      RCLCPP_FATAL(
        get_logger(), "Invalid shoot_status_rate=%f (must be finite)", shoot_status_rate_);
      throw std::runtime_error("invalid aimbot shoot status rate");
    }
    if (image_shoot_tolerance_x_px_ < 0.0 || image_shoot_tolerance_y_px_ < 0.0) {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid image_shoot_tolerance: x_px=%f, y_px=%f (must be >= 0)",
        image_shoot_tolerance_x_px_, image_shoot_tolerance_y_px_);
      throw std::runtime_error("invalid aimbot image shoot tolerance");
    }
    if (aim_tolerance_x_m_ < 0.0 || aim_tolerance_y_m_ < 0.0 || aim_tolerance_z_m_ < 0.0) {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid point3d.aim_tolerance: x_m=%f, y_m=%f, z_m=%f (all must be >= 0)",
        aim_tolerance_x_m_, aim_tolerance_y_m_, aim_tolerance_z_m_);
      throw std::runtime_error("invalid aimbot aim tolerance");
    }
    if (ballistic_max_shoot_range_m_ < 0.0) {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid point3d.ballistic.max_shoot_range_m=%f (must be >= 0; 0 disables the range gate)",
        ballistic_max_shoot_range_m_);
      throw std::runtime_error("invalid aimbot ballistic max shoot range");
    }
    if (!ballistic_table_path.empty()) {
      std::string ballistic_error;
      if (!BallisticTable::load(ballistic_table_path, ballistic_table_, ballistic_error)) {
        RCLCPP_FATAL(
          get_logger(), "Invalid ballistics table: %s", ballistic_error.c_str());
        throw std::runtime_error("invalid aimbot ballistics table");
      }
      RCLCPP_INFO(
        get_logger(),
        "Loaded ballistics table from '%s': %zu samples covering %.3f..%.3fm",
        ballistic_table_path.c_str(), ballistic_table_.size(),
        ballistic_table_.minRange(), ballistic_table_.maxRange());
    }
    if (point3d_max_range_m_ > 0.0 && point3d_min_range_m_ > point3d_max_range_m_) {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid point3d range: min_range_m=%f > max_range_m=%f",
        point3d_min_range_m_, point3d_max_range_m_);
      throw std::runtime_error("invalid aimbot point3d range");
    }

    {
      if (envelope_pitch_margin < 0.0) {
        RCLCPP_FATAL(
          get_logger(),
          "Invalid envelope.pitch_margin_rad=%f (must be >= 0)", envelope_pitch_margin);
        throw std::runtime_error("invalid aimbot envelope pitch margin");
      }
      if (enable_angle_limit_ && envelope_path.empty()) {
        RCLCPP_FATAL(
          get_logger(),
          "enable_angle_limit is true but envelope.table_path is empty");
        throw std::runtime_error("missing aimbot envelope table path");
      }
      if (!envelope_path.empty()) {
        std::string envelope_error;
        if (!TurretEnvelope::load(envelope_path, envelope_, envelope_error)) {
          RCLCPP_FATAL(get_logger(), "Invalid turret envelope: %s", envelope_error.c_str());
          throw std::runtime_error("invalid aimbot turret envelope");
        }
        envelope_.setMotorLimits(motor_limits);
        envelope_.setPitchMargin(envelope_pitch_margin);
        RCLCPP_INFO(
          get_logger(),
          "Loaded turret envelope from '%s': %zu points covering yaw %.3f..%.3frad",
          envelope_path.c_str(), envelope_.size(), envelope_.yawMin(), envelope_.yawMax());
      }
    }

    // ----------------------------
    // Subscriber
    // ----------------------------
    // ターゲット入力は排他。選択したモードの購読だけを生成する。
    if (target_input_mode_ == TargetInputMode::Point3D) {
      target_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "target_point_3d", 10,
        std::bind(&AimBot::targetPoint3dCallback, this, std::placeholders::_1));
    } else {
      target_image_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "target_image_position", 10,
        std::bind(&AimBot::targetImageCallback, this, std::placeholders::_1));
    }

    test_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/test_mode", 10, std::bind(&AimBot::testModeCallback, this, std::placeholders::_1));
    test_yaw_sub_ = create_subscription<std_msgs::msg::Float32>(
      "test_yaw_angle", 10,
      std::bind(&AimBot::testYawCallback, this, std::placeholders::_1));
    test_pitch_sub_ = create_subscription<std_msgs::msg::Float32>(
      "test_pitch_angle", 10,
      std::bind(&AimBot::testPitchCallback, this, std::placeholders::_1));

    manual_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
      "manual_mode", 10, std::bind(&AimBot::manualModeCallback, this, std::placeholders::_1));
    manual_pitch_sub_ = create_subscription<std_msgs::msg::Float32>(
      "manual_pitch_angle", 10,
      std::bind(&AimBot::manualPitchCallback, this, std::placeholders::_1));

    hazard_state_sub_ = create_subscription<std_msgs::msg::Bool>(
      "hazard_status", 10, std::bind(&AimBot::hazardCallback, this, std::placeholders::_1));

    // 自動射撃の有効/無効。これだけは照準側では決められないので外から受け取る。
    turret_auto_sub_ = create_subscription<std_msgs::msg::Bool>(
      "turret_auto", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {turret_auto_ = msg->data;});

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&AimBot::jointStateCallback, this, std::placeholders::_1));

    // ----------------------------
    // Publisher
    // ----------------------------
    can_pub_ = this->create_publisher<core_msgs::msg::CANArray>("/can/tx", 10);
    // 自動射撃の引き金。照準が合っているかを知っているのはこのノードなので、
    // turret_auto と突き合わせてここで判断する。
    shoot_fullauto_pub_ = this->create_publisher<std_msgs::msg::Bool>("shoot_fullauto", 10);

    // 「なぜ撃たないか」を外から見えるようにする。現場での切り分け用。
    if (shoot_status_rate_ > 0.0) {
      shoot_status_pub_ = this->create_publisher<std_msgs::msg::String>("shoot_status", 10);
      const auto status_period = std::chrono::duration<double>(1.0 / shoot_status_rate_);
      shoot_status_timer_ = create_wall_timer(
        status_period, std::bind(&AimBot::publishShootStatus, this));
    }

    // ----------------------------
    // Timer
    // ----------------------------
    const auto period = std::chrono::duration<double>(1.0 / rate_);
    timer_ = create_wall_timer(period, std::bind(&AimBot::timerCallback, this));

    if (target_input_mode_ == TargetInputMode::Point3D) {
      const std::string max_range_text = point3d_max_range_m_ > 0.0 ?
        std::to_string(point3d_max_range_m_) :
        std::string("inf");
      RCLCPP_INFO(
        get_logger(),
        "AimBot started. target_input_mode=point3d, target_point_3d=PointStamped(turret frame: x forward, y left, z up [m]; x<0 means not detected), point3d.frame=%s, camera_offset=(y=%.4f, z=%.4f)m, tolerance=(yaw=%.4f, pitch=%.4f)rad, range=[%.3f, %s]m, ballistic=%s(table_points=%zu), max_shoot_range=%.3fm, shoot_fullauto published when turret_auto, limit_command_rate=%s, target_lead_time=%.3fs, target_velocity_filter=(min_dt=%.3fs,max=%.2fm/s,alpha=%.2f), return_rate=(yaw=%.3f,pitch=%.3f)rad/s, target_lost_return_delay=%.2fs, test_mode_default=%s(topic override supported), startup_release_target=(%.3f, %.3f)",
        point3d_frame_ == Point3DFrame::Turret ? "turret" : "base",
        camera_offset_y_m_, camera_offset_z_m_,
        point3d_tolerance_yaw_rad_, point3d_tolerance_pitch_rad_,
        point3d_min_range_m_,
        max_range_text.c_str(),
        ballistic_table_.empty() ? "flat(straight trajectory)" : "measured table",
        ballistic_table_.size(),
        ballistic_max_shoot_range_m_,
        point3d_limit_command_rate_ ? "true" : "false",
        target_lead_time_sec_, target_velocity_min_dt_sec_,
        point3d_target_velocity_max_m_per_sec_, target_velocity_ema_alpha_,
        max_yaw_rate_, max_pitch_rate_,
        target_lost_return_to_startup_delay_sec_,
        enable_test_mode_ ? "true" : "false",
        startup_release_yaw_angle_, startup_release_pitch_angle_);
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "AimBot started. target_input_mode=image, target_image_position=PointStamped(center-origin x/y px, z:0=detected 1=none), image_size=(%.0f x %.0f), target_center_norm=(%.3f, %.3f), target_center_px=(%.1f, %.1f), tracking=%s, hfov=%.1fdeg, image_tolerance=(%.3f, %.3f), image_shoot_tolerance=(%.1f, %.1f)px, shoot_fullauto published when turret_auto, target_lead_time=%.3fs, target_velocity_filter=(min_dt=%.3fs,max=%.1fpx/s,alpha=%.2f), return_rate=(yaw=%.3f,pitch=%.3f)rad/s, target_lost_return_delay=%.2fs, test_mode_default=%s(topic override supported), startup_release_target=(%.3f, %.3f)",
      image_width_, image_height_, image_center_x_, image_center_y_,
      getImageTargetCenterX(), getImageTargetCenterY(),
      use_fov_image_tracking_ ? "fov" : "gain",
      horizontal_fov_deg_, image_tolerance_x_, image_tolerance_y_,
      image_shoot_tolerance_x_px_, image_shoot_tolerance_y_px_, target_lead_time_sec_,
      target_velocity_min_dt_sec_, target_velocity_max_px_per_sec_, target_velocity_ema_alpha_,
      max_yaw_rate_, max_pitch_rate_,
      target_lost_return_to_startup_delay_sec_,
      enable_test_mode_ ? "true" : "false",
      startup_release_yaw_angle_, startup_release_pitch_angle_);
  }

private:
  enum class ControlMode
  {
    Emergency,
    Manual,
    Test,
    AutoTrack
  };

  /// ZoneAB のピッチ上限を決める。
  ///
  /// 旧名の zone.pitch_zone2_upper も互換のために受け付けるが、
  /// 新しい zone.pitch_zone1_upper が明示されていればそちらを優先する。
  double resolveZoneAbUpper()
  {
    const double zone1_upper = core_shooter::declareAndGet<double>(
      *this, "zone.pitch_zone1_upper", 3.14159265359);
    const double zone2_upper = core_shooter::declareAndGet<double>(
      *this, "zone.pitch_zone2_upper", 0.52359877559);

    const auto & overrides =
      this->get_node_parameters_interface()->get_parameter_overrides();
    const bool zone1_set = overrides.find("zone.pitch_zone1_upper") != overrides.end();
    const bool zone2_set = overrides.find("zone.pitch_zone2_upper") != overrides.end();

    if (zone1_set && zone2_set && std::fabs(zone1_upper - zone2_upper) > 1e-9) {
      RCLCPP_WARN(
        get_logger(),
        "Both zone.pitch_zone1_upper=%f and zone.pitch_zone2_upper=%f are set. Using zone.pitch_zone1_upper as the ZoneAB upper limit.",
        zone1_upper, zone2_upper);
    } else if (!zone1_set && zone2_set) {
      RCLCPP_WARN(
        get_logger(),
        "zone.pitch_zone2_upper is treated as the ZoneAB upper limit for compatibility. Prefer zone.pitch_zone1_upper.");
    }
    return zone1_set ? zone1_upper : zone2_upper;
  }

  // ===== コールバック =====
  void hazardCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool prev = hazard_state_;
    hazard_state_ = msg->data;

    if (!hazard_state_ && prev) {
      if (startup_release_init_pending_) {
        // ノード再起動後、最初の緊急停止解除時のみ設定済みの原点へ初期化する。
        setCommandTarget(startup_release_yaw_angle_, startup_release_pitch_angle_);
        startup_release_init_pending_ = false;
        startup_release_hold_active_ = true;
        RCLCPP_INFO(
          this->get_logger(),
          "First emergency release after restart: initialize command target to yaw=%f, pitch=%f",
          command_yaw_angle_, command_pitch_angle_);
      }
    }
  }

  void manualModeCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool prev = manual_mode_active_;
    manual_mode_active_ = msg->data;

    if (!prev && manual_mode_active_) {
      // Manual mode should initialize yaw/pitch only on explicit manual ON edge.
      manual_mode_init_pending_ = true;
      manual_mode_return_active_ = true;
    }

    if (!manual_mode_active_ && prev) {
      manual_mode_init_pending_ = false;
      manual_mode_return_active_ = false;
      RCLCPP_INFO(this->get_logger(), "Manual mode OFF");
    }
  }

  void testModeCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (test_mode_.update(msg->data)) {
      RCLCPP_INFO(
        this->get_logger(), "Test mode %s (source=topic, param fallback=%s)",
        test_mode_.enabled() ? "ON" : "OFF", test_mode_.defaultValue() ? "true" : "false");
    }
  }

  void targetImageCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    // PointStamped semantics:
    //   detected     -> point = <x, y, 0> (x/y are centered image coordinates)
    //   not detected -> point = <0, 0, 1>
    // image_center_x/y specify the desired image center in normalized coordinates.
    const bool detected = (msg->point.z < 0.5);
    if (!detected) {
      has_target_ = false;
      target_image_x_ = 0.0;
      target_image_y_ = 0.0;
      resetTargetMotionPrediction();
      return;
    }

    const rclcpp::Time sample_time = getTargetSampleTime(*msg);
    target_image_x_ = msg->point.x;
    target_image_y_ = msg->point.y;
    has_target_ = true;
    updateTargetMotionPrediction(msg->point.x, msg->point.y, sample_time);
    last_target_time_ = this->now();
  }

  void targetPoint3dCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    // 入力はカメラ座標系（REP-103: x前方 / y左 / z上, 単位[m]）のターゲット座標。
    // 「検出なし」は x に負の値を入れて通知される（前方にしか的は存在しないため）。
    const Vector3 camera_point{msg->point.x, msg->point.y, msg->point.z};
    if (!isFinite(camera_point)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "target_point_3d contains non-finite values: ignored");
      return;
    }

    // 未検出。camera_offset は y/z にしか効かないので、この判定は
    // 砲塔座標へ移す前でも後でも結果が変わらない。
    if (camera_point.x < 0.0) {
      clearPoint3dTarget();
      return;
    }

    // 原点近傍は測距失敗や初期値の可能性が高いので、検出扱いしない。
    if (vectorNorm(camera_point) < point3d_min_range_m_) {
      clearPoint3dTarget();
      return;
    }

    // カメラ取付位置のぶんだけ平行移動して砲塔座標系へ移す。
    // これをしないと近距離ほど視差で狙いがずれる。
    const Vector3 point = toTurretFrame(camera_point);

    const double range = vectorNorm(point);
    if (point3d_max_range_m_ > 0.0 && range > point3d_max_range_m_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "target_point_3d range=%.3fm exceeds point3d.max_range_m=%.3fm: ignored",
        range, point3d_max_range_m_);
      clearPoint3dTarget();
      return;
    }

    const rclcpp::Time sample_time = getTargetSampleTime(*msg);
    target_point_ = point;
    has_target_ = true;
    updateTargetPoint3dPrediction(point, sample_time);
    last_target_time_ = this->now();
  }

  /// カメラ座標系の点を砲塔座標系へ移す。
  ///
  /// camera_offset は砲塔の回転中心から見たカメラ取付位置[m]。
  /// カメラ基準の座標にこれを足すと砲塔基準の座標になる。
  Vector3 toTurretFrame(const Vector3 & camera_point) const
  {
    return Vector3{
      camera_point.x,
      camera_point.y + camera_offset_y_m_,
      camera_point.z + camera_offset_z_m_};
  }

  void testYawCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    test_yaw_target_ = msg->data;
    has_test_yaw_target_ = true;
    last_test_yaw_time_ = this->now();
  }

  void testPitchCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    test_pitch_target_ = msg->data;
    has_test_pitch_target_ = true;
    last_test_pitch_time_ = this->now();
  }

  void manualPitchCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    manual_pitch_target_ = msg->data;
    has_manual_pitch_target_ = true;
    last_manual_pitch_time_ = this->now();
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const size_t position_size = msg->position.size();
    if (yaw_motor_id_ < 0 || pitch_motor_id_ < 0 ||
      static_cast<size_t>(yaw_motor_id_) >= position_size ||
      static_cast<size_t>(pitch_motor_id_) >= position_size)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "joint_states size mismatch: position=%zu, yaw_id=%d, pitch_id=%d",
        position_size, yaw_motor_id_, pitch_motor_id_);
      return;
    }

    yaw_angle_ = msg->position[yaw_motor_id_];
    pitch_angle_ = msg->position[pitch_motor_id_];
    has_joint_state_ = true;
  }

  void timerCallback()
  {
    // 各周期の既定は発射不可。射程内のターゲットに照準が合ったときだけ許可に変わる。
    aimed_at_target_ = false;
    runControlCycle();
    publishShootFullauto();
  }

  void runControlCycle()
  {
    const bool test_mode_enabled = isTestModeEnabled();
    ControlMode mode = ControlMode::AutoTrack;
    if (hazard_state_) {
      mode = ControlMode::Emergency;
    } else if (manual_mode_active_) {
      mode = ControlMode::Manual;
    } else if (test_mode_enabled) {
      mode = ControlMode::Test;
    }

    if (mode != ControlMode::AutoTrack) {
      auto_track_timeout_active_ = false;
      auto_track_timeout_returned_to_startup_ = false;
      startup_release_hold_active_ = false;
    }

    const bool entering_mode = setActiveControlMode(mode);

    // 既定の理由をモードから決める。AutoTrack はこの後の経路で上書きされる。
    switch (mode) {
      case ControlMode::Emergency: shoot_block_reason_ = "emergency stop"; break;
      case ControlMode::Manual: shoot_block_reason_ = "manual mode"; break;
      case ControlMode::Test: shoot_block_reason_ = "test mode"; break;
      case ControlMode::AutoTrack: shoot_block_reason_ = "no target"; break;
    }

    switch (mode) {
      case ControlMode::Emergency: {
          if (entering_mode && has_joint_state_) {
            // Latch actual angle once when entering emergency, then keep holding that command.
            setCommandTargetRaw(yaw_angle_, pitch_angle_);
          }
          publishCommandHold("emergency hold skipped: command target not initialized");
          return;
        }

      case ControlMode::Manual: {
          if (test_mode_enabled) {
            RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 2000,
              "manual_mode active: overriding test_mode outputs");
          }
          if (entering_mode) {
            if (manual_mode_init_pending_) {
              // Initialize manual yaw/pitch only when /manual_mode is explicitly turned ON.
              has_manual_pitch_target_ = false;
              manual_mode_init_pending_ = false;
              RCLCPP_INFO(
                this->get_logger(),
                "Manual mode ON: start interpolated move to yaw=%f, pitch=%f using max_yaw_rate/max_pitch_rate (pitch becomes controllable via manual_pitch_angle)",
                manual_mode_yaw_fixed_angle_, manual_mode_pitch_initial_angle_);
            }
          }

          if (!has_manual_pitch_target_) {
            RCLCPP_WARN_THROTTLE(
              this->get_logger(),
              *this->get_clock(), 2000,
              "manual_mode active but manual_pitch_angle not received yet: holding manual initial pitch command");
          }
          bool manual_input_timed_out = false;
          if (has_manual_pitch_target_ && isTimedOut(last_manual_pitch_time_)) {
            manual_input_timed_out = true;
            has_manual_pitch_target_ = false;
            RCLCPP_WARN_THROTTLE(
              this->get_logger(),
              *this->get_clock(), 2000,
              "manual pitch input timed out: hold current pitch");
          }
          const double manual_pitch_delta = has_manual_pitch_target_ ?
            std::clamp(manual_pitch_target_, -1.0, 1.0) :
            0.0;
          const bool manual_pitch_override_active =
            has_manual_pitch_target_ &&
            (!manual_mode_return_active_ || std::fabs(manual_pitch_delta) > 1e-6);
          double manual_pitch = command_pitch_angle_;
          if (manual_pitch_override_active) {
            manual_mode_return_active_ = false;
            manual_pitch =
              command_pitch_angle_ + pitch_direction_ * test_pitch_gain_ * manual_pitch_delta;
          } else if (manual_input_timed_out && has_joint_state_) {
            manual_pitch = pitch_angle_;
          }
          if (manual_mode_return_active_ && !manual_pitch_override_active) {
            if (!stepCommandTargetTowardManualModeInitial()) {
              publishCommandHold(
                "manual mode init skipped: command/joint_states not initialized");
              return;
            }
          } else {
            if (!stepCommandTargetYawTowardManualModeFixed(manual_pitch)) {
              publishCommandHold(
                "manual mode hold skipped: command/joint_states not initialized");
              return;
            }
          }
          publishCommandTarget();
          return;
        }

      case ControlMode::Test:
        if (entering_mode) {
          // Require fresh test inputs after every test-mode entry to avoid reusing stale values.
          has_test_yaw_target_ = false;
          has_test_pitch_target_ = false;
        }
        if (has_test_yaw_target_ && isTimedOut(last_test_yaw_time_)) {
          has_test_yaw_target_ = false;
          RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(), 2000,
            "test yaw input timed out: hold current yaw command");
        }
        if (has_test_pitch_target_ && isTimedOut(last_test_pitch_time_)) {
          has_test_pitch_target_ = false;
          RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(), 2000,
            "test pitch input timed out: hold current pitch command");
        }
        if (!has_test_yaw_target_ && !has_test_pitch_target_) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "test mode enabled but no fresh test yaw/pitch target received yet");
          publishCommandHold("test mode hold skipped: command target not initialized");
          return;
        }
        if (!ensureCommandTarget("test mode enabled but joint_states not received yet")) {
          return;
        }

        // test input is normalized delta command [-1, 1] integrated on the internal command target.
        {
          const double yaw_delta =
            has_test_yaw_target_ ? std::clamp(test_yaw_target_, -1.0, 1.0) : 0.0;
          const double next_yaw = command_yaw_angle_ + yaw_direction_ * test_yaw_gain_ * yaw_delta;

          const double pitch_delta =
            has_test_pitch_target_ ? std::clamp(test_pitch_target_, -1.0, 1.0) : 0.0;
          const double next_pitch =
            command_pitch_angle_ + pitch_direction_ * test_pitch_gain_ * pitch_delta;

          setCommandTarget(next_yaw, next_pitch);
          publishCommandTarget();
          return;
        }

      case ControlMode::AutoTrack:
        if (startup_release_hold_active_ &&
          (!has_target_ || (this->now() - last_target_time_).seconds() > target_timeout_sec_))
        {
          publishCommandHold("startup release hold skipped: command target not initialized");
          return;
        }
        startup_release_hold_active_ = false;

        if (!has_target_ || (this->now() - last_target_time_).seconds() > target_timeout_sec_) {
          if (!auto_track_timeout_active_) {
            auto_track_timeout_active_ = true;
            auto_track_timeout_start_ = this->now();
            auto_track_timeout_returned_to_startup_ = false;
            resetTargetMotionPrediction();
            if (has_joint_state_) {
              // 目標喪失時は最初の1回だけ現在角をラッチし、その後はその目標を保持する。
              setCommandTargetRaw(yaw_angle_, pitch_angle_);
              publishCommandTarget();
              return;
            }
          }
          const double timeout_elapsed_sec =
            (this->now() - auto_track_timeout_start_).seconds();
          if (timeout_elapsed_sec >= target_lost_return_to_startup_delay_sec_) {
            if (!stepCommandTargetTowardStartupRelease()) {
              publishCommandHold(
                "startup return skipped: command/joint_states not available");
              return;
            }
            if (!auto_track_timeout_returned_to_startup_) {
              auto_track_timeout_returned_to_startup_ = true;
              RCLCPP_INFO(
                this->get_logger(),
                "target_image_position timeout continued for %.2fs: start interpolated return to startup_release target yaw=%f, pitch=%f",
                timeout_elapsed_sec, startup_release_yaw_angle_, startup_release_pitch_angle_);
            }
            publishCommandTarget();
            return;
          }
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "target_image_position timeout");
          publishCommandHold(
            "auto timeout: joint_states not received yet, cannot hold current angle");
          return;
        }

        auto_track_timeout_active_ = false;
        auto_track_timeout_returned_to_startup_ = false;

        if (entering_mode && has_joint_state_) {
          // Auto mode baseline is actual joint angle; pitch_offset is applied once at auto entry.
          setCommandTarget(yaw_angle_, pitch_angle_ + pitch_offset_);
        }

        if (!ensureCommandTarget("auto track skipped: command/joint_states not available")) {
          return;
        }

        if (target_input_mode_ == TargetInputMode::Point3D) {
          publishPoint3dTrackingCommand();
          return;
        }

        // 入力は中心原点のピクセル座標。image_center_x/y で狙う画像中心をずらし、
        // その中心からの誤差で追尾する。
        {
          const auto [predicted_target_x, predicted_target_y] = getPredictedTargetImagePosition();
          const double x_error = predicted_target_x - getImageTargetCenterX();
          const double y_error = predicted_target_y - getImageTargetCenterY();

          // 狙っている画像中心にターゲットが十分近ければ発射を許す。
          // 判定基準は image_center_x/y（= 実際に狙っている点）なので、
          // 追尾の収束先と発射判定の基準が必ず一致する。
          aimed_at_target_ =
            std::fabs(x_error) <= image_shoot_tolerance_x_px_ &&
            std::fabs(y_error) <= image_shoot_tolerance_y_px_;
          if (!aimed_at_target_) {
            shoot_block_reason_ = formatReason(
              "off center: err=(%.1f, %.1f)px tol=(%.1f, %.1f)px",
              x_error, y_error, image_shoot_tolerance_x_px_, image_shoot_tolerance_y_px_);
          }

          const double yaw_base = has_joint_state_ ? yaw_angle_ : command_yaw_angle_;
          const double pitch_base = has_joint_state_ ?
            (pitch_angle_ + pitch_offset_) :
            command_pitch_angle_;

          // AutoTrack は現在角ベースの比例補正にして、画像ノイズでのドリフトを防ぐ。
          double yaw_target = yaw_base;
          double pitch_target = pitch_base;

          if (use_fov_image_tracking_) {
            const double horizontal_fov_rad = horizontal_fov_deg_ * kDegToRad;
            const double vertical_fov_rad = horizontal_fov_rad * (image_height_ / image_width_);
            const double half_hfov_rad = 0.5 * horizontal_fov_rad;
            const double half_vfov_rad = 0.5 * vertical_fov_rad;

            if (std::fabs(x_error) > image_tolerance_x_) {
              // rectilinear camera assumption:
              // x_px in [-image_width/2, image_width/2] -> yaw offset in [-hfov/2, hfov/2]
              const double x_norm = std::clamp((2.0 * x_error) / image_width_, -1.0, 1.0);
              const double yaw_offset_rad = std::atan(x_norm * std::tan(half_hfov_rad));
              yaw_target = yaw_base + yaw_direction_ * yaw_offset_rad;
            }
            if (std::fabs(y_error) > image_tolerance_y_) {
              // Vertical FOV is derived from horizontal FOV by image aspect ratio.
              const double y_norm = std::clamp((2.0 * y_error) / image_height_, -1.0, 1.0);
              const double pitch_offset_rad = std::atan(y_norm * std::tan(half_vfov_rad));
              pitch_target = pitch_base + pitch_direction_ * pitch_offset_rad;
            }
          } else {
            if (std::fabs(x_error) > image_tolerance_x_) {
              yaw_target = yaw_base + yaw_direction_ * yaw_image_gain_ * x_error;
            }
            if (std::fabs(y_error) > image_tolerance_y_) {
              pitch_target = pitch_base + pitch_direction_ * pitch_image_gain_ * y_error;
            }
          }

          setCommandTarget(yaw_target, pitch_target);
          publishCommandTarget();
          return;
        }
    }
  }

  /// 発射を許可する水平距離かどうか。max_shoot_range_m が 0 以下なら制限なし。
  bool isWithinShootRange(double horizontal_range_m) const
  {
    return ballistic_max_shoot_range_m_ <= 0.0 ||
           horizontal_range_m < ballistic_max_shoot_range_m_;
  }

  /// 砲身が目標に向いているかを、目標位置での着弾ずれ[m]に換算して判定する。
  ///
  /// residual_yaw/pitch_rad は弾道補正まで含めた「まだ振り残している角度」。
  /// これを目標までの距離倍して横(y)・縦(z)のずれに直し、閾値と比べる。
  /// x は弾道テーブルが覆っていない距離のはみ出し量（解が保証されない距離）を見る。
  bool isAimedAtTarget(
    double slant_range_m, double horizontal_range_m,
    double residual_yaw_rad, double residual_pitch_rad)
  {
    const double miss_y_m = slant_range_m * std::tan(residual_yaw_rad);
    const double miss_z_m = slant_range_m * std::tan(residual_pitch_rad);
    const double range_error_m = ballistic_table_.rangeError(horizontal_range_m);

    const bool aimed =
      std::fabs(range_error_m) <= aim_tolerance_x_m_ &&
      std::fabs(miss_y_m) <= aim_tolerance_y_m_ &&
      std::fabs(miss_z_m) <= aim_tolerance_z_m_;

    if (!aimed) {
      shoot_block_reason_ = formatReason(
        "not aimed: miss=(x=%.3f, y=%.3f, z=%.3f)m tol=(%.3f, %.3f, %.3f)m",
        range_error_m, miss_y_m, miss_z_m,
        aim_tolerance_x_m_, aim_tolerance_y_m_, aim_tolerance_z_m_);
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "not aimed yet: miss=(x=%.4f, y=%.4f, z=%.4f)m tolerance=(%.4f, %.4f, %.4f)m",
        range_error_m, miss_y_m, miss_z_m,
        aim_tolerance_x_m_, aim_tolerance_y_m_, aim_tolerance_z_m_);
    }
    return aimed;
  }

  /// 砲塔座標系の3次元座標から yaw/pitch 指令を算出して発行する。
  void publishPoint3dTrackingCommand()
  {
    const Vector3 predicted_target = getPredictedTargetPoint();
    const DirectionAngles target_direction = toDirectionAngles(predicted_target);

    // 実測した着弾ずれを打ち消す向きに射角を補正する（テーブル未設定なら直線弾道）。
    const double horizontal_range_m = std::hypot(predicted_target.x, predicted_target.y);
    const DirectionAngles ballistic_correction = ballistic_table_.correction(horizontal_range_m);
    const double aim_yaw_rad = target_direction.yaw_rad + ballistic_correction.yaw_rad;
    const double aim_pitch_rad = target_direction.pitch_rad + ballistic_correction.pitch_rad;

    // 射程外のターゲットは追尾だけ続け、発射は許可しない。
    const bool within_shoot_range = isWithinShootRange(horizontal_range_m);
    if (!within_shoot_range) {
      shoot_block_reason_ = formatReason(
        "out of range: %.2fm >= max_shoot_range %.2fm",
        horizontal_range_m, ballistic_max_shoot_range_m_);
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 2000,
        "target horizontal range=%.3fm is at or beyond point3d.ballistic.max_shoot_range_m=%.3fm: tracking continues but shooting is not permitted",
        horizontal_range_m, ballistic_max_shoot_range_m_);
    }

    // AutoTrack は画像追尾と同じく現在角ベースで補正し、指令のドリフトを防ぐ。
    const double yaw_base = has_joint_state_ ? yaw_angle_ : command_yaw_angle_;
    const double pitch_base = has_joint_state_ ?
      (pitch_angle_ + pitch_offset_) :
      command_pitch_angle_;

    double yaw_target = yaw_base;
    double pitch_target = pitch_base;

    // まだ振り残している角度（弾道補正込み）。不感帯に入っても値は潰さず、
    // 発射判定にはこの生の残差を使う。
    double residual_yaw_rad = 0.0;
    double residual_pitch_rad = 0.0;

    if (point3d_frame_ == Point3DFrame::Turret) {
      // 砲身基準の相対座標なので、方向角がそのまま角度誤差になる。
      residual_yaw_rad = aim_yaw_rad;
      residual_pitch_rad = aim_pitch_rad;
      if (std::fabs(aim_yaw_rad) > point3d_tolerance_yaw_rad_) {
        yaw_target = yaw_base + yaw_direction_ * aim_yaw_rad;
      }
      if (std::fabs(aim_pitch_rad) > point3d_tolerance_pitch_rad_) {
        pitch_target = pitch_base + pitch_direction_ * aim_pitch_rad;
      }
    } else {
      // 砲塔基部基準の絶対座標なので、方向角がそのまま目標角になる。
      const double absolute_yaw = yaw_direction_ * aim_yaw_rad;
      const double absolute_pitch = pitch_direction_ * aim_pitch_rad + pitch_offset_;
      residual_yaw_rad = absolute_yaw - yaw_base;
      residual_pitch_rad = absolute_pitch - pitch_base;
      if (std::fabs(absolute_yaw - yaw_base) > point3d_tolerance_yaw_rad_) {
        yaw_target = absolute_yaw;
      }
      if (std::fabs(absolute_pitch - pitch_base) > point3d_tolerance_pitch_rad_) {
        pitch_target = absolute_pitch;
      }
    }

    // 砲身が目標へ向いているか（弾道補正込み）を、射程判定と合わせて発射許可にする。
    aimed_at_target_ = within_shoot_range &&
      isAimedAtTarget(
      vectorNorm(predicted_target), horizontal_range_m,
      residual_yaw_rad, residual_pitch_rad);

    if (point3d_limit_command_rate_) {
      // 3次元入力は最大180degの誤差を取り得るため、1周期あたりの指令変化量を制限する。
      yaw_target = stepToward(command_yaw_angle_, yaw_target, max_yaw_rate_ / rate_);
      pitch_target = stepToward(command_pitch_angle_, pitch_target, max_pitch_rate_ / rate_);
    }

    setCommandTarget(yaw_target, pitch_target);
    publishCommandTarget();
  }

  double clampYaw(double angle) const
  {
    return std::clamp(angle, yaw_min_angle_, yaw_max_angle_);
  }

  double clampPitch(double angle) const
  {
    return std::clamp(angle, pitch_min_angle_, pitch_max_angle_);
  }

  /// 包絡線の定義域とモータ可動範囲の両方でヨーをクランプする。
  double clampEnvelopeYaw(double raw_yaw) const
  {
    if (envelope_.empty()) {
      return clampYaw(raw_yaw);
    }
    return clampYaw(std::clamp(raw_yaw, envelope_.yawMin(), envelope_.yawMax()));
  }

  /// 砲塔の可動包絡線で指令角を制限する。
  ///
  /// 実測角が無い間は指令角で代用する。制限器は状態を持たないので、
  /// 同じ入力なら常に同じ結果になる。
  std::pair<double, double> applyAngleLimit(double yaw, double pitch)
  {
    if (!enable_angle_limit_ || envelope_.empty()) {
      return {clampYaw(yaw), clampPitch(pitch)};
    }

    const double measured_yaw = has_joint_state_ ?
      yaw_angle_ :
      (has_command_target_ ? command_yaw_angle_ : yaw);
    const double measured_pitch = has_joint_state_ ?
      pitch_angle_ :
      (has_command_target_ ? command_pitch_angle_ : pitch);

    const auto result = envelope_.apply(yaw, pitch, measured_yaw, measured_pitch);
    if (result.recovering) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "pitch %.3frad is outside the envelope at yaw %.3frad: holding yaw until pitch recovers",
        measured_pitch, measured_yaw);
    }
    return {result.yaw, result.pitch};
  }

  void setCommandTarget(double yaw, double pitch)
  {
    const auto [limited_yaw, limited_pitch] = applyAngleLimit(yaw, pitch);
    command_yaw_angle_ = limited_yaw;
    command_pitch_angle_ = limited_pitch;
    has_command_target_ = true;
  }

  void setManualModeCommandTarget(double yaw, double pitch)
  {
    command_yaw_angle_ = enable_angle_limit_ ? clampEnvelopeYaw(yaw) : clampYaw(yaw);
    command_pitch_angle_ = clampPitch(pitch);
    has_command_target_ = true;
  }

  void setCommandTargetRaw(double yaw, double pitch)
  {
    command_yaw_angle_ = yaw;
    command_pitch_angle_ = pitch;
    has_command_target_ = true;
  }

  double stepToward(double current, double target, double max_step) const
  {
    if (max_step <= 0.0) {
      return target;
    }
    const double error = target - current;
    if (std::fabs(error) <= max_step) {
      return target;
    }
    return current + std::copysign(max_step, error);
  }

  bool stepCommandTargetTowardStartupRelease()
  {
    if (!ensureCommandTarget("startup return skipped: command/joint_states not available")) {
      return false;
    }
    const double yaw_step = max_yaw_rate_ / rate_;
    const double pitch_step = max_pitch_rate_ / rate_;
    const double next_yaw =
      stepToward(command_yaw_angle_, startup_release_yaw_angle_, yaw_step);
    const double next_pitch =
      stepToward(command_pitch_angle_, startup_release_pitch_angle_, pitch_step);
    setCommandTarget(next_yaw, next_pitch);
    return true;
  }

  bool stepCommandTargetTowardManualModeInitial()
  {
    if (!ensureManualCommandTarget(
        "manual mode init skipped: command/joint_states not initialized"))
    {
      return false;
    }
    const double yaw_step = max_yaw_rate_ / rate_;
    const double pitch_step = max_pitch_rate_ / rate_;
    const double next_yaw =
      stepToward(command_yaw_angle_, getManualModeFixedYawTarget(), yaw_step);
    const double next_pitch =
      stepToward(command_pitch_angle_, clampPitch(manual_mode_pitch_initial_angle_), pitch_step);
    setManualModeCommandTarget(next_yaw, next_pitch);
    return true;
  }

  bool stepCommandTargetYawTowardManualModeFixed(double pitch)
  {
    if (!ensureManualCommandTarget(
        "manual mode hold skipped: command/joint_states not initialized"))
    {
      return false;
    }
    const double yaw_step = max_yaw_rate_ / rate_;
    const double next_yaw =
      stepToward(command_yaw_angle_, getManualModeFixedYawTarget(), yaw_step);
    setManualModeCommandTarget(next_yaw, pitch);
    return true;
  }

  bool latchCommandTargetFromJointState()
  {
    if (!has_joint_state_) {
      return false;
    }
    setCommandTarget(yaw_angle_, pitch_angle_);
    return true;
  }

  bool latchManualCommandTargetFromJointState()
  {
    if (!has_joint_state_) {
      return false;
    }
    setManualModeCommandTarget(yaw_angle_, pitch_angle_);
    return true;
  }

  bool ensureCommandTarget(const char * warn_message)
  {
    if (has_command_target_) {
      return true;
    }
    if (latchCommandTargetFromJointState()) {
      return true;
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000, "%s", warn_message);
    return false;
  }

  bool ensureManualCommandTarget(const char * warn_message)
  {
    if (has_command_target_) {
      return true;
    }
    if (latchManualCommandTargetFromJointState()) {
      return true;
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000, "%s", warn_message);
    return false;
  }

  bool isTimedOut(const rclcpp::Time & last_time) const
  {
    return (this->now() - last_time).seconds() > target_timeout_sec_;
  }

  double getManualModeFixedYawTarget() const
  {
    return enable_angle_limit_ ? clampEnvelopeYaw(manual_mode_yaw_fixed_angle_) :
           clampYaw(manual_mode_yaw_fixed_angle_);
  }

  void publishCommandHold(const char * warn_message)
  {
    if (!has_command_target_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "%s", warn_message);
      return;
    }
    publishCommandTarget();
  }

  bool setActiveControlMode(ControlMode next_mode)
  {
    const bool changed = !has_active_control_mode_ || active_control_mode_ != next_mode;
    active_control_mode_ = next_mode;
    has_active_control_mode_ = true;
    return changed;
  }

  bool isTestModeEnabled() const
  {
    return test_mode_.enabled();
  }

  double getImageTargetCenterX() const
  {
    return (image_center_x_ - 0.5) * image_width_;
  }

  double getImageTargetCenterY() const
  {
    return (image_center_y_ - 0.5) * image_height_;
  }

  rclcpp::Time getTargetSampleTime(const geometry_msgs::msg::PointStamped & msg) const
  {
    if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0) {
      return this->now();
    }
    return rclcpp::Time(msg.header.stamp, this->get_clock()->get_clock_type());
  }

  void resetTargetMotionPrediction()
  {
    has_target_velocity_ = false;
    target_image_velocity_x_ = 0.0;
    target_image_velocity_y_ = 0.0;
    target_point_velocity_ = Vector3{};
    has_previous_target_sample_ = false;
  }

  void clearPoint3dTarget()
  {
    has_target_ = false;
    target_point_ = Vector3{};
    resetTargetMotionPrediction();
  }

  void storeTargetSample(double x, double y, const rclcpp::Time & sample_time)
  {
    previous_target_image_x_ = x;
    previous_target_image_y_ = y;
    previous_target_sample_time_ = sample_time;
    has_previous_target_sample_ = true;
  }

  void updateTargetMotionPrediction(double x, double y, const rclcpp::Time & sample_time)
  {
    if (!has_previous_target_sample_) {
      storeTargetSample(x, y, sample_time);
      return;
    }

    const double dt = (sample_time - previous_target_sample_time_).seconds();
    if (dt <= 0.0) {
      resetTargetMotionPrediction();
      storeTargetSample(x, y, sample_time);
      return;
    }
    if (dt < target_velocity_min_dt_sec_) {
      return;
    }

    const double instant_velocity_x = std::clamp(
      (x - previous_target_image_x_) / dt,
      -target_velocity_max_px_per_sec_,
      target_velocity_max_px_per_sec_);
    const double instant_velocity_y = std::clamp(
      (y - previous_target_image_y_) / dt,
      -target_velocity_max_px_per_sec_,
      target_velocity_max_px_per_sec_);

    if (!has_target_velocity_ || target_velocity_ema_alpha_ >= 1.0) {
      target_image_velocity_x_ = instant_velocity_x;
      target_image_velocity_y_ = instant_velocity_y;
    } else if (target_velocity_ema_alpha_ <= 0.0) {
      target_image_velocity_x_ = 0.0;
      target_image_velocity_y_ = 0.0;
    } else {
      const double keep = 1.0 - target_velocity_ema_alpha_;
      target_image_velocity_x_ =
        keep * target_image_velocity_x_ + target_velocity_ema_alpha_ * instant_velocity_x;
      target_image_velocity_y_ =
        keep * target_image_velocity_y_ + target_velocity_ema_alpha_ * instant_velocity_y;
    }
    has_target_velocity_ = true;
    storeTargetSample(x, y, sample_time);
  }

  std::pair<double, double> getPredictedTargetImagePosition() const
  {
    double predicted_x = target_image_x_;
    double predicted_y = target_image_y_;
    if (has_target_velocity_ && target_lead_time_sec_ > 0.0) {
      predicted_x += target_image_velocity_x_ * target_lead_time_sec_;
      predicted_y += target_image_velocity_y_ * target_lead_time_sec_;
    }
    const double half_width = 0.5 * image_width_;
    const double half_height = 0.5 * image_height_;
    predicted_x = std::clamp(predicted_x, -half_width, half_width);
    predicted_y = std::clamp(predicted_y, -half_height, half_height);
    return {predicted_x, predicted_y};
  }

  void storeTargetPoint3dSample(const Vector3 & point, const rclcpp::Time & sample_time)
  {
    previous_target_point_ = point;
    previous_target_sample_time_ = sample_time;
    has_previous_target_sample_ = true;
  }

  void updateTargetPoint3dPrediction(const Vector3 & point, const rclcpp::Time & sample_time)
  {
    if (!has_previous_target_sample_) {
      storeTargetPoint3dSample(point, sample_time);
      return;
    }

    const double dt = (sample_time - previous_target_sample_time_).seconds();
    if (dt <= 0.0) {
      resetTargetMotionPrediction();
      storeTargetPoint3dSample(point, sample_time);
      return;
    }
    if (dt < target_velocity_min_dt_sec_) {
      return;
    }

    const auto clampVelocity = [this](double value) {
        return std::clamp(
          value,
          -point3d_target_velocity_max_m_per_sec_,
          point3d_target_velocity_max_m_per_sec_);
      };
    const Vector3 instant_velocity{
      clampVelocity((point.x - previous_target_point_.x) / dt),
      clampVelocity((point.y - previous_target_point_.y) / dt),
      clampVelocity((point.z - previous_target_point_.z) / dt)};

    if (!has_target_velocity_ || target_velocity_ema_alpha_ >= 1.0) {
      target_point_velocity_ = instant_velocity;
    } else if (target_velocity_ema_alpha_ <= 0.0) {
      target_point_velocity_ = Vector3{};
    } else {
      const double keep = 1.0 - target_velocity_ema_alpha_;
      target_point_velocity_ = Vector3{
        keep * target_point_velocity_.x + target_velocity_ema_alpha_ * instant_velocity.x,
        keep * target_point_velocity_.y + target_velocity_ema_alpha_ * instant_velocity.y,
        keep * target_point_velocity_.z + target_velocity_ema_alpha_ * instant_velocity.z};
    }
    has_target_velocity_ = true;
    storeTargetPoint3dSample(point, sample_time);
  }

  Vector3 getPredictedTargetPoint() const
  {
    if (!has_target_velocity_ || target_lead_time_sec_ <= 0.0) {
      return target_point_;
    }
    return Vector3{
      target_point_.x + target_point_velocity_.x * target_lead_time_sec_,
      target_point_.y + target_point_velocity_.y * target_lead_time_sec_,
      target_point_.z + target_point_velocity_.z * target_lead_time_sec_};
  }

  /// 自動射撃の引き金を発行する。
  ///
  /// turret_auto が有効で、かつ射程内のターゲットに照準が合っているときだけ true。
  /// 非常停止・手動・テストモードでは runControlCycle が aimed_at_target_ を
  /// 立てないので、自動的に false になる。
  /// 理由文字列を組み立てる。制御周期で呼ぶので固定長バッファで済ませる。
  template<typename ... Args>
  static std::string formatReason(const char * format, Args ... args)
  {
    char buffer[192];
    const int written = std::snprintf(buffer, sizeof(buffer), format, args ...);
    if (written < 0) {
      return "format error";
    }
    return std::string(buffer);
  }

  /// 発射可否とその理由を発行する。「なぜ撃たないか」の切り分け用。
  ///
  /// turret_auto が落ちているのか、射程外なのか、照準が合っていないのか、
  /// ターゲットを見失っているのかが `ros2 topic echo` で分かるようにする。
  void publishShootStatus()
  {
    if (!shoot_status_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    if (!turret_auto_) {
      msg.data = "blocked: turret_auto off";
    } else if (aimed_at_target_) {
      msg.data = "firing";
    } else {
      msg.data = "blocked: " + shoot_block_reason_;
    }
    shoot_status_pub_->publish(msg);
  }

  void publishShootFullauto()
  {
    std_msgs::msg::Bool msg;
    msg.data = turret_auto_ && aimed_at_target_;
    shoot_fullauto_pub_->publish(msg);
  }

  void publishCommandTarget()
  {
    if (!has_command_target_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "publishCommandTarget skipped: command target not initialized");
      return;
    }
    motorPublish(yaw_motor_id_, static_cast<float>(command_yaw_angle_));
    motorPublish(pitch_motor_id_, static_cast<float>(command_pitch_angle_));
  }

  void motorPublish(int id, float data)
  {
    core_shooter::publishMotorCommand(can_pub_, id, data);
  }

  // ===== 内部変数 =====
  bool hazard_state_ = true;
  bool has_joint_state_ = false;

  double yaw_angle_ = 0.0;
  double pitch_angle_ = 0.0;
  double target_image_x_ = 0.0;
  double target_image_y_ = 0.0;
  double previous_target_image_x_ = 0.0;
  double previous_target_image_y_ = 0.0;
  double target_image_velocity_x_ = 0.0;
  double target_image_velocity_y_ = 0.0;
  Vector3 target_point_;
  Vector3 previous_target_point_;
  Vector3 target_point_velocity_;
  bool has_target_ = false;
  bool has_previous_target_sample_ = false;
  bool has_target_velocity_ = false;
  rclcpp::Time last_target_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_target_sample_time_{0, 0, RCL_ROS_TIME};
  double test_yaw_target_ = 0.0;
  double test_pitch_target_ = 0.0;
  bool has_test_yaw_target_ = false;
  bool has_test_pitch_target_ = false;
  core_shooter::TestModeGate test_mode_;
  bool manual_mode_active_ = false;
  bool manual_mode_init_pending_ = false;
  bool manual_mode_return_active_ = false;
  double manual_pitch_target_ = 0.0;
  bool has_manual_pitch_target_ = false;
  rclcpp::Time last_manual_pitch_time_{0, 0, RCL_ROS_TIME};
  bool has_command_target_ = false;
  double command_yaw_angle_ = 0.0;
  double command_pitch_angle_ = 0.0;
  bool auto_track_timeout_active_ = false;
  bool auto_track_timeout_returned_to_startup_ = false;
  rclcpp::Time auto_track_timeout_start_{0, 0, RCL_ROS_TIME};
  bool startup_release_init_pending_ = true;
  bool startup_release_hold_active_ = false;
  bool has_active_control_mode_ = false;
  ControlMode active_control_mode_ = ControlMode::AutoTrack;
  bool aimed_at_target_ = false;
  bool turret_auto_ = false;
  std::string shoot_block_reason_ = "starting up";

  double rate_;
  double pitch_offset_;
  double yaw_min_angle_;
  double yaw_max_angle_;
  double pitch_min_angle_;
  double pitch_max_angle_;
  TargetInputMode target_input_mode_ = TargetInputMode::Image;
  Point3DFrame point3d_frame_ = Point3DFrame::Turret;
  double point3d_tolerance_yaw_rad_ = 0.01;
  double point3d_tolerance_pitch_rad_ = 0.01;
  double point3d_min_range_m_ = 0.05;
  double point3d_max_range_m_ = 0.0;
  bool point3d_limit_command_rate_ = true;
  double point3d_target_velocity_max_m_per_sec_ = 10.0;
  BallisticTable ballistic_table_;
  double ballistic_max_shoot_range_m_ = 5.0;
  double shoot_status_rate_ = 2.0;
  double image_shoot_tolerance_x_px_ = 50.0;
  double image_shoot_tolerance_y_px_ = 50.0;
  double camera_offset_y_m_ = 0.0;
  double camera_offset_z_m_ = 0.0;
  double aim_tolerance_x_m_ = 0.5;
  double aim_tolerance_y_m_ = 0.08;
  double aim_tolerance_z_m_ = 0.08;
  double image_center_x_;
  double image_center_y_;
  double image_width_;
  double image_height_;
  double horizontal_fov_deg_;
  bool use_fov_image_tracking_ = true;
  double image_tolerance_x_;
  double image_tolerance_y_;
  double target_lead_time_sec_;
  double target_velocity_min_dt_sec_;
  double target_velocity_max_px_per_sec_;
  double target_velocity_ema_alpha_;
  double max_yaw_rate_;
  double max_pitch_rate_;
  double yaw_image_gain_;
  double pitch_image_gain_;
  double yaw_direction_;
  double pitch_direction_;
  double target_timeout_sec_;
  double target_lost_return_to_startup_delay_sec_;
  bool enable_test_mode_ = false;
  double test_yaw_gain_ = 0.05;
  double test_pitch_gain_ = 0.05;
  rclcpp::Time last_test_yaw_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_test_pitch_time_{0, 0, RCL_ROS_TIME};
  double manual_mode_yaw_fixed_angle_ = 0.0;
  double manual_mode_pitch_initial_angle_ = 0.0;
  double startup_release_yaw_angle_ = 0.0;
  double startup_release_pitch_angle_ = 0.0;
  bool enable_angle_limit_ = false;
  TurretEnvelope envelope_;
  int pitch_motor_id_;
  int yaw_motor_id_;
  // ROS通信
  rclcpp::Publisher<core_msgs::msg::CANArray>::SharedPtr can_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shoot_fullauto_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shoot_status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_image_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_point_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr test_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr test_yaw_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr test_pitch_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr manual_pitch_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hazard_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr turret_auto_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr shoot_status_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AimBot>());
  rclcpp::shutdown();
  return 0;
}
