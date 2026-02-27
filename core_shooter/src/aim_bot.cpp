#include <memory>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "core_msgs/msg/can.hpp"
#include "core_msgs/msg/can_array.hpp"
#include "core_shooter/forbidden_region_planner.hpp"

using namespace std::chrono_literals;

class AimBot : public rclcpp::Node
{
public:
  AimBot()
  : Node("aim_bot")
  {
    // ----------------------------
    // パラメータ宣言
    // ----------------------------
    this->declare_parameter<double>("rate", 30.0);
    this->declare_parameter<int>("pitch_motor_id", 10);
    this->declare_parameter<int>("yaw_motor_id", 7);
    this->declare_parameter<double>("pitch_offset", 0.0);
    this->declare_parameter<double>("yaw_min_angle", -3.14159265359);
    this->declare_parameter<double>("yaw_max_angle", 3.14159265359);
    this->declare_parameter<double>("pitch_min_angle", -3.14159265359);
    this->declare_parameter<double>("pitch_max_angle", 3.14159265359);
    this->declare_parameter<double>("image_center_x", 0.5);
    this->declare_parameter<double>("image_center_y", 0.5);
    this->declare_parameter<double>("image_width", 1280.0);
    this->declare_parameter<double>("image_height", 720.0);
    this->declare_parameter<double>("horizontal_fov_deg", 100.0);
    this->declare_parameter<bool>("use_fov_image_tracking", true);
    this->declare_parameter<double>("image_tolerance_x", 0.02);
    this->declare_parameter<double>("image_tolerance_y", 0.02);
    this->declare_parameter<double>("yaw_image_gain", 0.5);
    this->declare_parameter<double>("pitch_image_gain", 0.5);
    this->declare_parameter<double>("yaw_direction", 1.0);
    this->declare_parameter<double>("pitch_direction", 1.0);
    this->declare_parameter<double>("target_timeout_sec", 0.2);
    this->declare_parameter<bool>("enable_test_mode", false);
    this->declare_parameter<double>("test_yaw_gain", 0.05);
    this->declare_parameter<double>("test_pitch_gain", 0.05);
    this->declare_parameter<double>("manual_mode_yaw_fixed_angle", 0.0);
    this->declare_parameter<double>("manual_mode_pitch_initial_angle", 0.0);
    this->declare_parameter<bool>("forbidden_region_enabled", false);
    this->declare_parameter<std::vector<double>>(
      "forbidden_region_points_yaw",
      std::vector<double>{});
    this->declare_parameter<std::vector<double>>(
      "forbidden_region_points_pitch",
      std::vector<double>{});
    this->declare_parameter<double>("forbidden_region_point_radius", 0.0);
    this->declare_parameter<bool>("forbidden_region_connect_as_polyline", true);
    this->declare_parameter<double>("forbidden_region_polyline_margin", 0.0);
    this->declare_parameter<bool>("forbidden_region_closed_loop", false);
    this->declare_parameter<double>("forbidden_region_detour_extra_margin", 0.02);

    // ----------------------------
    // パラメータ取得
    // ----------------------------
    this->get_parameter("rate", rate_);
    this->get_parameter("pitch_offset", pitch_offset_);
    this->get_parameter("yaw_min_angle", yaw_min_angle_);
    this->get_parameter("yaw_max_angle", yaw_max_angle_);
    this->get_parameter("pitch_min_angle", pitch_min_angle_);
    this->get_parameter("pitch_max_angle", pitch_max_angle_);
    this->get_parameter("image_center_x", image_center_x_);
    this->get_parameter("image_center_y", image_center_y_);
    this->get_parameter("image_width", image_width_);
    this->get_parameter("image_height", image_height_);
    this->get_parameter("horizontal_fov_deg", horizontal_fov_deg_);
    this->get_parameter("use_fov_image_tracking", use_fov_image_tracking_);
    this->get_parameter("image_tolerance_x", image_tolerance_x_);
    this->get_parameter("image_tolerance_y", image_tolerance_y_);
    this->get_parameter("yaw_image_gain", yaw_image_gain_);
    this->get_parameter("pitch_image_gain", pitch_image_gain_);
    this->get_parameter("yaw_direction", yaw_direction_);
    this->get_parameter("pitch_direction", pitch_direction_);
    this->get_parameter("target_timeout_sec", target_timeout_sec_);
    this->get_parameter("enable_test_mode", enable_test_mode_);
    this->get_parameter("test_yaw_gain", test_yaw_gain_);
    this->get_parameter("test_pitch_gain", test_pitch_gain_);
    this->get_parameter("manual_mode_yaw_fixed_angle", manual_mode_yaw_fixed_angle_);
    this->get_parameter("manual_mode_pitch_initial_angle", manual_mode_pitch_initial_angle_);
    this->get_parameter("forbidden_region_enabled", forbidden_region_enabled_);
    this->get_parameter("forbidden_region_points_yaw", forbidden_region_points_yaw_);
    this->get_parameter("forbidden_region_points_pitch", forbidden_region_points_pitch_);
    this->get_parameter("forbidden_region_point_radius", forbidden_region_point_radius_);
    this->get_parameter(
      "forbidden_region_connect_as_polyline", forbidden_region_connect_as_polyline_);
    this->get_parameter("forbidden_region_polyline_margin", forbidden_region_polyline_margin_);
    this->get_parameter("forbidden_region_closed_loop", forbidden_region_closed_loop_);
    this->get_parameter(
      "forbidden_region_detour_extra_margin", forbidden_region_detour_extra_margin_);
    this->get_parameter("pitch_motor_id", pitch_motor_id_);
    this->get_parameter("yaw_motor_id", yaw_motor_id_);

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
      image_tolerance_x_ < 0.0 || image_tolerance_y_ < 0.0 || target_timeout_sec_ < 0.0)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid image params: image_width=%f, image_height=%f, horizontal_fov_deg=%f, test_yaw_gain=%f, test_pitch_gain=%f, image_tolerance_x=%f, image_tolerance_y=%f, target_timeout_sec=%f",
        image_width_, image_height_, horizontal_fov_deg_, test_yaw_gain_, test_pitch_gain_,
        image_tolerance_x_, image_tolerance_y_, target_timeout_sec_);
      throw std::runtime_error("invalid aimbot image parameters");
    }
    if (use_fov_image_tracking_ &&
      (horizontal_fov_deg_ <= 0.0 || horizontal_fov_deg_ >= 180.0))
    {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid horizontal_fov_deg=%f for FOV image tracking (must be 0 < hfov < 180)",
        horizontal_fov_deg_);
      throw std::runtime_error("invalid aimbot horizontal fov");
    }
    if (forbidden_region_point_radius_ < 0.0 ||
      forbidden_region_polyline_margin_ < 0.0 ||
      forbidden_region_detour_extra_margin_ < 0.0)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid forbidden region params: point_radius=%f, polyline_margin=%f, detour_extra_margin=%f",
        forbidden_region_point_radius_, forbidden_region_polyline_margin_,
        forbidden_region_detour_extra_margin_);
      throw std::runtime_error("invalid forbidden region parameters");
    }

    configureForbiddenRegionPlanner();

    // ----------------------------
    // Subscriber
    // ----------------------------
    target_image_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "target_image_position", 10,
      std::bind(&AimBot::targetImageCallback, this, std::placeholders::_1));

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

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&AimBot::jointStateCallback, this, std::placeholders::_1));

    // ----------------------------
    // Publisher
    // ----------------------------
    can_pub_ = this->create_publisher<core_msgs::msg::CANArray>("/can/tx", 10);

    // ----------------------------
    // Timer
    // ----------------------------
    const auto period = std::chrono::duration<double>(1.0 / rate_);
    timer_ = create_wall_timer(period, std::bind(&AimBot::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "AimBot started. target_image_position=PointStamped(center-origin x/y px, z:0=detected 1=none), image_size=(%.0f x %.0f), tracking=%s, hfov=%.1fdeg, image_tolerance=(%.3f, %.3f), test_mode_default=%s(topic override supported)",
      image_width_, image_height_, use_fov_image_tracking_ ? "fov" : "gain",
      horizontal_fov_deg_, image_tolerance_x_, image_tolerance_y_,
      enable_test_mode_ ? "true" : "false");
  }

private:
  enum class ControlMode
  {
    Emergency,
    Manual,
    Test,
    AutoTrack
  };

  void configureForbiddenRegionPlanner()
  {
    core_shooter::ForbiddenRegionPlanner::Config config;
    config.enabled = forbidden_region_enabled_;
    config.points_yaw = forbidden_region_points_yaw_;
    config.points_pitch = forbidden_region_points_pitch_;
    config.point_radius = forbidden_region_point_radius_;
    config.connect_as_polyline = forbidden_region_connect_as_polyline_;
    config.polyline_margin = forbidden_region_polyline_margin_;
    config.closed_loop = forbidden_region_closed_loop_;
    config.detour_extra_margin = forbidden_region_detour_extra_margin_;
    config.yaw_min_angle = yaw_min_angle_;
    config.yaw_max_angle = yaw_max_angle_;
    config.pitch_min_angle = pitch_min_angle_;
    config.pitch_max_angle = pitch_max_angle_;

    try {
      forbidden_region_planner_.setConfig(config);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        get_logger(), "Failed to configure forbidden region planner: %s", e.what());
      throw;
    }

    if (forbidden_region_enabled_ && forbidden_region_planner_.boundaryPointCount() == 0) {
      RCLCPP_WARN(
        get_logger(),
        "forbidden_region_enabled=true but no boundary points configured; planner disabled effectively");
    }
    if (forbidden_region_closed_loop_ && forbidden_region_planner_.boundaryPointCount() < 3) {
      RCLCPP_WARN(
        get_logger(),
        "forbidden_region_closed_loop=true requires >=3 points (current=%zu); using open polyline",
        forbidden_region_planner_.boundaryPointCount());
    }
  }

  // ===== コールバック =====
  void hazardCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool prev = hazard_state_;
    hazard_state_ = msg->data;

    if (!hazard_state_ && prev) {
      if (zero_init_after_restart_pending_) {
        // ノード再起動後、最初の緊急停止解除時のみ yaw/pitch=0 を初期目標にする。
        setCommandTarget(0.0, 0.0);
        zero_init_after_restart_pending_ = false;
        RCLCPP_INFO(
          this->get_logger(),
          "First emergency release after restart: initialize command target to yaw=0, pitch=0");
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
    }

    if (!manual_mode_active_ && prev) {
      manual_mode_init_pending_ = false;
      RCLCPP_INFO(this->get_logger(), "Manual mode OFF");
    }
  }

  void testModeCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool prev_effective = isTestModeEnabled();
    test_mode_topic_value_ = msg->data;
    has_test_mode_topic_value_ = true;
    const bool next_effective = isTestModeEnabled();
    if (next_effective != prev_effective) {
      RCLCPP_INFO(
        this->get_logger(), "Test mode %s (source=topic, param fallback=%s)",
        next_effective ? "ON" : "OFF", enable_test_mode_ ? "true" : "false");
    }
  }

  void targetImageCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    // PointStamped semantics:
    //   detected     -> point = <x, y, 0> (x/y are centered image coordinates)
    //   not detected -> point = <0, 0, 1>
    const bool detected = (msg->point.z < 0.5);
    if (!detected) {
      has_target_ = false;
      target_image_x_ = 0.0;
      target_image_y_ = 0.0;
      return;
    }

    target_image_x_ = msg->point.x;
    target_image_y_ = msg->point.y;
    has_target_ = true;
    last_target_time_ = this->now();
  }

  void testYawCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    test_yaw_target_ = msg->data;
    has_test_yaw_target_ = true;
  }

  void testPitchCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    test_pitch_target_ = msg->data;
    has_test_pitch_target_ = true;
  }

  void manualPitchCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    manual_pitch_target_ = msg->data;
    has_manual_pitch_target_ = true;
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
    const bool test_mode_enabled = isTestModeEnabled();
    ControlMode mode = ControlMode::AutoTrack;
    if (hazard_state_) {
      mode = ControlMode::Emergency;
    } else if (manual_mode_active_) {
      mode = ControlMode::Manual;
    } else if (test_mode_enabled) {
      mode = ControlMode::Test;
    }

    const bool entering_mode = setActiveControlMode(mode);

    switch (mode) {
      case ControlMode::Emergency:
        if (entering_mode && has_joint_state_) {
          // Latch actual angle once when entering emergency, then keep holding that command.
          setCommandTarget(yaw_angle_, pitch_angle_);
        }
        publishCommandHold("emergency hold skipped: command target not initialized");
        return;

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
              setCommandTarget(manual_mode_yaw_fixed_angle_, manual_mode_pitch_initial_angle_);
              manual_mode_init_pending_ = false;
              RCLCPP_INFO(
                this->get_logger(),
                "Manual mode ON: set yaw=%f, pitch=%f (pitch becomes controllable via manual_pitch_angle)",
                command_yaw_angle_, command_pitch_angle_);
            }
          }

          if (!has_manual_pitch_target_) {
            RCLCPP_WARN_THROTTLE(
              this->get_logger(),
              *this->get_clock(), 2000,
              "manual_mode active but manual_pitch_angle not received yet: holding manual initial pitch command");
          }
          double manual_pitch = command_pitch_angle_;
          if (has_manual_pitch_target_) {
            const double pitch_delta = std::clamp(manual_pitch_target_, -1.0, 1.0);
            manual_pitch = command_pitch_angle_ + pitch_direction_ * test_pitch_gain_ * pitch_delta;
          }
          setCommandTarget(manual_mode_yaw_fixed_angle_, manual_pitch);
          publishCommandTarget();
          return;
        }

      case ControlMode::Test:
        if (entering_mode) {
          // Require fresh test inputs after every test-mode entry to avoid reusing stale values.
          has_test_yaw_target_ = false;
          has_test_pitch_target_ = false;
        }
        if (!has_test_yaw_target_ || !has_test_pitch_target_) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "test mode enabled but test yaw/pitch target not received yet");
          publishCommandHold("test mode hold skipped: command target not initialized");
          return;
        }
        if (!ensureCommandTarget("test mode enabled but joint_states not received yet")) {
          return;
        }

        // test input is normalized delta command [-1, 1] integrated on the internal command target.
        {
          const double yaw_delta = std::clamp(test_yaw_target_, -1.0, 1.0);
          const double next_yaw = command_yaw_angle_ + yaw_direction_ * test_yaw_gain_ * yaw_delta;

          const double pitch_delta = std::clamp(test_pitch_target_, -1.0, 1.0);
          const double next_pitch =
            command_pitch_angle_ + pitch_direction_ * test_pitch_gain_ * pitch_delta;

          setCommandTarget(next_yaw, next_pitch);
          publishCommandTarget();
          return;
        }

      case ControlMode::AutoTrack:
        if (!has_target_ || (this->now() - last_target_time_).seconds() > target_timeout_sec_) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "target_image_position timeout");
          publishCommandHold("no-input hold skipped: command target not initialized");
          return;
        }

        if (entering_mode && has_joint_state_) {
          // Auto mode baseline is actual joint angle; pitch_offset is applied once at auto entry.
          setCommandTarget(yaw_angle_, pitch_angle_ + pitch_offset_);
        }

        if (!ensureCommandTarget("auto track skipped: command/joint_states not available")) {
          return;
        }

        // 入力は中心原点のピクセル座標。追尾方式はパラメータで切替。
        {
          const double x_error = target_image_x_;
          const double y_error = target_image_y_;

          // モータ側が位置制御を行う前提で、内部の目標角を更新する
          double yaw_target = command_yaw_angle_;
          double pitch_target = command_pitch_angle_;

          if (use_fov_image_tracking_) {
            constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
            const double horizontal_fov_rad = horizontal_fov_deg_ * kDegToRad;
            const double vertical_fov_rad = horizontal_fov_rad * (image_height_ / image_width_);
            const double half_hfov_rad = 0.5 * horizontal_fov_rad;
            const double half_vfov_rad = 0.5 * vertical_fov_rad;

            if (std::fabs(x_error) > image_tolerance_x_) {
              // rectilinear camera assumption:
              // x_px in [-image_width/2, image_width/2] -> yaw offset in [-hfov/2, hfov/2]
              const double x_norm = std::clamp((2.0 * x_error) / image_width_, -1.0, 1.0);
              const double yaw_offset_rad = std::atan(x_norm * std::tan(half_hfov_rad));
              const double yaw_base = has_joint_state_ ? yaw_angle_ : command_yaw_angle_;
              yaw_target = yaw_base + yaw_direction_ * yaw_offset_rad;
            }
            if (std::fabs(y_error) > image_tolerance_y_) {
              // Vertical FOV is derived from horizontal FOV by image aspect ratio.
              const double y_norm = std::clamp((2.0 * y_error) / image_height_, -1.0, 1.0);
              const double pitch_offset_rad = std::atan(y_norm * std::tan(half_vfov_rad));
              const double pitch_base = has_joint_state_ ?
                (pitch_angle_ + pitch_offset_) :
                command_pitch_angle_;
              pitch_target = pitch_base + pitch_direction_ * pitch_offset_rad;
            }
          } else {
            if (std::fabs(x_error) > image_tolerance_x_) {
              yaw_target += yaw_direction_ * yaw_image_gain_ * x_error;
            }
            if (std::fabs(y_error) > image_tolerance_y_) {
              pitch_target += pitch_direction_ * pitch_image_gain_ * y_error;
            }
          }

          setCommandTarget(yaw_target, pitch_target);
          publishCommandTarget();
          return;
        }
    }
  }

  double clampYaw(double angle) const
  {
    return std::clamp(angle, yaw_min_angle_, yaw_max_angle_);
  }

  double clampPitch(double angle) const
  {
    return std::clamp(angle, pitch_min_angle_, pitch_max_angle_);
  }

  void setCommandTarget(double yaw, double pitch)
  {
    // Forbidden region planning is evaluated in joint_state measured angle space.
    // If joint_states are not available yet, planner falls back to target-only adjustment.
    const auto adjusted = forbidden_region_planner_.adjustTarget(
      yaw, pitch,
      has_joint_state_, yaw_angle_, pitch_angle_);
    command_yaw_angle_ = adjusted.yaw;
    command_pitch_angle_ = adjusted.pitch;
    has_command_target_ = true;
  }

  bool latchCommandTargetFromJointState(double pitch_bias = 0.0)
  {
    if (!has_joint_state_) {
      return false;
    }
    setCommandTarget(yaw_angle_, pitch_angle_ + pitch_bias);
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
    return has_test_mode_topic_value_ ? test_mode_topic_value_ : enable_test_mode_;
  }

  const char * controlModeName(ControlMode mode) const
  {
    switch (mode) {
      case ControlMode::Emergency:
        return "Emergency";
      case ControlMode::Manual:
        return "Manual";
      case ControlMode::Test:
        return "Test";
      case ControlMode::AutoTrack:
      default:
        return "AutoTrack";
    }
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

    // Log both command target and measured joint state for debugging.
    if (has_joint_state_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "[%s] cmd(yaw=%.4f,pitch=%.4f) joint(yaw=%.4f,pitch=%.4f) err(yaw=%.4f,pitch=%.4f)",
        controlModeName(active_control_mode_),
        command_yaw_angle_, command_pitch_angle_,
        yaw_angle_, pitch_angle_,
        command_yaw_angle_ - yaw_angle_, command_pitch_angle_ - pitch_angle_);
    } else {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "[%s] cmd(yaw=%.4f,pitch=%.4f) joint=unavailable",
        controlModeName(active_control_mode_), command_yaw_angle_, command_pitch_angle_);
    }
  }

  void motorPublish(int id, float data)
  {
    auto can_array = core_msgs::msg::CANArray();
    auto can = core_msgs::msg::CAN();
    can.id = id;
    can.data.push_back(data);
    can_array.array.push_back(can);
    can_pub_->publish(can_array);
  }

  // ===== 内部変数 =====
  bool hazard_state_ = true;
  bool has_joint_state_ = false;

  double yaw_angle_ = 0.0;
  double pitch_angle_ = 0.0;
  double target_image_x_ = 0.0;
  double target_image_y_ = 0.0;
  bool has_target_ = false;
  rclcpp::Time last_target_time_{0, 0, RCL_ROS_TIME};
  double test_yaw_target_ = 0.0;
  double test_pitch_target_ = 0.0;
  bool has_test_yaw_target_ = false;
  bool has_test_pitch_target_ = false;
  bool test_mode_topic_value_ = false;
  bool has_test_mode_topic_value_ = false;
  bool manual_mode_active_ = false;
  bool manual_mode_init_pending_ = false;
  double manual_pitch_target_ = 0.0;
  bool has_manual_pitch_target_ = false;
  bool has_command_target_ = false;
  double command_yaw_angle_ = 0.0;
  double command_pitch_angle_ = 0.0;
  bool zero_init_after_restart_pending_ = true;
  bool has_active_control_mode_ = false;
  ControlMode active_control_mode_ = ControlMode::AutoTrack;

  double rate_;
  double pitch_offset_;
  double yaw_min_angle_;
  double yaw_max_angle_;
  double pitch_min_angle_;
  double pitch_max_angle_;
  double image_center_x_;
  double image_center_y_;
  double image_width_;
  double image_height_;
  double horizontal_fov_deg_;
  bool use_fov_image_tracking_ = true;
  double image_tolerance_x_;
  double image_tolerance_y_;
  double yaw_image_gain_;
  double pitch_image_gain_;
  double yaw_direction_;
  double pitch_direction_;
  double target_timeout_sec_;
  bool enable_test_mode_ = false;
  double test_yaw_gain_ = 0.05;
  double test_pitch_gain_ = 0.05;
  double manual_mode_yaw_fixed_angle_ = 0.0;
  double manual_mode_pitch_initial_angle_ = 0.0;
  bool forbidden_region_enabled_ = false;
  std::vector<double> forbidden_region_points_yaw_;
  std::vector<double> forbidden_region_points_pitch_;
  double forbidden_region_point_radius_ = 0.0;
  bool forbidden_region_connect_as_polyline_ = true;
  double forbidden_region_polyline_margin_ = 0.0;
  bool forbidden_region_closed_loop_ = false;
  double forbidden_region_detour_extra_margin_ = 0.02;
  int pitch_motor_id_;
  int yaw_motor_id_;
  core_shooter::ForbiddenRegionPlanner forbidden_region_planner_;

  // ROS通信
  rclcpp::Publisher<core_msgs::msg::CANArray>::SharedPtr can_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_image_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr test_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr test_yaw_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr test_pitch_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr manual_pitch_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hazard_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AimBot>());
  rclcpp::shutdown();
  return 0;
}
