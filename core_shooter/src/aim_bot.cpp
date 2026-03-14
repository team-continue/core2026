#include <memory>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "core_msgs/msg/can.hpp"
#include "core_msgs/msg/can_array.hpp"

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
    this->declare_parameter<double>("target_lost_return_to_startup_delay_sec", 2.0);
    this->declare_parameter<bool>("enable_test_mode", false);
    this->declare_parameter<double>("test_yaw_gain", 0.05);
    this->declare_parameter<double>("test_pitch_gain", 0.05);
    this->declare_parameter<double>("manual_mode_yaw_fixed_angle", 0.0);
    this->declare_parameter<double>("manual_mode_pitch_initial_angle", 0.0);
    this->declare_parameter<double>("startup_release_yaw_angle", 0.0);
    this->declare_parameter<double>("startup_release_pitch_angle", 0.0);
    this->declare_parameter<bool>("enable_zone_angle_limit", false);
    this->declare_parameter<bool>("zone.yaw_reversed", false);
    this->declare_parameter<double>("zone.yaw_zone1_start", -3.14159265359);
    this->declare_parameter<double>("zone.yaw_boundary", -1.57079632679);
    this->declare_parameter<double>("zone.yaw_zone2_end", 1.57079632679);
    this->declare_parameter<double>("zone.yaw_zone3_end", 3.14159265359);
    this->declare_parameter<double>("zone.pitch_lower_limit", -3.14159265359);
    this->declare_parameter<double>("zone.pitch_zone2_upper", 0.52359877559);
    this->declare_parameter<double>("zone.pitch_zone2_lower", -3.14159265359);
    this->declare_parameter<double>("zone.pitch_zone2_upper_limit", 3.14159265359);
    this->declare_parameter<double>("zone.pitch_zone3_lower", -0.52359877559);
    this->declare_parameter<double>("zone.pitch_zone3_upper", 3.14159265359);
    this->declare_parameter<double>("zone.pitch_zone1_upper", 3.14159265359);
    this->declare_parameter<double>("control.hysteresis_rad", 0.017453292519943295);
    this->declare_parameter<double>("control.pitch_correct_tolerance", 0.01);

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
    this->get_parameter(
      "target_lost_return_to_startup_delay_sec", target_lost_return_to_startup_delay_sec_);
    this->get_parameter("enable_test_mode", enable_test_mode_);
    this->get_parameter("test_yaw_gain", test_yaw_gain_);
    this->get_parameter("test_pitch_gain", test_pitch_gain_);
    this->get_parameter("manual_mode_yaw_fixed_angle", manual_mode_yaw_fixed_angle_);
    this->get_parameter("manual_mode_pitch_initial_angle", manual_mode_pitch_initial_angle_);
    this->get_parameter("startup_release_yaw_angle", startup_release_yaw_angle_);
    this->get_parameter("startup_release_pitch_angle", startup_release_pitch_angle_);
    this->get_parameter("pitch_motor_id", pitch_motor_id_);
    this->get_parameter("yaw_motor_id", yaw_motor_id_);
    this->get_parameter("enable_zone_angle_limit", enable_zone_angle_limit_);
    this->get_parameter("zone.yaw_reversed", zone_yaw_reversed_);
    this->get_parameter("zone.yaw_zone1_start", zone_yaw_zone1_start_);
    this->get_parameter("zone.yaw_boundary", zone_yaw_boundary_);
    this->get_parameter("zone.yaw_zone2_end", zone_yaw_zone2_end_);
    this->get_parameter("zone.yaw_zone3_end", zone_yaw_zone3_end_);
    this->get_parameter("zone.pitch_lower_limit", zone_pitch_lower_limit_);
    this->get_parameter("zone.pitch_zone2_upper", zone_pitch_zone2_upper_);
    this->get_parameter("zone.pitch_zone2_lower", zone_pitch_zone2_lower_);
    this->get_parameter("zone.pitch_zone2_upper_limit", zone_pitch_zone2_upper_limit_);
    this->get_parameter("zone.pitch_zone3_lower", zone_pitch_zone3_lower_);
    this->get_parameter("zone.pitch_zone3_upper", zone_pitch_zone3_upper_);
    this->get_parameter("zone.pitch_zone1_upper", zone_pitch_zone1_upper_);
    this->get_parameter("control.hysteresis_rad", control_hysteresis_rad_);
    this->get_parameter("control.pitch_correct_tolerance", control_pitch_correct_tolerance_);
    const auto & parameter_overrides =
      this->get_node_parameters_interface()->get_parameter_overrides();
    zone_pitch_zone1_upper_overridden_ =
      parameter_overrides.find("zone.pitch_zone1_upper") != parameter_overrides.end();
    zone_pitch_zone2_upper_overridden_ =
      parameter_overrides.find("zone.pitch_zone2_upper") != parameter_overrides.end();
    zone_pitch_zoneab_upper_ = zone_pitch_zone1_upper_overridden_ ?
      zone_pitch_zone1_upper_ :
      zone_pitch_zone2_upper_;
    if (zone_pitch_zone1_upper_overridden_ && zone_pitch_zone2_upper_overridden_ &&
      std::fabs(zone_pitch_zone1_upper_ - zone_pitch_zone2_upper_) > 1e-9)
    {
      RCLCPP_WARN(
        get_logger(),
        "Both zone.pitch_zone1_upper=%f and zone.pitch_zone2_upper=%f are set. Using zone.pitch_zone1_upper as the ZoneAB upper limit.",
        zone_pitch_zone1_upper_, zone_pitch_zone2_upper_);
    } else if (!zone_pitch_zone1_upper_overridden_ && zone_pitch_zone2_upper_overridden_) {
      RCLCPP_WARN(
        get_logger(),
        "zone.pitch_zone2_upper is treated as the ZoneAB upper limit for compatibility. Prefer zone.pitch_zone1_upper.");
    }

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
      target_lost_return_to_startup_delay_sec_ < 0.0)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid image params: image_width=%f, image_height=%f, horizontal_fov_deg=%f, test_yaw_gain=%f, test_pitch_gain=%f, image_tolerance_x=%f, image_tolerance_y=%f, target_timeout_sec=%f, target_lost_return_to_startup_delay_sec=%f",
        image_width_, image_height_, horizontal_fov_deg_, test_yaw_gain_, test_pitch_gain_,
        image_tolerance_x_, image_tolerance_y_, target_timeout_sec_,
        target_lost_return_to_startup_delay_sec_);
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
    {
      std::string zone_config_error;
      if (!validateZoneConfig(
          zone_yaw_zone1_start_, zone_yaw_boundary_, zone_yaw_zone2_end_, zone_yaw_zone3_end_,
          zone_pitch_lower_limit_, zone_pitch_zoneab_upper_, zone_pitch_zone2_lower_,
          zone_pitch_zone2_upper_limit_, zone_pitch_zone3_lower_, zone_pitch_zone3_upper_,
          control_hysteresis_rad_, zone_config_error))
      {
        RCLCPP_FATAL(
          get_logger(),
          "Invalid zone angle limit parameters: %s",
          zone_config_error.c_str());
        throw std::runtime_error("invalid aimbot zone angle limit parameters");
      }
    }

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
      "AimBot started. target_image_position=PointStamped(center-origin x/y px, z:0=detected 1=none), image_size=(%.0f x %.0f), target_center_norm=(%.3f, %.3f), target_center_px=(%.1f, %.1f), tracking=%s, hfov=%.1fdeg, image_tolerance=(%.3f, %.3f), target_lost_return_delay=%.2fs, test_mode_default=%s(topic override supported), startup_release_target=(%.3f, %.3f)",
      image_width_, image_height_, image_center_x_, image_center_y_,
      getImageTargetCenterX(), getImageTargetCenterY(),
      use_fov_image_tracking_ ? "fov" : "gain",
      horizontal_fov_deg_, image_tolerance_x_, image_tolerance_y_,
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
  enum class YawZone
  {
    OutOfRange,
    ZoneAB,
    ZoneBC,
    ZoneCD
  };
  enum class PitchCorrectionMode
  {
    None,
    ToTargetPitch
  };

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
    // image_center_x/y specify the desired image center in normalized coordinates.
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
              setManualModeCommandTarget(
                manual_mode_yaw_fixed_angle_, manual_mode_pitch_initial_angle_);
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
          bool manual_input_timed_out = false;
          if (has_manual_pitch_target_ && isTimedOut(last_manual_pitch_time_)) {
            manual_input_timed_out = true;
            has_manual_pitch_target_ = false;
            RCLCPP_WARN_THROTTLE(
              this->get_logger(),
              *this->get_clock(), 2000,
              "manual pitch input timed out: hold current pitch");
          }
          double manual_pitch = command_pitch_angle_;
          if (has_manual_pitch_target_) {
            const double pitch_delta = std::clamp(manual_pitch_target_, -1.0, 1.0);
            manual_pitch = command_pitch_angle_ + pitch_direction_ * test_pitch_gain_ * pitch_delta;
          } else if (manual_input_timed_out && has_joint_state_) {
            manual_pitch = pitch_angle_;
          }
          setManualModeCommandTarget(manual_mode_yaw_fixed_angle_, manual_pitch);
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
            if (has_joint_state_) {
              // 目標喪失時は最初の1回だけ現在角をラッチし、その後はその目標を保持する。
              setCommandTargetRaw(yaw_angle_, pitch_angle_);
              publishCommandTarget();
              return;
            }
          }
          const double timeout_elapsed_sec =
            (this->now() - auto_track_timeout_start_).seconds();
          if (!auto_track_timeout_returned_to_startup_ &&
            timeout_elapsed_sec >= target_lost_return_to_startup_delay_sec_)
          {
            setCommandTarget(startup_release_yaw_angle_, startup_release_pitch_angle_);
            auto_track_timeout_returned_to_startup_ = true;
            RCLCPP_INFO(
              this->get_logger(),
              "target_image_position timeout continued for %.2fs: return to startup_release target yaw=%f, pitch=%f",
              timeout_elapsed_sec, command_yaw_angle_, command_pitch_angle_);
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

        // 入力は中心原点のピクセル座標。image_center_x/y で狙う画像中心をずらし、
        // その中心からの誤差で追尾する。
        {
          const double x_error = target_image_x_ - getImageTargetCenterX();
          const double y_error = target_image_y_ - getImageTargetCenterY();
          const double yaw_base = has_joint_state_ ? yaw_angle_ : command_yaw_angle_;
          const double pitch_base = has_joint_state_ ?
            (pitch_angle_ + pitch_offset_) :
            command_pitch_angle_;

          // AutoTrack は現在角ベースの比例補正にして、画像ノイズでのドリフトを防ぐ。
          double yaw_target = yaw_base;
          double pitch_target = pitch_base;

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

  double clampYaw(double angle) const
  {
    return std::clamp(angle, yaw_min_angle_, yaw_max_angle_);
  }

  double clampPitch(double angle) const
  {
    return std::clamp(angle, pitch_min_angle_, pitch_max_angle_);
  }

  double toZoneYawFrame(double raw_yaw) const
  {
    return zone_yaw_reversed_ ? -raw_yaw : raw_yaw;
  }

  double fromZoneYawFrame(double zone_yaw) const
  {
    return zone_yaw_reversed_ ? -zone_yaw : zone_yaw;
  }

  double clampZoneYaw(double raw_yaw) const
  {
    const double zone_yaw =
      std::clamp(toZoneYawFrame(clampYaw(raw_yaw)), zone_yaw_zone1_start_, zone_yaw_zone3_end_);
    return clampYaw(fromZoneYawFrame(zone_yaw));
  }

  bool validateZoneConfig(
    double yaw_zone1_start, double yaw_boundary, double yaw_zone2_end, double yaw_zone3_end,
    double pitch_lower_limit, double pitch_zoneab_upper, double pitch_zonebc_lower,
    double pitch_zonebc_upper,
    double pitch_zonecd_lower, double pitch_zonecd_upper,
    double hysteresis_rad, std::string & reason) const
  {
    if (!(yaw_zone1_start < yaw_boundary &&
      yaw_boundary < yaw_zone2_end &&
      yaw_zone2_end < yaw_zone3_end))
    {
      reason =
        "zone.yaw_zone1_start < zone.yaw_boundary < zone.yaw_zone2_end < zone.yaw_zone3_end is required";
      return false;
    }
    const bool pitch_ab_valid = pitch_lower_limit != pitch_zoneab_upper;
    const bool pitch_zone2_pair_valid = pitch_zonebc_lower != pitch_zonebc_upper;
    const bool pitch_zone3_pair_valid = pitch_zonecd_lower != pitch_zonecd_upper;
    if (!pitch_ab_valid || !pitch_zone2_pair_valid || !pitch_zone3_pair_valid) {
      reason = "zone pitch ranges are invalid";
      return false;
    }
    if (hysteresis_rad < 0.0) {
      reason = "control.hysteresis_rad must be >= 0";
      return false;
    }
    if (control_pitch_correct_tolerance_ < 0.0) {
      reason = "control.pitch_correct_tolerance must be >= 0";
      return false;
    }
    const double zone1_width = yaw_boundary - yaw_zone1_start;
    const double zone2_width = yaw_zone2_end - yaw_boundary;
    const double zone3_width = yaw_zone3_end - yaw_zone2_end;
    if (hysteresis_rad >= zone1_width || hysteresis_rad >= zone2_width ||
      hysteresis_rad >= zone3_width)
    {
      reason = "control.hysteresis_rad is too large for configured yaw zones";
      return false;
    }

    const auto has_pitch_overlap = [this](double a, double b) {
        const double interval_min = std::min(a, b);
        const double interval_max = std::max(a, b);
        const double effective_min = std::max(pitch_min_angle_, interval_min);
        const double effective_max = std::min(pitch_max_angle_, interval_max);
        return effective_min <= effective_max;
      };
    if (!has_pitch_overlap(pitch_lower_limit, pitch_zoneab_upper) ||
      !has_pitch_overlap(pitch_zonebc_lower, pitch_zonebc_upper) ||
      !has_pitch_overlap(pitch_zonecd_lower, pitch_zonecd_upper))
    {
      reason = "zone pitch limits conflict with pitch_min_angle/pitch_max_angle";
      return false;
    }
    return true;
  }

  YawZone classifyYawZone(double yaw)
  {
    if (yaw < zone_yaw_zone1_start_ || yaw > zone_yaw_zone3_end_) {
      return YawZone::OutOfRange;
    }

    const double hysteresis_rad = control_hysteresis_rad_;
    const YawZone reference_zone = has_last_yaw_zone_ ?
      last_yaw_zone_ :
      (yaw <= zone_yaw_boundary_ ? YawZone::ZoneAB :
      (yaw <= zone_yaw_zone2_end_ ? YawZone::ZoneBC : YawZone::ZoneCD));

    if (reference_zone == YawZone::ZoneAB) {
      return yaw > (zone_yaw_boundary_ + hysteresis_rad) ? YawZone::ZoneBC : YawZone::ZoneAB;
    }
    if (reference_zone == YawZone::ZoneBC) {
      if (yaw < (zone_yaw_boundary_ - hysteresis_rad)) {
        return YawZone::ZoneAB;
      }
      if (yaw > (zone_yaw_zone2_end_ + hysteresis_rad)) {
        return YawZone::ZoneCD;
      }
      return YawZone::ZoneBC;
    }
    return yaw < (zone_yaw_zone2_end_ - hysteresis_rad) ? YawZone::ZoneBC : YawZone::ZoneCD;
  }

  std::pair<double, double> getZonePitchLimits(YawZone zone) const
  {
    if (zone == YawZone::ZoneAB) {
      return {zone_pitch_lower_limit_, zone_pitch_zoneab_upper_};
    }
    if (zone == YawZone::ZoneCD) {
      return {zone_pitch_zone3_lower_, zone_pitch_zone3_upper_};
    }
    return {zone_pitch_zone2_lower_, zone_pitch_zone2_upper_limit_};
  }

  std::pair<double, double> applyZoneAngleLimit(double yaw, double pitch)
  {
    double limited_yaw = clampYaw(yaw);
    double limited_pitch = clampPitch(pitch);

    if (!enable_zone_angle_limit_) {
      pitch_correction_mode_ = PitchCorrectionMode::None;
      correction_target_zone_ = YawZone::OutOfRange;
      return {limited_yaw, limited_pitch};
    }

    limited_yaw = clampZoneYaw(limited_yaw);
    const double current_yaw = has_command_target_ ?
      clampZoneYaw(command_yaw_angle_) :
      limited_yaw;
    const double current_zone_yaw = toZoneYawFrame(current_yaw);
    const double target_zone_yaw = toZoneYawFrame(limited_yaw);
    const double current_pitch = has_command_target_ ? command_pitch_angle_ : limited_pitch;
    const double feedback_pitch = has_joint_state_ ? pitch_angle_ : current_pitch;
    const YawZone current_zone = classifyYawZone(current_zone_yaw);
    const YawZone target_zone = classifyYawZone(target_zone_yaw);
    if (current_zone == YawZone::OutOfRange || target_zone == YawZone::OutOfRange) {
      pitch_correction_mode_ = PitchCorrectionMode::None;
      correction_target_zone_ = YawZone::OutOfRange;
      return {limited_yaw, limited_pitch};
    }

    const auto [target_pitch_lower_raw, target_pitch_upper_raw] = getZonePitchLimits(target_zone);
    const double target_pitch_min = std::min(target_pitch_lower_raw, target_pitch_upper_raw);
    const double target_pitch_max = std::max(target_pitch_lower_raw, target_pitch_upper_raw);
    if (pitch_correction_mode_ != PitchCorrectionMode::None &&
      correction_target_zone_ != target_zone)
    {
      pitch_correction_mode_ = PitchCorrectionMode::None;
      correction_target_zone_ = YawZone::OutOfRange;
    }
    if (pitch_correction_mode_ == PitchCorrectionMode::None && target_zone != current_zone) {
      if (feedback_pitch < (target_pitch_min - control_pitch_correct_tolerance_)) {
        pitch_correction_mode_ = PitchCorrectionMode::ToTargetPitch;
        correction_hold_yaw_ = current_yaw;
        correction_target_zone_ = target_zone;
        correction_target_pitch_ = target_pitch_min;
      } else if (feedback_pitch > (target_pitch_max + control_pitch_correct_tolerance_)) {
        pitch_correction_mode_ = PitchCorrectionMode::ToTargetPitch;
        correction_hold_yaw_ = current_yaw;
        correction_target_zone_ = target_zone;
        correction_target_pitch_ = target_pitch_max;
      }
    }
    if (pitch_correction_mode_ == PitchCorrectionMode::ToTargetPitch) {
      limited_yaw = clampZoneYaw(correction_hold_yaw_);
      const auto [corr_pitch_lower_raw, corr_pitch_upper_raw] =
        getZonePitchLimits(correction_target_zone_);
      const double corr_pitch_min = std::min(corr_pitch_lower_raw, corr_pitch_upper_raw);
      const double corr_pitch_max = std::max(corr_pitch_lower_raw, corr_pitch_upper_raw);
      limited_pitch = std::clamp(
        correction_target_pitch_,
        std::max(pitch_min_angle_, corr_pitch_min),
        std::min(pitch_max_angle_, corr_pitch_max));
      if (std::fabs(feedback_pitch - correction_target_pitch_) <=
        control_pitch_correct_tolerance_)
      {
        pitch_correction_mode_ = PitchCorrectionMode::None;
        correction_target_zone_ = YawZone::OutOfRange;
      }
    }
    if (pitch_correction_mode_ != PitchCorrectionMode::None) {
      const YawZone hold_zone = classifyYawZone(toZoneYawFrame(limited_yaw));
      has_last_yaw_zone_ = hold_zone != YawZone::OutOfRange;
      if (has_last_yaw_zone_) {
        last_yaw_zone_ = hold_zone;
      }
      return {limited_yaw, limited_pitch};
    }

    const YawZone zone = classifyYawZone(target_zone_yaw);
    const auto [zone_pitch_lower, zone_pitch_upper] = getZonePitchLimits(zone);
    const double pitch_lower =
      std::max(pitch_min_angle_, std::min(zone_pitch_lower, zone_pitch_upper));
    const double pitch_upper =
      std::min(pitch_max_angle_, std::max(zone_pitch_lower, zone_pitch_upper));
    if (pitch_lower > pitch_upper) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "zone pitch limits are invalid after merge with pitch caps. fallback to base clamp only.");
      return {limited_yaw, clampPitch(pitch)};
    }

    limited_pitch = std::clamp(limited_pitch, pitch_lower, pitch_upper);
    has_last_yaw_zone_ = true;
    last_yaw_zone_ = zone;
    return {limited_yaw, limited_pitch};
  }

  void setCommandTarget(double yaw, double pitch)
  {
    const auto [limited_yaw, limited_pitch] = applyZoneAngleLimit(yaw, pitch);
    command_yaw_angle_ = limited_yaw;
    command_pitch_angle_ = limited_pitch;
    has_command_target_ = true;
  }

  void setManualModeCommandTarget(double yaw, double pitch)
  {
    command_yaw_angle_ = enable_zone_angle_limit_ ? clampZoneYaw(yaw) : clampYaw(yaw);
    command_pitch_angle_ = clampPitch(pitch);
    has_command_target_ = true;
    pitch_correction_mode_ = PitchCorrectionMode::None;
    correction_target_zone_ = YawZone::OutOfRange;
  }

  void setCommandTargetRaw(double yaw, double pitch)
  {
    command_yaw_angle_ = yaw;
    command_pitch_angle_ = pitch;
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

  bool isTimedOut(const rclcpp::Time & last_time) const
  {
    return (this->now() - last_time).seconds() > target_timeout_sec_;
  }

  void holdCurrentAngle(const char * warn_message)
  {
    if (!has_joint_state_) {
      publishCommandHold(warn_message);
      return;
    }
    setCommandTargetRaw(yaw_angle_, pitch_angle_);
    publishCommandTarget();
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

  double getImageTargetCenterX() const
  {
    return (image_center_x_ - 0.5) * image_width_;
  }

  double getImageTargetCenterY() const
  {
    return (image_center_y_ - 0.5) * image_height_;
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
  bool enable_zone_angle_limit_ = false;
  bool zone_yaw_reversed_ = false;
  double zone_yaw_zone1_start_ = -3.14159265359;
  double zone_yaw_boundary_ = -1.57079632679;
  double zone_yaw_zone2_end_ = 1.57079632679;
  double zone_yaw_zone3_end_ = 3.14159265359;
  double zone_pitch_lower_limit_ = -3.14159265359;
  double zone_pitch_zone2_upper_ = 0.52359877559;
  double zone_pitch_zone2_lower_ = -3.14159265359;
  double zone_pitch_zone2_upper_limit_ = 3.14159265359;
  double zone_pitch_zone3_lower_ = -0.52359877559;
  double zone_pitch_zone3_upper_ = 3.14159265359;
  double zone_pitch_zone1_upper_ = 3.14159265359;
  double zone_pitch_zoneab_upper_ = 0.52359877559;
  double control_hysteresis_rad_ = 0.017453292519943295;
  double control_pitch_correct_tolerance_ = 0.01;
  bool zone_pitch_zone1_upper_overridden_ = false;
  bool zone_pitch_zone2_upper_overridden_ = false;
  bool has_last_yaw_zone_ = false;
  YawZone last_yaw_zone_ = YawZone::ZoneAB;
  PitchCorrectionMode pitch_correction_mode_ = PitchCorrectionMode::None;
  double correction_hold_yaw_ = 0.0;
  YawZone correction_target_zone_ = YawZone::OutOfRange;
  double correction_target_pitch_ = 0.0;
  int pitch_motor_id_;
  int yaw_motor_id_;
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
