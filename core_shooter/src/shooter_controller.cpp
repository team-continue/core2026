#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "core_msgs/msg/can.hpp"
#include "core_msgs/msg/can_array.hpp"

#include "core_shooter/can_command.hpp"
#include "core_shooter/parameter_utils.hpp"
#include "core_shooter/test_mode_gate.hpp"

namespace
{
/// 装填モータは 1 発ごとに半回転（π rad）進める。
constexpr double kLoadingAnglePerShot = M_PI;
/// 目標角の 95% 到達で発射完了とみなす（残り 5% = 0.05π）。
constexpr double kShootCompletionMarginRad = M_PI * 0.05;
/// joint_states のうち、シャーシに対する砲塔角が入るインデックス。
constexpr size_t kTurretAngleJointIndex = 4;

/// 発射機構の種類。yaml の shoot_mechanism で左右別々に選ぶ。
enum class ShootMechanism
{
  RotaryLoader,  ///< 装填モータを 1 発ごとに π rad 回す方式
  BeltFlick      ///< サーボでベルトを揺動させ（角度1→角度2→角度1）ディスクを飛ばす方式
};

std::optional<ShootMechanism> parseShootMechanism(const std::string & name)
{
  if (name == "rotary_loader") {
    return ShootMechanism::RotaryLoader;
  }
  if (name == "belt_flick") {
    return ShootMechanism::BeltFlick;
  }
  return std::nullopt;
}

/// belt_flick のサーボ動作の段階。
enum class FlickPhase
{
  Idle,
  InitStrike,  ///< INIT: 角度2へ移動中
  InitHome,    ///< INIT: 角度1へ移動中
  Strike,      ///< 発射: 角度2へ移動中
  Hold,        ///< 発射: 角度2で保持中
  Return       ///< 発射: 角度1へ移動中
};

const char * flickPhaseName(FlickPhase phase)
{
  switch (phase) {
    case FlickPhase::Idle: return "IDLE";
    case FlickPhase::InitStrike: return "INIT_STRIKE";
    case FlickPhase::InitHome: return "INIT_HOME";
    case FlickPhase::Strike: return "STRIKE";
    case FlickPhase::Hold: return "HOLD";
    case FlickPhase::Return: return "RETURN";
  }
  return "UNKNOWN";
}

/// サーボ移動の判定結果。
enum class FlickMoveResult
{
  Moving,
  Reached,
  TimedOut
};

/// シューターモータ（ベルト駆動）を回し始められない理由。
enum class ShootMotorStartBlocker
{
  None,
  InitNotCompleted,
  ServoNotAtHome
};

constexpr double degToRad(double deg)
{
  return deg * M_PI / 180.0;
}
}  // namespace


class ShooterController : public rclcpp::Node
{
public:
  ShooterController()
  : Node("shooter_controller")
  {
    //========================================
    // shoot id parameters
    //========================================
    shoot_motor_id_ = core_shooter::declareAndGet<int>(*this, "shoot_motor_id", 100);
    loading_motor_id_ = core_shooter::declareAndGet<int>(*this, "loading_motor_id", 100);
    if (shoot_motor_id_ < 0 || loading_motor_id_ < 0) {
      RCLCPP_FATAL(
        this->get_logger(), "Invalid motor ids: shoot_motor_id=%d, loading_motor_id=%d",
        shoot_motor_id_, loading_motor_id_);
      throw std::runtime_error("invalid shooter motor ids");
    }

    RCLCPP_INFO(
      this->get_logger(), "shoot_motor_id: %d, loading_motor_id: %d", shoot_motor_id_,
      loading_motor_id_);

    //========================================
    // shoot parameters
    //========================================
    burst_count_ = core_shooter::declareAndGet<int>(*this, "burst_count", 3);
    shoot_interval_ms_ = core_shooter::declareAndGet<int>(*this, "shoot_interval_ms", 500);
    burst_interval_ms_ = core_shooter::declareAndGet<int>(*this, "burst_interval_ms", 500);
    fullauto_interval_ms_ = core_shooter::declareAndGet<int>(*this, "fullauto_interval_ms", 500);
    shoot_motor_rotation_cmd_activation_delay_sec_ = core_shooter::declareAndGet<double>(
      *this, "shoot_motor_rotation_cmd_activation_delay_sec", 2.0);
    enable_test_mode_ = core_shooter::declareAndGet<bool>(*this, "enable_test_mode", false);
    test_mode_.setDefault(enable_test_mode_);
    if (
      burst_count_ <= 0 || shoot_interval_ms_ < 0 || burst_interval_ms_ < 0 ||
      fullauto_interval_ms_ < 0 ||
      shoot_motor_rotation_cmd_activation_delay_sec_ < 0.0)
    {
      RCLCPP_FATAL(
        this->get_logger(),
        "Invalid shoot params: burst_count=%d, shoot_interval_ms=%d, burst_interval_ms=%d, fullauto_interval_ms=%d, shoot_motor_rotation_cmd_activation_delay_sec=%f",
        burst_count_, shoot_interval_ms_, burst_interval_ms_, fullauto_interval_ms_,
        shoot_motor_rotation_cmd_activation_delay_sec_);
      throw std::runtime_error("invalid shoot parameters");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "burst: %d, shoot_interval_ms: %d, burst_interval_ms: %d, fullauto_interval_ms: %d, shoot_motor_rotation_cmd_activation_delay_sec: %.3f, test_mode_default=%s(topic override supported)",
      burst_count_, shoot_interval_ms_, burst_interval_ms_, fullauto_interval_ms_,
      shoot_motor_rotation_cmd_activation_delay_sec_, enable_test_mode_ ? "true" : "false");

    //========================================
    // shoot mechanism parameters
    //========================================
    const std::string shoot_mechanism_name =
      core_shooter::declareAndGet<std::string>(*this, "shoot_mechanism", "rotary_loader");
    const auto shoot_mechanism = parseShootMechanism(shoot_mechanism_name);
    if (!shoot_mechanism) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Invalid shoot_mechanism=\"%s\" (expected \"rotary_loader\" or \"belt_flick\")",
        shoot_mechanism_name.c_str());
      throw std::runtime_error("invalid shoot_mechanism");
    }
    shoot_mechanism_ = *shoot_mechanism;

    flick_home_rad_ =
      degToRad(core_shooter::declareAndGet<double>(*this, "flick_home_angle_deg", 0.0));
    flick_strike_rad_ =
      degToRad(core_shooter::declareAndGet<double>(*this, "flick_strike_angle_deg", 5.0));
    flick_hold_sec_ = core_shooter::declareAndGet<double>(*this, "flick_hold_sec", 0.2);
    flick_tolerance_rad_ =
      degToRad(core_shooter::declareAndGet<double>(*this, "flick_angle_tolerance_deg", 0.5));
    flick_move_timeout_sec_ =
      core_shooter::declareAndGet<double>(*this, "flick_move_timeout_sec", 1.0);
    if (shoot_mechanism_ == ShootMechanism::BeltFlick) {
      const double half_stroke_rad = std::abs(flick_strike_rad_ - flick_home_rad_) / 2.0;
      if (
        flick_hold_sec_ <= 0.0 || flick_move_timeout_sec_ <= 0.0 ||
        flick_tolerance_rad_ <= 0.0 || flick_tolerance_rad_ >= half_stroke_rad)
      {
        RCLCPP_FATAL(
          this->get_logger(),
          "Invalid flick params: home_rad=%f, strike_rad=%f, hold_sec=%f, tolerance_rad=%f, move_timeout_sec=%f (hold/timeout/tolerance must be > 0, tolerance < |strike-home|/2)",
          flick_home_rad_, flick_strike_rad_, flick_hold_sec_, flick_tolerance_rad_,
          flick_move_timeout_sec_);
        throw std::runtime_error("invalid flick parameters");
      }
    }

    RCLCPP_INFO(
      this->get_logger(),
      "shoot_mechanism: %s, flick home_rad: %f, strike_rad: %f, hold_sec: %f, tolerance_rad: %f, move_timeout_sec: %f",
      shoot_mechanism_name.c_str(), flick_home_rad_, flick_strike_rad_, flick_hold_sec_,
      flick_tolerance_rad_, flick_move_timeout_sec_);

    //========================================
    // panel limit parameters
    //========================================
    limit_rad_ =
      core_shooter::declareAndGet<std::vector<double>>(*this, "limit_rad", std::vector<double>(4, 0.0));
    if (limit_rad_.size() != 4) {
      RCLCPP_FATAL(
        this->get_logger(), "Invalid limit_rad size=%zu (expected 4)", limit_rad_.size());
      throw std::runtime_error("invalid limit_rad size");
    }
    RCLCPP_INFO(
      this->get_logger(), "limit_rad: %f, %f, %f, %f", limit_rad_[0], limit_rad_[1], limit_rad_[2],
      limit_rad_[3]);

    enable_panel_synchronizer_ =
      core_shooter::declareAndGet<bool>(*this, "enable_panel_synchronizer", true);
    RCLCPP_INFO(
      this->get_logger(), "enable_panel_synchronizer: %d", enable_panel_synchronizer_);

    //========================================
    // loading motor parameters
    //========================================
    loading_motor_speed_ =
      static_cast<float>(core_shooter::declareAndGet<double>(*this, "loading_motor_speed", 3.0));
    loading_motor_initial_angle_ = core_shooter::declareAndGet<std::vector<double>>(
      *this, "loading_motor_initial_angle", std::vector<double>(3, 0.0));
    if (loading_motor_initial_angle_.size() != 3) {
      RCLCPP_FATAL(
        this->get_logger(), "Invalid loading_motor_initial_angle size=%zu (expected 3)",
        loading_motor_initial_angle_.size());
      throw std::runtime_error("invalid loading_motor_initial_angle size");
    }

    RCLCPP_INFO(
      this->get_logger(), "loading_motor_speed: %f, loading_motor1_initial_angle: %f, loading_motor2_initial_angle: %f, loading_motor3_initial_angle: %f",
      loading_motor_speed_, loading_motor_initial_angle_[0], loading_motor_initial_angle_[1],
      loading_motor_initial_angle_[2]);

    //========================================
    // shoot motor parameters
    //========================================
    shoot_motor_target_speed_ = core_shooter::declareAndGet<std::vector<double>>(
      *this, "target_speed", std::vector<double>(3, 0.0));
    if (shoot_motor_target_speed_.size() != 3) {
      RCLCPP_FATAL(
        this->get_logger(), "Invalid target_speed size=%zu (expected 3)",
        shoot_motor_target_speed_.size());
      throw std::runtime_error("invalid target_speed size");
    }

    RCLCPP_INFO(
      this->get_logger(), "target_speed1: %f, target_speed2: %f, target_speed3: %f",
      shoot_motor_target_speed_[0], shoot_motor_target_speed_[1], shoot_motor_target_speed_[2]);

    //========================================
    // jam parameters
    //========================================
    enable_jam_detection_ =
      core_shooter::declareAndGet<bool>(*this, "enable_jam_detection", true);
    jam_detect_time_sec_ =
      core_shooter::declareAndGet<double>(*this, "jam_detect_time_sec", 0.1);
    if (jam_detect_time_sec_ < 0.0) {
      RCLCPP_FATAL(
        this->get_logger(), "Invalid jam_detect_time_sec=%f (must be >= 0)",
        jam_detect_time_sec_);
      throw std::runtime_error("invalid jam_detect_time_sec");
    }

    RCLCPP_INFO(
      this->get_logger(), "enable_jam_detection: %d, jam_detect_time_sec_: %f",
      enable_jam_detection_, jam_detect_time_sec_);

    //========================================
    // debug parameters
    //========================================
    set_initial_rad_ = core_shooter::declareAndGet<double>(*this, "set_initial_rad", 0.0);

    RCLCPP_INFO(this->get_logger(), "set_initial_rad_: %f", set_initial_rad_);

    //========================================
    // subscribers sensor
    //========================================
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/hardware/joint_states", 10,
      std::bind(&ShooterController::jointStateCallback, this, std::placeholders::_1));
    jam_sensor_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "jam", 10,
      std::bind(&ShooterController::jamSensorCallback, this, std::placeholders::_1));
    hazard_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "hazard_status", 10,
      std::bind(&ShooterController::hazardStatusCallback, this, std::placeholders::_1));
    test_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/test_mode", 10,
      std::bind(&ShooterController::testModeCallback, this, std::placeholders::_1));
    // belt_flick はリグリップを使わないため購読しない（regrip_active_ は false のまま）
    if (shoot_mechanism_ == ShootMechanism::BeltFlick) {
      RCLCPP_INFO(this->get_logger(), "belt_flick: regrip_active is not used");
    } else {
      regrip_active_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "regrip_active", rclcpp::QoS(1).transient_local(),
        std::bind(&ShooterController::regripActiveCallback, this, std::placeholders::_1));
    }

    //========================================
    // subscribers shoot cmd
    //========================================
    shoot_cmd_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "shoot_cmd", 1,
      std::bind(&ShooterController::shootCmdCallback, this, std::placeholders::_1));

    //========================================
    // subscribers ui
    //========================================
    shoot_motor_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "shoot_motor", 1,
      std::bind(&ShooterController::shootMotorCallback, this, std::placeholders::_1));

    //========================================
    // publishers
    //========================================
    shoot_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "shoot_status", 10);
    can_pub_ = this->create_publisher<core_msgs::msg::CANArray>(
      "/can/tx", 10);
    jam_state_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "jam_state", 10);
    loading_error_state_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "loading_motor_error_state", 10);
    shooting_error_state_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "shoot_motor_error_state", 10);

    //========================================
    // timer callback
    //========================================
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&ShooterController::timerCallback, this));
  }

private:
  //========================================
  // sensor callbacks
  //========================================
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const size_t position_size = msg->position.size();
    if (loading_motor_id_ < 0 ||
      static_cast<size_t>(loading_motor_id_) >= position_size ||
      position_size <= kTurretAngleJointIndex)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 2000,
        "joint_states size mismatch: position=%zu, loading_id=%d (need loading index and position[4])",
        position_size, loading_motor_id_);
      return;
    }

    loading_motor_rad_ = msg->position[loading_motor_id_];
    turret_angle_from_chassis_ = msg->position[kTurretAngleJointIndex];
    joint_state_received_ = true;
  }

  void jamSensorCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    jam_photo_reflector_raw_ = msg->data;
    rclcpp::Time now = this->get_clock()->now();

    if (jam_photo_reflector_raw_) {
      // true が初めて来た瞬間を記録
      if (!jam_tracking_) {
        jam_start_time_ = now;
        jam_tracking_ = true;
      }

      // true が続いている場合、100ms 経過したかチェック
      auto elapsed = now - jam_start_time_;
      if (elapsed > rclcpp::Duration::from_seconds(jam_detect_time_sec_)) {
        is_jam_detected_ = true;
      }
    } else {
      // false になったらリセット
      jam_tracking_ = false;
      is_jam_detected_ = false;
    }

    RCLCPP_INFO(this->get_logger(), "flag = %s", is_jam_detected_ ? "true" : "false");
  }

  void hazardStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool previous_hazard_state = hazard_state_;
    hazard_state_ = msg->data;

    // 禁止解除時にショット完了をtrueにする
    if (previous_hazard_state && !hazard_state_) {
      shoot_completed_ = true;
      RCLCPP_INFO(get_logger(), "Clear Emergency, Set shoot_completed_ TRUE");
    }
  }

  void testModeCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (test_mode_.update(msg->data)) {
      RCLCPP_INFO(
        this->get_logger(),
        "Test mode %s in shooter_controller (source=topic, param fallback=%s)",
        test_mode_.enabled() ? "ON" : "OFF", test_mode_.defaultValue() ? "true" : "false");
    }
  }

  void regripActiveCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (regrip_active_ != msg->data) {
      RCLCPP_INFO(
        this->get_logger(), "Regrip %s -> shooting %s",
        msg->data ? "ACTIVE" : "INACTIVE", msg->data ? "blocked" : "allowed");
    }
    regrip_active_ = msg->data;
  }

  //========================================
  // ui callbacks
  //========================================
  void shootCmdCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    shoot_repeat_count_ = msg->data;
  }

  void shootMotorCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (shoot_mechanism_ == ShootMechanism::BeltFlick) {
      handleShootMotorCommandBeltFlick(msg->data);
      return;
    }

    float commanded_speed = 0.0f;
    switch (state_) {
      case EMERGENCY:
        commanded_speed = 0.0f;
        break;

      default:
        commanded_speed = mapShootMotorSpeed(msg->data);
        break;
    }
    setSpeed(shoot_motor_id_, commanded_speed);
    const bool nonzero_command = commanded_speed > 0.0f;
    if (nonzero_command) {
      if (!shoot_motor_rotation_cmd_requested_) {
        shoot_motor_rotation_cmd_requested_ = true;
        shoot_motor_rotation_cmd_start_time_ = this->now();
      }
    } else {
      resetShootMotorRotationCommandState();
    }
    updateShootMotorRotationCommandFlag();
  }

  /// shoot_motor の入力（0..1）を目標速度に変換する。
  float mapShootMotorSpeed(float input) const
  {
    if (input > 0.7) {
      return static_cast<float>(shoot_motor_target_speed_[0]);
    } else if (input > 0.4) {
      return static_cast<float>(shoot_motor_target_speed_[1]);
    } else if (input > 0.1) {
      return static_cast<float>(shoot_motor_target_speed_[2]);
    }
    return 0.0f;
  }

  //========================================
  // main loop
  //========================================
  void timerCallback()
  {
    if (shoot_mechanism_ == ShootMechanism::BeltFlick) {
      timerCallbackBeltFlick();
      return;
    }

    updateShootMotorRotationCommandFlag();
    jamStatePublish(is_jam_detected_);

    switch (state_) {
      case INIT:
        {
          // shoot指令
          RCLCPP_INFO(get_logger(), "Initilize, %f", loading_motor_rad_);

          init_sync_in_progress_ = true;
          setAngle(loading_motor_id_, getShootMotorRotationCount() * kLoadingAnglePerShot);
          last_shoot_time_ = this->now();

          shoot_completed_ = false;
          state_ = SHOOT;
          RCLCPP_INFO(get_logger(), "change SHOOT (Initilize)");
          break;
        }
      case CMD_WAIT:
        {
          if (hazard_state_) {
            state_ = EMERGENCY;
            resetShootMotorRotationCommandState();
            RCLCPP_INFO(get_logger(), "Set EMERGENCY in CMD_WAIT");
            break;
          }

          // repeat > 1 && repeat < 0
          if (shoot_repeat_count_ == 0) {
            break;
          }
          bool result = shootDecision();
          if (result) {
            // shoot指令
            shoot_count_++;
            RCLCPP_INFO(
              get_logger(), "internal cnt = %d, %f", shoot_count_,
              kLoadingAnglePerShot + shoot_count_ * kLoadingAnglePerShot);

            init_sync_in_progress_ = false;
            setAngle(
              loading_motor_id_,
              getShootMotorRotationCount() * kLoadingAnglePerShot + kLoadingAnglePerShot);
            last_shoot_time_ = this->now();

            shoot_completed_ = false;
            shootStatePublish(shoot_completed_);

            state_ = SHOOT;
            RCLCPP_INFO(get_logger(), "change SHOOT");
          }
          RCLCPP_INFO(this->get_logger(), "Remaining number of repeats: %d", shoot_repeat_count_);
          break;
        }
      case SHOOT:
        if (hazard_state_) {
          state_ = EMERGENCY;
          resetShootMotorRotationCommandState();
          setAngle(loading_motor_id_, loading_motor_rad_);

          RCLCPP_INFO(get_logger(), "Set EMERGENCY in SHOOT");
          break;
        }

        // target_angle_ の95%を超えたら遷移
        RCLCPP_INFO_THROTTLE(
          get_logger(),
          *this->get_clock(), 1000, "target: %f, current: %f", target_angle_, loading_motor_rad_);

        if (target_angle_ - kShootCompletionMarginRad < loading_motor_rad_) {
          shoot_completed_ = true;

          if (!init_sync_in_progress_) {
            shootStatePublish(shoot_completed_);
          } else {
            RCLCPP_INFO(get_logger(), "Initial sync completed without publishing shoot_status");
          }

          if (shoot_repeat_count_ >= 1) {
            shoot_repeat_count_--;
          }
          init_sync_in_progress_ = false;
          state_ = CMD_WAIT;
          RCLCPP_INFO(get_logger(), "change CMD_WAIT");
        }
        break;

      case EMERGENCY:
        shoot_completed_ = false;
        shoot_repeat_count_ = 0;
        resetShootMotorRotationCommandState();

        setSpeed(shoot_motor_id_, 0.0f);

        if (!hazard_state_) {
          // state_ = CMD_WAIT;
          state_ = INIT;
          RCLCPP_INFO(get_logger(), "Clear emergency, change CMD_WAIT");
        }
        break;
      default:
        break;
    }
  }

  //========================================
  // belt_flick sequence
  //========================================
  void timerCallbackBeltFlick()
  {
    updateShootMotorRotationCommandFlag();
    jamStatePublish(is_jam_detected_);

    if (hazard_state_ && state_ != EMERGENCY) {
      enterEmergencyBeltFlick();
      return;
    }

    tryStartShootMotorBeltFlick();

    switch (state_) {
      case INIT:
        updateFlickInit();
        break;

      case CMD_WAIT:
        if (shoot_repeat_count_ == 0) {
          break;
        }
        if (shootDecision()) {
          startFlickShot();
        }
        break;

      case SHOOT:
        updateFlickShot();
        break;

      case EMERGENCY:
        shoot_completed_ = false;
        shoot_repeat_count_ = 0;
        requested_shoot_motor_speed_ = 0.0f;
        shoot_motor_running_ = false;
        resetShootMotorRotationCommandState();
        setSpeed(shoot_motor_id_, 0.0f);

        if (!hazard_state_) {
          resetFlickState();
          state_ = INIT;
          RCLCPP_INFO(get_logger(), "Clear emergency, reset flick sequence and change INIT");
        }
        break;

      default:
        break;
    }
  }

  /// hazard を受けたら、サーボを現在位置で止めて EMERGENCY に移る。
  void enterEmergencyBeltFlick()
  {
    // joint_states 未受信のときは現在角が不明なので、角度指令を出さない
    if (joint_state_received_) {
      setAngle(loading_motor_id_, static_cast<float>(loading_motor_rad_));
    }
    RCLCPP_INFO(
      get_logger(), "Set EMERGENCY in %s (flick phase=%s), hold servo at %f",
      stateName(state_), flickPhaseName(flick_phase_), loading_motor_rad_);
    flick_phase_ = FlickPhase::Idle;
    requested_shoot_motor_speed_ = 0.0f;
    state_ = EMERGENCY;
  }

  /// EMERGENCY 復帰時に、それまでの動作をすべて捨てる。
  void resetFlickState()
  {
    flick_phase_ = FlickPhase::Idle;
    shoot_repeat_count_ = 0;
    shoot_completed_ = false;
    init_sync_in_progress_ = false;
    requested_shoot_motor_speed_ = 0.0f;
    shoot_motor_running_ = false;
    resetShootMotorRotationCommandState();
  }

  /// INIT: 角度2 → 角度1 と往復させてから CMD_WAIT に移る。
  void updateFlickInit()
  {
    switch (flick_phase_) {
      case FlickPhase::Idle:
        RCLCPP_INFO(get_logger(), "Flick init start");
        startFlickMove(flick_strike_rad_, FlickPhase::InitStrike);
        break;

      case FlickPhase::InitStrike:
        if (finishFlickMove()) {
          startFlickMove(flick_home_rad_, FlickPhase::InitHome);
        }
        break;

      case FlickPhase::InitHome:
        if (finishFlickMove()) {
          flick_phase_ = FlickPhase::Idle;
          shoot_completed_ = true;
          state_ = CMD_WAIT;
          RCLCPP_INFO(get_logger(), "Flick init completed, change CMD_WAIT");
        }
        break;

      default:
        RCLCPP_WARN(
          get_logger(), "Unexpected flick phase %s in INIT, restart init",
          flickPhaseName(flick_phase_));
        flick_phase_ = FlickPhase::Idle;
        break;
    }
  }

  /// CMD_WAIT → SHOOT: 角度2へ動かし始める。
  void startFlickShot()
  {
    shoot_count_++;
    RCLCPP_INFO(
      get_logger(), "Flick shot start: internal cnt = %d, remaining repeats = %d",
      shoot_count_, shoot_repeat_count_);

    last_shoot_time_ = this->now();
    shoot_completed_ = false;
    shootStatePublish(shoot_completed_);

    startFlickMove(flick_strike_rad_, FlickPhase::Strike);
    state_ = SHOOT;
  }

  /// SHOOT: 角度2に到達 → hold → 角度1に到達で 1 発完了。
  void updateFlickShot()
  {
    switch (flick_phase_) {
      case FlickPhase::Strike:
        if (finishFlickMove()) {
          flick_phase_ = FlickPhase::Hold;
          flick_phase_start_time_ = this->now();
        }
        break;

      case FlickPhase::Hold:
        if ((this->now() - flick_phase_start_time_).seconds() >= flick_hold_sec_) {
          startFlickMove(flick_home_rad_, FlickPhase::Return);
        }
        break;

      case FlickPhase::Return:
        if (finishFlickMove()) {
          flick_phase_ = FlickPhase::Idle;
          shoot_completed_ = true;
          shootStatePublish(shoot_completed_);
          if (shoot_repeat_count_ >= 1) {
            shoot_repeat_count_--;
          }
          state_ = CMD_WAIT;
          RCLCPP_INFO(get_logger(), "Flick shot completed, change CMD_WAIT");
        }
        break;

      default:
        RCLCPP_WARN(
          get_logger(), "Unexpected flick phase %s in SHOOT, return to CMD_WAIT",
          flickPhaseName(flick_phase_));
        flick_phase_ = FlickPhase::Idle;
        state_ = CMD_WAIT;
        break;
    }
  }

  /// サーボに角度を指令し、到達待ちのタイマーを開始する。
  void startFlickMove(double target_rad, FlickPhase phase)
  {
    setAngle(loading_motor_id_, static_cast<float>(target_rad));
    flick_target_rad_ = target_rad;
    flick_phase_ = phase;
    flick_phase_start_time_ = this->now();
  }

  bool isFlickAt(double target_rad) const
  {
    return joint_state_received_ &&
           std::abs(loading_motor_rad_ - target_rad) <= flick_tolerance_rad_;
  }

  FlickMoveResult evaluateFlickMove(double elapsed_sec) const
  {
    if (isFlickAt(flick_target_rad_)) {
      return FlickMoveResult::Reached;
    }
    if (elapsed_sec >= flick_move_timeout_sec_) {
      return FlickMoveResult::TimedOut;
    }
    return FlickMoveResult::Moving;
  }

  /// 移動が終わった（到達 or 時間切れ）なら true。
  /// 時間切れは warning 扱いとし、到達したものとして次の段階へ進める。
  bool finishFlickMove()
  {
    const double elapsed_sec = (this->now() - flick_phase_start_time_).seconds();
    switch (evaluateFlickMove(elapsed_sec)) {
      case FlickMoveResult::Reached:
        if (flick_move_warning_active_) {
          flick_move_warning_active_ = false;
          errorStatePublish(loading_error_state_pub_, false);
          RCLCPP_INFO(
            get_logger(), "Flick servo reached target again (phase=%s)",
            flickPhaseName(flick_phase_));
        }
        return true;

      case FlickMoveResult::TimedOut:
        flick_move_warning_active_ = true;
        errorStatePublish(loading_error_state_pub_, true);
        RCLCPP_WARN(
          get_logger(),
          "Flick servo did not reach target within %.3f s (phase=%s, target=%f rad, current=%f rad, joint_state_received=%s). Continue sequence.",
          flick_move_timeout_sec_, flickPhaseName(flick_phase_), flick_target_rad_,
          loading_motor_rad_, joint_state_received_ ? "true" : "false");
        return true;

      case FlickMoveResult::Moving:
        return false;
    }
    return false;
  }

  //========================================
  // belt_flick shoot motor (belt drive)
  //========================================
  void handleShootMotorCommandBeltFlick(float input)
  {
    const float speed =
      (state_ == EMERGENCY || hazard_state_) ? 0.0f : mapShootMotorSpeed(input);
    requested_shoot_motor_speed_ = speed;

    if (speed <= 0.0f) {
      setSpeed(shoot_motor_id_, 0.0f);
      shoot_motor_running_ = false;
      resetShootMotorRotationCommandState();
      updateShootMotorRotationCommandFlag();
      return;
    }

    if (shoot_motor_running_) {
      // 回転中の速度変更は阻害しない
      setSpeed(shoot_motor_id_, speed);
      return;
    }

    tryStartShootMotorBeltFlick();
  }

  /// 保留中の回転指令があり、阻害要因がなければベルトを回し始める。
  void tryStartShootMotorBeltFlick()
  {
    if (shoot_motor_running_ || requested_shoot_motor_speed_ <= 0.0f) {
      return;
    }

    switch (shootMotorStartBlocker()) {
      case ShootMotorStartBlocker::InitNotCompleted:
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *this->get_clock(), 1000,
          "Cannot start shoot motor: flick servo initialization is not completed (state=%s, phase=%s)",
          stateName(state_), flickPhaseName(flick_phase_));
        return;

      case ShootMotorStartBlocker::ServoNotAtHome:
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *this->get_clock(), 1000,
          "Cannot start shoot motor: flick servo is not at home angle (home=%f rad, current=%f rad, tolerance=%f rad, joint_state_received=%s)",
          flick_home_rad_, loading_motor_rad_, flick_tolerance_rad_,
          joint_state_received_ ? "true" : "false");
        return;

      case ShootMotorStartBlocker::None:
        break;
    }

    setSpeed(shoot_motor_id_, requested_shoot_motor_speed_);
    shoot_motor_running_ = true;
    shoot_motor_rotation_cmd_requested_ = true;
    shoot_motor_rotation_cmd_start_time_ = this->now();
    updateShootMotorRotationCommandFlag();
    RCLCPP_INFO(
      get_logger(), "Shoot motor started: speed=%f", requested_shoot_motor_speed_);
  }

  ShootMotorStartBlocker shootMotorStartBlocker() const
  {
    if (state_ == INIT || state_ == EMERGENCY) {
      return ShootMotorStartBlocker::InitNotCompleted;
    }
    if (!isFlickAt(flick_home_rad_)) {
      return ShootMotorStartBlocker::ServoNotAtHome;
    }
    return ShootMotorStartBlocker::None;
  }

  void errorStatePublish(
    const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & publisher, bool state)
  {
    auto message = std_msgs::msg::Bool();
    message.data = state;
    publisher->publish(message);
  }

  const char * stateName(int state) const
  {
    switch (state) {
      case INIT: return "INIT";
      case CMD_WAIT: return "CMD_WAIT";
      case SHOOT: return "SHOOT";
      case EMERGENCY: return "EMERGENCY";
      default: return "UNKNOWN";
    }
  }

  bool shootDecision()
  {
    // Output logs every second.
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(), 1000, "shoot completed: %d, Angle: %d, Interval: %d", shoot_completed_,
      isValidAngle(), isShootIntervalElapsed());

    if (canShoot()) {
      RCLCPP_INFO(this->get_logger(), "========== Shoot ==========");
      return true;
    } else {
      // Output logs every second.
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Shoot condition not met");

      // 打てなかった場合はそのタスクを削除する
      if (shoot_repeat_count_ >= 1) {
        shoot_repeat_count_ -= 1;
      }
      return false;
    }
  }

  float getShootMotorRotationCount() const
  {
    const double remainder = std::fmod(loading_motor_rad_, kLoadingAnglePerShot);
    float rotate =
      static_cast<float>((loading_motor_rad_ - remainder) / kLoadingAnglePerShot);

    if (remainder > kLoadingAnglePerShot / 2.0) {
      rotate += 1;
    }
    return rotate;
  }

  //========================================
  // canShoot
  //========================================
  bool canShoot()
  {
    // hazard_status=true は危険状態なので射撃不可
    const bool test_mode_enabled = isTestModeEnabled();
    // TODO(ytk-wsl): Temporarily disable jam gating until debounce behavior is revisited.
    // const bool jam_ok = test_mode_enabled || !enable_jam_detection_ || !isJamDetected();
    const bool jam_ok = true;
    const bool shoot_motor_ready = test_mode_enabled || isShootMotorRotationCommandActive();
    const bool regrip_ok = test_mode_enabled || !regrip_active_;
    return !hazard_state_ && regrip_ok && isValidAngle() && isShootIntervalElapsed() &&
           shoot_motor_ready && jam_ok;
  }

  bool isValidAngle()
  {
    // 隙間撃ちの設定がfalseの場合は常にtrueを返す
    if (!enable_panel_synchronizer_) {
      return true;
    }

    // 　limit_rad_[1]
    //　　＼　　　limit_rad_[0]
    //　　　＼　／
    //  true ◯  true  ---- 0 [rad]
    //　　　／　＼
    //　　／　　　limit_rad_[3]
    // 　limit_rad_[2]

    // theta を正規化
    double rad = turret_angle_from_chassis_;
    rad = fmod(rad, 2 * M_PI);
    if (rad < 0) {
      rad += 2 * M_PI;
    }

    // 2つの区間をループで判定
    for (int i = 0; i < 4; i += 2) {
      double start = limit_rad_[i];
      double end = limit_rad_[i + 1];

      if ((start <= end && rad >= start && rad <= end) ||
        (start > end && (rad >= start || rad <= end)))
      {
        return false;
      }
    }

    return true;
  }

  bool isShootIntervalElapsed()
  {
    if (shoot_repeat_count_ == 1) {
      return (this->now() - last_shoot_time_).seconds() * 1000 >= shoot_interval_ms_;
    } else if (shoot_repeat_count_ >= 2) {
      return (this->now() - last_shoot_time_).seconds() * 1000 >= burst_interval_ms_;
    } else if (shoot_repeat_count_ <= -1) {
      return (this->now() - last_shoot_time_).seconds() * 1000 >= fullauto_interval_ms_;
    }
    return false;
  }

  bool isShootMotorRotationCommandActive() const
  {
    return shoot_motor_rotation_cmd_active_;
  }

  bool isTestModeEnabled() const
  {
    return test_mode_.enabled();
  }

  void updateShootMotorRotationCommandFlag()
  {
    if (!shoot_motor_rotation_cmd_requested_) {
      shoot_motor_rotation_cmd_active_ = false;
      return;
    }

    const auto elapsed = this->now() - shoot_motor_rotation_cmd_start_time_;
    shoot_motor_rotation_cmd_active_ =
      elapsed >= rclcpp::Duration::from_seconds(shoot_motor_rotation_cmd_activation_delay_sec_);
  }

  void resetShootMotorRotationCommandState()
  {
    shoot_motor_rotation_cmd_requested_ = false;
    shoot_motor_rotation_cmd_active_ = false;
  }


  //========================================
  // motorPub
  //========================================
  // モータの目標角度を設定
  void setAngle(int motor, float angle)
  {
    motorPublish(motor, angle);
    target_angle_ = angle;
  }

  // 発射モータのスピードを設定
  void setSpeed(int motor, float speed)
  {
    motorPublish(motor, speed);
  }

  // モータの速度をCANで送信
  void motorPublish(int id, float data)
  {
    core_shooter::publishMotorCommand(can_pub_, id, data);
  }

  //========================================
  // jam detection
  //========================================
  bool isJamDetected()
  {
    bool jam_state;
    if (is_jam_detected_) {
      jam_state = true;
    } else if (jam_photo_reflector_raw_) {
      jam_state = true;
    } else {
      jam_state = false;
    }
    jamStatePublish(jam_state);
    return jam_state;
  }

  void jamStatePublish(bool state)
  {
    auto message = std_msgs::msg::Bool();
    message.data = state;
    jam_state_pub_->publish(message);
  }

  //========================================
  // shoot status
  //========================================

  void shootStatePublish(bool state)
  {
    auto message = std_msgs::msg::Bool();
    message.data = state;
    shoot_status_pub_->publish(message);
  }

  //========================================
  // Subscription valids
  //========================================
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr jam_sensor_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hazard_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr test_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr regrip_active_sub_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr shoot_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr shoot_motor_sub_;

  //========================================
  // publisher valids
  //========================================
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shoot_status_pub_;
  rclcpp::Publisher<core_msgs::msg::CANArray>::SharedPtr can_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr jam_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr loading_error_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shooting_error_state_pub_;

  //========================================
  // timer callback valids
  //========================================
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_shoot_time_ = now();

  //========================================
  // subscription valids
  //========================================
  double loading_motor_rad_ = 0.0;
  double turret_angle_from_chassis_ = 0.0;

  bool jam_photo_reflector_raw_ = false;

  bool hazard_state_ = true;
  core_shooter::TestModeGate test_mode_;
  bool regrip_active_ = false;
  bool shoot_motor_rotation_cmd_requested_ = false;
  bool shoot_motor_rotation_cmd_active_ = false;
  rclcpp::Time shoot_motor_rotation_cmd_start_time_ = now();

  //========================================
  // parameter valids
  //========================================
  int shoot_motor_id_;
  int loading_motor_id_;

  int burst_count_;
  int shoot_interval_ms_;
  int burst_interval_ms_;
  int fullauto_interval_ms_;

  std::vector<double> limit_rad_;
  bool enable_panel_synchronizer_;

  float loading_motor_speed_;
  std::vector<double> loading_motor_initial_angle_;

  double set_initial_rad_;

  std::vector<double> shoot_motor_target_speed_;

  double jam_detect_time_sec_;
  bool enable_jam_detection_ = true;
  bool enable_test_mode_ = false;
  double shoot_motor_rotation_cmd_activation_delay_sec_ = 2.0;

  ShootMechanism shoot_mechanism_ = ShootMechanism::RotaryLoader;
  double flick_home_rad_ = 0.0;
  double flick_strike_rad_ = 0.0;
  double flick_hold_sec_ = 0.2;
  double flick_tolerance_rad_ = 0.0;
  double flick_move_timeout_sec_ = 1.0;

  //========================================
  // belt_flick valids
  //========================================
  FlickPhase flick_phase_ = FlickPhase::Idle;
  double flick_target_rad_ = 0.0;
  rclcpp::Time flick_phase_start_time_ = now();
  bool flick_move_warning_active_ = false;
  bool joint_state_received_ = false;
  float requested_shoot_motor_speed_ = 0.0f;
  bool shoot_motor_running_ = false;

  //========================================
  // valids
  //========================================
  bool shoot_completed_ = true;
  bool init_sync_in_progress_ = false;

  // the count of repeated shots. (x < -1: fullauto, x = 0: none, x > 1: burst )
  int shoot_repeat_count_ = 0;

  enum STATE
  {
    INIT,
    CMD_WAIT,
    SHOOT,
    EMERGENCY
  };
  STATE state_ = CMD_WAIT;
  int shoot_count_ = 0;

  float target_angle_ = M_PI;

  rclcpp::Time jam_start_time_;
  bool jam_tracking_ = false;
  bool is_jam_detected_ = false;


  //========================================
  // debug parameters
  //========================================
  // turret_angle_from_chassis_ = set_initial_rad_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShooterController>());
  rclcpp::shutdown();
  return 0;
}
