// 砲塔 MPC の実機試走ノード。
//
// aim_bot に組み込む前に、同定モデルと MPC パラメータを実機で確かめるためのもの。
// 画像ターゲットの代わりに「仮想の的」（三角波 = 等速往復 / 正弦波 / ステップ）を角度空間で動かし、
// 1軸だけを MPC（または比較用の直接指令）で追従させて CSV に記録する。
//
// 安全機構（同定データ取得ツール turret_sysid_logger と同じ考え方）:
//   - 指令範囲 angle_min/angle_max を必須にし、仮想の的の振れ幅がはみ出す設定は起動時に拒否
//   - 実測角の範囲逸脱・joint_states 途絶・追従誤差の持続・非常停止で即座に「現在角保持」して終了
//   - Ctrl+C でも現在角保持の指令を送ってから終了
//
// aim_bot と同時に動かすと /can/tx の指令が競合するので、必ず aim_bot を止めてから使うこと。

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "core_msgs/msg/can_array.hpp"

#include "core_shooter/can_command.hpp"
#include "core_shooter/parameter_utils.hpp"
#include "core_shooter/turret_model.hpp"
#include "core_shooter/turret_mpc.hpp"

using namespace std::chrono_literals;

namespace
{

std::atomic<bool> g_stop_requested{false};

void onSignal(int)
{
  g_stop_requested = true;
}

/// 試走の進行段階。
enum class Phase
{
  WaitJoint,  ///< joint_states の受信待ち
  Approach,   ///< 現在角から中心角へ低速で移動
  PreHold,    ///< 中心角で静止
  Run,        ///< 仮想の的を追従
  Return,     ///< 中心角へ戻る
  Abort,      ///< 異常・中断。現在角を保持
  Done,
};

const char * phaseName(Phase p)
{
  switch (p) {
    case Phase::WaitJoint: return "wait_joint";
    case Phase::Approach: return "approach";
    case Phase::PreHold: return "pre_hold";
    case Phase::Run: return "run";
    case Phase::Return: return "return";
    case Phase::Abort: return "abort";
    case Phase::Done: return "done";
  }
  return "unknown";
}

/// from から to へ max_step だけ近づける。届く場合は to をそのまま返す（到達判定を厳密にするため）。
double moveToward(double from, double to, double max_step)
{
  if (!(max_step > 0.0) || std::fabs(to - from) <= max_step) {
    return to;
  }
  return from + (to > from ? max_step : -max_step);
}

}  // namespace

class TurretMpcTrial : public rclcpp::Node
{
public:
  TurretMpcTrial()
  : Node("turret_mpc_trial")
  {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    motor_id_ = core_shooter::declareAndGet<int>(*this, "motor_id", -1);
    hold_motor_id_ = core_shooter::declareAndGet<int>(*this, "hold_motor_id", -1);
    axis_label_ = core_shooter::declareAndGet<std::string>(*this, "axis_label", "axis");
    const std::string model_path =
      core_shooter::declareAndGet<std::string>(*this, "model_path", "");
    const std::string mpc_config_path =
      core_shooter::declareAndGet<std::string>(*this, "mpc_config_path", "");
    controller_ = core_shooter::declareAndGet<std::string>(*this, "controller", "mpc");
    angle_min_ = core_shooter::declareAndGet<double>(*this, "angle_min", nan);
    angle_max_ = core_shooter::declareAndGet<double>(*this, "angle_max", nan);
    range_margin_ = core_shooter::declareAndGet<double>(*this, "range_margin_rad", 0.05);
    ref_type_ = core_shooter::declareAndGet<std::string>(*this, "reference.type", "triangle");
    ref_amplitude_ = core_shooter::declareAndGet<double>(*this, "reference.amplitude_rad", 0.2);
    ref_rate_ = core_shooter::declareAndGet<double>(*this, "reference.rate_rad_per_sec", 0.5);
    ref_frequency_ = core_shooter::declareAndGet<double>(*this, "reference.frequency_hz", 0.5);
    ref_step_period_ = core_shooter::declareAndGet<double>(
      *this, "reference.step_period_sec", 2.0);
    ref_center_param_ = core_shooter::declareAndGet<double>(*this, "reference.center_rad", nan);
    ref_preview_ = core_shooter::declareAndGet<std::string>(
      *this, "reference.preview", "constant_velocity");
    duration_ = core_shooter::declareAndGet<double>(*this, "duration_sec", 10.0);
    pre_hold_ = core_shooter::declareAndGet<double>(*this, "pre_hold_sec", 1.0);
    approach_rate_ = core_shooter::declareAndGet<double>(
      *this, "approach_rate_rad_per_sec", 0.3);
    joint_timeout_ = core_shooter::declareAndGet<double>(*this, "joint_timeout_sec", 0.2);
    max_tracking_error_ = core_shooter::declareAndGet<double>(
      *this, "max_tracking_error_rad", 0.5);
    tracking_error_time_ = core_shooter::declareAndGet<double>(
      *this, "tracking_error_time_sec", 0.5);
    abort_on_hazard_ = core_shooter::declareAndGet<bool>(*this, "abort_on_hazard", true);
    const std::string hazard_topic = core_shooter::declareAndGet<std::string>(
      *this, "hazard_topic", "/system/emergency/hazard_status");
    const std::string can_topic =
      core_shooter::declareAndGet<std::string>(*this, "can_topic", "/hardware/can/tx");
    const std::string joint_topic =
      core_shooter::declareAndGet<std::string>(*this, "joint_states_topic", "/joint_states");
    std::string csv_path = core_shooter::declareAndGet<std::string>(*this, "output_csv", "");

    // ---- 設定の検証（危険な設定では起動しない） ----
    if (motor_id_ < 0) {
      fail("motor_id must be set (left yaw=5 pitch=7, right yaw=6 pitch=11)");
    }
    if (!std::isfinite(angle_min_) || !std::isfinite(angle_max_) || angle_min_ >= angle_max_) {
      fail("angle_min / angle_max must be set explicitly (absolute motor angle [rad])");
    }
    if (controller_ != "mpc" && controller_ != "direct") {
      fail("controller must be 'mpc' or 'direct'");
    }
    if (ref_type_ != "triangle" && ref_type_ != "sine" && ref_type_ != "step") {
      fail("reference.type must be triangle, sine or step");
    }
    if (ref_preview_ != "constant_velocity" && ref_preview_ != "exact") {
      fail("reference.preview must be constant_velocity or exact");
    }
    if (!(ref_amplitude_ > 0.0) || !(ref_rate_ > 0.0) || !(ref_frequency_ > 0.0) ||
      !(ref_step_period_ > 0.0) || !(duration_ > 0.0) || !(approach_rate_ > 0.0))
    {
      fail("reference amplitude/rate/frequency/period, duration and approach rate must be > 0");
    }

    std::string reason;
    if (!core_shooter::loadTurretAxisModel(model_path, model_, reason)) {
      fail("model_path: " + reason);
    }
    config_.command_min = angle_min_;
    config_.command_max = angle_max_;
    if (!core_shooter::loadTurretMpcConfig(mpc_config_path, config_, reason)) {
      fail("mpc_config_path: " + reason);
    }
    if (!mpc_.configure(model_, config_, reason)) {
      fail("MPC configure failed: " + reason);
    }

    if (csv_path.empty()) {
      char stamp[32];
      const std::time_t now = std::time(nullptr);
      std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));
      csv_path = "turret_mpc_trial_" + axis_label_ + "_" + controller_ + "_" + stamp + ".csv";
    }
    csv_.open(csv_path);
    if (!csv_) {
      fail("cannot open output_csv: " + csv_path);
    }
    csv_ << "t,phase,reference,command,measured,hold_measured,constrained,disturbance,"
      "compute_us\n";

    can_pub_ = create_publisher<core_msgs::msg::CANArray>(can_topic, 10);
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_topic, 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        onJointState(*msg);
      });
    hazard_sub_ = create_subscription<std_msgs::msg::Bool>(
      hazard_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {hazard_ = msg->data;});

    // rclcpp::Time の既定値は SYSTEM_TIME なので、now()（ROS_TIME）と引き算できるよう揃えておく
    start_time_ = now();
    phase_start_ = start_time_;
    last_joint_time_ = start_time_;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(config_.sample_time_sec), [this]() {onTimer();});

    const auto & dm = mpc_.discreteModel();
    RCLCPP_INFO(
      get_logger(),
      "turret_mpc_trial: axis=%s motor_id=%d hold_motor_id=%d controller=%s range=[%.3f, %.3f] "
      "reference=%s(amp=%.3f rad) preview=%s rate=%.1fHz horizon=%d delay=%zu+%.4fs csv=%s",
      axis_label_.c_str(), motor_id_, hold_motor_id_, controller_.c_str(), angle_min_,
      angle_max_, ref_type_.c_str(), ref_amplitude_, ref_preview_.c_str(),
      1.0 / config_.sample_time_sec, config_.horizon_steps, dm.delay_steps,
      dm.fractional_delay_sec, csv_path.c_str());
  }

  bool finished() const {return phase_ == Phase::Done;}
  bool failed() const {return failed_;}

private:
  [[noreturn]] void fail(const std::string & message)
  {
    RCLCPP_FATAL(get_logger(), "%s", message.c_str());
    throw std::runtime_error(message);
  }

  void onJointState(const sensor_msgs::msg::JointState & msg)
  {
    const auto size = msg.position.size();
    if (static_cast<std::size_t>(motor_id_) >= size ||
      (hold_motor_id_ >= 0 && static_cast<std::size_t>(hold_motor_id_) >= size))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "joint_states too short: size=%zu", size);
      return;
    }
    measured_ = msg.position[motor_id_];
    if (hold_motor_id_ >= 0) {
      hold_measured_ = msg.position[hold_motor_id_];
    }
    last_joint_time_ = now();
    has_joint_ = true;
  }

  /// 仮想の的の角度 [rad]（中心からの相対）と角速度 [rad/s]。t は Run 開始からの経過時間。
  std::pair<double, double> virtualTarget(double t) const
  {
    const double a = ref_amplitude_;
    if (ref_type_ == "sine") {
      const double w = 2.0 * M_PI * ref_frequency_;
      return {a * std::sin(w * t), a * w * std::cos(w * t)};
    }
    if (ref_type_ == "step") {
      const double ph = std::fmod(t, ref_step_period_);
      return {ph < ref_step_period_ / 2.0 ? a : -a, 0.0};
    }
    // triangle: 0 から +a、-a、0 と等速往復
    const double v = ref_rate_;
    const double period = 4.0 * a / v;
    const double ph = std::fmod(t, period);
    if (ph < a / v) {
      return {v * ph, v};
    }
    if (ph < 3.0 * a / v) {
      return {a - v * (ph - a / v), -v};
    }
    return {-a + v * (ph - 3.0 * a / v), v};
  }

  void abort(const std::string & why)
  {
    if (phase_ == Phase::Abort || phase_ == Phase::Done) {
      return;
    }
    RCLCPP_ERROR(get_logger(), "ABORT: %s -> holding current angle", why.c_str());
    failed_ = true;
    enterPhase(Phase::Abort);
    hold_command_ = measured_;
  }

  void enterPhase(Phase p)
  {
    phase_ = p;
    phase_start_ = now();
    RCLCPP_INFO(get_logger(), "phase: %s", phaseName(p));
  }

  void send(double command)
  {
    core_shooter::publishMotorCommand(can_pub_, motor_id_, static_cast<float>(command));
    if (hold_motor_id_ >= 0 && hold_target_) {
      core_shooter::publishMotorCommand(
        can_pub_, hold_motor_id_,
        static_cast<float>(*hold_target_));
    }
  }

  void onTimer()
  {
    const rclcpp::Time t_now = now();
    const double phase_elapsed = (t_now - phase_start_).seconds();
    if (g_stop_requested && phase_ != Phase::Abort && phase_ != Phase::Done) {
      abort("interrupted (Ctrl+C)");
    }

    // ---- 監視 ----
    if (phase_ != Phase::Abort && phase_ != Phase::Done) {
      if (phase_ == Phase::WaitJoint) {
        if (!has_joint_ && (t_now - start_time_).seconds() > 3.0) {
          RCLCPP_ERROR(get_logger(), "no joint_states within 3 s; nothing was commanded");
          failed_ = true;
          phase_ = Phase::Done;
          return;
        }
      } else {
        if ((t_now - last_joint_time_).seconds() > joint_timeout_) {
          abort("joint_states timeout");
        } else if (measured_ < angle_min_ - range_margin_ ||
          measured_ > angle_max_ + range_margin_)
        {
          abort("measured angle " + std::to_string(measured_) + " is outside the allowed range");
        } else if (abort_on_hazard_ && hazard_) {
          abort("emergency stop (hazard_status=true)");
        }
      }
    }

    double reference = std::numeric_limits<double>::quiet_NaN();
    double command = last_command_;
    bool constrained = false;
    double disturbance = 0.0;
    double compute_us = 0.0;

    switch (phase_) {
      case Phase::WaitJoint: {
          if (!has_joint_) {
            return;
          }
          if (measured_ < angle_min_ || measured_ > angle_max_) {
            RCLCPP_ERROR(
              get_logger(), "current angle %.3f is outside [%.3f, %.3f]; refusing to move",
              measured_, angle_min_, angle_max_);
            failed_ = true;
            phase_ = Phase::Done;
            return;
          }
          center_ = std::isfinite(ref_center_param_) ? ref_center_param_ : measured_;
          if (center_ - ref_amplitude_ < angle_min_ || center_ + ref_amplitude_ > angle_max_) {
            RCLCPP_ERROR(
              get_logger(),
              "reference [%.3f, %.3f] exceeds the allowed range [%.3f, %.3f]; refusing to start",
              center_ - ref_amplitude_, center_ + ref_amplitude_, angle_min_, angle_max_);
            failed_ = true;
            phase_ = Phase::Done;
            return;
          }
          if (hold_motor_id_ >= 0) {
            hold_target_ = hold_measured_;
          }
          last_command_ = measured_;
          command = measured_;
          enterPhase(Phase::Approach);
          break;
        }
      case Phase::Approach:
        command = moveToward(last_command_, center_, approach_rate_ * config_.sample_time_sec);
        reference = center_;
        if (command == center_) {
          enterPhase(Phase::PreHold);
        }
        break;
      case Phase::PreHold:
        command = center_;
        reference = center_;
        if (phase_elapsed >= pre_hold_) {
          mpc_.reset(measured_, center_);
          enterPhase(Phase::Run);
        }
        break;
      case Phase::Run: {
          const double t = phase_elapsed;
          const auto [offset, rate] = virtualTarget(t);
          reference = center_ + offset;
          if (controller_ == "mpc") {
            std::vector<double> ref;
            const double ts = config_.sample_time_sec;
            if (ref_preview_ == "exact") {
              for (int j = 1; j <= config_.horizon_steps; ++j) {
                ref.push_back(center_ + virtualTarget(t + j * ts).first);
              }
            } else {
              ref = core_shooter::makeConstantVelocityReference(
                reference, rate, config_.horizon_steps, ts);
            }
            const auto start = std::chrono::steady_clock::now();
            const auto sol = mpc_.compute(measured_, ref);
            compute_us = std::chrono::duration<double, std::micro>(
              std::chrono::steady_clock::now() - start).count();
            command = sol.command;
            constrained = sol.constrained;
            disturbance = sol.disturbance;
            mpc_.commit(command);
          } else {
            command = std::clamp(reference, angle_min_, angle_max_);
            command = moveToward(last_command_, command, config_.max_command_step);
          }
          // 追従誤差の持続（引っ掛かり・衝突の疑い）
          if (std::fabs(measured_ - reference) > max_tracking_error_) {
            tracking_error_time_acc_ += config_.sample_time_sec;
            if (tracking_error_time_acc_ > tracking_error_time_) {
              abort("tracking error stayed above max_tracking_error_rad");
              command = hold_command_;
            }
          } else {
            tracking_error_time_acc_ = 0.0;
          }
          if (phase_ == Phase::Run && t >= duration_) {
            enterPhase(Phase::Return);
          }
          break;
        }
      case Phase::Return:
        command = moveToward(last_command_, center_, approach_rate_ * config_.sample_time_sec);
        reference = center_;
        if (command == center_ && phase_elapsed > 0.5) {
          RCLCPP_INFO(get_logger(), "trial finished");
          phase_ = Phase::Done;
        }
        break;
      case Phase::Abort:
        command = hold_command_;
        if (phase_elapsed > 0.3) {
          phase_ = Phase::Done;
        }
        break;
      case Phase::Done:
        return;
    }

    // 最終防衛線: どの段階でも範囲外の指令は送らない
    command = std::clamp(command, angle_min_, angle_max_);
    send(command);
    last_command_ = command;

    csv_ << (t_now - start_time_).seconds() << ',' << phaseName(phase_) << ',' << reference <<
      ',' << command << ',' << measured_ << ',' << hold_measured_ << ',' << (constrained ? 1 : 0) <<
      ',' << disturbance << ',' << compute_us << '\n';
  }

  // パラメータ
  int motor_id_ = -1;
  int hold_motor_id_ = -1;
  std::string axis_label_;
  std::string controller_;
  double angle_min_ = 0.0;
  double angle_max_ = 0.0;
  double range_margin_ = 0.05;
  std::string ref_type_;
  double ref_amplitude_ = 0.2;
  double ref_rate_ = 0.5;
  double ref_frequency_ = 0.5;
  double ref_step_period_ = 2.0;
  double ref_center_param_ = 0.0;
  std::string ref_preview_;
  double duration_ = 10.0;
  double pre_hold_ = 1.0;
  double approach_rate_ = 0.3;
  double joint_timeout_ = 0.1;
  double max_tracking_error_ = 0.5;
  double tracking_error_time_ = 0.5;
  bool abort_on_hazard_ = true;

  // 状態
  core_shooter::TurretAxisModel model_;
  core_shooter::TurretMpcConfig config_;
  core_shooter::TurretMpc mpc_;
  Phase phase_ = Phase::WaitJoint;
  bool failed_ = false;
  bool has_joint_ = false;
  bool hazard_ = false;
  double measured_ = 0.0;
  double hold_measured_ = std::numeric_limits<double>::quiet_NaN();
  std::optional<double> hold_target_;
  double center_ = 0.0;
  double last_command_ = 0.0;
  double hold_command_ = 0.0;
  double tracking_error_time_acc_ = 0.0;
  rclcpp::Time start_time_;
  rclcpp::Time phase_start_;
  rclcpp::Time last_joint_time_;
  std::ofstream csv_;

  rclcpp::Publisher<core_msgs::msg::CANArray>::SharedPtr can_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hazard_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  // Ctrl+C で即終了すると保持指令を送れないので、rclcpp のシグナル処理を使わず自前で受ける。
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  int exit_code = 0;
  try {
    auto node = std::make_shared<TurretMpcTrial>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !node->finished()) {
      executor.spin_once(10ms);
    }
    exit_code = node->failed() ? 1 : 0;
  } catch (const std::exception & e) {
    std::fprintf(stderr, "turret_mpc_trial: %s\n", e.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
