#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "core_mode/led_selector.hpp"
#include "core_msgs/msg/led_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

// hazard_states の並び (emergency_handler の publishHazardState と同じ)
constexpr std::size_t kHazardMicrocontroller = 2;
constexpr std::size_t kHazardBridge = 3;
constexpr std::size_t kHazardDestroy = 4;

constexpr std::array<const char *, 2> kTurretSides{"left", "right"};

// shooter_controller がモータを回す shoot_motor の下限 (これより大きければ回す)
constexpr float kShootMotorOnThreshold = 0.1F;

// 砲塔 1 つ分の最後に受けた値
struct TurretState
{
  bool manual_mode = true;
  bool jam = false;
  bool loading_error = false;
  bool shooting_error = false;
  std::optional<rclcpp::Time> last_shot_event;
  bool shoot_motor_on = false;
  std::string aim_status;
  std::optional<rclcpp::Time> aim_status_stamp;
};

}  // namespace

class LedManager : public rclcpp::Node
{
public:
  LedManager()
  : Node("led_manager")
  {
    //========================================
    // parameters
    //========================================
    const double publish_rate_hz = this->declare_parameter("publish_rate_hz", 10.0);
    if (!(publish_rate_hz > 0.0)) {
      throw std::invalid_argument("publish_rate_hz must be positive");
    }
    state_timeout_ = millisecondsParameter("state_timeout_ms", 1000);
    input_timeout_ = millisecondsParameter("input_timeout_ms", 1000);
    manual_fire_window_ = millisecondsParameter("manual_fire_window_ms", 500);
    const auto shooter_namespace =
      this->declare_parameter<std::string>("shooter_namespace", "/mecha/shooter");
    loadCommandTable();

    //========================================
    // subscribers
    //========================================
    // emergency_handler の QoS に合わせる
    const auto hazard_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    hazard_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "hazard_status", hazard_qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {software_emergency_ = msg->data;});
    hazard_states_sub_ = this->create_subscription<std_msgs::msg::Int8MultiArray>(
      "hazard_states", hazard_qos,
      [this](const std_msgs::msg::Int8MultiArray::SharedPtr msg) {hazard_states_ = msg->data;});
    behavior_state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "behavior_state", 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        behavior_state_ = msg->data;
        behavior_state_stamp_ = this->now();
      });
    // rotation は値が変わったときだけ publish されるので、最後の値を使い続ける
    rotation_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "rotation", 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {rotation_ = msg->data;});

    for (std::size_t i = 0; i < kTurretSides.size(); ++i) {
      subscribeTurret(i, shooter_namespace + "/" + kTurretSides[i]);
    }

    //========================================
    // publishers
    //========================================
    // 上のテープ: led/upper = 中央 (全体)、led/upper_left・led/upper_right = 左右の砲塔
    led_upper_pub_ = this->create_publisher<core_msgs::msg::LedCommand>("led/upper", 10);
    led_upper_left_pub_ =
      this->create_publisher<core_msgs::msg::LedCommand>("led/upper_left", 10);
    led_upper_right_pub_ =
      this->create_publisher<core_msgs::msg::LedCommand>("led/upper_right", 10);
    led_bottom_pub_ = this->create_publisher<core_msgs::msg::LedCommand>("led/bottom", 10);

    // core_hardware は指令の途絶で LED を「指令なし」にするので、変化がなくても一定周期で出す
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz), [this]() {publishCommands();});

    RCLCPP_INFO(this->get_logger(), "LedManager initialized");
  }

private:
  std::chrono::milliseconds millisecondsParameter(const std::string & name, int64_t default_ms)
  {
    const int64_t value = this->declare_parameter<int64_t>(name, default_ms);
    if (value <= 0) {
      throw std::invalid_argument(name + " must be positive");
    }
    return std::chrono::milliseconds(value);
  }

  // commands.<状態名>: [パターン, 色]
  void loadCommandTable()
  {
    const auto load_with_limit = [this](
      const std::string & name, core_mode::LedCommand & command,
      int64_t max_pattern, int64_t max_color) {
        const auto value = this->declare_parameter<std::vector<int64_t>>(
          "commands." + name,
          std::vector<int64_t>{command.pattern, command.color});
        if (value.size() != 2 || value[0] < 0 || value[0] > max_pattern || value[1] < 0 ||
          value[1] > max_color)
        {
          throw std::invalid_argument(
                  "commands." + name + " must be [pattern, color] with pattern in 0.." +
                  std::to_string(max_pattern) + " and color in 0.." + std::to_string(max_color));
        }
        command.pattern = static_cast<uint8_t>(value[0]);
        command.color = static_cast<uint8_t>(value[1]);
      };
    const auto load = [&load_with_limit](const std::string & name,
      core_mode::LedCommand & command) {
        load_with_limit(name, command, 255, 255);
      };
    // 砲塔の区間は、core_hardware が左右を 1 枠にまとめて送るので 1 byte (パターン 4 bit・色 3 bit) に収める。
    // 異常の表示もこの区間に出るので同じ制限にする
    const auto load_turret = [&load_with_limit](
      const std::string & name, core_mode::LedCommand & command) {
        load_with_limit(name, command, 15, 7);
      };
    load_turret("microcontroller_timeout", table_.microcontroller_timeout);
    load_turret("controller_disconnected", table_.controller_disconnected);
    load_turret("destroy", table_.destroy);
    load_turret("software_emergency", table_.software_emergency);
    load_turret("turret_error", table_.turret_error);
    load_turret("turret_auto_firing", table_.turret_auto_firing);
    load_turret("turret_manual_firing", table_.turret_manual_firing);
    load_turret("turret_manual_motor_on", table_.turret_manual_motor_on);
    load_turret("turret_enemy_detected", table_.turret_enemy_detected);
    load_turret("turret_auto_motor_on", table_.turret_auto_motor_on);
    load_turret("turret_auto_fire_off", table_.turret_auto_fire_off);
    load_turret("turret_auto_idle", table_.turret_auto_idle);
    load_turret("turret_manual", table_.turret_manual);
    load("chassis_auto_rotating", table_.chassis_auto_rotating);
    load("chassis_manual_rotating", table_.chassis_manual_rotating);
    load("chassis_auto", table_.chassis_auto);
    load("chassis_manual", table_.chassis_manual);
    load("behavior_manual", table_.behavior_manual);
    load("behavior_attack", table_.behavior_attack);
    load("behavior_auto_selected", table_.behavior_auto_selected);
    load("behavior_auto_waypoint", table_.behavior_auto_waypoint);
    load("behavior_auto_idle", table_.behavior_auto_idle);
    load("behavior_unknown", table_.behavior_unknown);
  }

  void subscribeTurret(std::size_t index, const std::string & prefix)
  {
    TurretState & turret = turrets_[index];
    const auto bool_sub = [this](const std::string & topic, bool & target) {
        return this->create_subscription<std_msgs::msg::Bool>(
          topic, 10, [&target](const std_msgs::msg::Bool::SharedPtr msg) {target = msg->data;});
      };
    turret_subs_.push_back(bool_sub(prefix + "/manual_mode", turret.manual_mode));
    turret_subs_.push_back(bool_sub(prefix + "/jam_state", turret.jam));
    turret_subs_.push_back(bool_sub(prefix + "/loading_motor_error_state", turret.loading_error));
    turret_subs_.push_back(bool_sub(prefix + "/shoot_motor_error_state", turret.shooting_error));
    // shooter_controller は 1 発ごとに撃ち始めで false、撃ち終わりで true を出す
    turret_subs_.push_back(
      this->create_subscription<std_msgs::msg::Bool>(
        prefix + "/shoot_status", 10,
        [this, &turret](const std_msgs::msg::Bool::SharedPtr) {
          turret.last_shot_event = this->now();
        }));
    // 発射モータの指令。UI の切り替え時にだけ、手動・自動に関係なく左右へ出るので、最後の値を使い続ける
    shoot_motor_subs_.push_back(
      this->create_subscription<std_msgs::msg::Float32>(
        prefix + "/shoot_motor", 10,
        [&turret](const std_msgs::msg::Float32::SharedPtr msg) {
          turret.shoot_motor_on = msg->data > kShootMotorOnThreshold;
        }));
    aim_status_subs_.push_back(
      this->create_subscription<std_msgs::msg::String>(
        prefix + "/aim_status", 10,
        [this, &turret](const std_msgs::msg::String::SharedPtr msg) {
          turret.aim_status = msg->data;
          turret.aim_status_stamp = this->now();
        }));
  }

  bool hazardState(std::size_t index) const
  {
    return index < hazard_states_.size() && hazard_states_[index] != 0;
  }

  bool isWithin(const std::optional<rclcpp::Time> & stamp, std::chrono::milliseconds window) const
  {
    return stamp.has_value() && (this->now() - *stamp) <= rclcpp::Duration(window);
  }

  core_mode::LedInputs collectInputs() const
  {
    core_mode::LedInputs in;
    in.microcontroller_timeout = hazardState(kHazardMicrocontroller);
    in.controller_disconnected = hazardState(kHazardBridge);
    in.destroy = hazardState(kHazardDestroy);
    in.software_emergency = software_emergency_;
    if (isWithin(behavior_state_stamp_, state_timeout_)) {
      in.behavior_state = behavior_state_;
    }
    in.rotating = rotation_ != 0;
    for (std::size_t i = 0; i < turrets_.size(); ++i) {
      const TurretState & turret = turrets_[i];
      in.turrets[i].manual_mode = turret.manual_mode;
      in.turrets[i].error = turret.jam || turret.loading_error || turret.shooting_error;
      in.turrets[i].manual_firing = isWithin(turret.last_shot_event, manual_fire_window_);
      in.turrets[i].shoot_motor_on = turret.shoot_motor_on;
      in.turrets[i].aim = isWithin(turret.aim_status_stamp, input_timeout_) ?
        core_mode::classifyAimStatus(turret.aim_status) : core_mode::AimStatus::kUnknown;
    }
    return in;
  }

  void publishCommands()
  {
    const core_mode::LedCommands commands =
      core_mode::selectLedCommands(collectInputs(), table_);
    led_upper_pub_->publish(toMsg(commands.upper_overall));
    led_upper_left_pub_->publish(toMsg(commands.upper_left));
    led_upper_right_pub_->publish(toMsg(commands.upper_right));
    led_bottom_pub_->publish(toMsg(commands.bottom));
  }

  static core_msgs::msg::LedCommand toMsg(const core_mode::LedCommand & command)
  {
    core_msgs::msg::LedCommand msg;
    msg.pattern = command.pattern;
    msg.color = command.color;
    return msg;
  }

  core_mode::LedCommandTable table_;
  std::chrono::milliseconds state_timeout_{1000};
  std::chrono::milliseconds input_timeout_{1000};
  std::chrono::milliseconds manual_fire_window_{500};

  bool software_emergency_ = false;
  std::vector<int8_t> hazard_states_;
  int32_t behavior_state_ = 0;
  std::optional<rclcpp::Time> behavior_state_stamp_;
  int32_t rotation_ = 0;
  std::array<TurretState, 2> turrets_{};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hazard_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr hazard_states_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr behavior_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rotation_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> turret_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> aim_status_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> shoot_motor_subs_;
  rclcpp::Publisher<core_msgs::msg::LedCommand>::SharedPtr led_upper_pub_;
  rclcpp::Publisher<core_msgs::msg::LedCommand>::SharedPtr led_upper_left_pub_;
  rclcpp::Publisher<core_msgs::msg::LedCommand>::SharedPtr led_upper_right_pub_;
  rclcpp::Publisher<core_msgs::msg::LedCommand>::SharedPtr led_bottom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LedManager>());
  rclcpp::shutdown();
  return 0;
}
