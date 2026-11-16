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

// 砲塔 1 つ分の最後に受けた値
struct TurretState
{
  bool manual_mode = true;
  bool jam = false;
  bool loading_error = false;
  bool shooting_error = false;
  std::optional<rclcpp::Time> last_shot_event;
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
    enemy_detected_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "enemy_detected", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {enemy_detected_ = msg->data;});

    for (std::size_t i = 0; i < kTurretSides.size(); ++i) {
      subscribeTurret(i, shooter_namespace + "/" + kTurretSides[i]);
    }

    //========================================
    // publishers
    //========================================
    led_upper_pub_ = this->create_publisher<core_msgs::msg::LedCommand>("led/upper", 10);
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
    const auto load = [this](const std::string & name, core_mode::LedCommand & command) {
        const auto value = this->declare_parameter<std::vector<int64_t>>(
          "commands." + name,
          std::vector<int64_t>{command.pattern, command.color});
        if (value.size() != 2 || value[0] < 0 || value[0] > 255 || value[1] < 0 ||
          value[1] > 255)
        {
          throw std::invalid_argument(
                  "commands." + name + " must be [pattern, color] with values in 0..255");
        }
        command.pattern = static_cast<uint8_t>(value[0]);
        command.color = static_cast<uint8_t>(value[1]);
      };
    load("microcontroller_timeout", table_.microcontroller_timeout);
    load("controller_disconnected", table_.controller_disconnected);
    load("destroy", table_.destroy);
    load("software_emergency", table_.software_emergency);
    load("turret_error", table_.turret_error);
    load("turret_auto_firing", table_.turret_auto_firing);
    load("turret_manual_firing", table_.turret_manual_firing);
    load("turret_enemy_detected", table_.turret_enemy_detected);
    load("turret_auto_idle", table_.turret_auto_idle);
    load("turret_manual", table_.turret_manual);
    load("chassis_auto_rotating", table_.chassis_auto_rotating);
    load("chassis_manual_rotating", table_.chassis_manual_rotating);
    load("chassis_auto", table_.chassis_auto);
    load("chassis_manual", table_.chassis_manual);
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
    in.enemy_detected = enemy_detected_;
    for (std::size_t i = 0; i < turrets_.size(); ++i) {
      const TurretState & turret = turrets_[i];
      in.turrets[i].manual_mode = turret.manual_mode;
      in.turrets[i].error = turret.jam || turret.loading_error || turret.shooting_error;
      in.turrets[i].manual_firing = isWithin(turret.last_shot_event, manual_fire_window_);
      in.turrets[i].auto_firing =
        isWithin(turret.aim_status_stamp, input_timeout_) && turret.aim_status == "firing";
    }
    return in;
  }

  void publishCommands()
  {
    const core_mode::LedCommandPair commands =
      core_mode::selectLedCommands(collectInputs(), table_);
    led_upper_pub_->publish(toMsg(commands.upper));
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
  bool enemy_detected_ = false;
  std::array<TurretState, 2> turrets_{};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hazard_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr hazard_states_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr behavior_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rotation_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enemy_detected_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> turret_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> aim_status_subs_;
  rclcpp::Publisher<core_msgs::msg::LedCommand>::SharedPtr led_upper_pub_;
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
