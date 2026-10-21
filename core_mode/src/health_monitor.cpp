// 緊急停止に至らない異常(センサの通信断・電圧低下など)を収集し、/diagnostics に出力する。
// hazard(緊急停止)とは分離した表示用途のノード。

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace
{

using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;

//========================================
// 判定ロジック(副作用なし)
//========================================
struct TopicCheckConfig
{
  std::string name;
  std::string topic;
  std::string type;
  int timeout_ms;
  uint8_t timeout_level;
};

struct BatteryConfig
{
  std::string topic;
  int timeout_ms;
  double warn_voltage;
  double error_voltage;
};

KeyValue make_key_value(const std::string & key, const std::string & value)
{
  KeyValue kv;
  kv.key = key;
  kv.value = value;
  return kv;
}

DiagnosticStatus make_status(
  const std::string & name, const std::string & hardware_id, uint8_t level,
  const std::string & message, std::vector<KeyValue> values)
{
  DiagnosticStatus status;
  status.name = name;
  status.hardware_id = hardware_id;
  status.level = level;
  status.message = message;
  status.values = std::move(values);
  return status;
}

// age_ms: 最後の受信からの経過時間。未受信なら nullopt
DiagnosticStatus evaluate_topic(
  const TopicCheckConfig & config, std::optional<double> age_ms)
{
  const auto name = "sensor/" + config.name;
  if (!age_ms) {
    return make_status(name, config.topic, DiagnosticStatus::STALE, "no data", {});
  }
  std::vector<KeyValue> values{make_key_value("age_ms", std::to_string(static_cast<int>(*age_ms)))};
  if (*age_ms > config.timeout_ms) {
    return make_status(
      name, config.topic, config.timeout_level,
      "timeout (" + std::to_string(config.timeout_ms) + " ms)", std::move(values));
  }
  return make_status(name, config.topic, DiagnosticStatus::OK, "OK", std::move(values));
}

DiagnosticStatus evaluate_battery(
  const BatteryConfig & config, std::optional<double> age_ms,
  const std::optional<sensor_msgs::msg::BatteryState> & battery)
{
  const std::string name = "power/battery";
  if (!age_ms || !battery || *age_ms > config.timeout_ms) {
    return make_status(name, config.topic, DiagnosticStatus::STALE, "no data", {});
  }
  std::vector<KeyValue> values{
    make_key_value("voltage", std::to_string(battery->voltage)),
    make_key_value("percentage", std::to_string(battery->percentage)),
  };
  if (battery->voltage < config.error_voltage) {
    return make_status(name, config.topic, DiagnosticStatus::ERROR, "voltage critical", std::move(values));
  }
  if (battery->voltage < config.warn_voltage) {
    return make_status(name, config.topic, DiagnosticStatus::WARN, "voltage low", std::move(values));
  }
  return make_status(name, config.topic, DiagnosticStatus::OK, "OK", std::move(values));
}

uint8_t parse_level(const std::string & level, const std::string & param_name)
{
  if (level == "warn") {return DiagnosticStatus::WARN;}
  if (level == "error") {return DiagnosticStatus::ERROR;}
  throw std::invalid_argument(
          param_name + " must be \"warn\" or \"error\", got \"" + level + "\"");
}

}  // namespace

class HealthMonitor : public rclcpp::Node
{
public:
  HealthMonitor()
  : Node("health_monitor")
  {
    //========================================
    // parameters
    //========================================
    const auto publish_period_ms = this->declare_parameter("publish_period_ms", 1000);
    const auto check_names =
      this->declare_parameter<std::vector<std::string>>("topic_checks", std::vector<std::string>{});
    for (const auto & name : check_names) {
      topic_checks_.push_back(TopicCheckConfig{
          name,
          this->declare_parameter<std::string>(name + ".topic"),
          this->declare_parameter<std::string>(name + ".type"),
          static_cast<int>(this->declare_parameter(name + ".timeout_ms", 1000)),
          parse_level(
            this->declare_parameter<std::string>(name + ".level", "warn"), name + ".level"),
        });
    }

    battery_enabled_ = this->declare_parameter("battery.enabled", true);
    battery_config_ = BatteryConfig{
      this->declare_parameter<std::string>("battery.topic", "/hardware/battery"),
      static_cast<int>(this->declare_parameter("battery.timeout_ms", 2000)),
      this->declare_parameter("battery.warn_voltage", 22.2),
      this->declare_parameter("battery.error_voltage", 21.0),
    };

    //========================================
    // subscribers
    //========================================
    // 型に依存せず受信時刻だけを見るため GenericSubscription を使う。
    // SensorDataQoS(best effort) は reliable / best effort どちらの publisher とも接続できる
    last_received_.resize(topic_checks_.size());
    for (size_t i = 0; i < topic_checks_.size(); ++i) {
      topic_subscriptions_.push_back(
        create_generic_subscription(
          topic_checks_[i].topic, topic_checks_[i].type, rclcpp::SensorDataQoS(),
          [this, i](std::shared_ptr<rclcpp::SerializedMessage>) {
            last_received_[i] = get_clock()->now();
          }));
    }

    if (battery_enabled_) {
      battery_subscription_ = create_subscription<sensor_msgs::msg::BatteryState>(
        battery_config_.topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
          battery_last_received_ = get_clock()->now();
          battery_ = *msg;
        });
    }

    //========================================
    // publishers
    //========================================
    diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

    //========================================
    // timer
    //========================================
    timer_ = create_wall_timer(
      std::chrono::milliseconds(publish_period_ms),
      std::bind(&HealthMonitor::publish_diagnostics, this));

    RCLCPP_INFO(
      get_logger(), "< health_monitor > topic checks: %zu, battery: %s",
      topic_checks_.size(), battery_enabled_ ? battery_config_.topic.c_str() : "disabled");
  }

private:
  //========================================
  // main logic
  //========================================
  void publish_diagnostics()
  {
    const auto now = get_clock()->now();
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now;
    for (size_t i = 0; i < topic_checks_.size(); ++i) {
      array.status.push_back(evaluate_topic(topic_checks_[i], age_ms(now, last_received_[i])));
    }
    if (battery_enabled_) {
      array.status.push_back(
        evaluate_battery(battery_config_, age_ms(now, battery_last_received_), battery_));
    }
    diagnostics_publisher_->publish(array);
  }

  static std::optional<double> age_ms(
    const rclcpp::Time & now, const std::optional<rclcpp::Time> & last)
  {
    if (!last) {return std::nullopt;}
    return (now - *last).seconds() * 1000.0;
  }

  //========================================
  // Subscription members
  //========================================
  std::vector<rclcpp::GenericSubscription::SharedPtr> topic_subscriptions_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscription_;

  //========================================
  // publisher members
  //========================================
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;

  //========================================
  // timer callback member variable
  //========================================
  rclcpp::TimerBase::SharedPtr timer_;

  //========================================
  // state
  //========================================
  std::vector<TopicCheckConfig> topic_checks_;
  std::vector<std::optional<rclcpp::Time>> last_received_;
  bool battery_enabled_ = true;
  BatteryConfig battery_config_;
  std::optional<rclcpp::Time> battery_last_received_;
  std::optional<sensor_msgs::msg::BatteryState> battery_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HealthMonitor>());
  rclcpp::shutdown();
  return 0;
}
