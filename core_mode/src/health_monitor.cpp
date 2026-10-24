// 緊急停止に至らない異常(センサの通信断・電圧低下など)を収集し、/diagnostics に出力する。
// hazard(緊急停止)とは分離した表示用途のノード。

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "core_msgs/msg/hardware_link_status.hpp"
#include "core_msgs/msg/motor_status_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace
{

using core_msgs::msg::CanBusStatus;
using core_msgs::msg::HardwareLinkStatus;
using core_msgs::msg::MotorStatus;
using core_msgs::msg::MotorStatusArray;
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

struct LinkStatusConfig
{
  std::string topic;
  int timeout_ms;
};

struct MotorStatusConfig
{
  std::string topic;
  int timeout_ms;
  std::vector<std::string> names;  // names[id] を motor/<name> とする
  double temp_warn_c;
  double temp_error_c;
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

std::string format_fixed(double value, int precision)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

std::string format_hex(uint8_t value)
{
  std::ostringstream stream;
  stream << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0') <<
    static_cast<int>(value);
  return stream.str();
}

// 未受信なら "no data"、timeout_ms を超えて途絶えていれば "timeout (... ms)"。受信中なら nullopt
std::optional<std::string> stale_reason(std::optional<double> age_ms, int timeout_ms)
{
  if (!age_ms) {return "no data";}
  if (*age_ms > timeout_ms) {return "timeout (" + std::to_string(timeout_ms) + " ms)";}
  return std::nullopt;
}

std::vector<KeyValue> with_age(std::vector<KeyValue> values, std::optional<double> age_ms)
{
  if (age_ms) {values.push_back(make_key_value("age_ms", format_fixed(*age_ms, 0)));}
  return values;
}

// upstream_down: 経路の手前の区間が切れている理由。切れていれば状態は分からないため STALE とする
DiagnosticStatus evaluate_connection(
  const std::string & name, const std::string & hardware_id, bool connected,
  const std::optional<std::string> & upstream_down, std::vector<KeyValue> values)
{
  if (upstream_down) {
    return make_status(
      name, hardware_id, DiagnosticStatus::STALE, *upstream_down, std::move(values));
  }
  if (!connected) {
    return make_status(
      name, hardware_id, DiagnosticStatus::ERROR, "disconnected", std::move(values));
  }
  return make_status(name, hardware_id, DiagnosticStatus::OK, "OK", std::move(values));
}

// 経路は PC(core_hardware) → daemon → upper 基板(EtherCAT) → bottom 基板(CAN) の順につながる
std::vector<DiagnosticStatus> evaluate_link_status(
  const LinkStatusConfig & config, std::optional<double> age_ms,
  const std::optional<HardwareLinkStatus> & link)
{
  const std::string daemon = "link/daemon";
  const std::string ethercat = "link/ethercat";
  const std::string bottom = "link/bottom";
  const auto reason = stale_reason(age_ms, config.timeout_ms);
  if (reason || !link) {
    const auto message = reason.value_or("no data");
    const auto values = with_age({}, age_ms);
    return {
      make_status(daemon, config.topic, DiagnosticStatus::STALE, message, values),
      make_status(ethercat, config.topic, DiagnosticStatus::STALE, message, values),
      make_status(bottom, config.topic, DiagnosticStatus::STALE, message, values),
    };
  }
  std::optional<std::string> ethercat_upstream_down;
  std::optional<std::string> bottom_upstream_down;
  if (!link->daemon_connected) {
    ethercat_upstream_down = "daemon disconnected";
    bottom_upstream_down = "daemon disconnected";
  } else if (!link->ethercat_connected) {
    bottom_upstream_down = "ethercat disconnected";
  }
  // 不明 (NaN) のときは出さない
  std::vector<KeyValue> ethercat_values;
  if (!std::isnan(link->ethercat_rx_age_ms)) {
    ethercat_values.push_back(
      make_key_value("ethercat_rx_age_ms", format_fixed(link->ethercat_rx_age_ms, 0)));
  }
  return {
    evaluate_connection(daemon, config.topic, link->daemon_connected, std::nullopt, {}),
    evaluate_connection(
      ethercat, config.topic, link->ethercat_connected, ethercat_upstream_down, ethercat_values),
    evaluate_connection(
      bottom, config.topic, link->bottom_connected, bottom_upstream_down, {}),
  };
}

std::string motor_name(const std::vector<std::string> & names, size_t id)
{
  return "motor/" + (id < names.size() ? names[id] : std::to_string(id));
}

std::string can_name(const CanBusStatus & bus, size_t index)
{
  return "can/" + (bus.name.empty() ? std::to_string(index) : bus.name);
}

// 取得できない値 (NaN) は出さない
std::vector<KeyValue> motor_values(const MotorStatus & motor)
{
  std::vector<KeyValue> values{
    make_key_value("id", std::to_string(motor.id)),
    make_key_value("enabled", motor.enabled ? "true" : "false"),
    make_key_value("error_code", format_hex(motor.error_code)),
  };
  const auto add_if_available = [&values](const std::string & key, float value, int precision) {
      if (!std::isnan(value)) {
        values.push_back(make_key_value(key, format_fixed(value, precision)));
      }
    };
  add_if_available("temp", motor.temp, 1);
  add_if_available("temp_rotor", motor.temp_rotor, 1);
  add_if_available("voltage", motor.voltage, 2);
  return values;
}

// 取得できた温度 (主温度・ローター温度) の高いほう。どちらも取得できなければ nullopt
std::optional<double> max_temperature(const MotorStatus & motor)
{
  std::optional<double> result;
  for (const float temp : {motor.temp, motor.temp_rotor}) {
    if (!std::isnan(temp) && (!result || temp > *result)) {result = temp;}
  }
  return result;
}

DiagnosticStatus evaluate_motor(const MotorStatusConfig & config, const MotorStatus & motor)
{
  const auto name = motor_name(config.names, motor.id);
  auto values = motor_values(motor);
  if (!motor.connected) {
    return make_status(
      name, config.topic, DiagnosticStatus::ERROR, "disconnected", std::move(values));
  }
  if (motor.fault) {
    return make_status(
      name, config.topic, DiagnosticStatus::ERROR,
      "fault (code " + format_hex(motor.error_code) + ")", std::move(values));
  }
  const auto temp = max_temperature(motor);
  if (temp && *temp >= config.temp_error_c) {
    return make_status(
      name, config.topic, DiagnosticStatus::ERROR,
      "temperature critical (" + format_fixed(*temp, 1) + " C)", std::move(values));
  }
  if (temp && *temp >= config.temp_warn_c) {
    return make_status(
      name, config.topic, DiagnosticStatus::WARN,
      "temperature high (" + format_fixed(*temp, 1) + " C)", std::move(values));
  }
  return make_status(name, config.topic, DiagnosticStatus::OK, "OK", std::move(values));
}

DiagnosticStatus evaluate_can_bus(
  const std::string & hardware_id, const CanBusStatus & bus, size_t index)
{
  const auto name = can_name(bus, index);
  if (!bus.valid) {
    return make_status(name, hardware_id, DiagnosticStatus::STALE, "not available", {});
  }
  std::vector<KeyValue> values{
    make_key_value("tx_error_count", std::to_string(bus.tx_error_count)),
    make_key_value("rx_error_count", std::to_string(bus.rx_error_count)),
  };
  if (bus.bus_off) {
    return make_status(name, hardware_id, DiagnosticStatus::ERROR, "bus off", std::move(values));
  }
  if (bus.error_passive) {
    return make_status(
      name, hardware_id, DiagnosticStatus::WARN, "error passive", std::move(values));
  }
  return make_status(name, hardware_id, DiagnosticStatus::OK, "OK", std::move(values));
}

// 未受信のときはどのモータが診断対象か分からないため、names の全 ID を STALE とする。
// タイムアウトのときは最後に受信した構成 (supported なモータと CAN バス) を STALE とする
std::vector<DiagnosticStatus> evaluate_motor_status(
  const MotorStatusConfig & config, std::optional<double> age_ms,
  const std::optional<MotorStatusArray> & status)
{
  const auto reason = stale_reason(age_ms, config.timeout_ms);
  std::vector<DiagnosticStatus> result;
  if (!status) {
    for (size_t id = 0; id < config.names.size(); ++id) {
      result.push_back(
        make_status(
          motor_name(config.names, id), config.topic, DiagnosticStatus::STALE,
          reason.value_or("no data"), {make_key_value("id", std::to_string(id))}));
    }
    return result;
  }
  for (const auto & motor : status->motors) {
    if (!motor.supported) {continue;}  // 診断対象外のモータは出さない
    result.push_back(
      reason ?
      make_status(
        motor_name(config.names, motor.id), config.topic, DiagnosticStatus::STALE, *reason,
        with_age({make_key_value("id", std::to_string(motor.id))}, age_ms)) :
      evaluate_motor(config, motor));
  }
  for (size_t i = 0; i < status->can_buses.size(); ++i) {
    result.push_back(
      reason ?
      make_status(
        can_name(status->can_buses[i], i), config.topic, DiagnosticStatus::STALE, *reason,
        with_age({}, age_ms)) :
      evaluate_can_bus(config.topic, status->can_buses[i], i));
  }
  return result;
}

uint8_t parse_level(const std::string & level, const std::string & param_name)
{
  if (level == "warn") {return DiagnosticStatus::WARN;}
  if (level == "error") {return DiagnosticStatus::ERROR;}
  throw std::invalid_argument(
          param_name + " must be \"warn\" or \"error\", got \"" + level + "\"");
}

void validate_timeout(int timeout_ms, const std::string & param_name)
{
  if (timeout_ms <= 0) {
    throw std::invalid_argument(param_name + " must be > 0, got " + std::to_string(timeout_ms));
  }
}

void validate_motor_status_config(const MotorStatusConfig & config)
{
  validate_timeout(config.timeout_ms, "motor_status.timeout_ms");
  // NaN も弾くため否定で比較する
  if (!(config.temp_warn_c <= config.temp_error_c)) {
    throw std::invalid_argument(
            "motor_status.temp_warn_c (" + format_fixed(config.temp_warn_c, 1) +
            ") must be <= motor_status.temp_error_c (" + format_fixed(config.temp_error_c, 1) +
            ")");
  }
  // 名前が空・重複していると /diagnostics の項目を区別できない
  std::set<std::string> seen;
  for (size_t id = 0; id < config.names.size(); ++id) {
    if (config.names[id].empty()) {
      throw std::invalid_argument(
              "motor_status.names[" + std::to_string(id) + "] must not be empty");
    }
    if (!seen.insert(config.names[id]).second) {
      throw std::invalid_argument(
              "motor_status.names has duplicate name \"" + config.names[id] + "\"");
    }
  }
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

    link_status_enabled_ = this->declare_parameter("link_status.enabled", true);
    link_status_config_ = LinkStatusConfig{
      this->declare_parameter<std::string>("link_status.topic", "/hardware/link_status"),
      static_cast<int>(this->declare_parameter("link_status.timeout_ms", 500)),
    };
    validate_timeout(link_status_config_.timeout_ms, "link_status.timeout_ms");

    // names の正は yaml (モータ ID と名前の対応はハードウェア側の取り決め)。未指定なら motor/<id>
    motor_status_enabled_ = this->declare_parameter("motor_status.enabled", true);
    motor_status_config_ = MotorStatusConfig{
      this->declare_parameter<std::string>("motor_status.topic", "/hardware/motor_status"),
      static_cast<int>(this->declare_parameter("motor_status.timeout_ms", 500)),
      this->declare_parameter<std::vector<std::string>>(
        "motor_status.names", std::vector<std::string>{}),
      this->declare_parameter("motor_status.temp_warn_c", 60.0),
      this->declare_parameter("motor_status.temp_error_c", 70.0),
    };
    validate_motor_status_config(motor_status_config_);

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

    if (link_status_enabled_) {
      link_status_subscription_ = create_subscription<HardwareLinkStatus>(
        link_status_config_.topic, rclcpp::SensorDataQoS(),
        [this](const HardwareLinkStatus::SharedPtr msg) {
          link_status_last_received_ = get_clock()->now();
          link_status_ = *msg;
        });
    }

    if (motor_status_enabled_) {
      motor_status_subscription_ = create_subscription<MotorStatusArray>(
        motor_status_config_.topic, rclcpp::SensorDataQoS(),
        [this](const MotorStatusArray::SharedPtr msg) {
          motor_status_last_received_ = get_clock()->now();
          motor_status_ = *msg;
        });
    }

    //========================================
    // publishers
    //========================================
    // launch で system/health 名前空間に置き /system/health/diagnostics とする
    diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);

    //========================================
    // timer
    //========================================
    timer_ = create_wall_timer(
      std::chrono::milliseconds(publish_period_ms),
      std::bind(&HealthMonitor::publish_diagnostics, this));

    RCLCPP_INFO(
      get_logger(),
      "< health_monitor > topic checks: %zu, battery: %s, link_status: %s, motor_status: %s",
      topic_checks_.size(), battery_enabled_ ? battery_config_.topic.c_str() : "disabled",
      link_status_enabled_ ? link_status_config_.topic.c_str() : "disabled",
      motor_status_enabled_ ? motor_status_config_.topic.c_str() : "disabled");
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
    if (link_status_enabled_) {
      const auto statuses = evaluate_link_status(
        link_status_config_, age_ms(now, link_status_last_received_), link_status_);
      array.status.insert(array.status.end(), statuses.begin(), statuses.end());
    }
    if (motor_status_enabled_) {
      const auto statuses = evaluate_motor_status(
        motor_status_config_, age_ms(now, motor_status_last_received_), motor_status_);
      array.status.insert(array.status.end(), statuses.begin(), statuses.end());
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
  rclcpp::Subscription<HardwareLinkStatus>::SharedPtr link_status_subscription_;
  rclcpp::Subscription<MotorStatusArray>::SharedPtr motor_status_subscription_;

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
  bool link_status_enabled_ = true;
  LinkStatusConfig link_status_config_;
  std::optional<rclcpp::Time> link_status_last_received_;
  std::optional<HardwareLinkStatus> link_status_;
  bool motor_status_enabled_ = true;
  MotorStatusConfig motor_status_config_;
  std::optional<rclcpp::Time> motor_status_last_received_;
  std::optional<MotorStatusArray> motor_status_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HealthMonitor>());
  rclcpp::shutdown();
  return 0;
}
