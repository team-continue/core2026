#include <chrono>
#include <cstdint>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "core_msgs/msg/can.hpp"
#include "core_msgs/msg/can_array.hpp"

#include <algorithm>
#include <deque>
#include <cmath>
#include <vector>

#include "core_shooter/can_command.hpp"
#include "core_shooter/parameter_utils.hpp"

using namespace std::chrono_literals;

namespace
{
/// この枚数以下ではホールドしない（弾が薄くなり押さえが効かないため）。
constexpr int kMinHoldDisks = 10;
/// std_msgs/Int8 で表現できる最大値。
constexpr int kInt8Max = 127;
}  // namespace

class MagazineManager : public rclcpp::Node
{
public:
  MagazineManager()
  : Node("magazine_manager")
  {
    //========================================
    // parameters
    //========================================
    max_disks_ = core_shooter::declareAndGet<int>(*this, "max_disks", 27);
    disk_thickness_ = core_shooter::declareAndGet<double>(*this, "disk_thickness", 1.0);
    sensor_height_ = core_shooter::declareAndGet<double>(*this, "sensor_height", 100.0);
    window_size_ = core_shooter::declareAndGet<int>(*this, "window_size", 3);

    if (max_disks_ < 0 || max_disks_ > kInt8Max) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Invalid max_disks=%d (must be in [0, 127])",
        max_disks_);
      throw std::runtime_error("invalid max_disks");
    }
    if (window_size_ <= 0) {
      RCLCPP_FATAL(
        this->get_logger(), "Invalid window_size=%d (must be > 0)", window_size_);
      throw std::runtime_error("invalid window_size");
    }
    if (disk_thickness_ <= 0.0) {
      RCLCPP_FATAL(
        this->get_logger(), "Invalid disk_thickness=%f (must be > 0)", disk_thickness_);
      throw std::runtime_error("invalid disk_thickness");
    }
    if (sensor_height_ <= 0.0) {
      RCLCPP_FATAL(
        this->get_logger(), "Invalid sensor_height=%f (must be > 0)", sensor_height_);
      throw std::runtime_error("invalid sensor_height");
    }

    remaining_disks_ = max_disks_;

    RCLCPP_INFO(
      this->get_logger(),
      "max_disks: %d, disk_thickness: %f, sensor_height: %f",
      max_disks_, disk_thickness_, sensor_height_);

    //========================================
    // disk hold motor parameters
    //========================================
    disk_hold_right_motor_id_ =
      core_shooter::declareAndGet<int>(*this, "disk_hold_right_motor_id", 100);
    disk_hold_left_motor_id_ =
      core_shooter::declareAndGet<int>(*this, "disk_hold_left_motor_id", 101);
    disk_hold_motor_left_angle_ = core_shooter::declareAndGet<std::vector<double>>(
      *this, "disk_hold_motor_left_angle", std::vector<double>{0.0, 1.0});
    disk_hold_motor_right_angle_ = core_shooter::declareAndGet<std::vector<double>>(
      *this, "disk_hold_motor_right_angle", std::vector<double>{0.0, 1.0});
    if (disk_hold_motor_left_angle_.size() != 2 || disk_hold_motor_right_angle_.size() != 2) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Invalid disk_hold_motor_*_angle size: left=%zu right=%zu (expected 2:[close,open])",
        disk_hold_motor_left_angle_.size(), disk_hold_motor_right_angle_.size());
      throw std::runtime_error("invalid disk_hold_motor_*_angle size");
    }

    //========================================
    // regrip parameters
    //========================================
    regrip_enabled_ = core_shooter::declareAndGet<bool>(*this, "regrip_enabled", true);
    regrip_release_ms_ = core_shooter::declareAndGet<int>(*this, "regrip_release_ms", 200);
    regrip_trigger_shots_ = core_shooter::declareAndGet<int>(*this, "regrip_trigger_shots", 6);
    if (regrip_release_ms_ < 0) {
      RCLCPP_FATAL(
        this->get_logger(), "Invalid regrip_release_ms=%d (must be >= 0)", regrip_release_ms_);
      throw std::runtime_error("invalid regrip_release_ms");
    }
    if (regrip_trigger_shots_ <= 0) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Invalid regrip_trigger_shots=%d (must be > 0)",
        regrip_trigger_shots_);
      throw std::runtime_error("invalid regrip_trigger_shots");
    }

    //========================================
    // disk hold parameters (redundant check)
    //========================================
    hold_disable_height_margin_mm_ =
      core_shooter::declareAndGet<double>(*this, "hold_disable_height_margin_mm", 0.5);
    if (hold_disable_height_margin_mm_ < 0.0) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Invalid hold_disable_height_margin_mm=%f (must be >= 0)",
        hold_disable_height_margin_mm_);
      throw std::runtime_error("invalid hold_disable_height_margin_mm");
    }

    //========================================
    // subscribers
    //========================================
    shoot_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "shoot_status", 10,
      std::bind(&MagazineManager::shootStatusCallback, this, std::placeholders::_1));

    reloading_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "reloading", 10,
      std::bind(&MagazineManager::reloadingCallback, this, std::placeholders::_1));

    reloading_increment_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "reloading_increment", 10,
      std::bind(&MagazineManager::reloadingIncrementCallback, this, std::placeholders::_1));

    disk_distance_sensor_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "disk_distance_sensor", 10,
      std::bind(&MagazineManager::diskDistanceSensorCallback, this, std::placeholders::_1));

    hold_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "disk_hold_state", 10,
      std::bind(&MagazineManager::holdStateCallback, this, std::placeholders::_1));

    hazard_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "hazard_status", 10,
      std::bind(&MagazineManager::hazardStatusCallback, this, std::placeholders::_1));

    //========================================
    // publishers
    //========================================
    remaining_disk_pub_ = this->create_publisher<std_msgs::msg::Int8>(
      "remaining_disk", rclcpp::QoS(10).transient_local());
    regrip_active_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "regrip_active", rclcpp::QoS(1).transient_local());
    can_pub_ = this->create_publisher<core_msgs::msg::CANArray>("/can/tx", 10);

    //========================================
    // initialize (publish remaining disks first)
    //========================================
    remainingDisksPublish(remaining_disks_);
    publishRegripActive(false);

    //========================================
    // timer callback
    //========================================
    timer_ = this->create_wall_timer(10ms, std::bind(&MagazineManager::timerCallback, this));
  }

private:
  //========================================
  // disk hold status
  //========================================
  enum class State : uint8_t
  {
    IDLE_RELEASED = 0,
    HOLDING,
    REGRIP_RELEASING
  };

  //========================================
  // magazine callbacks
  //========================================
  void shootStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    // 初回受信は現在状態の同期として扱い、誤って減算しない
    if (!shoot_status_initialized_) {
      prev_shoot_status_ = msg->data;
      shoot_status_initialized_ = true;
      return;
    }

    const bool rising = msg->data && !prev_shoot_status_;
    prev_shoot_status_ = msg->data;

    if (rising) {
      // 押さえ中はセンサが歪むので、通常はカウントで減算
      decrementRemainingDisk();

      // 保持可能枚数を超えている間だけ、hold中の射撃回数を数える
      if (
        state_ == State::HOLDING && hold_on_ && !hazard_active_ &&
        remaining_disks_ > kMinHoldDisks)
      {
        ++hold_shots_since_grip_;
        RCLCPP_INFO(this->get_logger(), "hold_shots_since_grip: %d", hold_shots_since_grip_);
        maybeStartRegrip();
      }
    }
  }

  void reloadingCallback(std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      remaining_disks_ = max_disks_;
      invalidateLastSensorEstimate();
      buffer_.clear();
      regrip_valid_sensor_sample_received_ = false;
      regrip_sensor_sync_completed_ = false;
      remainingDisksPublish(remaining_disks_);
    }
  }

  void reloadingIncrementCallback(std_msgs::msg::Int8::SharedPtr msg)
  {
    if (msg->data > 0) {
      // 押さえの影響でセンサは信用できない想定 → ここでは同期しない
      remaining_disks_ = clampRemainingDisks(
        remaining_disks_ + msg->data, "reloading_increment");
      invalidateLastSensorEstimate();
      buffer_.clear();
      regrip_valid_sensor_sample_received_ = false;
      regrip_sensor_sync_completed_ = false;
      remainingDisksPublish(remaining_disks_);
    }
  }

  void diskDistanceSensorCallback(std_msgs::msg::Int32::SharedPtr msg)
  {
    // ★重要：移動平均は REGRIP（開放）中だけ更新する
    // 押さえ中の値で buffer_ を汚すと、開いた直後に正しい推定ができない
    if (state_ != State::REGRIP_RELEASING) {
      return;
    }

    double val = static_cast<double>(msg->data);
    if (!isDistanceMeasurementValid(val)) {
      return;
    }

    regrip_valid_sensor_sample_received_ = true;

    // 移動平均フィルタ
    buffer_.push_back(val);
    if (buffer_.size() > static_cast<size_t>(window_size_)) {
      buffer_.pop_front();
    }

    if (buffer_.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "disk distance buffer is empty (window_size=%d)", window_size_);
      return;
    }

    double sum = 0.0;
    for (double v : buffer_) {sum += v;}
    distance_ = sum / buffer_.size();
  }

  //========================================
  // remaining disk estimator
  //========================================
  /// 射撃 1 発ぶんとして残弾を 1 枚減らす。
  void decrementRemainingDisk()
  {
    if (remaining_disks_ > 0) {
      remaining_disks_--;
    }
    remainingDisksPublish(remaining_disks_);
  }

  /// 距離センサの移動平均から残弾を推定して同期する。
  /// 押さえ中はセンサ値が歪むため、regrip で開いている間だけ呼ぶこと。
  bool syncRemainingDiskFromSensor()
  {
    if (disk_thickness_ <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "disk_thickness must be > 0 for estimation");
      return false;
    }

    const double estimated_stack_height_mm = sensor_height_ - distance_;
    if (estimated_stack_height_mm <= 0.0) {
      RCLCPP_INFO(this->get_logger(), "disk sensor height error");
      return false;
    }

    const int estimated =
      static_cast<int>(std::round(estimated_stack_height_mm / disk_thickness_));
    remaining_disks_ = clampRemainingDisks(estimated, "sensor estimate");

    // 冗長チェック用に保持（最後に「見えた」値）
    last_sensor_estimated_disks_ = remaining_disks_;
    last_sensor_height_mm_ = estimated_stack_height_mm;

    remainingDisksPublish(remaining_disks_);
    return true;
  }

  bool isDistanceMeasurementValid(double distance_mm)
  {
    const double max_valid_distance_mm = sensor_height_ + disk_thickness_;
    if (distance_mm < 0.0 || distance_mm > max_valid_distance_mm) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignore out-of-range disk distance=%f mm (valid range: 0..%f)",
        distance_mm, max_valid_distance_mm);
      return false;
    }

    return true;
  }

  void invalidateLastSensorEstimate()
  {
    last_sensor_estimated_disks_ = -1;
    last_sensor_height_mm_ = -1.0;
  }

  int clampRemainingDisks(int value, const char * source)
  {
    if (value < 0) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s produced %d remaining disks. Clamp to 0.",
        source, value);
      return 0;
    }

    if (value > max_disks_) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s produced %d remaining disks. Clamp to max_disks=%d.",
        source, value, max_disks_);
      return max_disks_;
    }

    return value;
  }

  void remainingDisksPublish(int data)
  {
    std_msgs::msg::Int8 message;
    message.data = static_cast<int8_t>(std::clamp(data, 0, kInt8Max));
    remaining_disk_pub_->publish(message);
  }

  void publishRegripActive(bool active)
  {
    std_msgs::msg::Bool message;
    message.data = active;
    regrip_active_pub_->publish(message);
  }

  //========================================
  // disk hold callbacks
  //========================================
  void holdStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    // ボタン指示（hazard / <=10枚 は timerCallback で優先上書き）
    hold_request_on_ = msg->data;

    // ボタン押下時は即release側へ寄せる（最終決定は timerCallback）
    if (hold_request_on_) {
      hold_on_ = false;
      hold_shots_since_grip_ = 0;
      if (state_ != State::REGRIP_RELEASING) {
        state_ = State::IDLE_RELEASED;
      }
    }
  }

  // hazard：hold出力を強制releaseし、通常復帰後は timerCallback の優先順に戻す
  void hazardStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool prev = hazard_active_;
    hazard_active_ = msg->data;

    if (hazard_active_ && !prev) {
      hold_on_ = false;
      state_ = State::IDLE_RELEASED;
      hold_shots_since_grip_ = 0;
      publishHoldCommand(false);
      RCLCPP_ERROR(this->get_logger(), "HAZARD ACTIVE -> force RELEASE + reset");
      return;
    }

    if (!hazard_active_ && prev) {
      // 復帰後は通常ロジックに復帰（hold_on_ 指示に従う）
      hold_on_ = false;
      state_ = State::IDLE_RELEASED;
      hold_shots_since_grip_ = 0;
      timerCallback();  // hazard解除を即時反映（条件を満たせばgrip）
      RCLCPP_WARN(this->get_logger(), "HAZARD CLEARED -> resume normal hold logic");
    }
  }

  //========================================
  // disk hold loop
  //========================================
  void maybeStartRegrip()
  {
    if (
      !regrip_enabled_ || state_ != State::HOLDING ||
      hold_shots_since_grip_ < regrip_trigger_shots_ || remaining_disks_ <= kMinHoldDisks)
    {
      return;
    }

    state_ = State::REGRIP_RELEASING;
    hold_shots_since_grip_ = 0;
    regrip_valid_sensor_sample_received_ = false;
    regrip_sensor_sync_completed_ = false;

    // 押さえ中の値を捨てて、release後の距離センサだけで同期する。
    buffer_.clear();

    regrip_release_until_ =
      this->now() + rclcpp::Duration(0, static_cast<int64_t>(regrip_release_ms_) * 1000 * 1000);

    publishHoldCommand(false);
    publishRegripActive(true);

    RCLCPP_WARN(
      this->get_logger(),
      "Regrip triggered after %d shots: total=%d -> release %d ms",
      regrip_trigger_shots_, remaining_disks_, regrip_release_ms_);
  }

  /// hold を強制解除し、ホールド状態をリセットする。
  void forceRelease()
  {
    hold_on_ = false;
    state_ = State::IDLE_RELEASED;
    hold_shots_since_grip_ = 0;
    publishHoldCommand(false);
    publishRegripActive(false);
  }

  void timerCallback()
  {
    // ============================================================
    // PRIORITY 1: HAZARDなら必ずrelease(false)
    // ============================================================
    if (hazard_active_) {
      forceRelease();
      return;
    }

    // ============================================================
    // PRIORITY 2: 保持可能枚数以下なら必ずrelease(false)（冗長判定含む）
    // ============================================================
    const bool cannot_hold_by_count = (remaining_disks_ <= kMinHoldDisks);
    const bool cannot_hold_by_last_sensor =
      (last_sensor_estimated_disks_ >= 0 && last_sensor_estimated_disks_ <= kMinHoldDisks);
    const bool cannot_hold_by_last_height =
      (last_sensor_height_mm_ >= 0.0 &&
      last_sensor_height_mm_ <=
      (disk_thickness_ * kMinHoldDisks + hold_disable_height_margin_mm_));

    if (cannot_hold_by_count || cannot_hold_by_last_sensor || cannot_hold_by_last_height) {
      forceRelease();
      return;
    }

    // ============================================================
    // PRIORITY 3: ボタン押下時はrelease
    // ============================================================
    if (hold_request_on_) {
      forceRelease();
      return;
    }

    // ============================================================
    // NORMAL: ボタン未押下 かつ 保持可能枚数超え
    // ============================================================
    if (state_ == State::IDLE_RELEASED) {
      state_ = State::HOLDING;
      hold_shots_since_grip_ = 0;
    }

    // (D) 状態に応じて0/1指令（0: release, 1: grip）
    switch (state_) {
      case State::IDLE_RELEASED:
        // 直前で HOLDING へ遷移させているため通常は到達しない（列挙の網羅用）。
        hold_on_ = false;
        publishHoldCommand(false);
        break;

      case State::HOLDING:
        hold_on_ = true;
        publishHoldCommand(true);
        break;

      case State::REGRIP_RELEASING:
        // 開放中
        hold_on_ = false;
        publishHoldCommand(false);

        // ★REGRIP中はセンサが見える想定
        // 移動平均の窓が揃ってから同期（安定化）
        if (!regrip_sensor_sync_completed_ &&
          static_cast<int>(buffer_.size()) >= window_size_)
        {
          regrip_sensor_sync_completed_ = syncRemainingDiskFromSensor();
        }

        if (this->now() >= regrip_release_until_) {
          if (!regrip_sensor_sync_completed_) {
            if (regrip_valid_sensor_sample_received_) {
              RCLCPP_WARN(
                this->get_logger(),
                "Regrip finished without successful sensor sync. Keep shot-count estimate.");
            } else {
              RCLCPP_WARN(
                this->get_logger(),
                "Regrip finished without distance sensor samples. Keep shot-count estimate.");
            }
          }

          state_ = State::HOLDING;
          hold_shots_since_grip_ = 0;

          RCLCPP_WARN(this->get_logger(), "Regrip done -> HOLDING");
        }
        break;
    }

    publishRegripActive(state_ == State::REGRIP_RELEASING);
  }

  //========================================
  // publish
  //========================================
  void publishHoldCommand(bool hold)
  {
    constexpr size_t CLOSE_INDEX = 0;
    constexpr size_t OPEN_INDEX = 1;
    const size_t index = hold ? CLOSE_INDEX : OPEN_INDEX;
    const float left_angle = static_cast<float>(disk_hold_motor_left_angle_[index]);
    const float right_angle = static_cast<float>(disk_hold_motor_right_angle_[index]);

    motorPublish(disk_hold_left_motor_id_, left_angle);
    motorPublish(disk_hold_right_motor_id_, right_angle);
  }

  void motorPublish(int id, float data)
  {
    core_shooter::publishMotorCommand(can_pub_, id, data);
  }

private:
  //========================================
  // Subscription valids
  //========================================
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shoot_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reloading_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr reloading_increment_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr disk_distance_sensor_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hold_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hazard_status_sub_;

  //========================================
  // publisher valids
  //========================================
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr remaining_disk_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr regrip_active_pub_;
  rclcpp::Publisher<core_msgs::msg::CANArray>::SharedPtr can_pub_;

  //========================================
  // timer valids
  //========================================
  rclcpp::TimerBase::SharedPtr timer_;

  //========================================
  // sensor/filter valids
  //========================================
  double distance_ = 0.0;
  std::deque<double> buffer_;

  //========================================
  // parameter valids
  //========================================
  // magazine
  int max_disks_ = 27;
  int remaining_disks_ = 27;
  double disk_thickness_ = 1.0;
  double sensor_height_ = 100.0;
  int window_size_ = 3;

  // disk hold motor params
  int disk_hold_left_motor_id_ = 101;
  int disk_hold_right_motor_id_ = 100;
  std::vector<double> disk_hold_motor_left_angle_{0.0, 1.0};   // [close, open]
  std::vector<double> disk_hold_motor_right_angle_{0.0, 1.0};  // [close, open]

  // disk hold state
  bool hold_request_on_ = false;  // operator request from disk_hold_state topic
  bool hold_on_ = false;  // effective hold state after hazard/remaining/regrip conditions
  bool prev_shoot_status_ = false;
  bool shoot_status_initialized_ = false;
  bool hazard_active_ = true;
  State state_ = State::IDLE_RELEASED;
  int hold_shots_since_grip_ = 0;

  // regrip
  bool regrip_enabled_ = true;
  int regrip_release_ms_ = 200;
  int regrip_trigger_shots_ = 6;
  bool regrip_valid_sensor_sample_received_ = false;
  bool regrip_sensor_sync_completed_ = false;
  rclcpp::Time regrip_release_until_{0, 0, RCL_ROS_TIME};

  // redundant sensor check (valid ONLY when synced during regrip)
  double hold_disable_height_margin_mm_ = 0.5;

  int last_sensor_estimated_disks_ = -1;  // -1: 未取得
  double last_sensor_height_mm_ = -1.0;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MagazineManager>());
  rclcpp::shutdown();
  return 0;
}
