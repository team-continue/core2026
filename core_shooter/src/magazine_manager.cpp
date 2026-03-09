#include <chrono>
#include <cstdint>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "core_msgs/msg/can.hpp"
#include "core_msgs/msg/can_array.hpp"

#include <deque>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

class MagazineManager : public rclcpp::Node
{
public:
  MagazineManager()
  : Node("magazine_manager")
  {
    //========================================
    // parameters
    //========================================
    this->declare_parameter<int>("remaining_disks", 27);
    this->declare_parameter<double>("disk_thickness", 1.0);
    this->declare_parameter<double>("sensor_height", 100.0);
    this->declare_parameter<int>("window_size", 3);

    this->get_parameter("remaining_disks", remaining_disks_);
    this->get_parameter("disk_thickness", disk_thickness_);
    this->get_parameter("sensor_height", sensor_height_);
    this->get_parameter("window_size", window_size_);

    if (remaining_disks_ < 0 || remaining_disks_ > 127) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Invalid remaining_disks=%d (must be in [0, 127])",
        remaining_disks_);
      throw std::runtime_error("invalid remaining_disks");
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

    remaining_disks_ = remaining_disks_;

    RCLCPP_INFO(
      this->get_logger(),
      "remaining_disks: %d, disk_thickness: %f, sensor_height: %f",
      remaining_disks_, disk_thickness_, sensor_height_);

    //========================================
    // disk hold motor parameters
    //========================================
    this->declare_parameter<int>("disk_hold_right_motor_id", 100);
    this->declare_parameter<int>("disk_hold_left_motor_id", 101);
    this->declare_parameter<std::vector<double>>(
      "disk_hold_motor_left_angle", std::vector<double>{0.0, 1.0});
    this->declare_parameter<std::vector<double>>(
      "disk_hold_motor_right_angle", std::vector<double>{0.0, 1.0});

    this->get_parameter("disk_hold_right_motor_id", disk_hold_right_motor_id_);
    this->get_parameter("disk_hold_left_motor_id", disk_hold_left_motor_id_);
    this->get_parameter("disk_hold_motor_left_angle", disk_hold_motor_left_angle_);
    this->get_parameter("disk_hold_motor_right_angle", disk_hold_motor_right_angle_);
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
    this->declare_parameter<bool>("regrip_enabled", true);
    this->declare_parameter<int>("regrip_release_ms", 200);

    this->get_parameter("regrip_enabled", regrip_enabled_);
    this->get_parameter("regrip_release_ms", regrip_release_ms_);
    if (regrip_release_ms_ < 0) {
      RCLCPP_FATAL(
        this->get_logger(), "Invalid regrip_release_ms=%d (must be >= 0)", regrip_release_ms_);
      throw std::runtime_error("invalid regrip_release_ms");
    }

    //========================================
    // disk hold parameters (redundant check)
    //========================================
    this->declare_parameter<double>("hold_disable_height_margin_mm", 0.5);
    this->get_parameter("hold_disable_height_margin_mm", hold_disable_height_margin_mm_);

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
    can_pub_ = this->create_publisher<core_msgs::msg::CANArray>("/can/tx", 10);

    //========================================
    // initialize (publish remaining disks first)
    //========================================
    remainingDisksPublish(remaining_disks_);

    //========================================
    // timer callback
    //========================================
    timer_ = this->create_wall_timer(10ms, std::bind(&MagazineManager::on_timer, this));
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
      remainingDiskEstimator(-1);

      // hold=true中の射撃回数を数え、10回でregripする
      if (state_ == State::HOLDING && hold_on_ && !hazard_active_ && remaining_disks_ > 10) {
        ++hold_shots_since_grip_;
        RCLCPP_INFO(this->get_logger(), "hold_shots_since_grip: %d", hold_shots_since_grip_);
      }
    }
  }

  void reloadingCallback(std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      remaining_disks_ = remaining_disks_;
      remainingDisksPublish(remaining_disks_);
    }
  }

  void reloadingIncrementCallback(std_msgs::msg::Int8::SharedPtr msg)
  {
    if (msg->data > 0) {
      // 押さえの影響でセンサは信用できない想定 → ここでは同期しない
      remaining_disks_ = clampRemainingDisks(
        remaining_disks_ + msg->data, "reloading_increment");
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

    // 移動平均フィルタ
    buffer_.push_back(val);
    if (buffer_.size() > (size_t)window_size_) {
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
  //   data = -1 : decrement by shot
  //   data =  0 : sync from sensor (ONLY when regrip/open)
  //========================================
  void remainingDiskEstimator(int data)
  {
    // 1枚減らす
    if (data == -1) {
      if (remaining_disks_ > 0) {
        remaining_disks_--;
      }
      remainingDisksPublish(remaining_disks_);
      return;
    }

    // センサ同期（リグリップで開いている時だけ呼ぶ）
    if (data == 0) {
      if (disk_thickness_ <= 0.0) {
        RCLCPP_ERROR(this->get_logger(), "disk_thickness must be > 0 for estimation");
        return;
      }
      estimated_stack_height_mm_ = sensor_height_ - distance_;

      if (estimated_stack_height_mm_ <= 0.0) {
        RCLCPP_INFO(this->get_logger(), "disk sensor height error");
        return;
      }

      int estimated = std::round(estimated_stack_height_mm_ / disk_thickness_);
      remaining_disks_ = clampRemainingDisks(estimated, "sensor estimate");

      // 冗長チェック用に保持（最後に「見えた」値）
      last_sensor_estimated_disks_ = remaining_disks_;
      last_sensor_height_mm_ = estimated_stack_height_mm_;

      remainingDisksPublish(remaining_disks_);
      return;
    }
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

    if (value > remaining_disks_) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s produced %d remaining disks. Clamp to remaining_disks=%d.",
        source, value, remaining_disks_);
      return remaining_disks_;
    }

    return value;
  }

  void remainingDisksPublish(int data)
  {
    std_msgs::msg::Int8 message;
    if (data < 0) {
      message.data = 0;
    } else if (data > 127) {
      message.data = 127;
    } else {
      message.data = (int8_t)data;
    }
    remaining_disk_pub_->publish(message);
  }

  //========================================
  // disk hold callbacks
  //========================================
  void holdStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    // ボタン指示（hazard / <=10枚 は on_timer で優先上書き）
    hold_request_on_ = msg->data;

    // ボタン押下時は即release側へ寄せる（最終決定は on_timer）
    if (hold_request_on_) {
      hold_on_ = false;
      hold_shots_since_grip_ = 0;
      if (state_ != State::REGRIP_RELEASING) {
        state_ = State::IDLE_RELEASED;
      }
    }
  }

  // hazard：hold出力を強制releaseし、通常復帰後は on_timer の優先順に戻す
  void hazardStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool prev = hazard_active_;
    hazard_active_ = msg->data;

    if (hazard_active_ && !prev) {
      hold_on_ = false;
      state_ = State::IDLE_RELEASED;
      hold_shots_since_grip_ = 0;
      publish_hold_command(false);
      RCLCPP_ERROR(this->get_logger(), "HAZARD ACTIVE -> force RELEASE + reset");
      return;
    }

    if (!hazard_active_ && prev) {
      // 復帰後は通常ロジックに復帰（hold_on_ 指示に従う）
      hold_on_ = false;
      state_ = State::IDLE_RELEASED;
      hold_shots_since_grip_ = 0;
      on_timer();  // hazard解除を即時反映（条件を満たせばgrip）
      RCLCPP_WARN(this->get_logger(), "HAZARD CLEARED -> resume normal hold logic");
    }
  }

  //========================================
  // disk hold loop
  //========================================
  void on_timer()
  {
    // ============================================================
    // PRIORITY 1: HAZARDなら必ずrelease(false)
    // ============================================================
    if (hazard_active_) {
      hold_on_ = false;
      state_ = State::IDLE_RELEASED;
      hold_shots_since_grip_ = 0;
      publish_hold_command(false);
      return;
    }

    // ============================================================
    // PRIORITY 2: 10枚以下なら必ずrelease(false)（冗長判定含む）
    // ============================================================
    if (remaining_disks_ <= 10) {
      hold_on_ = false;
      state_ = State::IDLE_RELEASED;
      hold_shots_since_grip_ = 0;
      publish_hold_command(false);
      return;
    }

    // ============================================================
    // PRIORITY 3: ボタン押下時はrelease
    // ============================================================
    if (hold_request_on_) {
      hold_on_ = false;
      state_ = State::IDLE_RELEASED;
      hold_shots_since_grip_ = 0;
      publish_hold_command(false);
      return;
    }

    // ============================================================
    // NORMAL: ボタン未押下 かつ 11枚以上
    // ============================================================
    if (state_ == State::IDLE_RELEASED) {
      state_ = State::HOLDING;
      hold_shots_since_grip_ = 0;
    }

    // hold=trueになってから10回撃ったら false->true (regrip)
    if (regrip_enabled_ && state_ == State::HOLDING && hold_shots_since_grip_ >= 10) {
      state_ = State::REGRIP_RELEASING;
      hold_shots_since_grip_ = 0;

      // ★移動平均バッファをクリア（押さえ中の値を捨てる）
      buffer_.clear();

      regrip_release_until_ =
        this->now() + rclcpp::Duration(0, static_cast<int64_t>(regrip_release_ms_) * 1000 * 1000);

      RCLCPP_WARN(
        this->get_logger(),
        "Regrip triggered after 10 shots: total=%d -> release %d ms",
        remaining_disks_, regrip_release_ms_);
    }

    // (D) 状態に応じて0/1指令（0: release, 1: grip）
    switch (state_) {
      case State::IDLE_RELEASED:
        hold_on_ = false;
        publish_hold_command(false);
        break;

      case State::HOLDING:
        hold_on_ = true;
        publish_hold_command(true);
        break;

      case State::REGRIP_RELEASING:
        // 開放中
        hold_on_ = false;
        publish_hold_command(false);

        // ★REGRIP中はセンサが見える想定
        // 移動平均の窓が揃ってから同期（安定化）
        if ((int)buffer_.size() >= window_size_) {
          remainingDiskEstimator(0);
        }

        if (this->now() >= regrip_release_until_) {
          state_ = State::HOLDING;
          hold_shots_since_grip_ = 0;

          RCLCPP_WARN(this->get_logger(), "Regrip done -> HOLDING");
        }
        break;
    }
  }

  //========================================
  // publish
  //========================================
  void publish_hold_command(bool hold)
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
    auto can_array = core_msgs::msg::CANArray();
    auto can = core_msgs::msg::CAN();
    can.id = id;
    can.data.push_back(data);
    can_array.array.push_back(can);
    can_pub_->publish(can_array);
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
  int remaining_disks_ = 27;
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
  rclcpp::Time regrip_release_until_{0, 0, RCL_ROS_TIME};

  // redundant sensor check (valid ONLY when synced during regrip)
  double hold_disable_height_margin_mm_ = 0.5;
  double estimated_stack_height_mm_ = 0.0;

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
