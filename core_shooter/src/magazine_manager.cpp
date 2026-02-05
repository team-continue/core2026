#include <chrono>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "core_msgs/msg/can.hpp"
#include "core_msgs/msg/can_array.hpp"

#include <deque>
#include <cmath>

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
    declare_parameter("remaining_disks", 27);
    declare_parameter("disk_thickness", 1.0);
    declare_parameter("sensor_height", 100.0);
    declare_parameter("window_size", 3);

    remaining_disks_ = this->get_parameter("remaining_disks").as_int();
    disk_thickness_ = this->get_parameter("disk_thickness").as_double();
    sensor_height_ = this->get_parameter("sensor_height").as_double();
    window_size_ = this->get_parameter("window_size").as_int();

    RCLCPP_INFO(
      this->get_logger(),
      "remaining_disks: %d, disk_thickness: %f, sensor_height: %f",
      remaining_disks_, disk_thickness_, sensor_height_);

    //========================================
    // disk hold motor parameters
    //========================================
    declare_parameter("disk_hold_right_motor_id", 100);
    declare_parameter("disk_hold_left_motor_id", 101);

    declare_parameter("hold_left_angle", 30.0);
    declare_parameter("hold_right_angle", 30.0);
    declare_parameter("release_left_angle", -10.0);
    declare_parameter("release_right_angle", -10.0);

    disk_hold_right_motor_id_ = this->get_parameter("disk_hold_right_motor_id").as_int();
    disk_hold_left_motor_id_ = this->get_parameter("disk_hold_left_motor_id").as_int();

    hold_left_angle_ = this->get_parameter("hold_left_angle").as_double();
    hold_right_angle_ = this->get_parameter("hold_right_angle").as_double();
    release_left_angle_ = this->get_parameter("release_left_angle").as_double();
    release_right_angle_ = this->get_parameter("release_right_angle").as_double();

    //========================================
    // regrip parameters
    //========================================
    declare_parameter("regrip_enabled", true);
    declare_parameter("regrip_release_ms", 200);

    // 下段残2でregripは仕様固定（必要ならパラメータ化OK）
    regrip_enabled_ = this->get_parameter("regrip_enabled").as_bool();
    regrip_release_ms_ = this->get_parameter("regrip_release_ms").as_int();

    //========================================
    // disk hold parameters (redundant check)
    //========================================
    declare_parameter("hold_disable_height_margin_mm", 0.5);
    hold_disable_height_margin_mm_ =
      this->get_parameter("hold_disable_height_margin_mm").as_double();

    //========================================
    // subscribers
    //========================================
    shoot_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "shoot_status", 10,
      std::bind(&MagazineManager::shootStatusCallback, this, std::placeholders::_1));

    reloading_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "reloading", 10,
      std::bind(&MagazineManager::reloadingCallback, this, std::placeholders::_1));

    disk_distance_sensor_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "disk_distance_sensor", 10,
      std::bind(&MagazineManager::diskDistanceSensorCallback, this, std::placeholders::_1));

    sub_hold_state_ = this->create_subscription<std_msgs::msg::Bool>(
      "disk_hold_state", 10,
      std::bind(&MagazineManager::holdStateCallback, this, std::placeholders::_1));

    hazard_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "hazard_status", 10,
      std::bind(&MagazineManager::hazardStatusCallback, this, std::placeholders::_1));

    //========================================
    // publishers
    //========================================
    remaining_disk_pub_ = this->create_publisher<std_msgs::msg::Int8>("remaining_disk", 10);
    can_pub_ = this->create_publisher<core_msgs::msg::CANArray>("/can/tx", 10);

    //========================================
    // timer callback
    //========================================
    timer_ = this->create_wall_timer(10ms, std::bind(&MagazineManager::on_timer, this));

    //========================================
    // initialize
    //========================================
    remainingDisksPublish(remaining_disks_);
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
    if (msg->data) {
      // 押さえ中はセンサが歪むので、通常はカウントで減算
      remainingDiskEstimator(-1);
    }
  }

  void reloadingCallback(std_msgs::msg::Int8::SharedPtr msg)
  {
    if (msg->data > 0) {
      // 押さえの影響でセンサは信用できない想定 → ここでは同期しない
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
      estimated_stack_height_mm_ = sensor_height_ - distance_;

      if (estimated_stack_height_mm_ <= 0.0) {
        RCLCPP_INFO(this->get_logger(), "disk sensor height error");
        return;
      }

      int estimated = std::round(estimated_stack_height_mm_ / disk_thickness_);
      if (estimated < 0) {
        estimated = 0;
      }

      remaining_disks_ = estimated;

      // 冗長チェック用に保持（最後に「見えた」値）
      last_sensor_estimated_disks_ = estimated;
      last_sensor_height_mm_ = estimated_stack_height_mm_;

      remainingDisksPublish(remaining_disks_);
      return;
    }
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
    hold_on_ = msg->data;

    if (!hold_on_) {
      state_ = State::IDLE_RELEASED;
      regrip_done_ = false;
      latched_valid_ = false;
    } else {
      if (state_ == State::IDLE_RELEASED) {
        state_ = State::HOLDING;
      }
    }
  }

  // hazard：押さえ状態（ラッチ）をリセット
  void hazardStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool prev = hazard_active_;
    hazard_active_ = msg->data;

    if (hazard_active_ && !prev) {
      state_ = State::IDLE_RELEASED;
      regrip_done_ = false;
      latched_valid_ = false;
      RCLCPP_ERROR(this->get_logger(), "HAZARD ACTIVE -> force RELEASE + reset");
      return;
    }

    if (!hazard_active_ && prev) {
      // 復帰後も「押さえリセット」仕様 → 再ラッチされるまでrelease
      state_ = State::IDLE_RELEASED;
      regrip_done_ = false;
      latched_valid_ = false;
      RCLCPP_WARN(this->get_logger(), "HAZARD CLEARED -> reset hold latch");
    }
  }

  //========================================
  // disk hold loop
  //========================================
  void on_timer()
  {
    // ============================================================
    // PRIORITY 1: HAZARD STOP（最優先）→ 押さえ状態もリセット
    // ============================================================
    if (hazard_active_) {
      state_ = State::IDLE_RELEASED;
      regrip_done_ = false;
      latched_valid_ = false;
      publish_angles(release_left_angle_, release_right_angle_);
      return;
    }

    // ============================================================
    // PRIORITY 2: 全体が10枚以下なら掴めない（冗長判定含む）
    //   - センサは「リグリップ時のみ」信用できるので last_* を使う
    // ============================================================
    const bool cannot_hold_by_count = (remaining_disks_ <= 10);
    const bool cannot_hold_by_last_sensor =
      (last_sensor_estimated_disks_ >= 0 && last_sensor_estimated_disks_ <= 10);
    const bool cannot_hold_by_last_height =
      (last_sensor_height_mm_ >= 0.0 &&
      last_sensor_height_mm_ <= (disk_thickness_ * 10.0 + hold_disable_height_margin_mm_));

    if (cannot_hold_by_count || cannot_hold_by_last_sensor || cannot_hold_by_last_height) {
      state_ = State::IDLE_RELEASED;
      regrip_done_ = false;
      latched_valid_ = false;
      publish_angles(release_left_angle_, release_right_angle_);
      return;
    }

    // ============================================================
    // PRIORITY 3: hold OFFなら開く（操作で解除）
    // ============================================================
    if (!hold_on_) {
      state_ = State::IDLE_RELEASED;
      regrip_done_ = false;
      latched_valid_ = false;
      publish_angles(release_left_angle_, release_right_angle_);
      return;
    }

    // ============================================================
    // NORMAL: hold ON かつ 11枚以上 → 「下から10枚目を固定」
    // ============================================================

    // (A) まだ固定基準がないなら、いまの全体枚数から「上側枚数」をラッチ
    if (!latched_valid_) {
      latched_above_disks_ = remaining_disks_ - 10;
      if (latched_above_disks_ < 0) {
        latched_above_disks_ = 0;
      }
      latched_valid_ = true;
      regrip_done_ = false;
      state_ = State::HOLDING;
    }

    // (B) 固定ディスクより下（=下段側）の残数を計算
    int below_remaining = remaining_disks_ - latched_above_disks_ - 1;
    if (below_remaining < 0) {
      below_remaining = 0;
    }

    // (C) 下段残数が2になったら regrip（1回だけ）
    if (regrip_enabled_ && !regrip_done_ && below_remaining == 2) {
      if (state_ != State::REGRIP_RELEASING) {
        state_ = State::REGRIP_RELEASING;

        // ★移動平均バッファをクリア（押さえ中の値を捨てる）
        buffer_.clear();

        regrip_release_until_ =
          this->now() + rclcpp::Duration(0, static_cast<int64_t>(regrip_release_ms_) * 1000 * 1000);

        RCLCPP_WARN(
          this->get_logger(),
          "Regrip triggered: total=%d, latched_above=%d, below_remaining=%d -> release %d ms",
          remaining_disks_, latched_above_disks_, below_remaining, regrip_release_ms_);
      }
    }

    // (D) 状態に応じて角度指令
    switch (state_) {
      case State::IDLE_RELEASED:
        publish_angles(release_left_angle_, release_right_angle_);
        break;

      case State::HOLDING:
        publish_angles(hold_left_angle_, hold_right_angle_);
        break;

      case State::REGRIP_RELEASING:
        // 開放中
        publish_angles(release_left_angle_, release_right_angle_);

        // ★REGRIP中はセンサが見える想定
        // 移動平均の窓が揃ってから同期（安定化）
        if ((int)buffer_.size() >= window_size_) {
          remainingDiskEstimator(0);
        }

        if (this->now() >= regrip_release_until_) {
          state_ = State::HOLDING;
          regrip_done_ = true;

          // regrip後に「新しい10枚目」を固定し直す：ラッチ更新
          latched_above_disks_ = remaining_disks_ - 10;
          if (latched_above_disks_ < 0) {
            latched_above_disks_ = 0;
          }
          latched_valid_ = true;

          RCLCPP_WARN(this->get_logger(), "Regrip done -> HOLDING");
        }
        break;
    }
  }

  //========================================
  // publish
  //========================================
  void publish_angles(double left_angle, double right_angle)
  {
    motorPublish(disk_hold_left_motor_id_, (float)left_angle);
    motorPublish(disk_hold_right_motor_id_, (float)right_angle);
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
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr reloading_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr disk_distance_sensor_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_hold_state_;
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
  double disk_thickness_ = 1.0;
  double sensor_height_ = 100.0;
  int window_size_ = 3;

  // disk hold motor params
  int disk_hold_left_motor_id_ = 101;
  int disk_hold_right_motor_id_ = 100;

  double hold_left_angle_ = 30.0;
  double hold_right_angle_ = 30.0;
  double release_left_angle_ = -10.0;
  double release_right_angle_ = -10.0;

  // disk hold state
  bool hold_on_ = false;
  bool hazard_active_ = false;
  State state_ = State::IDLE_RELEASED;

  // regrip
  bool regrip_enabled_ = true;
  int regrip_release_ms_ = 200;
  bool regrip_done_ = false;
  rclcpp::Time regrip_release_until_{0, 0, RCL_ROS_TIME};

  // redundant sensor check (valid ONLY when synced during regrip)
  double hold_disable_height_margin_mm_ = 0.5;
  double estimated_stack_height_mm_ = 0.0;

  int last_sensor_estimated_disks_ = -1;  // -1: 未取得
  double last_sensor_height_mm_ = -1.0;

  // latch for "10th disk hold" logic
  bool latched_valid_ = false;
  int latched_above_disks_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MagazineManager>());
  rclcpp::shutdown();
  return 0;
}
