#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <math.h>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "core_msgs/msg/can.hpp"
#include "core_msgs/msg/can_array.hpp"


class ShooterController : public rclcpp::Node
{
public:
  ShooterController()
  : Node("shooter_controller")
  {
    //========================================
    // shoot id parameters
    //========================================
    declare_parameter("shoot_motor_id", 100);
    declare_parameter("loading_motor_id", 100);

    shoot_motor_id_ = this->get_parameter("shoot_motor_id").as_int();
    loading_motor_id_ = this->get_parameter("loading_motor_id").as_int();

    RCLCPP_INFO(
      this->get_logger(), "shoot_motor_id: %d, loading_motor_id: %d", shoot_motor_id_,
      loading_motor_id_);

    //========================================
    // shoot parameters
    //========================================
    declare_parameter("burst_count", 3);
    declare_parameter("shoot_interval_ms", 500);
    declare_parameter("burst_interval_ms", 500);
    declare_parameter("fullauto_interval_ms", 500);

    burst_count_ = this->get_parameter("burst_count").as_int();
    shoot_interval_ms_ = this->get_parameter("shoot_interval_ms").as_int();
    burst_interval_ms_ = this->get_parameter("burst_interval_ms").as_int();
    fullauto_interval_ms_ = this->get_parameter("fullauto_interval_ms").as_int();

    RCLCPP_INFO(
      this->get_logger(), "burst: %d, burst_interval_ms: %d, burst_interval_ms: %d, fullauto_interval_ms: %d", burst_count_, shoot_interval_ms_, burst_interval_ms_,
      fullauto_interval_ms_);

    //========================================
    // panel limit parameters
    //========================================
    declare_parameter("limit_rad", std::vector<double>(4, 0.0));
    declare_parameter("enable_panel_synchronizer", true);
    declare_parameter("yaw_reflect", 1.0);

    limit_rad_ = this->get_parameter("limit_rad").as_double_array();
    RCLCPP_INFO(
      this->get_logger(), "limit_rad: %f, %f, %f, %f", limit_rad_[0], limit_rad_[1], limit_rad_[2],
      limit_rad_[3]);

    enable_panel_synchronizer_ = this->get_parameter("enable_panel_synchronizer").as_bool();
    yaw_reflect_ = this->get_parameter("yaw_reflect").as_double();
    RCLCPP_INFO(
      this->get_logger(), "enable_panel_synchronizer: %d, yaw_reflect: %f", enable_panel_synchronizer_,
      yaw_reflect_);

    //========================================
    // loading motor parameters
    //========================================
    declare_parameter("loading_motor_speed", 3.0);
    declare_parameter("loading_motor_initial_angle", std::vector<double>(4, 0.0));

    loading_motor_speed_ = this->get_parameter("loading_motor_speed").as_double();
    loading_motor_initial_angle_ =
      this->get_parameter("loading_motor_initial_angle").as_double_array();

    RCLCPP_INFO(
      this->get_logger(), "loading_motor_speed: %f, loading_motor1_initial_angle: %f, loading_motor2_initial_angle: %f, loading_motor3_initial_angle: %f",
      loading_motor_speed_, loading_motor_initial_angle_[0], loading_motor_initial_angle_[1],
      loading_motor_initial_angle_[2]);

    //========================================
    // shoot motor parameters
    //========================================
    declare_parameter("min_speed", 1200.0);
    declare_parameter("target_speed", std::vector<double>(3, 0.0));
    declare_parameter("motor_target_threshold", 0.95);

    min_speed_ = this->get_parameter("min_speed").as_double();
    shoot_motor_target_speed_ = this->get_parameter("target_speed").as_double_array();
    motor_target_threshold_ = this->get_parameter("motor_target_threshold").as_double();

    RCLCPP_INFO(
      this->get_logger(), "min_speed: %f, target_speed1: %f, target_speed2: %f, target_speed3: %f, motor_target_threshold: %f",
      min_speed_, shoot_motor_target_speed_[0], shoot_motor_target_speed_[1],
      shoot_motor_target_speed_[2], motor_target_threshold_);

    //========================================
    // jam parameters
    //========================================
    declare_parameter("jam_detect_time_sec", 0.1);

    jam_detect_time_sec_ = this->get_parameter("jam_detect_time_sec").as_double();

    RCLCPP_INFO(this->get_logger(), "jam_detect_time_sec_: %f", jam_detect_time_sec_);

    //========================================
    // debug parameters
    //========================================
    declare_parameter("set_initial_rad", 0.0);

    set_initial_rad_ = this->get_parameter("set_initial_rad").as_double();

    RCLCPP_INFO(this->get_logger(), "set_initial_rad_: %f", set_initial_rad_);

    //========================================
    // subscribers sensor
    //========================================
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&ShooterController::jointStateCallback, this, std::placeholders::_1));
    target_omega_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/target_omega", 10,
      std::bind(&ShooterController::targetOmegaCallback, this, std::placeholders::_1));
    jam_sensor_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "jam", 10,
      std::bind(&ShooterController::jamSensorCallback, this, std::placeholders::_1));
    hazard_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "hazard_status", 10,
      std::bind(&ShooterController::hazardStatusCallback, this, std::placeholders::_1));

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
    const size_t velocity_size = msg->velocity.size();
    if (loading_motor_id_ < 0 || shoot_motor_id_ < 0 ||
      static_cast<size_t>(loading_motor_id_) >= position_size ||
      static_cast<size_t>(shoot_motor_id_) >= velocity_size ||
      position_size <= 4)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "joint_states size mismatch: position=%zu, velocity=%zu, loading_id=%d, shoot_id=%d",
        position_size, velocity_size, loading_motor_id_, shoot_motor_id_);
      return;
    }

    loading_motor_rad_ = msg->position[loading_motor_id_];
    shoot_motor_rad_ = msg->velocity[shoot_motor_id_];
    turret_angle_from_chassis_ = msg->position[4];
  }

  void targetOmegaCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    target_omega_ = msg->data;
  }

  void jamSensorCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    jam_photo_reflector_raw_ = msg->data;
    rclcpp::Time now = this->get_clock()->now();

    if (jam_photo_reflector_raw_) {
      // true が初めて来た瞬間を記録
      if (!jam_tracking_) {
        start_time_ = now;
        jam_tracking_ = true;
      }

      // true が続いている場合、100ms 経過したかチェック
      auto elapsed = now - start_time_;
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

  //========================================
  // ui callbacks
  //========================================
  void shootCmdCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    shoot_repeat_count_ = msg->data;
  }

  void shootMotorCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    switch (state) {
      case EMERGENCY:
        setSpeed(shoot_motor_id_, 0.0);
        break;

      default:
        if (msg->data > 0.7) {
          setSpeed(shoot_motor_id_, shoot_motor_target_speed_[0]);
        } else if (msg->data > 0.4) {
          setSpeed(shoot_motor_id_, shoot_motor_target_speed_[1]);
        } else if (msg->data > 0.1) {
          setSpeed(shoot_motor_id_, shoot_motor_target_speed_[2]);
        } else {
          setSpeed(shoot_motor_id_, 0.0);
        }
        break;
    }
  }

  //========================================
  // main loop
  //========================================
  void timerCallback()
  {
    jamStatePublish(is_jam_detected_);

    switch (state) {
      case INIT:
        {
          // shoot指令
          RCLCPP_INFO(get_logger(), "Initilize, %f", loading_motor_rad_);

          setAngle(loading_motor_id_, getShootMotorRotationCount() * M_PI);
          last_shoot_time_ = this->now();

          shoot_completed_ = false;
          state = SHOOT;
          RCLCPP_INFO(get_logger(), "change SHOOT (Initilize)");
          break;
        }
      case CMD_WAIT:
        {
          if (hazard_state_) {
            state = EMERGENCY;
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
            shoot_cnt++;
            RCLCPP_INFO(get_logger(), "internal cnt = %d, %f", shoot_cnt, M_PI + shoot_cnt * M_PI);

            setAngle(loading_motor_id_, getShootMotorRotationCount() * M_PI + M_PI);
            last_shoot_time_ = this->now();

            shoot_completed_ = false;
            shootStatePublish(shoot_completed_);

            state = SHOOT;
            RCLCPP_INFO(get_logger(), "change SHOOT");
          }
          RCLCPP_INFO(this->get_logger(), "Remaining number of repeats: %d", shoot_repeat_count_);
          break;
        }
      case SHOOT:
        if (hazard_state_) {
          state = EMERGENCY;
          setAngle(loading_motor_id_, loading_motor_rad_);

          RCLCPP_INFO(get_logger(), "Set EMERGENCY in SHOOT");
          break;
        }

        // target_angleの95%を超えたら遷移
        RCLCPP_INFO_THROTTLE(
          get_logger(),
          *this->get_clock(), 1000, "target: %f, current: %f", target_angle, loading_motor_rad_);

        if (target_angle - (M_PI * 0.05) < loading_motor_rad_) {
          shoot_completed_ = true;
          shootStatePublish(shoot_completed_);

          if (shoot_repeat_count_ >= 1) {
            shoot_repeat_count_--;
          }
          state = CMD_WAIT;
          RCLCPP_INFO(get_logger(), "change CMD_WAIT");
        }
        break;

      case EMERGENCY:
        shoot_completed_ = false;
        shoot_repeat_count_ = 0;

        if (!hazard_state_) {
          // state = CMD_WAIT;
          state = INIT;
          RCLCPP_INFO(get_logger(), "Clear emergency, change CMD_WAIT");
        }
        break;
      default:
        break;
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
        shoot_repeat_count_ = 0;
      }
      return false;
    }
  }

  float getShootMotorRotationCount()
  {
    float rotate = (loading_motor_rad_ - std::fmod(loading_motor_rad_, M_PI)) / M_PI;

    if (std::fmod(loading_motor_rad_, M_PI) > M_PI_2) {
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
    return !hazard_state_ && isValidAngle() && isShootIntervalElapsed() && !isJamDetected();
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

    double shift = yaw_reflect_ * target_omega_;

    // theta を逆方向にシフト + 正規化
    double rad = turret_angle_from_chassis_ - shift;
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

  //========================================
  // motorPub
  //========================================
  // モータの目標角度を設定
  void setAngle(int motor, float angle)
  {
    motorPublish(motor, angle);
    target_angle = angle;
  }

  // 発射モータのスピードを設定
  void setSpeed(int motor, float speed)
  {
    motorPublish(motor, speed);
  }

  // モータの速度をCANで送信
  void motorPublish(int id, float data)
  {
    auto can_array = core_msgs::msg::CANArray();
    auto can = core_msgs::msg::CAN();
    can.id = id;
    can.data.push_back(data);
    can_array.array.push_back(can);
    can_pub_->publish(can_array);
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
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_omega_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr jam_sensor_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hazard_status_sub_;

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
  rclcpp::Clock system_clock(rcl_clock_type_t RCL_SYSTEM_TIME);
  // rclcpp::Clock system_clock(RCL_SYSTEM_TIME);

  //========================================
  // subscription valids
  //========================================
  double loading_motor_rad_ = 0.0;
  double shoot_motor_rad_ = 0.0;
  double turret_angle_from_chassis_ = 0.0;

  double target_omega_ = 0.0;
  bool jam_photo_reflector_raw_ = false;

  bool hazard_state_ = false;

  //========================================
  // parameter valids
  //========================================
  std::string shoot_cmd_topic_;
  int shoot_motor_id_;
  int loading_motor_id_;

  int burst_count_;
  int shoot_interval_ms_;
  int burst_interval_ms_;
  int fullauto_interval_ms_;

  std::vector<double> limit_rad_;
  bool enable_panel_synchronizer_;
  double yaw_reflect_;

  float loading_motor_speed_;
  std::vector<double> loading_motor_initial_angle_;

  bool shoot_completed_disable_;
  bool shoot_ready_state_disable_;
  double set_initial_rad_;

  double min_speed_;
  std::vector<double> shoot_motor_target_speed_;
  double motor_target_threshold_;

  double jam_detect_time_sec_;

  //========================================
  // valids
  //========================================
  bool shoot_completed_ = true;

  // the count of repeated shots. (x < -1: fullauto, x = 0: none, x > 1: burst )
  int shoot_repeat_count_ = 0;

  enum STATE
  {
    INIT,
    CMD_WAIT,
    SHOOT,
    EMERGENCY
  };
  STATE state = CMD_WAIT;
  int shoot_cnt = 0;

  float target_angle = M_PI;

  rclcpp::Time start_time_;
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
