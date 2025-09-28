#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <math.h> 
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "core_msgs/msg/can.hpp"
#include "core_msgs/msg/can_array.hpp"


class ShootController : public rclcpp::Node
{
public:
    ShootController() : Node("shooter_controller")
    {
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

        RCLCPP_INFO(this->get_logger(), "< shoot config >\n burst: %d, burst_interval_ms: %d, burst_interval_ms: %d, fullauto_interval_ms: %d", burst_count_, shoot_interval_ms_, burst_interval_ms_, fullauto_interval_ms_);

        //========================================
        // panel limit parameters
        //========================================
        declare_parameter("limit_rad",std::vector<double>(4,0.0));
        declare_parameter("enable_panel_synchronizer", true);
        declare_parameter("yaw_reflect", 1.0);

        limit_rad_ = this->get_parameter("limit_rad").as_double_array();
        RCLCPP_INFO(this->get_logger(), "limit_rad: %f, %f, %f, %f", limit_rad_[0], limit_rad_[1], limit_rad_[2], limit_rad_[3]);

        enable_panel_synchronizer_ = this->get_parameter("enable_panel_synchronizer").as_bool();
        yaw_reflect_ = this->get_parameter("yaw_reflect").as_double();
        RCLCPP_INFO(this->get_logger(), "enable_panel_synchronizer: %d, yaw_reflect: %f", enable_panel_synchronizer_, yaw_reflect_);

        //========================================
        // loading motor parameters
        //========================================
        loading_motor_speed_ = this->get_parameter("loading_motor_speed").as_double();
        loading_motor_initial_angle_ = this->get_parameter("loading_motor_initial_angle").as_double_array();

        declare_parameter("loading_motor_speed", 3.0);
        declare_parameter("loading_motor_initial_angle",std::vector<double>(4,0.0));

        RCLCPP_INFO(this->get_logger(), "loading_motor_speed: %f, loading_motor1_initial_angle: %f, loading_motor2_initial_angle: %f, loading_motor3_initial_angle: %f", 
        loading_motor_speed_, loading_motor_initial_angle_[0], loading_motor_initial_angle_[1], loading_motor_initial_angle_[2]);
        
        //========================================
        // shoot motor parameters
        //========================================
        declare_parameter("min_speed", 1200.0);
        declare_parameter("target_speed",std::vector<double>(3,0.0));
        declare_parameter("motor_target_threshold", 0.95);

        min_speed_ = this->get_parameter("min_speed").as_double();
        shoot_motor_target_speed_ = this->get_parameter("limit_rad").as_double_array();
        motor_target_threshold_ = this->get_parameter("motor_target_threshold").as_double();

        RCLCPP_INFO(this->get_logger(), "min_speed: %f, target_speed1: %f, target_speed2: %f, target_speed3: %f, motor_target_threshold: %f",
        min_speed_, shoot_motor_target_speed_[0], shoot_motor_target_speed_[1], shoot_motor_target_speed_[2], motor_target_threshold_);

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
            std::bind(&ShootController::jointStateCallback, this, std::placeholders::_1));
        target_omega_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/target_omega", 10,
            std::bind(&ShootController::targetOmegaCallback, this, std::placeholders::_1));
        jam_sensor_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "jam", 10,
            std::bind(&ShootController::jamSensorCallback, this, std::placeholders::_1));
        hazard_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/system/emergency/hazard_status", 10,
            std::bind(&ShootController::hazardStatusCallback, this, std::placeholders::_1));

        //========================================
        // subscribers ui
        //========================================
        shoot_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "shoot", 1, 
            std::bind(&ShootController::shootCallback, this, std::placeholders::_1));
        shoot_burst_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "shoot_burst", 1, 
            std::bind(&ShootController::shootBurstCallback, this, std::placeholders::_1));
        shoot_fullauto_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "shoot_fullauto", 1, 
            std::bind(&ShootController::shootFullautoCallback, this, std::placeholders::_1));
        // shoot_fullburst_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        //     "shoot_fullburst", 1, 
        //     std::bind(&ShootController::shootFullburstCallback, this, std::placeholders::_1));

        shoot_right_shoulder_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "shoot_right_shoulder", 1, 
            std::bind(&ShootController::shootCallback, this, std::placeholders::_1));
        shoot_left_shoulder_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "shoot_right_shoulder", 1, 
            std::bind(&ShootController::shootCallback, this, std::placeholders::_1));

        discharge_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "discharge", 1,
            std::bind(&ShootController::dischargeCallback, this, std::placeholders::_1));

        shoot_motor_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/ui/shoot_motor", 1,
            std::bind(&ShootController::shootMotorCallback, this, std::placeholders::_1));

        //========================================
        // publishers
        //========================================
        shoot_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "~/shoot_status", 10);
        can_pub_ = this->create_publisher<core_msgs::msg::CANArray>(
            "/can/tx", 10);
        jam_state_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "~/jam_state", 10);
        loading_error_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "~/loading_motor_error", 10);
        shooting_error_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "~/shooting_motor_error", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&ShootController::timerCallback, this));
    }

private:
    //========================================
    // sensor callbacks
    //========================================
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        motor1_rad_ = msg->position[7];
        motor2_rad_ = - msg->position[8];
        motor3_rad_ = - msg->position[9];
    }

    void targetOmegaCallback(const std_msgs::msg::Float32::SharedPtr msg) 
    {
        target_omega_ = msg->data;
    }

    void jamSensorCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        jam_photo_reflector_raw_ = msg->data;
        rclcpp::Time now = this->get_clock()->now();

        if (jam_photo_reflector_raw_){
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
        if (hazerd_state_ && msg->data) {
            // キンテイ解除時にショット完了をtrueにする
            shoot_completed_ = true;
            RCLCPP_INFO(get_logger(), "Clear Emergency, Set shoot_completed_ TRUE");
        }
        hazerd_state_ = msg->data;
    }

    //========================================
    // ui callbacks
    //========================================
    void shootCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && canShoot()) {
            shoot_repeat_count = 1;
        } else if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "On trigger: 'Shoot' - Not ready.");
        }
    }

    void shootBurstCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && canShoot()) {
            shoot_repeat_count = 3;
        } else if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "On trigger: 'Burst' - Not ready.");
        }
    }

    void shootFullautoCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            // Flip: ON
            if (msg->data && canShoot()) {
                shoot_repeat_count = -1;
            } else if (msg->data) {
                RCLCPP_INFO(this->get_logger(), "On trigger: 'Fullauto' - Not ready.");
            }
        } else if(!msg->data) {
            // Flip: OFF
            if (shoot_repeat_count == -1) {
                shoot_repeat_count = 0;
                return;
            }
        }
    }

    // void shootFullburstCallback(const std_msgs::msg::Bool::SharedPtr msg){}

    void dischargeCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data){
            // state = DISCHARGE;
            // RCLCPP_INFO(get_logger(), "change DISCHARGE");
            RCLCPP_INFO(get_logger(), "DISCHARGE not enable");
        }
    }

    void shootMotorCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        switch (state)
        {
            case EMERGENCY:
                setSpeed(CENTER_TURRET_SHOOTING_MOTOR, 0.0);
                break;

            default:
                if (msg->data > 0.7) {
                    setSpeed(CENTER_TURRET_SHOOTING_MOTOR, shoot_motor_target_speed_[0]);
                } else if (msg->data > 0.4){
                    setSpeed(CENTER_TURRET_SHOOTING_MOTOR, shoot_motor_target_speed_[1]);
                } else if (msg->data > 0.1){
                    setSpeed(CENTER_TURRET_SHOOTING_MOTOR, shoot_motor_target_speed_[2]);
                } else {
                    setSpeed(CENTER_TURRET_SHOOTING_MOTOR, 0.0);
                }
                break;
        }
    }

    //========================================
    // main loop
    //========================================
    void timerCallback() {
        switch (state)
        {
        case CMD_WAIT:
        {
            if (is_emergency_unlock) {
                state = EMERGENCY;
                RCLCPP_INFO(get_logger(), "Set EMERGENCY in CMD_WAIT");
                break;
            }

            // repeat > 1 && repeat < 0
            if (shoot_repeat_count == 0) {
                break;
            }
            bool result = shootDecision();
            if (result) {
                // shoot指令
                shoot_cnt++;
                RCLCPP_INFO(get_logger(), "internal cnt = %d, %f", shoot_cnt, M_PI + shoot_cnt * M_PI);

                float rotate = (motor1_rad_ - std::fmod(motor1_rad_, M_PI)) / M_PI;
                if (std::fmod(motor1_rad_, M_PI) > M_PI_2) {
                    rotate += 1;
                }
                setAngle(CENTER_TURRET_LOADING_MOTOR, rotate * M_PI + M_PI);

                shoot_completed_ = false;
                state = SHOOT;
                RCLCPP_INFO(get_logger(), "change SHOOT");
            }
            RCLCPP_INFO(this->get_logger(), "Remaining number of repeats: %d", shoot_repeat_count);
            break;
        }
        case SHOOT:
            if (is_emergency_unlock) {
                state = EMERGENCY;
                RCLCPP_INFO(get_logger(), "Set EMERGENCY in SHOOT");
                break;
            }

            // target_angleの95%を超えたら遷移
            RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "target: %f, current: %f", target_angle, motor1_rad_);

            if (target_angle - (M_PI * 0.05) < motor1_rad_) {
                shoot_completed_ = true;
                if (shoot_repeat_count >= 1) {
                    shoot_repeat_count--;
                }
                state = CMD_WAIT;
                RCLCPP_INFO(get_logger(), "change CMD_WAIT");
            }
            break;

        case DISCHARGE:
        {
            float rotate = (motor1_rad_ - std::fmod(motor1_rad_, M_PI)) / M_PI;
            if (std::fmod(motor1_rad_, M_PI) > M_PI_2) {
                rotate += 1;
            }
            setAngle(CENTER_TURRET_LOADING_MOTOR, rotate * M_PI + M_PI * 2); // 1回転する

            shoot_completed_ = false;
            state = SHOOT;
            RCLCPP_INFO(get_logger(), "change SHOOT");
            break;
        }

        case EMERGENCY:
            shoot_completed_ = false;

            if (!is_emergency_unlock) {
                state = CMD_WAIT;
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
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "shoot completed: %d, Angle: %d, Interval: %d", shoot_completed_ , isValidAngle() , isShootIntervalElapsed());
        
        if (canShoot()) {
            RCLCPP_INFO(this->get_logger(), "========== Shoot ==========");
            return true;
        } else {
            // Output logs every second.
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Shoot condition not met");
            return false;
        }
    }

    //========================================
    // canShoot
    //========================================
    bool canShoot() {
        return hazerd_state_ && isValidAngle() && isShootIntervalElapsed() && !isJamDetected();
    }

    bool isValidAngle()
    {
        // ロジック直そうね！
        // If the angle limit is disabled, always return true.
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
        bool ret = false;
        float front_rad_offset = 1.9 - target_omega_ * yaw_reflect_; // [rad]
        float rad = turret_angle_from_chassis_ + front_rad_offset;
        
        // ラジアンを0 ~ 2PIに変換する。
        rad = std::fmod(rad, 2 * M_PI);
        // 変換した際に負の値の場合、ポジティブ方向で絶対角度にする。
        if (rad < 0) {
            rad = M_PI * 2 - rad;
        }

        RCLCPP_INFO(get_logger(), "raw: %f, rad: %f, front: %f", turret_angle_from_chassis_, rad, front_rad_offset);

        if (0 < rad && rad < limit_rad_[0]) {
            ret = true;
        } else if (limit_rad_[1] < rad && rad < limit_rad_[2]) {
            ret = true;
        } else if (limit_rad_[3] < rad && rad < 2 * M_PI) {
            ret = true;
        }

        return ret;
    }

    bool isShootIntervalElapsed()
    {
        if (shoot_repeat_count == 1){
            return (this->now() - last_shoot_time_).seconds() * 1000 >= shoot_interval_ms_;
        } else if (shoot_repeat_count >= 2){
            return (this->now() - last_shoot_time_).seconds() * 1000 >= burst_interval_ms_;
        } else if (shoot_repeat_count <= -1){
            return (this->now() - last_shoot_time_).seconds() * 1000 >= fullauto_interval_ms_;
        }
        return false;
    }

    //========================================
    // motorPub
    //========================================
    enum MOTOR{
        CENTER_TURRET_LOADING_MOTOR = 7,
        LEFT_TURRET_LOADING_MOTOR = 8,
        RIGHT_TURRET_LOADING_MOTOR = 9,
        CENTER_TURRET_SHOOTING_MOTOR = 10,
        LEFT_TURRET_SHOOTING_MOTOR = 11,
        RIGHT_TURRET_SHOOTING_MOTOR = 12,
    };

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
        if (is_jam_detected_) {
            return true;
        } else if (jam_photo_reflector_raw_) {
            return true;
        }
        return false;
    }

    //========================================
    // Subscription valids
    //========================================
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_omega_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr jam_sensor_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hazard_status_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shoot_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shoot_burst_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shoot_fullauto_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shoot_fullburst_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shoot_right_shoulder_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shoot_left_shoulder_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr discharge_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr shoot_motor_sub_;
 
    //========================================
    // publisher valids
    //========================================
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shoot_status_pub_ ;    
    rclcpp::Publisher<core_msgs::msg::CANArray>::SharedPtr can_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr jam_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr loading_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shooting_error_pub_;    
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_shoot_time_ = now();
    rclcpp::Clock system_clock(rcl_clock_type_t RCL_SYSTEM_TIME);

    //========================================
    // subscription valids
    //========================================

    double motor1_rad_;
    double motor2_rad_;
    double motor3_rad_;
    double turret_angle_from_chassis_;

    double target_omega_ = 0.0;
    double jam_photo_reflector_raw_;

    bool hazerd_state_ = true;

    //========================================
    // parameter valids
    //========================================
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
    int shoot_repeat_count = 0;

    enum STATE{
        CMD_WAIT,
        SHOOT,
        DISCHARGE,
        EMERGENCY
    };
    STATE state = CMD_WAIT;
    int shoot_cnt = 0;

    bool is_emergency_unlock;
    float target_angle = M_PI;

    rclcpp::Time start_time_;
    bool jam_tracking_ = false;
    bool is_jam_detected_ = false;


    //========================================
    // debug parameters
    //========================================
    // turret_angle_from_chassis_ = set_initial_rad_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ShootController>());

    // SingleThreadedExecutor の作成
    // rclcpp::executors::SingleThreadedExecutor executor;
    // rclcpp::executors::MultiThreadedExecutor executor;
    
    // ノードをExecutorに追加
    // auto shoot_controller = std::make_shared<ShootController>();
    // auto shoot_subscriber = std::make_shared<ShootSubscriber>();
    // executor.add_node(shoot_controller);
    // executor.add_node(shoot_subscriber);

    // 実行（この関数はブロッキングする）
    // executor.spin();

    rclcpp::shutdown();
    return 0;
}