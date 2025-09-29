#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <math.h> 
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "core_msgs/msg/can.hpp"
#include "core_msgs/msg/can_array.hpp"


class ShooterCmdGate : public rclcpp::Node
{
public:
    ShooterCmdGate() : Node("shooter_cmd_gate")
    {
        //========================================
        // parameters
        //========================================


        //========================================
        // subscribers ui
        //======================================== 
        once_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "shoot_once", 1, 
            std::bind(&ShooterCmdGate::onceCallback, this, std::placeholders::_1));
        burst_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "shoot_burst", 1, 
            std::bind(&ShooterCmdGate::burstCallback, this, std::placeholders::_1));
        fullauto_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "shoot_fullauto", 1, 
            std::bind(&ShooterCmdGate::fullautoCallback, this, std::placeholders::_1));
        fullburst_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "shoot_fullburst", 1, 
            std::bind(&ShooterCmdGate::fullburstCallback, this, std::placeholders::_1));
        left_shoulder_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "shoot_left_shoulder", 1, 
            std::bind(&ShooterCmdGate::shootLeftCallback, this, std::placeholders::_1));
        right_shoulder_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "shoot_right_shoulder", 1, 
            std::bind(&ShooterCmdGate::shootRightCallback, this, std::placeholders::_1));

        //========================================
        // publishers
        //========================================
        center_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "center_shoot_cmd", 10);
        left_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "left_shoot_cmd", 10);
        right_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "right_shoot_cmd", 10);

        //========================================
        // timer callback
        //========================================
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&ShooterCmdGate::timerCallback, this));
    }

    void onceCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            center_shoot_repeat_count_ = 1;
            RCLCPP_INFO(this->get_logger(), "On trigger: Shoot");
        }
    }

    void burstCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            center_shoot_repeat_count_ = 3;
            RCLCPP_INFO(this->get_logger(), "On trigger: Burst");
        }
    }

    void fullautoCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            // Flip: ON
            if (msg->data) {
                center_shoot_repeat_count_ = -1;
                RCLCPP_INFO(this->get_logger(), "On trigger: Fullauto");
            }
        } else if(!msg->data) {
            // Flip: OFF
            if (center_shoot_repeat_count_ == -1) {
                center_shoot_repeat_count_ = 0;
                return;
            }
        }
    }

    void fullburstCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            center_shoot_repeat_count_ = 1;
            left_shoot_repeat_count_ = 1;
            right_shoot_repeat_count_ = 1;
            RCLCPP_INFO(this->get_logger(), "On trigger: Fullburst");
        }
    }

    void shootLeftCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            left_shoot_repeat_count_ = 1;
            RCLCPP_INFO(this->get_logger(), "On trigger: Right Shoot");
        }
    }

    void shootRightCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            right_shoot_repeat_count_ = 1;
            RCLCPP_INFO(this->get_logger(), "On trigger: Right Shoot");
        }
    }

    void timerCallback() {
        auto center_msg = std_msgs::msg::Int32();
        center_msg.data = center_shoot_repeat_count_;
        center_cmd_pub_->publish(center_msg);

        auto left_msg = std_msgs::msg::Int32();
        left_msg.data = left_shoot_repeat_count_;
        left_cmd_pub_->publish(left_msg);

        auto right_msg = std_msgs::msg::Int32();
        right_msg.data = right_shoot_repeat_count_;
        right_cmd_pub_->publish(right_msg);   
    }

    //========================================
    // Subscription valids
    //========================================
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr once_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr burst_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fullauto_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fullburst_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_shoulder_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_shoulder_sub_;

    //========================================
    // publisher valids
    //========================================
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr center_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_cmd_pub_;

    //========================================
    // timer callback valids
    //========================================
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_shoot_time_ = now();
    rclcpp::Clock system_clock(rcl_clock_type_t RCL_SYSTEM_TIME);

    //========================================
    // valids
    //========================================
    // the count of repeated shots. (x < -1: fullauto, x = 0: none, x > 1: burst )
    int center_shoot_repeat_count_ = 0;
    int left_shoot_repeat_count_ = 0;
    int right_shoot_repeat_count_ = 0;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ShooterCmdGate>());

    rclcpp::shutdown();
    return 0;
}