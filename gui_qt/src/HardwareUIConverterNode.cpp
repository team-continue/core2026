#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class HardwareUIConverterNode : public rclcpp::Node {
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_degree_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr qe_degree_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_mps_pub_;

public:
    HardwareUIConverterNode() 
    :   Node("hardware_ui_converter_node")
    {
        // subscription
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "~/input/imu", 10, std::bind(&HardwareUIConverterNode::imu_callback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&HardwareUIConverterNode::joint_state_callback, this, std::placeholders::_1));

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&HardwareUIConverterNode::cmd_vel_callback, this, std::placeholders::_1));

        // publisher
        yaw_degree_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/output/yaw_degree", 10);
        qe_degree_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/output/qe_degree", 10);
        speed_mps_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/output/speed_mps", 10);
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Get the yaw angle from the received orientation and publish it.
        double roll, pitch, yaw;
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        std_msgs::msg::Float32 yaw_msg;
        yaw_msg.data = yaw * 180.0 / M_PI;
        yaw_degree_pub_->publish(yaw_msg);
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // 発射角度サーボ上
        auto shoot_angle_rad = msg->position[6];
        std_msgs::msg::Float32 angle_msg;
        angle_msg.data = shoot_angle_rad * 180.0 / M_PI;
        qe_degree_pub_->publish(angle_msg);
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto x = msg->linear.x;
        auto y = msg->linear.y;
        auto v = std::sqrt(x*x + y*y);
        std_msgs::msg::Float32 speed_msg;
        speed_msg.data = v;
        speed_mps_pub_->publish(speed_msg);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardwareUIConverterNode>());
    rclcpp::shutdown();
    return 0;
}