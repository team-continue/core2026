#include "core_hardware/Packet.h"
#include "core_msgs/msg/can.hpp"
#include "core_msgs/msg/can_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

#define JOINT_NUM 15

class HaruHardware : public rclcpp::Node {
    rclcpp::Publisher<core_msgs::msg::CANArray>::SharedPtr can_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr wireless_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr destroy_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr hp_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hardware_emergency_pub_;

    rclcpp::Subscription<core_msgs::msg::CANArray>::SharedPtr can_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    Packet packet_;
    uint8_t rx_data_[BUFFER_SIZE], tx_data_[BUFFER_SIZE];
    bool connect = false;
    int tx_len_ = 0;
    std::string port_;
    sensor_msgs::msg::JointState joint_states_;
    std_msgs::msg::UInt8MultiArray wireless_;
    std_msgs::msg::Bool destroy_;
    std_msgs::msg::UInt8 hp_;
    std_msgs::msg::Bool hardware_emergency_;

   public:
    HaruHardware()
        : Node("core_hardware_usb") {
        declare_parameter("port", "/dev/ttyACM0");
        port_ = this->get_parameter("port").as_string();
        can_pub_ = this->create_publisher<core_msgs::msg::CANArray>("can/rx", 10);
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        wireless_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("wireless", 10);
        destroy_pub_ = this->create_publisher<std_msgs::msg::Bool>("destroy", 10);
        hp_pub_ = this->create_publisher<std_msgs::msg::UInt8>("hp", 10);
        hardware_emergency_pub_ = this->create_publisher<std_msgs::msg::Bool>("hardware_emergency", 10);

        can_sub_ = this->create_subscription<core_msgs::msg::CANArray>(
            "can/tx", 10, std::bind(&HaruHardware ::can_cb, this, _1));
        timer_ = this->create_wall_timer(10ms, std::bind(&HaruHardware::timer_cb, this));

        joint_states_.name = std::vector<std::string>{};
        joint_states_.effort.resize(JOINT_NUM, 0);
        joint_states_.velocity.resize(JOINT_NUM, 0);
        joint_states_.position.resize(JOINT_NUM, 0);
    }

   private:
    void timer_cb() {
        int uint8_len=0, flt_len=0, rx_len=0, i=0;
        float* flt_addr=nullptr;
        uint8_t can_id=0, *uint8_addr=nullptr;
        // int *int_addr=nullptr;

        core_msgs::msg::CANArray can_array;

        try {
            // もしつながっていなかったら
            if (!connect) {
                // Teensyと接続する
                packet_.connect(port_.c_str());
                connect = true;
                RCLCPP_INFO(this->get_logger(), "connect");
            } else {
                // 送信
                packet_.send(tx_data_, tx_len_);
                tx_len_ = 0;
                // 受信
                rx_len = packet_.read(rx_data_);
                // CANに変換
                while(i<rx_len){
                    can_id = rx_data_[i];
                    uint8_len = rx_data_[i + 1];
                    flt_len = uint8_len / 4;
                    uint8_addr = &rx_data_[i + 2];
                    flt_addr = (float*)uint8_addr;
                    // int_addr = (int*)uint8_addr;
                    i += (uint8_len + 2);
                    if(uint8_len == 0){
                        // 何もしない
                    // DM 4 + Robostride 3 + Feetech 8 = 15
                    }else if(can_id < JOINT_NUM && flt_len == 6) {
                        core_msgs::msg::CAN can;
                        can.id = can_id;
                        can.data.resize(flt_len);
                        can.data.assign(flt_addr, flt_addr + flt_len);
                        can_array.array.push_back(can);
                        joint_states_.effort[can_id] = can.data[1];
                        joint_states_.velocity[can_id] = can.data[3];
                        joint_states_.position[can_id] = can.data[5];
                    } else if (can_id == 100 && uint8_len == 1) {
                        // damege
                        hp_.data = uint8_addr[0];
                        hp_pub_->publish(hp_);
                    } else if (can_id == 101 && uint8_len == 1) {
                        // destroy
                        destroy_.data = (uint8_addr[0] == 1);
                        destroy_pub_->publish(destroy_);
                    } else if (can_id == 102) {
                        // wireless
                        wireless_.data.assign(uint8_addr, uint8_addr + uint8_len);
                        wireless_pub_->publish(wireless_);
                    } else if (can_id == 104 && uint8_len == 1) {
                        hardware_emergency_.data = (uint8_addr[0] == 1);
                        hardware_emergency_pub_->publish(hardware_emergency_);
                    }
                }
                if (can_array.array.size() > 0){
                    can_pub_->publish(can_array);
                    joint_pub_->publish(joint_states_);
                }
            }
            // 接続できなかった
        } catch (...) {
            connect = false;
            RCLCPP_INFO(this->get_logger(), "no connect");
            // １秒待つ
            rclcpp::sleep_for(1000ms);
        }
    }
    void can_cb(const core_msgs::msg::CANArray::SharedPtr msg) {
        int flt_len, uint8_len;
        uint8_t* uint8_addr;
        for (auto& can : msg->array) {
            flt_len = can.data.size();
            uint8_len = flt_len * 4;
            if (tx_len_ + uint8_len > BUFFER_SIZE)
                return;
            tx_data_[tx_len_] = can.id;
            tx_data_[tx_len_ + 1] = uint8_len;
            uint8_addr = (uint8_t*)can.data.data();
            std::copy(uint8_addr, uint8_addr + uint8_len, tx_data_ + tx_len_ + 2);
            tx_len_ += (uint8_len + 2);
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HaruHardware>());
    rclcpp::shutdown();
    return 0;
}
