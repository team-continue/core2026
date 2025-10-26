
// #include "tf2_geometry_msgs/msg/tf2_geometry_msgs.hpp"

#include <algorithm>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "core_msgs/msg/can.hpp"
#include "core_msgs/msg/can_array.hpp"

#define MIN(a, b) ((a) < (b) ? (a) : (b))

using namespace boost::asio;
using namespace std::chrono_literals;

#define FLT_SIZE 4

#define LEN_BUFF_TX 32768
#define LEN_BUFF_RX 32768

#define LAST_ID (0xff)

class CanNode : public rclcpp::Node {
    rclcpp::Publisher<core_msgs::msg::CANArray>::SharedPtr can_pub_;
    rclcpp::Subscription<core_msgs::msg::CANArray>::SharedPtr can_array_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    boost::asio::io_service io;
    boost::asio::io_context& io_;
    boost::asio::serial_port port_;

    std::string path_;
    bool connected_ = false;
    uint8_t buff_tx_[LEN_BUFF_TX] = {0};
    int len_buff_tx_ = 0;
    uint8_t buff_rx_usb_[LEN_BUFF_RX] = {0};
    int len_buff_rx_usb_ = 0;
    uint8_t buff_decoded_[LEN_BUFF_RX] = {0};

   public:
    CanNode() : Node("core_hardware_usb"), io(), port_(io), io_(io) {
        // パラメーター
        declare_parameter("update_freq", 0.005);
        declare_parameter("port", "/dev/teensy");

        path_ = this->get_parameter("port").as_string();
        float update_freq = this->get_parameter("update_freq").as_double();

        // ROS初期化
        can_pub_ = this->create_publisher<core_msgs::msg::CANArray>("can/rx", 10);
        can_array_sub_ = this->create_subscription<core_msgs::msg::CANArray>(
            "can/tx", 10, std::bind(&CanNode::can_cb, this, std::placeholders::_1));

        // メインルーチン
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::seconds{1} * update_freq);
        timer_ = this->create_wall_timer(duration, std::bind(&CanNode::timer_cb, this));
    }
    ~CanNode() {
        RCLCPP_INFO(this->get_logger(), "stop");
        port_.cancel();
        port_.close();
        io_.stop();
    }
    void run() {
        // USB非同期受信
        RCLCPP_INFO(this->get_logger(), "port: %s", path_.c_str());
        rclcpp::WallRate loop_rate(1000ms);
        while (rclcpp::ok()) {
            try {
                port_.open(path_);
                if (port_.is_open()) {
                    port_.set_option(boost::asio::serial_port_base::baud_rate(9600));
                    start_receive();
                    io_.restart();
                    connected_ = true;
                    RCLCPP_INFO(this->get_logger(), "connected_!");
                    io.run();
                }
            } catch (...) {
                connected_ = false;
                RCLCPP_WARN(this->get_logger(), "connect failed");
            }
            loop_rate.sleep();
        }
        connected_ = false;
    }

   private:
    void can_cb(const core_msgs::msg::CANArray::SharedPtr msg) {
        for (auto& c : msg->array) {
            // 送信するデータをためていく
            setTxData(c.id, (uint8_t*)c.data.data(), c.data.size() * FLT_SIZE);
        }
    }
    // 数200Hzで非同期送信
    void timer_cb() {
        // 最後のデータを入れてる
        float data[1];
        setTxData(LAST_ID, (uint8_t*)data, 0);

        // Debug
        // printBuffer(buff_tx_, len_buff_tx_);

        // 送信
        // printBuffer(buff_encoded, packet_size + 2);
        if (connected_) {
            try {
                port_.write_some(buffer(buff_tx_, len_buff_tx_));
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "send failed");
            }
        }
        len_buff_tx_ = 0;
    }
    // 受信割り込みを設定
    void start_receive() {
        // バッファが大きくなったらクリア
        if (len_buff_rx_usb_ > (LEN_BUFF_RX - 64))
            len_buff_rx_usb_ = 0;
        // 非同期受信
        port_.async_read_some(
            boost::asio::buffer(len_buff_rx_usb_ + buff_rx_usb_, LEN_BUFF_RX - len_buff_rx_usb_),
            boost::bind(&CanNode::handle_receive, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    }
    // 受信割り込み関数
    void handle_receive(const boost::system::error_code& error,
                        std::size_t bytes_transferred) {
        if (boost::system::errc::success == error) {
            // 受信バッファサイズを更新
            len_buff_rx_usb_ += bytes_transferred;
            // Debug
            // printBuffer(buff_rx_usb_, len_buff_rx_usb_);
            // std::cout << "find header" << std::endl;
            // 受信バッファからヘッダー(0x00)を探す

            int i, len_read = 0;
            core_msgs::msg::CANArray can_array;

            for (i = 0; i < len_buff_rx_usb_; ++i) {
                // ヘッダーが見つかった
                if (buff_rx_usb_[i] != 0x00)
                    continue;
                // COBSデコード

                int len_encoded = (i - len_read) + 1;
                // std::cout << "decode" <<", "<<i<<", "<<len_read<<", " << len_encoded<<std::endl;
                // printBuffer(buff_rx_usb_ + len_read, len_encoded);
                // std::cout << "-->" << std::endl;
                int size = cobsDecode(buff_rx_usb_ + len_read, len_encoded, buff_decoded_);
                // debug
                // printBuffer(buff_decoded_, size);
                // デコードに失敗した
                if (len_encoded != (size + 1)) {
                    len_read = (i + 1);
                    continue;
                }
                // buff_decoded: 0x00, id, len, data
                if (buff_decoded_[0] != 0x00) {
                    len_read = (i + 1);
                    continue;
                }
                int id_can = buff_decoded_[1];
                int len_uint8 = buff_decoded_[2];
                // 0x00, id, len, data
                if (1 + 2 + len_uint8 != size) {
                    len_read = (i + 1);
                    continue;
                }

                // CANデータ
                core_msgs::msg::CAN can;
                can.id = id_can;
                int len_flt = len_uint8 / FLT_SIZE;
                float* data_flt = (float*)&buff_decoded_[3];
                can.data.resize(len_flt);
                std::copy(data_flt, data_flt + len_flt, can.data.begin());
                can_array.array.push_back(can);
                len_read = (i + 1);
            }
            if (can_array.array.size() > 0) {
                can_pub_->publish(can_array);
            }
            // 退避
            len_buff_rx_usb_ -= len_read;
            memcpy(buff_decoded_, buff_rx_usb_ + len_read, len_buff_rx_usb_);
            // 左詰める
            memcpy(buff_rx_usb_, buff_decoded_, len_buff_rx_usb_);
            start_receive();
        } else {
            port_.cancel();
            port_.close();
            len_buff_rx_usb_ = 0;
        }
    }
    // 送信用バッファにデータを格納
    void setTxData(uint8_t id, uint8_t* data, uint8_t len) {
        //  id + data + len + 1(COBS) + 1(0x00)
        if ((len_buff_tx_ + len + 2 + 2) >= LEN_BUFF_TX)
            return;
        if (!isCANData(id, len))
            return;
        uint8_t tmp[128] = {0};
        tmp[0] = id;
        tmp[1] = len;
        memcpy(tmp + 2, data, len);
        size_t size = cobsEncode(tmp, len + 2, buff_tx_ + len_buff_tx_);
        buff_tx_[len_buff_tx_ + size] = 0x00;  // COBSのデリミタ
        len_buff_tx_ += (size + 1);
    }
    bool isCANData(int id, int len) {
        int size = len / FLT_SIZE;
        if (size > 16 || size == 7 || size == 9 || size == 10 || size == 11 ||
            size == 13 || size == 14 || size == 15) {
            RCLCPP_WARN(this->get_logger(), "id: %d, data.size() is 0, 1, 2, 3, 4, 5, 6, 8, 12, 16", id);
            return false;
        }
        return true;
    }
    size_t cobsEncode(const uint8_t* data, size_t length, uint8_t* buffer) {
        uint8_t* encode = buffer;   // Encoded byte pointer
        uint8_t* codep = encode++;  // Output code pointer
        uint8_t code = 1;           // Code value

        for (const uint8_t* byte = data; length--; ++byte) {
            if (*byte)  // Byte not zero, write it
                *encode++ = *byte, ++code;
            else {
                *codep = code, code = 1, codep = encode;
                if (!*byte || length)
                    ++encode;
            }
        }
        *codep = code;  // Write final code value

        return (size_t)(encode - buffer);
    }
    size_t cobsDecode(const uint8_t* buffer, size_t length, uint8_t* data) {
        const uint8_t* byte = buffer;  // Encoded input byte pointer
        uint8_t* decode = data;        // Decoded output byte pointer

        for (uint8_t block = 0; byte < buffer + length; --block) {
            if (block)  // Decode block byte
                *decode++ = *byte++;
            else {
                block = *byte++;  // Fetch the next block length
                if (block)        // Encoded zero, write it unless it's delimiter.
                    *decode++ = 0;
            }
        }
        return (size_t)(decode - data);
    }
    void printBuffer(uint8_t* buf, int size) {
        std::cout << "B " << size << ": ";
        for (int i = 0; i < size; ++i) {
            std::cout << (unsigned int)(uint8_t)buf[i] << " ";
        }
        std::cout << std::endl;
    }
    void printVector(const std::vector<int>& vec) {
        int size = vec.size();
        std::cout << "V " << size << ": ";
        for (int i = 0; i < size; ++i) {
            std::cout << vec.at(i) << " ";
        }
        std::cout << std::endl;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanNode>();
    std::thread t(&CanNode::run, node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}