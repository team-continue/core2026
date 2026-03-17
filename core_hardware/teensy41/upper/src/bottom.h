#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "motor.h"

#define BOTTOM_REQUEST_ID 0xFF
#define BOTTOM_RESPONSE_ID 0x00
#define BOTTOM_CAN_TIMEOUT_MS 100
#define BOTTOM_ROS2_TIMEOUT_MS 1000

template <CAN_DEV_TABLE _bus, FLEXCAN_RXQUEUE_TABLE _rxSize, FLEXCAN_TXQUEUE_TABLE _txSize>
class Bottom : public MotorBase {
 public:
  explicit Bottom(FlexCAN_T4<_bus, _rxSize, _txSize> *can) : can_(can) {}

  void setPacketFrame(const float *data, int len) override {
    if (data == nullptr || len <= 0) {
      return;
    }

    led_byte_1_ = static_cast<uint8_t>(data[0]);
    if (len > 1) {
      led_byte_2_ = static_cast<uint8_t>(data[1]);
    }
    last_recv_ros2_ts_ms_ = millis();
  }

  void setLedBytes(uint8_t led_byte_1, uint8_t led_byte_2) {
    led_byte_1_ = led_byte_1;
    led_byte_2_ = led_byte_2;
    last_recv_ros2_ts_ms_ = millis();
  }

  void writeCanFrame() override {
    connect_can = (millis() - last_recv_can_ts_ms_) < BOTTOM_CAN_TIMEOUT_MS;
    connect_ros2 = (millis() - last_recv_ros2_ts_ms_) < BOTTOM_ROS2_TIMEOUT_MS;
    connect = connect_can;

    CAN_message_t msg{};
    msg.id = BOTTOM_REQUEST_ID;
    msg.len = 2;
    msg.buf[0] = led_byte_1_;
    msg.buf[1] = led_byte_2_;
    can_->write(msg);
  }

  bool setCanFrame(const CAN_message_t &msg) override {
    if (msg.id != BOTTOM_RESPONSE_ID || msg.len != 2) {
      return false;
    }

    last_recv_can_ts_ms_ = millis();
    destroy_ = msg.buf[0] != 0;
    hp_ = msg.buf[1];

    motor_state.status = destroy_ ? 1 : 0;
    motor_state.torque_nm = static_cast<float>(hp_);

    connect_can = true;
    connect = true;
    return true;
  }

  bool destroy() const { return destroy_; }
  uint8_t hp() const { return hp_; }

 private:
  FlexCAN_T4<_bus, _rxSize, _txSize> *can_;
  unsigned long last_recv_can_ts_ms_ = 0;
  unsigned long last_recv_ros2_ts_ms_ = 0;
  uint8_t led_byte_1_ = 0;
  uint8_t led_byte_2_ = 0;
  bool destroy_ = false;
  uint8_t hp_ = 0;
};
