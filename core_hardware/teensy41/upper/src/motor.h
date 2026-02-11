#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>

struct MotorRef {
  int mode = 0;
  float position_rad = 0.0f;
  float velocity_rad_s = 0.0f;
  float velocity_limit_rad_s = 0.0f;
  float torque_nm = 0.0f;
  float kp_vel = 0.0f;
  float ki_vel = 0.0f;
  float kp_pos = 5.0f;
  float kd_pos = 0.1f;
};

struct MotorState {
  float position_rad = 0.0f;
  float velocity_rad_s = 0.0f;
  float torque_nm = 0.0f;
  float temp_mos = 0.0f;
  float temp_rotor = 0.0f;
  uint8_t status = 0;
};

class MotorBase {
 public:
  virtual ~MotorBase() = default;

  virtual void setPacketFrame(const float *data, int len) = 0;
  virtual void writeCanFrame() = 0;
  virtual bool setCanFrame(const CAN_message_t &msg) = 0;

  MotorRef motor_ref;
  MotorState motor_state;
  bool connect_ros2 = false;
  bool connect_can = false;
  bool connect = false;
};
