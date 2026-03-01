#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "damiao.h"
#include "robostride.h"

#define CAN1_NUM_DAMIAO 4
#define CAN1_NUM_ROBOSTRIDE 1
#define CAN1_NUM_MOTOR (CAN1_NUM_DAMIAO + CAN1_NUM_ROBOSTRIDE)
#define CAN1_RESEND_INTERVAL_MS 1
#define CAN1_TIMEOUT_MS 1000
#define CAN2_RS05_SPEED_LIMIT 1.0 // rad/s
#define CAN2_RS05_ACC_LIMIT 3.0f // rad/s^2
#define CAN2_RS05_INIT_RUN_MODE PosPP_control_mode
#define CAN2_RS5_SET_GAIN false
#define CAN2_RS5_CHECK_GAIN false


volatile bool can1_waiting_reply = false;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
Damiao<CAN1, RX_SIZE_256, TX_SIZE_16> damiao_motor[CAN1_NUM_DAMIAO] = {
  Damiao(&can1, 0x11, 1),
  Damiao(&can1, 0x12, 2),
  Damiao(&can1, 0x13, 3),
  Damiao(&can1, 0x14, 4)
};
RoboStride<CAN1, RX_SIZE_256, TX_SIZE_16> robostride_can1[CAN1_NUM_ROBOSTRIDE] = {
  RoboStride(&can1, 0x01, 0x01, (int)ActuatorType::ROBSTRIDE_06)
};

MotorBase *can1_motor[CAN1_NUM_MOTOR] = {
  &damiao_motor[0],
  &damiao_motor[1],
  &damiao_motor[2],
  &damiao_motor[3],
  &robostride_can1[0]
};

void can1_cb(const CAN_message_t &msg) {
  for (int i = 0; i < CAN1_NUM_MOTOR; ++i) {
    if (can1_motor[i]->setCanFrame(msg)) {
      can1_waiting_reply = false;
      break;
    }
  }
}

void can1_init() {
  can1.begin();
  can1.setBaudRate(1000000);
  can1.setMaxMB(16);
  can1.enableFIFO();

  while (true) {
    bool init_success = true;

    for (size_t i = 0; i < CAN1_NUM_DAMIAO; ++i) {
      delay(100);
      if (!damiao_motor[i].init()) {
        Serial.print("Error: Damiao ");
        Serial.print(i + 1);
        Serial.println(" Init Failed!");
        init_success = false;
      }
    }

    for (size_t i = 0; i < CAN1_NUM_ROBOSTRIDE; ++i) {
      delay(100);
      if (!robostride_can1[i].init(CAN2_RS05_SPEED_LIMIT, CAN2_RS05_ACC_LIMIT, CAN2_RS05_INIT_RUN_MODE, CAN2_RS5_SET_GAIN, CAN2_RS5_CHECK_GAIN)) {
        Serial.print("Error: RoboStride(can1) ");
        Serial.print(i + 1);
        Serial.println(" Init Failed!");
        init_success = false;
      }
    }

    if (init_success) {
      break;
    }
  }
  Serial.println("can1 Motors Init Success!");

  can1.enableFIFOInterrupt();
  can1.onReceive(can1_cb);
  can1.mailboxStatus();
}

void can1_loop() {
  static int motor_control_i = 0;
  static uint32_t motor_last_send_us[CAN1_NUM_MOTOR] = {0};
  const uint32_t now_us = micros();
  const int idx = motor_control_i;

  const bool period_elapsed =
      (!can1_waiting_reply) &&
      ((uint32_t)(now_us - motor_last_send_us[idx]) >= CAN1_RESEND_INTERVAL_MS * 1000UL);
  const bool timeout_elapsed =
      ((uint32_t)(now_us - motor_last_send_us[idx]) >= CAN1_TIMEOUT_MS * 1000UL);
  if (!(period_elapsed || timeout_elapsed)) {
    return;
  }

  can1_motor[idx]->writeCanFrame();
  can1_waiting_reply = true;
  motor_last_send_us[idx] = now_us;
  motor_control_i = (motor_control_i + 1) % CAN1_NUM_MOTOR;
}
