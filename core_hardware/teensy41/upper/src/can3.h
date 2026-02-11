#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "damiao.h"
#include "robostride.h"

#define CAN3_NUM_DAMIAO 4
#define CAN3_NUM_ROBOSTRIDE 1
#define CAN3_NUM_MOTOR (CAN3_NUM_DAMIAO + CAN3_NUM_ROBOSTRIDE)
#define CAN3_RESEND_INTERVAL_MS 1
#define CAN3_TIMEOUT_MS 1000
#define CAN3_RS06_CURRENT_MAX 10.0f
#define CAN3_RS06_TORQUE_MAX 10.0f

volatile bool can3_waiting_reply = false;

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;
Damiao<CAN3, RX_SIZE_256, TX_SIZE_16> damiao_motor[CAN3_NUM_DAMIAO] = {
  Damiao(&can3, 0x11, 1),
  Damiao(&can3, 0x12, 2),
  Damiao(&can3, 0x13, 3),
  Damiao(&can3, 0x14, 4)
};
RoboStride<CAN3, RX_SIZE_256, TX_SIZE_16> robostride_can3[CAN3_NUM_ROBOSTRIDE] = {
  RoboStride(&can3, 0x01, 0x01, (int)ActuatorType::ROBSTRIDE_06)
};

MotorBase *can3_motor[CAN3_NUM_MOTOR] = {
  &damiao_motor[0],
  &damiao_motor[1],
  &damiao_motor[2],
  &damiao_motor[3],
  &robostride_can3[0]
};

void can3_cb(const CAN_message_t &msg) {
  for (int i = 0; i < CAN3_NUM_MOTOR; ++i) {
    if (can3_motor[i]->setCanFrame(msg)) {
      can3_waiting_reply = false;
      break;
    }
  }
}

void can3_init() {
  can3.begin();
  can3.setBaudRate(1000000);
  can3.setMaxMB(16);
  can3.enableFIFO();

  while (true) {
    bool init_success = true;

    for (size_t i = 0; i < CAN3_NUM_DAMIAO; ++i) {
      delay(100);
      if (!damiao_motor[i].init()) {
        Serial.print("Error: Damiao ");
        Serial.print(i + 1);
        Serial.println(" Init Failed!");
        init_success = false;
      }
    }

    for (size_t i = 0; i < CAN3_NUM_ROBOSTRIDE; ++i) {
      delay(100);
      if (!robostride_can3[i].init(CAN3_RS06_CURRENT_MAX, CAN3_RS06_TORQUE_MAX)) {
        Serial.print("Error: RoboStride(CAN3) ");
        Serial.print(i + 1);
        Serial.println(" Init Failed!");
        init_success = false;
      }
    }

    if (init_success) {
      break;
    }
  }
  Serial.println("CAN3 Motors Init Success!");

  can3.enableFIFOInterrupt();
  can3.onReceive(can3_cb);
  can3.mailboxStatus();
}

void can3_loop() {
  static int motor_control_i = 0;
  static uint32_t motor_last_send_us[CAN3_NUM_MOTOR] = {0};
  const uint32_t now_us = micros();
  const int idx = motor_control_i;

  const bool period_elapsed =
      (!can3_waiting_reply) &&
      ((uint32_t)(now_us - motor_last_send_us[idx]) >= CAN3_RESEND_INTERVAL_MS * 1000UL);
  const bool timeout_elapsed =
      ((uint32_t)(now_us - motor_last_send_us[idx]) >= CAN3_TIMEOUT_MS * 1000UL);
  if (!(period_elapsed || timeout_elapsed)) {
    return;
  }

  can3_motor[idx]->writeCanFrame();
  can3_waiting_reply = true;
  motor_last_send_us[idx] = now_us;
  motor_control_i = (motor_control_i + 1) % CAN3_NUM_MOTOR;
}
