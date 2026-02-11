#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "robostride.h"   // あなたの RoboStride クラスが入ってるヘッダ

#define CAN2_NUM_ROBOSTRIDE 1
#define CAN2_RESEND_INTERVAL_MS 10 // 約1ms間隔
#define CAN2_TIMEOUT_MS 1000 // 1000msでタイムアウトとみなす
volatile bool can2_waiting_reply = false;

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
RoboStride<CAN2, RX_SIZE_256, TX_SIZE_16> can2_robostride[CAN2_NUM_ROBOSTRIDE] = {
  RoboStride(&can2, 0x01, 0x01, (int)ActuatorType::ROBSTRIDE_05)
};
// CAN2 受信割り込み処理
void can2_cb(const CAN_message_t &msg) {
  for (int i = 0; i < CAN2_NUM_ROBOSTRIDE; ++i) {
    if (can2_robostride[i].setCanFrame(msg)) {
      can2_waiting_reply = false;
      // Serial.println("CAN2 RoboStride Received Reply");
      break;
    }
  }
}
void can2_init(){
can2.begin();
  can2.setBaudRate(1000000);
  can2.setMaxMB(16);
  can2.enableFIFO();

  while(true){
    bool init_success = true;
    for (size_t i = 0; i < CAN2_NUM_ROBOSTRIDE; ++i) {
      delay(100);
      if (!can2_robostride[i].init(10, 10)) {
        Serial.print("Error: Motor ");
        Serial.print(i + 1);
        Serial.println(" Init Failed!");
        init_success = false;
        // break;
      }
    }
    if (init_success) {
      break;
    }
  }
  Serial.println("CAN2 RoboStride Init Success!");

  can2.enableFIFOInterrupt();
  can2.onReceive(can2_cb);
  can2.mailboxStatus();
}

// CAN2 ループ処理
void can2_loop(){
  static int robostride_control_i = 0;
  static uint32_t robostride_last_send_us[CAN2_NUM_ROBOSTRIDE] = {0};
  const uint32_t now_us = micros();
  if((!can2_waiting_reply && (now_us - robostride_last_send_us[robostride_control_i] >= CAN2_RESEND_INTERVAL_MS * 1000)) || //100Hz以上
      (now_us - robostride_last_send_us[robostride_control_i]) >= CAN2_TIMEOUT_MS * 1000){ // 10Hz
      // or timeout
      can2_robostride[robostride_control_i].writeCanFrame();
        can2_waiting_reply = true;
      robostride_control_i = (robostride_control_i + 1) % CAN2_NUM_ROBOSTRIDE;
      // can3_damiao0_send_ts = millis();
      robostride_last_send_us[robostride_control_i] = now_us;
  }
}