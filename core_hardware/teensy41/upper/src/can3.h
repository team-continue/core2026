#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "damiao.h" 
#include "robostride.h"   // あなたの RoboStride クラスが入ってるヘッダ

#define CAN3_NUM_ROBOSTRIDE 1
#define CAN3_RESEND_INTERVAL_MS 1 // 約1ms間隔
#define CAN3_TIMEOUT_MS 1000 // 1000msでタイムアウトとみなす
volatile bool can3_waiting_reply = false;

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;
RoboStride<CAN3, RX_SIZE_256, TX_SIZE_16> can3_robostride[CAN3_NUM_ROBOSTRIDE] = {
  RoboStride(&can3, 0x00, 0x01, (int)ActuatorType::ROBSTRIDE_06)
};
// CAN3 受信割り込み処理
void can3_cb(const CAN_message_t &msg) {
  for (int i = 0; i < CAN3_NUM_ROBOSTRIDE; ++i) {
    if (can3_robostride[i].setCanFrame(msg)) {
      can3_waiting_reply = false;
      break;
    }
  }
}
void can3_init(){
  can3.begin();
  can3.setBaudRate(1000000);
  can3.setMaxMB(16);
  can3.enableFIFO();
  while(true){
    bool init_success = true;
    for (size_t i = 0; i < CAN3_NUM_ROBOSTRIDE; ++i) {
      delay(100);
      if (!can3_robostride[i].init(10, 10)) {
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
    Serial.println("CAN3 RoboStride Init Success!");

  can3.enableFIFOInterrupt();
  can3.onReceive(can3_cb);
  can3.mailboxStatus();
}

// CAN3 ループ処理
void can3_loop(){
  static int robostride_control_i = 0;
  static uint32_t robostride_last_send_us[CAN3_NUM_ROBOSTRIDE] = {0};
  const uint32_t now_us = micros();
  if((!can3_waiting_reply && (now_us - robostride_last_send_us[robostride_control_i] >= CAN3_RESEND_INTERVAL_MS * 1000)) || //100Hz以上
      (now_us - robostride_last_send_us[robostride_control_i]) >= CAN3_TIMEOUT_MS * 1000){ // 10Hz
      // or timeout
      can3_robostride[robostride_control_i].writeCanFrame();
        can3_waiting_reply = true;
      robostride_control_i = (robostride_control_i + 1) % CAN3_NUM_ROBOSTRIDE;
      // can3_damiao0_send_ts = millis();
      robostride_last_send_us[robostride_control_i] = now_us;
  }
}