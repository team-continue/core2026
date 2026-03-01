#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "robostride.h"   // あなたの RoboStride クラスが入ってるヘッダ

#define CAN2_NUM_MOTOR 2
#define CAN2_NUM_ROBOSTRIDE CAN2_NUM_MOTOR
#define CAN2_RESEND_INTERVAL_MS 1 // 約1ms間隔
#define CAN2_TIMEOUT_MS 1000 // 1000msでタイムアウトとみなす
#define CAN2_RS05_SPEED_LIMIT 1.0 // rad/s
#define CAN2_RS05_ACC_LIMIT 3.0f // rad/s^2
#define CAN2_RS05_INIT_RUN_MODE PosPP_control_mode

#define CAN2_RS5_SET_GAIN false
#define CAN2_RS5_CHECK_GAIN false

volatile bool can2_waiting_reply = false;

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
RoboStride<CAN2, RX_SIZE_256, TX_SIZE_16> can2_motor[CAN2_NUM_MOTOR] = {
  RoboStride(&can2, 253, 0x01, (int)ActuatorType::ROBSTRIDE_05),
  RoboStride(&can2, 253, 0x02, (int)ActuatorType::ROBSTRIDE_05)
};
RoboStride<CAN2, RX_SIZE_256, TX_SIZE_16> *can2_robostride = can2_motor;  // backward compatibility
// CAN2 受信割り込み処理
void can2_cb(const CAN_message_t &msg) {
  for (int i = 0; i < CAN2_NUM_MOTOR; ++i) {
    if (can2_motor[i].setCanFrame(msg)) {
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

  // can2_motor[0].Set_Baud_Rate(RoboStrideBaudRate::BAUD_1M);
  // can2_motor[0].Set_Baud_Rate(RoboStrideBaudRate::BAUD_500K);

  delay(2000);

  for(int i=0;i<CAN2_NUM_MOTOR;i++){
    while(true){
      if (can2_motor[i].init(CAN2_RS05_SPEED_LIMIT, CAN2_RS05_ACC_LIMIT, CAN2_RS05_INIT_RUN_MODE, CAN2_RS5_SET_GAIN, CAN2_RS5_CHECK_GAIN) ) {
        Serial.printf("RS5 %d Sucess\n", i+1);
        delay(1000);
        break;
      }
      delay(1000);
      Serial.printf("RS5 %d Init Failed, retrying...\n", i+1);
   }
}

// #if CAN2_NUM_MOTOR > 1
// #endif

  Serial.println("CAN2 RoboStride 2 Init Success!");

  can2.enableFIFOInterrupt();
  can2.onReceive(can2_cb);
  can2.mailboxStatus();
}

// CAN2 ループ処理
void can2_loop(){
  static int robostride_control_i = 0;
  static uint32_t motor_last_send_us[CAN2_NUM_MOTOR] = {0};
  const uint32_t now_us = micros();
  const int idx = robostride_control_i;
  if((!can2_waiting_reply && (now_us - motor_last_send_us[idx] >= CAN2_RESEND_INTERVAL_MS * 1000)) || //100Hz以上
      (now_us - motor_last_send_us[idx]) >= CAN2_TIMEOUT_MS * 1000){ // 10Hz
      // or timeout
      can2_motor[idx].writeCanFrame();
      can2_waiting_reply = true;
      motor_last_send_us[idx] = now_us;
      robostride_control_i = (robostride_control_i + 1) % CAN2_NUM_MOTOR;
      // can3_damiao0_send_ts = millis();
  }
}
