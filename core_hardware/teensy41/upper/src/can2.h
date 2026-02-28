#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "robostride.h"   // あなたの RoboStride クラスが入ってるヘッダ

// [RoboStride:param] start master_id=1 motor_id=1
// [RoboStride:param] idx=0x7005 name=run_mode value=0.000000
// [RoboStride:param] idx=0x7006 name=iq_ref value=0.000000
// [RoboStride:param] idx=0x700A name=spd_ref value=0.000000
// [RoboStride:param] idx=0x700B name=imit_torque value=5.500000
// [RoboStride:param] idx=0x7010 name=cur_kp value=0.125000
// [RoboStride:param] idx=0x7011 name=cur_ki value=0.015800
// [RoboStride:param] idx=0x7014 name=cur_filt_gain value=0.100000
// [RoboStride:param] idx=0x7016 name=loc_ref value=4.659590
// [RoboStride:param] idx=0x7017 name=limit_spd value=10.000000
// [RoboStride:param] idx=0x7018 name=limit_cur value=11.000000
// [RoboStride:param] idx=0x7019 name=mechPos value=4.659590
// [RoboStride:param] idx=0x701A name=iqf value=0.000000
// [RoboStride:param] idx=0x701B name=mechVel value=-0.061554
// [RoboStride:param] idx=0x701C name=VBUS value=24.737885
// [RoboStride:param] idx=0x701D name=rotation value=0.0000
// [RoboStride:param] idx=0x701E name=loc_kp value=10.00000
// [RoboStride:param] idx=0x701F name=spd_kp value=2.000000
// [RoboStride:param] idx=0x7020 name=spd_ki value=0.021000
// [RoboStride:param] idx=0x7021 name=spd_filt_gain value=0.050000

// [RoboStride:param] start master_id=1 motor_id=2
// [RoboStride:param] idx=0x7005 name=run_mode value=0.000000
// [RoboStride:param] idx=0x7006 name=iq_ref value=0.000000
// [RoboStride:param] idx=0x700A name=spd_ref value=0.000000
// [RoboStride:param] idx=0x700B name=imit_torque value=5.500000
// [RoboStride:param] idx=0x7010 name=cur_kp value=0.125000
// [RoboStride:param] idx=0x7011 name=cur_ki value=0.015800
// [RoboStride:param] idx=0x7014 name=cur_filt_gain value=0.100000
// [RoboStride:param] idx=0x7016 name=loc_ref value=3.364417
// [RoboStride:param] idx=0x7017 name=limit_spd value=10.000000
// [RoboStride:param] idx=0x7018 name=limit_cur value=11.000000
// [RoboStride:param] idx=0x7019 name=mechPos value=3.364515
// [RoboStride:param] idx=0x701A name=iqf value=0.000000
// [RoboStride:param] idx=0x701B name=mechVel value=-0.011572
// [RoboStride:param] idx=0x701C name=VBUS value=24.843153
// [RoboStride:param] idx=0x701D name=rotation value=0.000000
// [RoboStride:param] idx=0x701E name=loc_kp value=30.000000
// [RoboStride:param] idx=0x701F name=spd_kp value=2.000000
// [RoboStride:param] idx=0x7020 name=spd_ki value=0.021000
// [RoboStride:param] idx=0x7021 name=spd_filt_gain value=0.050000
// [RoboStride:param] done success=19 fail=0

#define CAN2_NUM_MOTOR 2
#define CAN2_NUM_ROBOSTRIDE CAN2_NUM_MOTOR
#define CAN2_RESEND_INTERVAL_MS 10 // 約1ms間隔
#define CAN2_TIMEOUT_MS 1000 // 1000msでタイムアウトとみなす
#define CAN2_RS05_MIT_TORQUE 1.0 //Nm
#define CAN2_RS05_MIT_SPEED 3.14 // rad/s
#define CAN2_RS05_MIT_ACC 10.0 // rad/s^2
#define CAN2_RS05_INIT_RUN_MODE PosPP_control_mode
#define CAN2_RS05_ACC_LIMIT 100.0f // rad/s^2
#define CAN2_RS5_SET_GAIN true

volatile bool can2_waiting_reply = false;

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
RoboStride<CAN2, RX_SIZE_256, TX_SIZE_16> can2_motor[CAN2_NUM_MOTOR] = {
  RoboStride(&can2, 0x01, 0x01, (int)ActuatorType::ROBSTRIDE_05),
  RoboStride(&can2, 0x01, 0x02, (int)ActuatorType::ROBSTRIDE_05)
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

  // can2_motor[0].printAllParameters();
  // can2_motor[1].printAllParameters();

  delay(1000);

  while(true){
    if (can2_motor[0].init(CAN2_RS05_MIT_TORQUE, CAN2_RS05_MIT_SPEED, CAN2_RS05_MIT_ACC, CAN2_RS05_INIT_RUN_MODE, CAN2_RS5_SET_GAIN)) {
      Serial.println("RS5 1 Sucess");
      break;
    }
    delay(1000);
    Serial.println("RS5 1 Init Failed, retrying...");
  }
  Serial.println("CAN2 RoboStride 1 Init Success!");
  delay(1000);

  while(true){
    if (can2_motor[1].init(CAN2_RS05_MIT_TORQUE, CAN2_RS05_MIT_SPEED, CAN2_RS05_MIT_ACC, CAN2_RS05_INIT_RUN_MODE, CAN2_RS5_SET_GAIN)) {
      Serial.println("RS5 2 Sucess");
      break;
    }
    delay(1000);
    Serial.println("RS5 2 Init Failed, retrying...");
  }

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
  if((!can2_waiting_reply && (now_us - motor_last_send_us[robostride_control_i] >= CAN2_RESEND_INTERVAL_MS * 1000)) || //100Hz以上
      (now_us - motor_last_send_us[robostride_control_i]) >= CAN2_TIMEOUT_MS * 1000){ // 10Hz
      // or timeout
      can2_motor[robostride_control_i].writeCanFrame();
        can2_waiting_reply = true;
      robostride_control_i = (robostride_control_i + 1) % CAN2_NUM_MOTOR;
      // can3_damiao0_send_ts = millis();
      motor_last_send_us[robostride_control_i] = now_us;
  }
}
