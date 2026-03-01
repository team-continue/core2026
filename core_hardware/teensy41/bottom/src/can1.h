#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "pin.h"
#include "client.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;//teensyのcan1を登録　RX_SIZE_256→受信buffer データをためとく　TX_SIZE_16→送信buffer

unsigned long can1_receive_ts = 0;
CAN_message_t can1_msg;

void can1_cb(const CAN_message_t &msg) {
  // Serial.println("ok");//動かすときはここをコメントアウトにしないとエラー
  if(msg.id == 0xff && msg.len == 0){//0x1fe//(torque)
      can1_msg.buf[0] = client.destoy;
      can1_msg.buf[1] = client.hp;
      can1.write(can1_msg);
    can1_receive_ts = millis();
  }
}

void can1_init(){
    can1_msg.id = 0;
    can1_msg.len = 2;

    can1.begin();
    can1.setBaudRate(1000000);//canの通信速度設定
    can1.setMaxMB(16);
    can1.enableFIFO();
    can1.enableFIFOInterrupt();
    can1.onReceive(can1_cb);
    can1.mailboxStatus();
}