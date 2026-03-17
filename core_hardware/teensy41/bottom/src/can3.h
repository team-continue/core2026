#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "pin.h"
#include "client.h"
#include "led.h"

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;//teensyのcan3を登録　RX_SIZE_256→受信buffer データをためとく　TX_SIZE_16→送信buffer
Led led1(LED1_SERIAL_PIN, 1, 20);
Led led2(LED2_SERIAL_PIN, 1, 20);

unsigned long can3_receive_ts = 0;
CAN_message_t can3_msg;

void can3_cb(const CAN_message_t &msg) {
  // Serial.println("ok");//動かすときはここをコメントアウトにしないとエラー
  if(msg.id == 0xff && msg.len == 2){// LED_TAPE[1], LED_TAPE[2]
      led1.write(msg.buf[0]);
      led2.write(msg.buf[1]);
      can3_msg.buf[0] = client.destoy;
      can3_msg.buf[1] = client.hp;
      can3.write(can3_msg);
    can3_receive_ts = millis();
  }
}

void can3_init(){
    can3_msg.id = 0;
    can3_msg.len = 2;

    can3.begin();
    can3.setBaudRate(1000000);//canの通信速度設定
    can3.setMaxMB(16);
    can3.enableFIFO();
    can3.enableFIFOInterrupt();
    can3.onReceive(can3_cb);
    can3.mailboxStatus();
}
