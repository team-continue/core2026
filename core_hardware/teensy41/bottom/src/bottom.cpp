#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "client.h"

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can1;//teensyのcan1を登録　RX_SIZE_256→受信buffer データをためとく　TX_SIZE_16→送信buffer
Client client;

unsigned long prev_connect_upper_ts = 0;
unsigned long prev_ts = 0;

#define PIN_EMERGENCY 19
#define PIN_DESTROY 15

CAN_message_t msg2;
bool led = false;
int count = 0;

void setup() {
  Serial.begin(9600);
  client.init();
  // 非常停止
  pinMode(PIN_EMERGENCY, OUTPUT);
  // LED
  pinMode(LED_BUILTIN, OUTPUT);
  // 撃破信号
  pinMode(PIN_DESTROY, INPUT);

  // Robomas 用
  can1.begin();
  can1.setBaudRate(1000000);//canの通信速度設定
  can1.setMaxMB(16);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(robomas_cb);
  can1.mailboxStatus();

  prev_connect_upper_ts = millis();
}

void robomas_cb(const CAN_message_t &msg) {
  // Serial.println("ok");//動かすときはここをコメントアウトにしないとエラー
  if(msg.id == 0x1ff && msg.len == 8){//0x1fe//(torque)
    if(++count == 50){
      count = 0;
      msg2.id = 0;
      msg2.len = 2;
      msg2.buf[0] = digitalRead(PIN_DESTROY);
      msg2.buf[1] = client.hp;
      can1.write(msg2);
    }
    prev_connect_upper_ts = millis();
  }
}

void loop() {
  unsigned long now_ts = millis();
  if(now_ts - prev_ts >= 50){
    prev_ts = now_ts;
    client.update();
    if((now_ts - prev_connect_upper_ts) >= 500){
      digitalWrite(LED_BUILTIN, HIGH);
    }else{
      led = !led;
      digitalWrite(LED_BUILTIN, led);
    }
  }
}
