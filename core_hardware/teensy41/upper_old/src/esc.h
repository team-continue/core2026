#include "wiring.h"
#pragma once

#include "Servo.h"

#define MAX_SIGNAL 2000  //PWM信号における最大のパルス幅[マイクロ秒]
#define MIN_SIGNAL 1000  //PWM信号における最小のパルス幅[マイクロ秒]
#define ESC_PIN 24  //ESCへの出力ピン

Servo _esc;

class ESC{
  public:
    ESC(){}
    void begin(){
      _esc.attach(ESC_PIN);  // attaches the servo on pin 20
    }
    void init(){
      delay(1000);
      // Serial.println("Writing maximum output.");
      _esc.writeMicroseconds(MAX_SIGNAL);  //ESCへ最大のパルス幅を指示します
      // Serial.println("Wait 2 seconds.");
      delay(2000);
      // Serial.println("Writing minimum output");
      _esc.writeMicroseconds(MIN_SIGNAL);  //ESCへ最小のパルス幅を指示します
      // Serial.println("Wait 2 seconds. Then motor starts");
      delay(2000);
    }
    void write(float data){
      int value = data;
      value = constrain(value, MIN_SIGNAL, MAX_SIGNAL);
      _esc.writeMicroseconds(value);
    }
};