#pragma once

#include <Arduino.h>
#include "Servo.h"
#include "pin.h"

#define MAX_SIGNAL 1400  //PWM信号における最大のパルス幅[マイクロ秒]
#define MIN_SIGNAL 1000  //PWM信号における最小のパルス幅[マイクロ秒]

Servo _esc;

class ESC{
  int _pin;
  public:
    ESC(int pin): _pin(pin){}
    void init(){
      _esc.attach(_pin);  // attaches the servo on pin 20
      // delay(1000);
      // Serial.println("Writing maximum output.");
      // _esc.writeMicroseconds(MAX_SIGNAL);  //ESCへ最大のパルス幅を指示します
      // Serial.println("Wait 2 seconds.");
      // delay(2000);
      // Serial.println("Writing minimum output");
      // _esc.writeMicroseconds(MIN_SIGNAL);  //ESCへ最小のパルス幅を指示します
      // Serial.println("Wait 2 seconds. Then motor starts");
      // delay(2000);
    }
    void write(float data){
      int value = data;
      value = constrain(value, MIN_SIGNAL, MAX_SIGNAL);
      _esc.writeMicroseconds(value);
    }
};

ESC esc= ESC(PIN_ESC2);
