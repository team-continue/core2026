#pragma once

#include <Arduino.h>
#include "Servo.h"
#include "pin.h"

#define MAX_SIGNAL 1400  //PWM信号における最大のパルス幅[マイクロ秒]
#define MIN_SIGNAL 1000  //PWM信号における最小のパルス幅[マイクロ秒]
#define CONTROL_MS 10      //ESCに指示を出す周期[ms]
#define ESC_ROS2_TIMEOUT_MS 200 // ROS2からの信号が切れたとみなすまでの時間[ms]

Servo _esc;

class ESC{
  int _pin;
  unsigned long _last_write_time = 0;
  unsigned long _control_ms = 10; //100Hz
  int value = 0;
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
      _last_write_time = millis();
    }
    void write(float data){
      value = data;
      _last_write_time = millis();
    }
    void loop(){
      // もし最後のwriteから1秒以上経過している場合、ESCに最小のパルス幅を指示します
      bool connect_ros2 = (millis() - _last_write_time) < ESC_ROS2_TIMEOUT_MS;
      if(!connect_ros2){
        value = 0;
      }
      if(millis() - _control_ms > CONTROL_MS){
        value = constrain(value, MIN_SIGNAL, MAX_SIGNAL);
        _esc.writeMicroseconds(value);
        _control_ms = millis();
      }
    }
};

ESC esc= ESC(PIN_ESC2);
