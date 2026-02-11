#include "wiring.h"
#pragma once

#include "Servo.h"

#define MAX_SIGNAL 2000  //PWM信号における最大のパルス幅[マイクロ秒]
#define MIN_SIGNAL 1000  //PWM信号における最小のパルス幅[マイクロ秒]

Servo _esc;

class ESC{
  int _pin;
  public:
    ESC(int pin): _pin(pin){}
    void begin(){
      _esc.attach(_pin);  // attaches the servo on pin 20
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

#define ESC_NUM 2
#define ESC1_PIN 35
#define ESC2_PIN 24
ESC esc[ESC_NUM] = {ESC(ESC1_PIN), ESC(ESC2_PIN)};
void esc_init(){
  for(int i=0;i<ESC_NUM;++i){
    esc[i].begin();
    esc[i].init();
  }
}
