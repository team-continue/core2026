#include "can3.h"
#include "client.h"
#include "led.h"
#include "pin.h"

void led_timer_cb();
IntervalTimer led_timer;

void setup() {
  Serial.begin(9600);

  // 非常停止
  pinMode(PIN_EMERGENCY, OUTPUT);
  // LED
  pinMode(LED_BUILTIN, OUTPUT);
  led1.init();
  led2.init();
  client.init();
  can3_init();

  led_timer.begin(led_timer_cb, 50000);//50msごとにled_timer_cbを呼び出す
}

void loop() {
  led1.update();
  led2.update();
}

void led_timer_cb() {
  static bool led = false;
  unsigned long now_ts = millis();
  client.update();
  if((now_ts - can3_receive_ts) >= 500){
    digitalWrite(LED_BUILTIN, HIGH);
  }else{
    led = !led;
    digitalWrite(LED_BUILTIN, led);
  }
}
