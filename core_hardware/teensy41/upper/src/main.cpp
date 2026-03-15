#include <Arduino.h>
#include <Wire.h>
#include <FlexCAN_T4.h>

#include "pi.h"
#include "ecat.h"
#include "feetech.h"
#include "can3.h"
#include "can2.h"
#include "esc.h"
#include "damiao.h"
#include "robostride.h"
#include "led.h"
#include "pin.h"
#include "wireless.h"

#define DEFAULT_EMERGENCY_STATE HIGH

STS sts;
unsigned long prev_connect_ros2_ts_=0;
bool connect_ros2 = false;
uint8_t wireless_data[LEN_WIRELESS] = {0};
uint8_t hardware_enable[1] = {0};
uint8_t destory[1] = {0};
uint8_t damege[1] = {0};
unsigned long prev_ts = 0;
int counter1 = 0, counter2 = 0, led=0;
int len_wireless = 0;
WirelessModule wireless;
Led upper_led(LED_UPPER_SERIAL_PIN, 1, 20);

// LED timer
void led_timer_cb();
IntervalTimer led_timer;

// PCから受信時に一度呼ばれるやつ
void ecat_FrameCallBack(){
  // rosと接続中
  prev_connect_ros2_ts_ = millis();
  // PCに送信するデータを登録
  for(int i =0;i<CAN3_NUM_MOTOR;++i){
    float f[6] = {
      0,
      can3_motor[i]->motor_state.torque_nm,
      can3_motor[i]->motor_ref.velocity_rad_s,
      can3_motor[i]->motor_state.velocity_rad_s,
      can3_motor[i]->motor_ref.position_rad,
      can3_motor[i]->motor_state.position_rad
    };
    ecat_setFloat(i, f, 6);
  }
  for(int i =0;i<CAN2_NUM_MOTOR;++i){
    float f[6] = {
      0,
      can2_motor[i].feedback.torque_nm,
      can2_motor[i].ref.velocity_rad_s,
      can2_motor[i].feedback.velocity_rad_s,
      can2_motor[i].ref.position_rad,
      can2_motor[i].feedback.position_rad
    };
    ecat_setFloat(i + CAN3_NUM_MOTOR, f, 6);
  }

  for(int i =0;i<LEN_SERVO;++i){
    float f[6] = {
      0,
      sts.servos[i].current,
      sts.servos[i].ref_vel,
      sts.servos[i].vel,
      sts.servos[i].ref_pos,
      sts.servos[i].pos
    };
    ecat_setFloat(i+CAN3_NUM_MOTOR+CAN2_NUM_MOTOR, f, 6);
  }

  destory[0] = bottom_can3.destroy() ? 1 : 0;
  damege[0] = bottom_can3.hp();
  // damege
  ecat_setUint8(100, damege, 1); 
  // destory
  ecat_setUint8(101, destory, 1);
  // wireless
  if (wireless.update(wireless_data)) {
    ecat_setUint8(102, wireless_data, LEN_WIRELESS);
  }
  hardware_enable[0] = damiao_motor[0].connect ? 0 : 1;
  ecat_setUint8(104, hardware_enable, 1);
}

// // PCから受信時にパケットごとに呼ばれるやつ
void ecat_PacketCallBack(const uint8_t id, const float *data, const size_t len){
  switch(id){
    case 0:
    case 1:
    case 2:
    case 3:
    // case 4:
      can3_motor[id]->setPacketFrame(data, len);
      break;
    case 5:
    case 6:
      can2_motor[id-5].setPacketFrame(data, len);
      break;
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
      if(len>=1){
        sts.setRefPos(id - 7, data[len - 1]);
      }
      break;
    case 15:
    // case 16:
      if(len >= 1){
        if(data[0] < 0){
          // esc.init();
        }else{
          esc.write(data[len - 1]);
        }
      }
      break;
    // case 17:
    //   if(len>=1){
    //     digitalWrite(PIN_EMERGENCY, data[len-1] != 0.f);
    //   }
      break;
    default:
      break;
  }
}

void ecat_PacketCallBack(const uint8_t id, const uint8_t *data, const uint8_t len) {
  if (id != 100 || data == nullptr || len < 3) {
    return;
  }

  upper_led.write(data[0]);
  bottom_can3.setLedBytes(data[1], data[2]);
}

void setup(void) {
  // 非常停止
  pinMode(PIN_EMERGENCY, OUTPUT);
  digitalWrite(PIN_EMERGENCY, DEFAULT_EMERGENCY_STATE);
  // LED
  pinMode(LED_BUILTIN, OUTPUT);
  upper_led.init();

  ecat_begin();
  wireless.init();
  sts.init();
  can3_init();
  can2_init();
  esc.init();

  led_timer.begin(led_timer_cb, 50 * 1000); //20Hz
}

void led_timer_cb(){
  // ros2と接続しているか確認
  connect_ros2 = (millis() - prev_connect_ros2_ts_) < 500;
  if(!connect_ros2){
    digitalWrite(LED_BUILTIN, HIGH);
    sts.disable = true;
  }else{
    led = !led;
    digitalWrite(LED_BUILTIN, led);
    sts.disable = false;
  }
}

void loop() {
  upper_led.update();
  ecat_update();
  sts.loop();
  can3_loop();
  can2_loop();
}
