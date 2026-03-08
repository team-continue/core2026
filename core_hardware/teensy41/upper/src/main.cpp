#include <Arduino.h>
#include <Wire.h>
#include <FlexCAN_T4.h>

#include "pi.h"
#include "packet.h"
#include "feetech.h"
#include "can3.h"
#include "can2.h"
#include "esc.h"
#include "damiao.h"
#include "robostride.h"
#include "pin.h"

#define DEFAULT_EMERGENCY_STATE HIGH

STS sts;
unsigned long prev_connect_ros2_ts_=0;
bool connect_ros2 = false;
uint8_t wireless[LEN_WIRELESS] = {0};
uint8_t hardware_enable[1] = {0};
uint8_t destory[1] = {0};
uint8_t damege[1] = {0};
unsigned long prev_ts = 0;
int counter1 = 0, counter2 = 0, led=0;
int len_wireless = 0;
uint8_t wirelessbuffer_[MAX_DATA_LENGTH];

// LED timer
void led_timer_cb();
IntervalTimer led_timer;

// PCから受信時に一度呼ばれるやつ
void packet_FrameCallBack(){
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
    packet_setFloat(i, f, 6);
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
    packet_setFloat(i + CAN3_NUM_MOTOR, f, 6);
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
    packet_setFloat(i+CAN3_NUM_MOTOR+CAN2_NUM_MOTOR, f, 6);
  }

  // damege
  packet_setUint8(100, damege, 1); 
  // destory
  packet_setUint8(101, destory, 1);
  // wireless
  while(PORT_WIRELESS.available()){
    wireless[len_wireless] = PORT_WIRELESS.read();
    if(wireless[len_wireless] == '\n'){
      packet_setUint8(102, wireless, len_wireless);
      len_wireless = 0;
    }
    if(len_wireless == LEN_WIRELESS-2){
      len_wireless = 0;
    }
    len_wireless++;
  }
  hardware_enable[0] = damiao_motor[0].connect ? 0 : 1;
  packet_setUint8(104, hardware_enable, 1);
  packet_send();
}

// // PCから受信時にパケットごとに呼ばれるやつ
void packet_PacketCallBack(const uint8_t id, const float *data, const size_t len){
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
        sts.servos[id - 7].ref_pos = data[len - 1];
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
    case 17:
      if(len>=1){
        digitalWrite(PIN_EMERGENCY, data[len-1] != 0.f);
      }
      break;
    default:
      break;
  }
}

void setup(void) {
  // 非常停止
  pinMode(PIN_EMERGENCY, OUTPUT);
  digitalWrite(PIN_EMERGENCY, DEFAULT_EMERGENCY_STATE);
  // LED
  pinMode(LED_BUILTIN, OUTPUT);

  packet_begin();
  PORT_WIRELESS.begin(115200);
  PORT_WIRELESS.addMemoryForRead(&wirelessbuffer_, sizeof(wirelessbuffer_));
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
    packet_send();
  }else{
    led = !led;
    digitalWrite(LED_BUILTIN, led);
    sts.disable = false;
  }
}

void loop() {
  packet_update();
  sts.loop();
  can3_loop();
  can2_loop();
}