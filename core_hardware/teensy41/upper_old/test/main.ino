#include <Arduino.h>
#include <Wire.h>
#include <FlexCAN_T4.h>

#include "pi.h"
#include "packet.h"
#include  "imu.h"
#include "feetech.h"
#include "esc.h"
#include "damiao.h"
#include "robostride.h"

#define NUM_DAMIAO 4
#define NUM_ROBOSTRIDE 1
#define PIN_EMERGENCY 19
#define PORT_WIRELESS Serial5
#define LEN_WIRELESS 1024

Imu im(false);
ESC esc;

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;//teensyのcan1を登録　RX_SIZE_256→受信buffer データをためとく　TX_SIZE_16→送信buffer

CAN_message_t msg2, msg3;
STS sts;
unsigned long prev_connect_ros2_ts_=0;
uint8_t wireless[LEN_WIRELESS] = {0};
uint8_t hardware_enable[1] = {0};
uint8_t destory[1] = {0};
uint8_t damege[1] = {0};
int quat[3] = {0};
unsigned long prev_ts = 0;
int counter1 = 0, counter2 = 0, led=0;
int len_wireless = 0;
uint8_t wirelessbuffer_[MAX_DATA_LENGTH];

IntervalTimer main_timer;

Damiao<CAN3, RX_SIZE_256, TX_SIZE_16> damiao_motor[NUM_DAMIAO] = {
  Damiao(&can3, 0x00, 1),
  Damiao(&can3, 0x00, 2),
  Damiao(&can3, 0x00, 3),
  Damiao(&can3, 0x00, 4)
};

RoboStride<CAN3, RX_SIZE_256, TX_SIZE_16> robostride_motor[NUM_ROBOSTRIDE] = {
  RoboStride(&can3, 0x01, 1, 6)
};

// PCから受信時に一度呼ばれるやつ
void packet_FrameCallBack(){
  // rosと接続中
  prev_connect_ros2_ts_ = millis();
  // PCに送信するデータを登録
  for(int i =0;i<NUM_DAMIAO;++i){
    float f[6] = {0, damiao_motor[i].feedback.torque_nm, damiao_motor[i].ref.velocity_rad_s, damiao_motor[i].feedback.velocity_rad_s, damiao_motor[i].ref.position_rad, damiao_motor[i].feedback.position_rad};
    packet_setFloat(i, f, 6);
  }
  for(int i =0;i<NUM_ROBOSTRIDE;++i){
    float f[6] = {0, robostride_motor[i].feedback.torque_nm, robostride_motor[i].ref.velocity_rad_s, robostride_motor[i].feedback.velocity_rad_s, robostride_motor[i].ref.position_rad, robostride_motor[i].feedback.position_rad};
    packet_setFloat(4+i, f, 6);
  }

  for(int i =0;i<LEN_SERVO;++i){
    float f[6] = {0, sts.servos[i].current, sts.ref_vel[i], sts.servos[i].vel, sts.ref_pos[i], sts.servos[i].pos};
    
    packet_setFloat(i+5, f, 6);
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
  // imu
  if(im.getQuat(quat)){
    packet_setInt32(103, quat, 3); 
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
      damiao_motor[id].setPacketFrame(data, len);
      break;
    case 4:
      robostride_motor[0].setPacketFrame(data, len);
      break;
    case 5:
    case 6:
    case 7:
    case 8:
      if(len>=1){
        sts.ref_pos[id-5] = data[len-1];
        break;
      }
    case 9:
      if(len >= 1){
        if(data[0] < 0){
          // esc.init();
        }else{
          esc.write(data[0]);
        }  
      }
      break;
    case 10:
      if(len>=1)
        digitalWrite(PIN_EMERGENCY, data[1] != 0.f);
      break;
    default:
      break;
  }
}

void setup(void) {
  // 非常停止
  pinMode(PIN_EMERGENCY, OUTPUT);
  // LED
  pinMode(LED_BUILTIN, OUTPUT);

  packet_begin();
  PORT_WIRELESS.begin(115200);
  PORT_WIRELESS.addMemoryForRead(&wirelessbuffer_, sizeof(wirelessbuffer_));
  im.init();
  sts.init();
  esc.begin();
  // esc.init();

  // Robomas 用
  can3.begin();
  can3.setBaudRate(1000000);//canの通信速度設定
  can3.setMaxMB(16);
  can3.enableFIFO();
  can3.mailboxStatus();

  for(int i=0;i<NUM_DAMIAO;++i){
    damiao_motor[i].init();
  }

  for(int i=0;i<NUM_ROBOSTRIDE;++i){
    robostride_motor[i].init();
  }

  main_timer.begin(main_timer_cb, 100); //100Hz
}

void main_timer_cb() {
  for(int i=0;i<NUM_DAMIAO;++i){
    damiao_motor[i].writeCanFrame();
  }
  for(int i=0;i<NUM_ROBOSTRIDE;++i){
    robostride_motor[i].writeCanFrame();
  }

  sts.update();
    // timeout
  if(++counter2 == 5){
    counter2 = 0;
    // timeout
    if((millis() - prev_connect_ros2_ts_) >= 500){
      digitalWrite(LED_BUILTIN, HIGH);
      packet_send();
      sts.disable = true;
    }else{
      led = !led;
      digitalWrite(LED_BUILTIN, led);
      sts.disable = false;
    }
  }
}

void loop() {
  packet_update();
}