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
#define NUM_ROBOSTRIDE_CAN2 2
#define NUM_ROBOSTRIDE_CAN3 1
#define PIN_EMERGENCY 19
#define PORT_WIRELESS Serial5
#define LEN_WIRELESS 1024

#define ROBOSTRIDE_CURRENT_MAX 20.0 //A
#define ROBOSTRIDE_TORQUE_MAX 30.0 //Nm

#define DAMIAO_CAN_TIMEOUT 100 //ms
#define DAMIAO_CAN_CONTROL_PERIOD 1 //ms

Imu im(false);
ESC esc;

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
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

IntervalTimer main_timer, can3_timer, can2_timer;

Damiao<CAN3, RX_SIZE_256, TX_SIZE_16> damiao_motor[NUM_DAMIAO] = {
  Damiao(&can3, 0x11, 1),
  Damiao(&can3, 0x12, 2),
  Damiao(&can3, 0x13, 3),
  Damiao(&can3, 0x14, 4)
};

// RoboStride<CAN2, RX_SIZE_256, TX_SIZE_16> robostride_can2[NUM_ROBOSTRIDE_CAN2] = {
//   RoboStride(&can2, 0x01, 1, (int)ActuatorType::ROBSTRIDE_05),
//   RoboStride(&can2, 0x01, 2, (int)ActuatorType::ROBSTRIDE_05)
// };

RoboStride<CAN3, RX_SIZE_256, TX_SIZE_16> robostride_can3[NUM_ROBOSTRIDE_CAN3] = {
  RoboStride(&can3, 1, 1, (int)ActuatorType::ROBSTRIDE_06),
};

void main_timer_cb();
void can3_cb(const CAN_message_t &msg);

// PCから受信時に一度呼ばれるやつ
void packet_FrameCallBack(){
  // rosと接続中
  prev_connect_ros2_ts_ = millis();
  // PCに送信するデータを登録
  for(int i =0;i<NUM_DAMIAO;++i){
    float f[6] = {0, damiao_motor[i].feedback.torque_nm, damiao_motor[i].ref.velocity_rad_s, damiao_motor[i].feedback.velocity_rad_s, damiao_motor[i].ref.position_rad, damiao_motor[i].feedback.position_rad};
    packet_setFloat(i, f, 6);
  }
  for(int i =0;i<NUM_ROBOSTRIDE_CAN3;++i){
    float f[6] = {0, robostride_can3[i].feedback.torque_nm, robostride_can3[i].ref.velocity_rad_s, robostride_can3[i].feedback.velocity_rad_s, robostride_can3[i].ref.position_rad, robostride_can3[i].feedback.position_rad};
    packet_setFloat(i+4, f, 6);
  }
  // for(int i =0;i<NUM_ROBOSTRIDE_CAN2;++i){
  //   float f[6] = {0, robostride_can2[i].feedback.torque_nm, robostride_can2[i].ref.velocity_rad_s, robostride_can2[i].feedback.velocity_rad_s, robostride_can2[i].ref.position_rad, robostride_can2[i].feedback.position_rad};
  //   packet_setFloat(i+4+1, f, 6);
  // }

  // for(int i =0;i<LEN_SERVO;++i){
  //   float f[6] = {0, sts.servos[i].current, sts.ref_vel[i], sts.servos[i].vel, sts.ref_pos[i], sts.servos[i].pos};
  //   packet_setFloat(i+4+1+2, f, 6);
  // }

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
      robostride_can3[id-4].setPacketFrame(data, len);
      break;
    // case 5:
    // case 6:
    //   robostride_can2[id-5].setPacketFrame(data, len);
    //   break;
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
      if(len>=1){
        sts.ref_pos[id-7] = data[len-1];
        break;
      }
    case 15:
      if(len >= 1){
        if(data[0] < 0){
          // esc.init();
        }else{
          esc.write(data[0]);
        }  
      }
      break;
    case 16:
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
  // sts.init();
  esc.begin();
  // esc.init();

  // Robomas 用
  can3.begin();
  can3.setBaudRate(1000000);//canの通信速度設定
  can3.setMaxMB(16);
  can3.enableFIFO();

  for(int i=0;i<NUM_DAMIAO;++i){
    damiao_motor[i].init();
  }
  // for(int i=0;i<NUM_ROBOSTRIDE_CAN2;++i){
  //   robostride_can2[i].init(ROBOSTRIDE_CURRENT_MAX, ROBOSTRIDE_TORQUE_MAX);
  // }
  for(int i=0;i<NUM_ROBOSTRIDE_CAN3;++i){
    robostride_can3[i].init(ROBOSTRIDE_CURRENT_MAX, ROBOSTRIDE_TORQUE_MAX);
  }

  main_timer.begin(main_timer_cb, 10 * 1000); //100Hz
}

void main_timer_cb(){
  // sts.update();
    // timeout
  if(++counter2 == 5){
    counter2 = 0;
    // timeout
    if((millis() - prev_connect_ros2_ts_) >= 500){
      digitalWrite(LED_BUILTIN, HIGH);
      sts.disable = true;
      packet_send();
    }else{
      led = !led;
      digitalWrite(LED_BUILTIN, led);
      sts.disable = false;
    }
  }
}

void can2_timer_cb(){

}

bool can3_communicating = false;

void loop() {
  packet_update();

  static int damiao_counter = 0;
  static uint32_t can3_damiao_send_ts[5] = {0};

  uint32_t now_ts = micros();

  // 100Hzで送信
  if((!can3_communicating && (now_ts - can3_damiao_send_ts[damiao_counter] >= DAMIAO_CAN_CONTROL_PERIOD * 1000)) || //100Hz以上
      (now_ts - can3_damiao_send_ts[damiao_counter]) >= DAMIAO_CAN_TIMEOUT * 1000){ // 10Hz
      // or timeout
      if(damiao_counter < 4){
        damiao_motor[damiao_counter].writeCanFrame();
        can3_communicating = true;
      }else{
        robostride_can3[0].writeCanFrame();
      }
      damiao_counter = (damiao_counter + 1) % 5;
      // can3_damiao0_send_ts = millis();
      can3_damiao_send_ts[damiao_counter] = now_ts;
  }
  CAN_message_t msg;
  if(can3.read(msg)){
    for(int i=0;i<NUM_DAMIAO;++i){
      if(damiao_motor[i].readCanFrame(msg)){
        can3_communicating = false;
      }
    }
  }
}