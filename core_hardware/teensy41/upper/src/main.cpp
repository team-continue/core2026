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

#define WIRELESS_TIMEOUT_MS 3000

STS sts;
unsigned long prev_connect_ros2_ts_=0;
// led_timer_cb (割り込み) で書き、loop() で読む
volatile bool connect_ros2 = false;
bool connect_bottom = false;
bool connect_wireless = false;
bool hardware_emergency = false;
unsigned long wireless_prev_connect_ms = 0;
uint8_t wireless_data[LEN_WIRELESS] = {0};
uint8_t hardware_enable[1] = {0};
uint8_t destory[1] = {0};
uint8_t damege[1] = {0};
unsigned long prev_ts = 0;
int led=0;
WirelessModule wireless;
uint16_t upper_led_count = 43;
float upper_led_chase_speed_leds_per_sec = 100.0f;
float upper_led_multi_chase_speed_leds_per_sec = 10.0f;
float upper_led_blink_period_sec = 0.2f;
float upper_led_double_flash_period_sec = 0.5f;
uint8_t upper_led_center_out_multi_base_brightness = 0;
uint8_t upper_led_brightness = 32;
Led upper_led(LED_UPPER_SERIAL_PIN, upper_led_count, 20,
              upper_led_chase_speed_leds_per_sec,
              upper_led_multi_chase_speed_leds_per_sec,
              upper_led_blink_period_sec,
              upper_led_double_flash_period_sec,
              upper_led_center_out_multi_base_brightness,
              upper_led_brightness);

// ROS から届いた LED 指令。[0] = upper テープ、[1] = bottom テープ
uint8_t ros_led_pattern[2] = {Led::kOff, Led::kOff};
uint8_t ros_led_color[2] = {Led::kWhite, Led::kWhite};
#define ECAT_LINK_CHECK_INTERVAL_MS 50
bool ecat_link = false;
unsigned long prev_ecat_link_check_ms = 0;
void update_led_outputs();

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
    ecat_setUint8(102, wireless_data, 7);
    wireless_prev_connect_ms = millis();
  }
  uint8_t color_payload[1] = { bottom_can3.color() };
  ecat_setUint8(103, color_payload, 1);
  hardware_enable[0] = hardware_emergency ? 0 : 1;
  ecat_setUint8(104, hardware_enable, 1);
}

// // PCから受信時にパケットごとに呼ばれるやつ
void ecat_PacketCallBack(const uint8_t id, const float *data, const size_t len){
  // 下の基板に接続できない場合は，命令を処理しない -> すべて停止
  // TODO: 無線機器の導入後に connect_wireless の条件を戻す。
  if(!connect_bottom /* || !connect_wireless */){
    return;
  }
  switch(id){
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
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
        esc.write(data[len - 1]);
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
  // data は LED_TAPE[0..2] の (パターン, 色) の並び。表示は update_led_outputs() で決める
  if (id != 100 || data == nullptr || len < 4) {
    return;
  }
  ros_led_pattern[0] = data[0];
  ros_led_color[0] = data[1];
  ros_led_pattern[1] = data[2];
  ros_led_color[1] = data[3];
}

// 接続の状態と ROS からの指令から、上下のテープの表示を決める。
// PC 側が表示を決められない段階 (リンクなし・受信なし・指令なし) はここで表示を出す。
void update_led_outputs(){
  const unsigned long now_ms = millis();
  if (now_ms - prev_ecat_link_check_ms >= ECAT_LINK_CHECK_INTERVAL_MS) {
    prev_ecat_link_check_ms = now_ms;
    ecat_link = ecat_link_up();
  }

  uint8_t pattern[2];
  uint8_t color[2];
  for (int i = 0; i < 2; ++i) {
    if (!ecat_link) {
      // PC が落ちた、またはケーブルが抜けた
      pattern[i] = Led::kBlinkSlow;
      color[i] = Led::kBlue;
    } else if (!connect_ros2) {
      // PC は起動しているが daemon / core_hardware から受信していない
      pattern[i] = Led::kFade;
      color[i] = Led::kBlue;
    } else if (ros_led_pattern[i] == Led::kOff) {
      // EtherCAT は疎通しているが LED の指令がない (led_manager が未起動・停止)
      pattern[i] = Led::kFade;
      color[i] = Led::kGreen;
    } else {
      pattern[i] = ros_led_pattern[i];
      color[i] = ros_led_color[i];
    }
  }
  upper_led.write(pattern[0], color[0]);
  bottom_can3.setLedBytes(pattern[1], color[1]);
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
  const uint32_t now_ms = millis();
  // ros2と接続しているか確認
  connect_ros2 = (now_ms - prev_connect_ros2_ts_) < 500;
  connect_bottom = bottom_can3.isCanConnected(now_ms);
  connect_wireless = (now_ms - wireless_prev_connect_ms) < WIRELESS_TIMEOUT_MS;
  // TODO: 無線機器の導入後に connect_wireless の条件を戻す。
  hardware_emergency = (!connect_ros2 || !connect_bottom /* || !connect_wireless */);
  if(hardware_emergency){
    digitalWrite(LED_BUILTIN, HIGH);
    sts.disable = true;
  }else{
    led = !led;
    digitalWrite(LED_BUILTIN, led);
    sts.disable = false;
  }
}

void loop() {
  update_led_outputs();
  upper_led.update();
  ecat_update();
  sts.loop();
  esc.loop();
  can3_loop();
  can2_loop();
}
