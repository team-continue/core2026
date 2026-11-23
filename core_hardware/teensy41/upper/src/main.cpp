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
// upper テープの区間。先頭 (Teensy の出力側) から 左の砲塔・全体・右の砲塔。境目は実機に合わせる
constexpr uint16_t UPPER_LED_LEFT_FIRST = 0;
constexpr uint16_t UPPER_LED_LEFT_COUNT = 14;
constexpr uint16_t UPPER_LED_OVERALL_FIRST = UPPER_LED_LEFT_FIRST + UPPER_LED_LEFT_COUNT;
constexpr uint16_t UPPER_LED_OVERALL_COUNT = 15;
constexpr uint16_t UPPER_LED_RIGHT_FIRST = UPPER_LED_OVERALL_FIRST + UPPER_LED_OVERALL_COUNT;
constexpr uint16_t UPPER_LED_RIGHT_COUNT = 14;
float upper_led_chase_speed_leds_per_sec = 100.0f;
float upper_led_multi_chase_speed_leds_per_sec = 10.0f;
float upper_led_blink_period_sec = 0.2f;
float upper_led_double_flash_period_sec = 0.5f;
uint8_t upper_led_center_out_multi_base_brightness = 0;
uint8_t upper_led_brightness = 32;
LedStrip upper_led_strip(LED_UPPER_SERIAL_PIN, upper_led_count, upper_led_brightness);
Led make_upper_led_segment(uint16_t first, uint16_t count) {
  return Led(upper_led_strip, first, count, 20,
             upper_led_chase_speed_leds_per_sec,
             upper_led_multi_chase_speed_leds_per_sec,
             upper_led_blink_period_sec,
             upper_led_double_flash_period_sec,
             upper_led_center_out_multi_base_brightness);
}
// 一方向に流れるパターンの向きは、ROS から指令ごとに受け取る (bit 7)
Led upper_led_overall = make_upper_led_segment(UPPER_LED_OVERALL_FIRST, UPPER_LED_OVERALL_COUNT);
Led upper_led_left = make_upper_led_segment(UPPER_LED_LEFT_FIRST, UPPER_LED_LEFT_COUNT);
Led upper_led_right = make_upper_led_segment(UPPER_LED_RIGHT_FIRST, UPPER_LED_RIGHT_COUNT);

// ROS から届いた LED 指令の区間
enum LedSegment : uint8_t {
  LED_UPPER_OVERALL = 0,  // LED_TAPE[0]
  LED_BOTTOM = 1,         // LED_TAPE[1]
  LED_UPPER_LEFT = 2,     // LED_TAPE[2] の下位 8 bit
  LED_UPPER_RIGHT = 3,    // LED_TAPE[2] の上位 8 bit
  LED_SEGMENT_NUM = 4,
};
uint8_t ros_led_pattern[LED_SEGMENT_NUM] = {Led::kOff, Led::kOff, Led::kOff, Led::kOff};
uint8_t ros_led_color[LED_SEGMENT_NUM] = {Led::kWhite, Led::kWhite, Led::kWhite, Led::kWhite};
bool ros_led_reverse[LED_SEGMENT_NUM] = {false, false, false, false};
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
  // data は LED_TAPE[0..2] の下位・上位 8 bit の並び。表示は update_led_outputs() で決める
  //   LED_TAPE[0], [1]: (パターン, 色の byte)。色の byte の下位 7 bit = 色、bit 7 = reverse
  //   LED_TAPE[2]: 左右の砲塔 1 byte ずつ。下位 4 bit = パターン、bit 4〜6 = 色、bit 7 = reverse
  //   reverse = 一方向に流れるパターンを区間の末尾から先頭へ流す
  if (id != 100 || data == nullptr || len < 6) {
    return;
  }
  ros_led_pattern[LED_UPPER_OVERALL] = data[0];
  ros_led_color[LED_UPPER_OVERALL] = data[1] & 0x7F;
  ros_led_reverse[LED_UPPER_OVERALL] = (data[1] & 0x80) != 0;
  ros_led_pattern[LED_BOTTOM] = data[2];
  ros_led_color[LED_BOTTOM] = data[3] & 0x7F;
  ros_led_reverse[LED_BOTTOM] = (data[3] & 0x80) != 0;
  ros_led_pattern[LED_UPPER_LEFT] = data[4] & 0x0F;
  ros_led_color[LED_UPPER_LEFT] = (data[4] >> 4) & 0x07;
  ros_led_reverse[LED_UPPER_LEFT] = (data[4] & 0x80) != 0;
  ros_led_pattern[LED_UPPER_RIGHT] = data[5] & 0x0F;
  ros_led_color[LED_UPPER_RIGHT] = (data[5] >> 4) & 0x07;
  ros_led_reverse[LED_UPPER_RIGHT] = (data[5] & 0x80) != 0;
}

// 接続の状態と ROS からの指令から、各区間の表示を決める。
// PC 側が表示を決められない段階 (リンクなし・受信なし・指令なし) はここで表示を出す。
// リンクなし・受信なしは全区間が同じ表示、指令なしは区間ごとに判断する。
void update_led_outputs(){
  const unsigned long now_ms = millis();
  if (now_ms - prev_ecat_link_check_ms >= ECAT_LINK_CHECK_INTERVAL_MS) {
    prev_ecat_link_check_ms = now_ms;
    ecat_link = ecat_link_up();
  }

  uint8_t pattern[LED_SEGMENT_NUM];
  uint8_t color[LED_SEGMENT_NUM];
  bool reverse[LED_SEGMENT_NUM];
  for (int i = 0; i < LED_SEGMENT_NUM; ++i) {
    if (!ecat_link || !connect_ros2) {
      // ケーブルが抜けた、または daemon から process data が来ない (PC が落ちた・起動中・daemon が停止)。
      // PC の電源が切れても NIC がリンクを保つことがあり、物理リンクだけでは PC の停止を見分けられないので、
      // フレームが来ないときもリンクなしと同じ表示にする
      pattern[i] = Led::kRainbowChase;
      color[i] = Led::kWhite;  // レインボー系は色を使わない
      reverse[i] = false;
    } else if (ros_led_pattern[i] == Led::kOff) {
      // EtherCAT は疎通しているが LED の指令がない (led_manager が未起動・停止)
      pattern[i] = Led::kFade;
      color[i] = Led::kGreen;
      reverse[i] = false;
    } else {
      pattern[i] = ros_led_pattern[i];
      color[i] = ros_led_color[i];
      reverse[i] = ros_led_reverse[i];
    }
  }
  upper_led_overall.write(
      pattern[LED_UPPER_OVERALL], color[LED_UPPER_OVERALL], reverse[LED_UPPER_OVERALL]);
  upper_led_left.write(pattern[LED_UPPER_LEFT], color[LED_UPPER_LEFT], reverse[LED_UPPER_LEFT]);
  upper_led_right.write(pattern[LED_UPPER_RIGHT], color[LED_UPPER_RIGHT], reverse[LED_UPPER_RIGHT]);
  // bottom には色の byte の bit 7 に reverse を載せたまま送る
  const uint8_t bottom_color_byte =
      static_cast<uint8_t>(color[LED_BOTTOM] | (reverse[LED_BOTTOM] ? 0x80 : 0x00));
  bottom_can3.setLedBytes(pattern[LED_BOTTOM], bottom_color_byte);
}

void setup(void) {
  // 非常停止
  pinMode(PIN_EMERGENCY, OUTPUT);
  digitalWrite(PIN_EMERGENCY, DEFAULT_EMERGENCY_STATE);
  // LED
  pinMode(LED_BUILTIN, OUTPUT);
  upper_led_strip.init();

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
  const unsigned long led_now_ms = millis();
  // 3 区間とも描いてから 1 回だけ送る (| は全区間の update を必ず呼ぶため)
  if (upper_led_overall.update(led_now_ms) | upper_led_left.update(led_now_ms) |
      upper_led_right.update(led_now_ms)) {
    upper_led_strip.show();
  }
  ecat_update();
  sts.loop();
  esc.loop();
  can3_loop();
  can2_loop();
}
