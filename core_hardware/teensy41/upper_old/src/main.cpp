#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "robostride.h"   // あなたの RoboStride クラスが入ってるヘッダ

// ========= CAN (Teensy 4.x: CAN1 example) =========
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;

// ========= User config =========
static constexpr uint8_t MASTER_ID = 0x01;  // host id
static constexpr uint8_t MOTOR_ID  = 0x01;  // target motor id
static constexpr int ACTUATOR_TYPE = (int)ActuatorType::ROBSTRIDE_06;

// RoboStride instance
RoboStride<CAN2, RX_SIZE_256, TX_SIZE_16> rs(&Can1, MASTER_ID, MOTOR_ID, ACTUATOR_TYPE);

// loop period
static constexpr uint32_t LOOP_DT_MS = 5;

// test mode
// 0:disable (len=0/1)
// 2:velocity (len=2)
// 3:motion   (len=3)
// 4:gains    (len=4)  ※あなたの実装は gain更新はmodeを変えない（ref.modeはそのまま）なので、
//                      “一回だけ送って、次の周期から別モード”みたいに使う
static int g_cmd = 2;

// params
static float vel_amp  = 3.0f;   // rad/s
static float vel_freq = 0.2f;   // Hz
static float pos_amp  = 0.5f;   // rad
static float pos_freq = 0.1f;   // Hz

static float gain_cur_kp = 0.125f; // 例
static float gain_cur_ki = 0.0158f;
static float gain_kp     = 500.0f;
static float gain_kd     = 5.0f;

static uint32_t last_loop_ms = 0;
static uint32_t last_print_ms = 0;

static void handleSerial() {
  if (!Serial.available()) return;
  char c = (char)Serial.read();

  switch (c) {
    case '0':
      g_cmd = 0;
      Serial.println("[cmd] disable");
      break;
    case '1':
      g_cmd = 2;
      Serial.println("[cmd] velocity");
      break;
    case '2':
      g_cmd = 3;
      Serial.println("[cmd] motion");
      break;
    case '3':
      g_cmd = 4;
      Serial.println("[cmd] gain update (one-shot)");
      break;

    case '+':
      vel_amp += 0.5f;
      Serial.printf("[cmd] vel_amp=%.2f\n", vel_amp);
      break;
    case '-':
      vel_amp = max(0.0f, vel_amp - 0.5f);
      Serial.printf("[cmd] vel_amp=%.2f\n", vel_amp);
      break;

    default:
      Serial.println("keys: 0=disable 1=velocity 2=motion 3=gain  +/-=vel_amp");
      break;
  }
}

static void printStatus() {
  if (millis() - last_print_ms < 200) return;
  last_print_ms = millis();

  Serial.printf("connect=%d  mode=%d Rpos=%.4f Rvel=%.4f  Spos=%.4f  Svel=%.4f  Stq=%.4f  temp=%.1f\n",
                (int)rs.connect,
                rs.ref.mode,
                rs.ref.position_rad,
                rs.ref.velocity_rad_s,
                rs.feedback.position_rad,
                rs.feedback.velocity_rad_s,
                rs.feedback.torque_nm,
                rs.feedback.temp_mos);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("=== RoboStride test: init/setPacketFrame/writeCanFrame ===");

  // CAN init
  Can1.begin();
  Can1.setBaudRate(1000000); // 1Mbps

  // RoboStride init (内部で stop + 受信試行など)
  rs.init(10, 10);

  Serial.println("keys: 0=disable 1=velocity 2=motion 3=gain  +/-=vel_amp");
}

void loop() {
  handleSerial();

  const uint32_t now = millis();
  if (now - last_loop_ms < LOOP_DT_MS) {
    // 余り時間は受信を回してもOK（writeCanFrame内でも回るが補助）
    // rs.receive_status_frame();
    return;
  }
  last_loop_ms = now;

  // ---- setPacketFrame 用のデータ作成 ----
  // あなたの setPacketFrame() の仕様:
  // len=0/1: disable
  // len=2: velocity  data[1]=vel
  // len=3: motion    data[1]=vel, data[2]=pos
  // len=4: gain set  data[0]=cur_kp, data[1]=cur_ki, data[2]=motion_kp, data[3]=motion_kd
  float data[4] = {0};

  const float t = 0.001f * (float)now;

  int len = 0;

  if (g_cmd == 0) {
    // disable
    len = 0;
    rs.setPacketFrame(data, len);
  }
  else if (g_cmd == 2) {
    // velocity mode
    const float vel = 3.14;
    data[1] = vel;
    len = 2;
    rs.setPacketFrame(data, len);
  }
  else if (g_cmd == 3) {
    // motion mode: pos + vel
    const float pos = pos_amp * sinf(2.0f * (float)M_PI * pos_freq * t);
    const float vel = 2.0f * (float)M_PI * pos_freq * pos_amp * cosf(2.0f * (float)M_PI * pos_freq * t);

    data[1] = vel;
    data[2] = pos;
    len = 3;
    rs.setPacketFrame(data, len);
  }
  else if (g_cmd == 4) {
    // gain update (one-shot)
    data[0] = gain_cur_kp;
    data[1] = gain_cur_ki;
    data[2] = gain_kp;
    data[3] = gain_kd;
    len = 4;
    rs.setPacketFrame(data, len);

    // 次周期からは motion に戻す（好みで velocity にしてもOK）
    g_cmd = 3;
  }

  // ---- writeFrame（= writeCanFrame） ----
  rs.writeCanFrame();

  // ---- optional: feedback update assist ----
//   rs.receive_status_frame();

  printStatus();
}
