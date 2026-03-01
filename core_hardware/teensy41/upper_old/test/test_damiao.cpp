#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "damiao.h"

// =====================
// CAN (Teensy 4.x)
// =====================
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can1;

// =====================
// User config
// =====================
static constexpr uint8_t MASTER_ID = 0x01; // 実質未使用
static constexpr uint8_t SLAVE_ID  = 0x01; // Damiao モータID

Damiao<CAN3, RX_SIZE_256, TX_SIZE_16> dm(&Can1, MASTER_ID, SLAVE_ID);

// loop period
static constexpr uint32_t LOOP_DT_MS = 5;

// command mode
// 0: disable
// 2: velocity
// 3: position
// 4: gain update (one-shot)
static int g_cmd = 2;

// parameters
static float vel_amp  = 3.0f;   // rad/s
static float vel_freq = 0.2f;   // Hz

static float pos_amp  = 0.5f;   // rad
static float pos_freq = 0.1f;   // Hz

static float kp_asr = 0.0f;
static float ki_asr = 0.0f;
static float kp_apr = 0.0f;
static float ki_apr = 0.0f;

static uint32_t last_loop_ms  = 0;
static uint32_t last_print_ms = 0;

// =====================
// Serial command
// =====================
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
      Serial.println("[cmd] position");
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
      Serial.println("keys: 0=disable 1=velocity 2=position 3=gain +/-=vel_amp");
      break;
  }
}

// =====================
// Print feedback
// =====================
static void printStatus() {
  if (millis() - last_print_ms < 200) return;
  last_print_ms = millis();

  Serial.printf(
    "connect=%d status=0x%02X pos=%.4f vel=%.4f tq=%.4f temp_mos=%d temp_rot=%d\n",
    (int)dm.connect,
    (unsigned)dm.feedback.status,
    dm.feedback.position_rad,
    dm.feedback.velocity_rad_s,
    dm.feedback.torque_nm,
    (int)dm.feedback.temp_mos,
    (int)dm.feedback.temp_rotor
  );
}

// =====================
// setup
// =====================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("=== Damiao test (NO HOMING) ===");

  // CAN init
  Can1.begin();
  Can1.setBaudRate(1000000); // Damiao仕様に合わせる

  // Damiao init (flash read)
  if (!dm.init()) {
    Serial.println("Damiao init failed");
  } else {
    Serial.println("Damiao init OK");
  }

  Serial.println("keys: 0=disable 1=velocity 2=position 3=gain +/-=vel_amp");
}

// =====================
// loop
// =====================
void loop() {
  handleSerial();

  const uint32_t now = millis();
  if (now - last_loop_ms < LOOP_DT_MS) {
    return;
  }
  last_loop_ms = now;

  float data[4] = {0};
  int len = 0;

  const float t = 0.001f * (float)now;

  // -----------------
  // setPacketFrame
  // -----------------
  if (g_cmd == 0) {
    // disable
    len = 0;
    dm.setPacketFrame(data, len);
  }
  else if (g_cmd == 2) {
    // velocity
    const float vel = vel_amp * sinf(2.0f * (float)M_PI * vel_freq * t);
    data[1] = vel;
    len = 2;
    dm.setPacketFrame(data, len);
  }
  else if (g_cmd == 3) {
    // position
    const float pos = pos_amp * sinf(2.0f * (float)M_PI * pos_freq * t);
    const float vel = 2.0f * (float)M_PI * pos_freq * pos_amp
                      * cosf(2.0f * (float)M_PI * pos_freq * t);
    data[1] = vel;
    data[2] = pos;
    len = 3;
    dm.setPacketFrame(data, len);
  }
  else if (g_cmd == 4) {
    // gain update (one-shot)
    data[0] = kp_asr;
    data[1] = ki_asr;
    data[2] = kp_apr;
    data[3] = ki_apr;
    len = 4;
    dm.setPacketFrame(data, len);

    // 次周期から position に戻す
    g_cmd = 3;
  }

  // -----------------
  // writeCanFrame
  // -----------------
  dm.writeCanFrame();

  printStatus();
}
