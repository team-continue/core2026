#include "can2.h"

// test mode
// 0:disable (len=0/1)
// 2:velocity (len=2)
// 3:motion   (len=3)
// 4:gains    (len=4)  ※あなたの実装は gain更新はmodeを変えない（ref.modeはそのまま）なので、
//                      “一回だけ送って、次の周期から別モード”みたいに使う
static int g_cmd = 3;

// params
static float target_vel = 3.0f;   // rad/s
static float target_pos = 0.0f;   // rad

static float gain_cur_kp = 0.125f; // 例
static float gain_cur_ki = 0.0158f;
static float gain_kp     = 500.0f;
static float gain_kd     = 5.0f;

static uint32_t last_print_ms = 0;
static char serial_line[64] = {0};
static size_t serial_line_len = 0;

static void printHelp() {
  Serial.println("keys: 0=disable 1=velocity 2=motion 3=gain +/-=vel");
  Serial.println("cmd : p <rad> (set motion target position)");
}

static bool handleSingleKey(const char key) {
  switch (key) {
    case '0':
      g_cmd = 0;
      Serial.println("[cmd] disable");
      return true;
    case '1':
      g_cmd = 2;
      Serial.printf("[cmd] velocity vel=%.2f\n", target_vel);
      return true;
    case '2':
      g_cmd = 3;
      Serial.printf("[cmd] motion pos=%.3f vel=%.2f\n", target_pos, target_vel);
      return true;
    case '3':
      g_cmd = 4;
      Serial.println("[cmd] gain update (one-shot)");
      return true;
    case '+':
      target_vel += 0.5f;
      Serial.printf("[cmd] target_vel=%.2f\n", target_vel);
      return true;
    case '-':
      target_vel = max(0.0f, target_vel - 0.5f);
      Serial.printf("[cmd] target_vel=%.2f\n", target_vel);
      return true;
    default:
      return false;
  }
}

static void handleSerial() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r') continue;

    // Keep existing one-key workflow without requiring newline.
    if (serial_line_len == 0 && !Serial.available() && handleSingleKey(c)) {
      continue;
    }

    if (c != '\n') {
      if (serial_line_len < (sizeof(serial_line) - 1)) {
        serial_line[serial_line_len++] = c;
      }
      continue;
    }

    serial_line[serial_line_len] = '\0';
    if (serial_line_len == 0) {
      continue;
    }

    if (serial_line_len == 1) {
      if (!handleSingleKey(serial_line[0])) {
        printHelp();
      }
    } else {
      float pos = 0.0f;
      if (sscanf(serial_line, "p %f", &pos) == 1 || sscanf(serial_line, "p%f", &pos) == 1) {
        target_pos = pos;
        g_cmd = 3;
        Serial.printf("[cmd] target_pos=%.3f rad (motion)\n", target_pos);
      } else {
        printHelp();
      }
    }

    serial_line_len = 0;
  }
}

static void printStatus() {
  if (millis() - last_print_ms < 200) return;
  last_print_ms = millis();

  Serial.printf("connect_ros2=%d  mode=%d, pos=%.4f  vel=%.4f  tq=%.4f  temp=%.1f, Kp=%.2f, Ki=%.2f\n",
                (int)can2_robostride[0].connect_ros2,
                can2_robostride[0].ref.mode,
                can2_robostride[0].feedback.position_rad,
                can2_robostride[0].feedback.velocity_rad_s,
                can2_robostride[0].feedback.torque_nm,
                can2_robostride[0].feedback.temp_mos,
                can2_robostride[0].ref.vel_kp,
                can2_robostride[0].ref.vel_ki);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("=== RoboStride test: init/setPacketFrame/writeCanFrame ===");

  can2_init();

  printHelp();
}

void loop() {
  handleSerial();
  printStatus();
  can2_loop();

  // ---- setPacketFrame 用のデータ作成 ----
  // あなたの setPacketFrame() の仕様:
  // len=0/1: disable
  // len=2: velocity  data[1]=vel
  // len=3: motion    data[1]=vel, data[2]=pos
  // len=4: gain set  data[0]=cur_kp, data[1]=cur_ki, data[2]=motion_kp, data[3]=motion_kd
  float data[4] = {0};

  int len = 0;

  if (g_cmd == 0) {
    // disable
    len = 0;
    can2_robostride[0].setPacketFrame(data, len);
  }
  else if (g_cmd == 2) {
    // velocity mode
    data[1] = target_vel;
    len = 2;
    can2_robostride[0].setPacketFrame(data, len);
  }
  else if (g_cmd == 3) {
    // motion mode: pos + vel
    data[1] = target_vel;
    data[2] = target_pos;
    len = 3;
    can2_robostride[0].setPacketFrame(data, len);
  }
  else if (g_cmd == 4) {
    // gain update (one-shot)
    data[0] = gain_cur_kp;
    data[1] = gain_cur_ki;
    data[2] = gain_kp;
    data[3] = gain_kd;
    len = 4;
    can2_robostride[0].setPacketFrame(data, len);

    // 次周期からは motion に戻す（好みで velocity にしてもOK）
    g_cmd = 0;
  }
}
