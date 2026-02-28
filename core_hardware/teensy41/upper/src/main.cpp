#include "can2.h"

enum TestCmd {
  CMD_DISABLE = 0,
  CMD_VELOCITY = 2,
  CMD_MOTION = 3,
  CMD_GAIN = 4
};

static int g_cmd[CAN2_NUM_MOTOR];
static float target_vel[CAN2_NUM_MOTOR];
static float target_pos[CAN2_NUM_MOTOR];
static float gain_cur_kp[CAN2_NUM_MOTOR];
static float gain_cur_ki[CAN2_NUM_MOTOR];
static float gain_kp[CAN2_NUM_MOTOR];
static float gain_kd[CAN2_NUM_MOTOR];

static uint32_t last_print_ms = 0;
static char serial_line[64] = {0};
static size_t serial_line_len = 0;

static void initTestState() {
  for (int i = 0; i < CAN2_NUM_MOTOR; ++i) {
    g_cmd[i] = CMD_MOTION;
    target_vel[i] = 3.0f;
    target_pos[i] = 3.14f;
    gain_cur_kp[i] = 0.125f;
    gain_cur_ki[i] = 0.0158f;
    gain_kp[i] = 500.0f;
    gain_kd[i] = 5.0f;
  }
}

static void printHelp() {
  Serial.println("keys: 0=disable-all +/-=vel-all");
  Serial.println("cmd : m<id> d");
  Serial.println("cmd : m<id> v <rad_s>");
  Serial.println("cmd : m<id> p <rad>");
  Serial.println("cmd : m<id> g <cur_kp> <cur_ki> <kp> <kd>");
  Serial.println("ex  : m0 v 3.0 / m1 p 1.57");
}

static bool validMotorId(const int id) {
  return (id >= 0) && (id < CAN2_NUM_MOTOR);
}

static bool handleSingleKey(const char key) {
  switch (key) {
    case '0':
      for (int i = 0; i < CAN2_NUM_MOTOR; ++i) {
        g_cmd[i] = CMD_DISABLE;
      }
      Serial.println("[cmd] all motors disable");
      return true;
    case '+':
      for (int i = 0; i < CAN2_NUM_MOTOR; ++i) {
        target_vel[i] += 0.5f;
        g_cmd[i] = CMD_VELOCITY;
      }
      Serial.printf("[cmd] all motors target_vel=%.2f\n", target_vel[0]);
      return true;
    case '-':
      for (int i = 0; i < CAN2_NUM_MOTOR; ++i) {
        target_vel[i] -= 0.5f;
        if (target_vel[i] < 0.0f) {
          target_vel[i] = 0.0f;
        }
        g_cmd[i] = CMD_VELOCITY;
      }
      Serial.printf("[cmd] all motors target_vel=%.2f\n", target_vel[0]);
      return true;
    default:
      return false;
  }
}

static bool handleMotorCommand(const char *line) {
  int id = -1;
  char op = '\0';
  if (sscanf(line, "m%d %c", &id, &op) != 2) {
    return false;
  }
  if (!validMotorId(id)) {
    Serial.printf("[err] invalid motor id: %d (valid: 0-%d)\n", id, CAN2_NUM_MOTOR - 1);
    return true;
  }

  if (op == 'd') {
    g_cmd[id] = CMD_DISABLE;
    Serial.printf("[cmd] M%d disable\n", id);
    return true;
  }
  if (op == 'v') {
    float vel = 0.0f;
    if (sscanf(line, "m%d %c %f", &id, &op, &vel) == 3) {
      target_vel[id] = vel;
      g_cmd[id] = CMD_VELOCITY;
      Serial.printf("[cmd] M%d velocity vel=%.3f\n", id, target_vel[id]);
      return true;
    }
    return false;
  }
  if (op == 'p') {
    float pos = 0.0f;
    if (sscanf(line, "m%d %c %f", &id, &op, &pos) == 3) {
      target_pos[id] = pos;
      g_cmd[id] = CMD_MOTION;
      Serial.printf("[cmd] M%d motion pos=%.3f\n", id, target_pos[id]);
      return true;
    }
    return false;
  }
  if (op == 'g') {
    float cur_kp = 0.0f;
    float cur_ki = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    if (sscanf(line, "m%d %c %f %f %f %f", &id, &op, &cur_kp, &cur_ki, &kp, &kd) == 6) {
      gain_cur_kp[id] = cur_kp;
      gain_cur_ki[id] = cur_ki;
      gain_kp[id] = kp;
      gain_kd[id] = kd;
      g_cmd[id] = CMD_GAIN;
      Serial.printf("[cmd] M%d gain update one-shot\n", id);
      return true;
    }
    return false;
  }

  return false;
}

static void handleSerial() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }

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

    bool ok = false;
    if (serial_line_len == 1) {
      ok = handleSingleKey(serial_line[0]);
    } else {
      ok = handleMotorCommand(serial_line);
    }
    if (!ok) {
      printHelp();
    }

    serial_line_len = 0;
  }
}

static void printStatus() {
  if (millis() - last_print_ms < 200) {
    return;
  }
  last_print_ms = millis();

  for (int i = 0; i < CAN2_NUM_MOTOR; ++i) {
    Serial.printf("M%d connect_ros2=%d mode=%d rp=%.4f rv=%.4f p=%.4f v=%.4f torque=%.4f temp=%.1f Kp=%.2f Ki=%.2f\n",
                  i,
                  (int)can2_robostride[i].connect_ros2,
                  can2_robostride[i].ref.mode,
                  can2_robostride[i].ref.position_rad,
                  can2_robostride[i].ref.velocity_rad_s,
                  can2_robostride[i].feedback.position_rad,
                    can2_robostride[i].feedback.velocity_rad_s,
                    can2_robostride[i].feedback.torque_nm,
                    can2_robostride[i].feedback.temp_mos,
                    can2_robostride[i].ref.kp_vel,
                    can2_robostride[i].ref.ki_vel);
    // Serial.printf("M%d connect_ros2=%d mode=%d
  }
}

static void applyMotorCommand(const int motor_id) {
  float data[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  int len = 0;

  if (g_cmd[motor_id] == CMD_DISABLE) {
    len = 1;
  } else if (g_cmd[motor_id] == CMD_VELOCITY) {
    data[1] = target_vel[motor_id];
    len = 1;
  } else if (g_cmd[motor_id] == CMD_MOTION) {
    data[0] = target_pos[motor_id];
    len = 1;
  } else if (g_cmd[motor_id] == CMD_GAIN) {
    data[0] = gain_cur_kp[motor_id];
    data[1] = gain_cur_ki[motor_id];
    data[2] = gain_kp[motor_id];
    data[3] = gain_kd[motor_id];
    len = 4;
  }

  can2_robostride[motor_id].setPacketFrame(data, len);

//   if (g_cmd[motor_id] == CMD_GAIN) {
//     g_cmd[motor_id] = CMD_DISABLE;
//   }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("=== CAN2 RoboStride multi-motor test ===");

  initTestState();

  can2_init();

  printHelp();
}

void loop() {
  handleSerial();
  printStatus();
  for (int i = 0; i < CAN2_NUM_MOTOR; ++i) {
    applyMotorCommand(i);
  }
  can2_loop();
}
