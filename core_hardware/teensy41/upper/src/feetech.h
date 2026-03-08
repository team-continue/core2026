#pragma once

#include "wiring.h"
#include "pi.h"
#include <cstring>
#include "pin.h"

#define STS_CONTROL_DT 0.02f
#define STS_LIMIT_VEL_STS3215 ((67.0f / 60.0f) * 2.0f * M_PI)
#define STS_LIMIT_VEL_STS3020 ((100.0f / 60.0f) * 2.0f * M_PI)
#define STS_PID_P 2.0

#define STS_CONTROL_INTERVAL_US 10000UL
#define STS_CONNECT_TIMEOUT_MS 1000UL
#define STS_RECONNECT_INTERVAL_MS 500UL
#define STS_SYNCREAD_TIMEOUT_MS 50UL
#define MAX_DATA_LENGTH 32768
#define LEN_SERVO 8

struct STS_SERVO {
  float pos = 0.0f;
  float vel = 0.0f;
  float current = 0.0f;

  float ref_vel = 0.0f;
  float ref_pos = 0.0f;
  float origin = 0.0f;

  float limit_vel = STS_LIMIT_VEL_STS3020;

  uint8_t torque = 0;
  PID pos_pid{STS_PID_P, 0.0f, STS_CONTROL_DT};
  bool vel_cmd = false;
  uint8_t id = 0;

  uint16_t prev_pos = UINT16_MAX;
  int rotate = 0;
};

class STS {
 public:
  unsigned long prev_connect_ts_ = 0;
  bool disable = false;
  bool connect = false;
  STS_SERVO servos[LEN_SERVO];

  STS() {
    const float default_origin[LEN_SERVO] = {
      M_PI,
      M_PI,
      M_PI+1.1,
      M_PI-0.2,
      M_PI,
      M_PI,
      M_PI-0.3,
      M_PI-0.4,
    };
    for (int i = 0; i < LEN_SERVO; ++i) {
      servos[i].id = i + 1;
      servos[i].origin = default_origin[i];
    }
    servos[1].vel_cmd = true;
    servos[5].vel_cmd = true;
  }

  void init() {
    SERVO_SERIAL.begin(1000000);
    SERVO_SERIAL.addMemoryForRead(bigserialbuffer_, sizeof(bigserialbuffer_));
    while (!SERVO_SERIAL) {
    }
    delay(1000);
    prev_connect_ts_ = millis();
    last_control_us_ = micros();
  }

  void torque(bool on) {
    byte ids[LEN_SERVO] = {0};
    const byte len_id = collectServoIds(ids);

    byte status_level[LEN_SERVO] = {0};
    byte torque_data[LEN_SERVO] = {0};
    byte lock[LEN_SERVO] = {0};
    for (int i = 0; i < len_id; ++i) {
      torque_data[i] = on ? 1 : 0;
      lock[i] = 1;
    }
    sts_syncWrite(ids, len_id, 8, 1, status_level);
    sts_syncWrite(ids, len_id, 40, 1, torque_data);
    sts_syncWrite(ids, len_id, 55, 1, lock);
  }

  void loop() {
    syncReadData();

    if (syncread_pending_) {
      if ((millis() - syncread_sent_ms_) <= STS_SYNCREAD_TIMEOUT_MS) {
        return;
      }
      syncread_pending_ = false;
    }

    const uint32_t now_us = micros();
    if ((uint32_t)(now_us - last_control_us_) < STS_CONTROL_INTERVAL_US) {
      return;
    }
    last_control_us_ = now_us;

    const uint32_t now_ms = millis();
    connect = ((now_ms - prev_connect_ts_) <= STS_CONNECT_TIMEOUT_MS);

    if (disable) {
      if (!prev_disable_) {
        torque(false);
        prev_disable_ = true;
      }
    } else {
      prev_disable_ = false;
    }

    if (!connect && !disable &&
        (now_ms - last_torque_on_ms_) >= STS_RECONNECT_INTERVAL_MS) {
      last_torque_on_ms_ = now_ms;
      torque(true);
    }

    syncWriteCommandByMode();

    byte ids[LEN_SERVO] = {0};
    const byte len_id =
      syncread_upper_half_ ? collectServoIdsInRange(ids, 5, 8)
                           : collectServoIdsInRange(ids, 1, 4);
    if (len_id > 0) {
      startSyncReadTracking(ids, len_id);
      sts_syncRead(ids, len_id, 56, 15);
      syncread_upper_half_ = !syncread_upper_half_;
    }
  }

 private:
  void syncReadData() {
    if (!SERVO_SERIAL.available()) {
      return;
    }

    while (SERVO_SERIAL.available()) {
      if (buffer_data_num >= MAX_DATA_LENGTH) {
        buffer_data_num = 0;
      }
      buffer[buffer_data_num++] = (byte)SERVO_SERIAL.read();
    }

    int parse_i = 0;
    while ((buffer_data_num - parse_i) >= 4) {
      if (buffer[parse_i] != 0xFF || buffer[parse_i + 1] != 0xFF) {
        ++parse_i;
        continue;
      }

      const byte id = buffer[parse_i + 2];
      const byte payload_len = buffer[parse_i + 3];
      if (payload_len < 2) {
        ++parse_i;
        continue;
      }

      const int packet_len = payload_len + 4;
      if ((buffer_data_num - parse_i) < packet_len) {
        break;
      }

      dispResvData(buffer + parse_i, id, packet_len);
      parse_i += packet_len;
    }

    if (parse_i > 0) {
      if (parse_i < buffer_data_num) {
        memmove(buffer, buffer + parse_i, buffer_data_num - parse_i);
      }
      buffer_data_num -= parse_i;
    }
  }

  uint8_t bigserialbuffer_[MAX_DATA_LENGTH];
  byte buffer[MAX_DATA_LENGTH];
  int buffer_data_num = 0;
  uint32_t last_control_us_ = 0;
  uint32_t last_torque_on_ms_ = 0;
  bool prev_disable_ = false;
  bool syncread_upper_half_ = false;
  bool syncread_pending_ = false;
  uint32_t syncread_sent_ms_ = 0;
  byte syncread_expected_ids_[LEN_SERVO] = {0};
  bool syncread_received_[LEN_SERVO] = {0};
  byte syncread_expected_len_ = 0;

  byte collectServoIds(byte *ids) const {
    byte len_id = 0;
    for (int i = 0; i < LEN_SERVO; ++i) {
      if (servos[i].id == 0) {
        continue;
      }
      ids[len_id++] = servos[i].id;
    }
    return len_id;
  }

  byte collectServoIdsInRange(byte *ids, byte min_id, byte max_id) const {
    byte len_id = 0;
    for (int i = 0; i < LEN_SERVO; ++i) {
      if (servos[i].id == 0) {
        continue;
      }
      if (servos[i].id < min_id || servos[i].id > max_id) {
        continue;
      }
      ids[len_id++] = servos[i].id;
    }
    return len_id;
  }

  void startSyncReadTracking(const byte *ids, byte len_id) {
    syncread_expected_len_ = len_id;
    memset(syncread_expected_ids_, 0, sizeof(syncread_expected_ids_));
    memset(syncread_received_, 0, sizeof(syncread_received_));
    for (int i = 0; i < len_id; ++i) {
      syncread_expected_ids_[i] = ids[i];
    }
    syncread_sent_ms_ = millis();
    syncread_pending_ = (len_id > 0);
  }

  void markSyncReadReceived(byte id) {
    if (!syncread_pending_) {
      return;
    }
    for (int i = 0; i < syncread_expected_len_; ++i) {
      if (syncread_expected_ids_[i] == id) {
        syncread_received_[i] = true;
        break;
      }
    }

    for (int i = 0; i < syncread_expected_len_; ++i) {
      if (!syncread_received_[i]) {
        return;
      }
    }
    syncread_pending_ = false;
  }

  int findServoIndex(const byte id) const {
    for (int i = 0; i < LEN_SERVO; ++i) {
      if (servos[i].id == id) {
        return i;
      }
    }
    return -1;
  }

  void syncWriteCommandByMode() {
    byte pos_ids[LEN_SERVO] = {0};
    byte vel_ids[LEN_SERVO] = {0};
    float pos_data[LEN_SERVO] = {0.0f};
    float vel_data[LEN_SERVO] = {0.0f};
    byte pos_len = 0;
    byte vel_len = 0;

    for (int i = 0; i < LEN_SERVO; ++i) {
      if (servos[i].id == 0) {
        continue;
      }

      if (servos[i].vel_cmd) {
        if (disable) {
          servos[i].ref_vel = 0.0f;
          servos[i].pos_pid.reset();
        } else {
          const float pos_error = servos[i].ref_pos - servos[i].pos;
          servos[i].ref_vel = servos[i].pos_pid.update(pos_error, servos[i].limit_vel);
        }
        vel_ids[vel_len] = servos[i].id;
        vel_data[vel_len++] = servos[i].ref_vel;
      } else {
        pos_ids[pos_len] = servos[i].id;
        pos_data[pos_len++] = servos[i].ref_pos + servos[i].origin;
      }
    }

    if (pos_len > 0) {
      sts_syncWritePos(pos_ids, pos_len, pos_data);
    }
    if (vel_len > 0) {
      sts_syncWriteVel(vel_ids, vel_len, vel_data);
    }
  }

  void sts_syncRead(byte *ids, byte len_id, byte index, byte len) {
    byte message[64];
    message[0] = 0xFF;
    message[1] = 0xFF;
    message[2] = 0xFE;
    message[3] = len_id + 4;
    message[4] = 0x82;
    message[5] = index;
    message[6] = len;
    memcpy(message + 7, ids, len_id);
    message[7 + len_id] = sts_calcCkSum(message, 8 + len_id);
    SERVO_SERIAL.write(message, 8 + len_id);
    SERVO_SERIAL.flush();
  }

  void sts_syncWrite(byte *ids, byte len_id, byte index, byte len, byte *data) {
    byte message[64];
    message[0] = 0xFF;
    message[1] = 0xFF;
    message[2] = 0xFE;
    message[3] = (len + 1) * len_id + 4;
    message[4] = 0x83;
    message[5] = index;
    message[6] = len;
    for (int i = 0; i < len_id; ++i) {
      const int start_addr = 7 + (len + 1) * i;
      message[start_addr] = ids[i];
      memcpy(message + start_addr + 1, data + i * len, len);
    }
    message[7 + (len + 1) * len_id] = sts_calcCkSum(message, 8 + (len + 1) * len_id);
    SERVO_SERIAL.write(message, 8 + (len + 1) * len_id);
    SERVO_SERIAL.flush();
  }

  void sts_writeVel(byte id, float data) {
    int16_t temp = (int16_t)(data / (2 * M_PI) * 4096.0f);
    if (temp < 0) {
      temp = -temp;
      bitWrite(temp, 15, 1);
    } else {
      bitWrite(temp, 15, 0);
    }
    sts_write(id, 46, 2, (byte *)&temp);
  }

  void sts_writePos(byte id, float data) {
    uint16_t buf[1];
    const int buf_int = (int)(data / M_PI / 2 * 4096);
    buf[0] = constrain(buf_int, 0, 4095);
    sts_write(id, 42, 2, (byte *)buf);
  }

  void sts_write(byte id, byte index, byte len, byte *data) {
    byte message[64];
    message[0] = 0xFF;
    message[1] = 0xFF;
    message[2] = id;
    message[3] = len + 3;
    message[4] = 0x03;
    message[5] = index;
    memcpy(message + 6, data, len);
    message[6 + len] = sts_calcCkSum(message, 7 + len);
    SERVO_SERIAL.write(message, 7 + len);
    SERVO_SERIAL.flush();
  }

  void sts_writeReg(byte id, byte index, byte data) {
    byte buf[1] = {data};
    sts_write(id, index, 1, buf);
  }

  void sts_syncWritePos(byte *ids, byte len_id, float *data) {
    uint16_t buf[32];
    for (int i = 0; i < len_id; ++i) {
      buf[i] = (uint16_t)constrain((data[i] / (2 * M_PI) * 4096.0f), 0.0f, 4095.0f);
    }
    sts_syncWrite(ids, len_id, 42, 2, (byte *)buf);
  }

  void sts_syncWriteVel(byte *ids, byte len_id, float *data) {
    int16_t buf[32];
    for (int i = 0; i < len_id; ++i) {
      int16_t temp = (int16_t)(data[i] / (2 * M_PI) * 4096.0f);
      if (temp < 0) {
        temp = -temp;
        bitWrite(temp, 15, 1);
      } else {
        bitWrite(temp, 15, 0);
      }
      buf[i] = temp;
    }
    sts_syncWrite(ids, len_id, 46, 2, (byte *)buf);
  }

  byte sts_calcCkSum(byte *arr, int len) {
    byte checksum = 0;
    for (int i = 2; i < len - 1; ++i) {
      checksum += arr[i];
    }
    return ~((byte)(checksum & 0xFF));
  }

  void dispResvData(byte *data, byte id, byte len) {
    if (len < 6) {
      return;
    }
    if (data[len - 1] != sts_calcCkSum(data, len)) {
      return;
    }

    const int i = findServoIndex(id);
    if (i < 0) {
      return;
    }

    prev_connect_ts_ = millis();
    markSyncReadReceived(id);

    if (len > 19) {
      const uint16_t pos = (uint16_t)(data[6] << 8 | data[5]) & 0x0fff;
      const uint16_t vel = (uint16_t)(data[8] << 8 | data[7]);
      const uint16_t current = (uint16_t)(data[19] << 8 | data[18]);

      int vel_i = vel & 0x7fff;
      if (bitRead(vel, 15)) {
        vel_i = -vel_i;
      }

      int curr_i = current & 0x7fff;
      if (bitRead(current, 15)) {
        curr_i = -curr_i;
      }

      if (servos[i].prev_pos == UINT16_MAX) {
        servos[i].prev_pos = pos;
      } else {
        const int diff = (int)pos - (int)servos[i].prev_pos;
        if (diff > (4096 / 2)) {
          servos[i].rotate--;
        } else if (diff < -(4096 / 2)) {
          servos[i].rotate++;
        }
      }

      servos[i].pos = ((float)pos / 4096.0f + servos[i].rotate) * M_PI * 2.0f - servos[i].origin;
      servos[i].prev_pos = pos;
      servos[i].vel = (float)vel_i / 4096.0f * M_PI * 2.0f;
      servos[i].current = (float)curr_i * 0.0065f;
    } else if (len > 5) {
      servos[i].torque = data[5];
    }
  }
};
