#include <Arduino.h>

#include "ecat.h"

namespace {

constexpr uint8_t kJointIds[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
constexpr uint8_t kWirelessBytes[] = {10, 11, 12, 13, 14, 15, 16};

float g_targets[16] = {0.0f};
bool g_emergency = false;
uint32_t g_rx_count = 0;

void build_feedback_frame(const uint8_t id, float *frame) {
  frame[0] = static_cast<float>(id * 10 + 0);
  frame[1] = 0.0f;
  frame[2] = static_cast<float>(id * 10 + 2);
  frame[3] = 1.0f;
  frame[4] = static_cast<float>(id * 10 + 4);
  frame[5] = 2.0f;

  const float target = g_targets[id];
  if (id <= 4) {
    frame[2] = target;
  } else if (id <= 14) {
    frame[4] = target;
  }
}

}  // namespace

void ecat_FrameCallBack() {
  for (const uint8_t id : kJointIds) {
    float frame[6];
    build_feedback_frame(id, frame);
    ecat_setFloat(id, frame, 6);
  }

  uint8_t zero = 1;
  uint8_t wireless[sizeof(kWirelessBytes)];
  for (size_t i = 0; i < sizeof(kWirelessBytes); ++i) {
    wireless[i] = kWirelessBytes[i];
  }
  
  ecat_setUint8(100, &zero, 1);
  zero = 1;
  ecat_setUint8(101, &zero, 1);
  ecat_setUint8(102, wireless, static_cast<int>(sizeof(wireless)));
  zero = 1;
  ecat_setUint8(104, &zero, 1);
  ecat_send();
}

void ecat_PacketCallBack(const uint8_t id, const float *data, const size_t len) {
  if (len == 0) {
    return;
  }

  if (id <= 15) {
    g_targets[id] = data[len - 1];
  } else if (id == 17) {
    g_emergency = (data[len - 1] != 0.0f);
  }

  ++g_rx_count;
  digitalWrite(LED_BUILTIN, (g_rx_count & 0x01U) != 0U ? HIGH : LOW);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  delay(100);
  Serial.println("upper ecat test");

  ecat_begin();
}

void loop() {
  ecat_update();
}
