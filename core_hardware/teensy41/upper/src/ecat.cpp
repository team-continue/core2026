#include "ecat.h"

#include <Arduino.h>

#include <limits.h>
#include <math.h>
#include <string.h>

#include "ecat_slv.h"
#include "utypes.h"

namespace {

_Objects &obj = Obj;
constexpr size_t kWirelessPayloadBytes = 7;

int clamp_to_int16(const float value) {
  if (value > 32767.0f) {
    return 32767;
  }
  if (value < -32768.0f) {
    return -32768;
  }
  return static_cast<int>(lroundf(value));
}

int position_index_from_id(const uint8_t id) {
  if (id <= 14) {
    return id;
  }
  return -1;
}

bool set_state_frame(const uint8_t id, const float *data, const int flt_len) {
  if (flt_len < 6) {
    return false;
  }

  const int pos_index = position_index_from_id(id);
  if (pos_index >= 0) {
    obj.motor_state_pos[pos_index] = data[5];
  }
  if (id < 15) {
    obj.motor_state_vel[id] = clamp_to_int16(data[3]);
    if (id >= 4) {
      obj.motor_state_torque[id] = clamp_to_int16(data[1]);
    }
    return true;
  }
  return false;
}

void dispatch_received_outputs() {
  float data[1] = {0.0f};
  for (uint8_t id = 0; id < 16; ++id) {
    data[0] = obj.motor_ref[id];
    ecat_PacketCallBack(id, data, 1);
  }

  data[0] = obj.system_ref[0] ? 1.0f : 0.0f;
  ecat_PacketCallBack(17, data, 1);

  // LED_TAPE[i] の下位 8 bit がパターン、上位 8 bit が色。テープごとに (パターン, 色) の順で並べる
  uint8_t led_data[6] = {
      static_cast<uint8_t>(obj.LED_TAPE[0] & 0xFF),
      static_cast<uint8_t>((obj.LED_TAPE[0] >> 8) & 0xFF),
      static_cast<uint8_t>(obj.LED_TAPE[1] & 0xFF),
      static_cast<uint8_t>((obj.LED_TAPE[1] >> 8) & 0xFF),
      static_cast<uint8_t>(obj.LED_TAPE[2] & 0xFF),
      static_cast<uint8_t>((obj.LED_TAPE[2] >> 8) & 0xFF),
  };
  ecat_PacketCallBack(100, led_data, 6);
}

void app_hook() {
  if ((CC_ATOMIC_GET(ESCvar.App.state) & APPSTATE_INPUT) == 0) {
    return;
  }
  if (!ecat_rxpdo_pending) {
    return;
  }

  dispatch_received_outputs();
  ecat_FrameCallBack();
  ecat_rxpdo_pending = false;
}

void safe_output_hook() {
  memset(obj.motor_ref, 0, sizeof(obj.motor_ref));
  obj.system_ref[0] = 0;
  memset(obj.LED_TAPE, 0, sizeof(obj.LED_TAPE));
}

}  // namespace

_Objects Obj = {};
volatile uint32_t ecat_prev_connect_ts = 0;
volatile bool ecat_rxpdo_pending = false;

bool ecat_setUint8(const uint8_t id, const uint8_t *data, const int u8_len) {
  if (data == nullptr || u8_len <= 0) {
    return false;
  }

  switch (id) {
    case 100:
      obj.core_state[0] = data[0];
      return true;
    case 101:
      obj.core_state[1] = data[0];
      return true;
    case 102: {
      uint8_t *torque_bytes = reinterpret_cast<uint8_t *>(obj.motor_state_torque);
      const size_t copy_len = static_cast<size_t>(u8_len) < kWirelessPayloadBytes ? static_cast<size_t>(u8_len) : kWirelessPayloadBytes;
      memset(torque_bytes, 0, kWirelessPayloadBytes);
      memcpy(torque_bytes, data, copy_len);
      return true;
    }
    case 103: {
      if (u8_len <= 0) {
        return false;
      }
      uint8_t *torque_bytes = reinterpret_cast<uint8_t *>(obj.motor_state_torque);
      torque_bytes[kWirelessPayloadBytes] = data[0];
      return true;
    }
    case 104:
      obj.system_state[0] = data[0] != 0;
      return true;
    default:
      return false;
  }
}

bool ecat_setFloat(const uint8_t id, const float *data, const int flt_len) {
  if (data == nullptr || flt_len <= 0) {
    return false;
  }

  if (id <= 14) {
    return set_state_frame(id, data, flt_len);
  }
  if (id == 15) {
    obj.motor_state_vel[14] = clamp_to_int16(data[flt_len - 1]);
    return true;
  }
  return false;
}

bool ecat_setInt32(const uint8_t id, const int *data, const int int_len) {
  return ecat_setUint8(id, reinterpret_cast<const uint8_t *>(data), int_len * static_cast<int>(sizeof(int)));
}

void ecat_send() {
}

void ecat_begin() {
  static esc_cfg_t config = {
      .user_arg = const_cast<char *>("core2026_upper"),
      .use_interrupt = 0,
      .watchdog_cnt = INT32_MAX,
      .set_defaults_hook = NULL,
      .pre_state_change_hook = NULL,
      .post_state_change_hook = NULL,
      .application_hook = app_hook,
      .safeoutput_override = safe_output_hook,
      .pre_object_download_hook = NULL,
      .post_object_download_hook = NULL,
      .rxpdo_override = NULL,
      .txpdo_override = NULL,
      .esc_hw_interrupt_enable = NULL,
      .esc_hw_interrupt_disable = NULL,
      .esc_hw_eep_handler = NULL,
      .esc_check_dc_handler = NULL,
  };

  memset(&obj, 0, sizeof(obj));
  ecat_prev_connect_ts = millis();
  ecat_rxpdo_pending = false;
  obj.serial = 1;
  ecat_slv_init(&config);
}

void ecat_update() {
  ecat_slv();
}

bool ecat_link_up() {
  // DL Status (0x0110) の port 0 の物理リンクのビット。
  // ET1100 互換の配置 (bit 4) を前提にしている。AX58100 で実機確認が必要。
  constexpr uint16_t kDlStatusPort0LinkMask = 0x0010;
  uint16_t dl_status = 0;
  ESC_read(ESCREG_DLSTATUS, &dl_status, sizeof(dl_status));
  dl_status = etohs(dl_status);
  return (dl_status & kDlStatusPort0LinkMask) != 0;
}
