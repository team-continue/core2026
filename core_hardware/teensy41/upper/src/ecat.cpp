#include "ecat.h"

#include <Arduino.h>

#include <limits.h>
#include <math.h>
#include <string.h>

#include "ecat_slv.h"
#include "utypes.h"

namespace {

_Objects &obj = Obj;

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
    obj.motor_state_torque[id] = clamp_to_int16(data[1]);
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
}

void app_hook() {
  if ((CC_ATOMIC_GET(ESCvar.App.state) & APPSTATE_INPUT) == 0) {
    return;
  }

  dispatch_received_outputs();
  ecat_FrameCallBack();
}

void safe_output_hook() {
  memset(obj.motor_ref, 0, sizeof(obj.motor_ref));
  obj.system_ref[0] = 0;
  memset(obj.LED_TAPE, 0, sizeof(obj.LED_TAPE));
}

}  // namespace

_Objects Obj = {};

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
  obj.serial = 1;
  ecat_slv_init(&config);
}

void ecat_update() {
  ecat_slv();
}
