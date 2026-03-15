#pragma once

#include <stdint.h>
#include <stddef.h>

extern volatile uint32_t ecat_prev_connect_ts;
extern volatile bool ecat_rxpdo_pending;

bool ecat_setUint8(const uint8_t id, const uint8_t *data, const int u8_len);
bool ecat_setFloat(const uint8_t id, const float *data, const int flt_len);
bool ecat_setInt32(const uint8_t id, const int *data, const int int_len);

void ecat_begin();
void ecat_update();
void ecat_send();

void ecat_FrameCallBack();
void ecat_PacketCallBack(const uint8_t id, const float *data, const size_t len);
void ecat_PacketCallBack(const uint8_t id, const uint8_t *data, const uint8_t len);
