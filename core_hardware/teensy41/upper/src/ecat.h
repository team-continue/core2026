#pragma once

#include <stdint.h>
#include <stddef.h>

#include "core_hardware/motor_diag_layout.h"

extern volatile uint32_t ecat_prev_connect_ts;
extern volatile bool ecat_rxpdo_pending;

// id 100: hp, 101: destroy, 102: 無線 (7 byte), 103: color,
// 104: hardware_enable, 105: bottom 基板との CAN 接続 (1: 接続)
bool ecat_setUint8(const uint8_t id, const uint8_t *data, const int u8_len);
bool ecat_setFloat(const uint8_t id, const float *data, const int flt_len);
bool ecat_setInt32(const uint8_t id, const int *data, const int int_len);
// モータ ID 0〜14 の診断情報 (motor_diag / motor_voltage)
bool ecat_setDiag(const uint8_t id, const core_hardware::motor_diag::MotorDiag &diag);
// CAN バスの状態 (bus_diag)。bus 0: can3, 1: can2
bool ecat_setBusDiag(const uint8_t bus, const core_hardware::motor_diag::BusDiag &diag);

void ecat_begin();
void ecat_update();
void ecat_send();

void ecat_FrameCallBack();
void ecat_PacketCallBack(const uint8_t id, const float *data, const size_t len);
void ecat_PacketCallBack(const uint8_t id, const uint8_t *data, const uint8_t len);
