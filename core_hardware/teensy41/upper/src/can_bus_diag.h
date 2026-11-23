#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>

// 診断情報のビット配置は ROS 層と共有する (core_hardware/include)
#include "core_hardware/motor_diag_layout.h"

#define CAN_BUS_DIAG_MAX_READ 16 // 1 回に読む error() の最大件数 (FlexCAN_T4 の記録の最大数)

// FlexCAN_T4 の error() の値から CAN バスの状態を作る。
// FLTCONF (ESR1 の bit 4〜5): 0 = error active, 1 = error passive, 2/3 = bus off
// (i.MX RT1062 のリファレンスマニュアルに基づく。実機で要確認)
inline core_hardware::motor_diag::BusDiag to_bus_diag(const CAN_error_t &err) {
  const uint8_t fltconf = static_cast<uint8_t>((err.ESR1 >> 4) & 0x3);
  core_hardware::motor_diag::BusDiag diag;
  diag.valid = true;
  diag.error_passive = fltconf == 1;
  diag.bus_off = fltconf >= 2;
  diag.tx_error_count = err.TX_ERR_COUNTER;
  diag.rx_error_count = err.RX_ERR_COUNTER;
  return diag;
}

// CAN バスの状態 (PDO の bus_diag) を読む。
// error() は割り込みで記録された ESR1 / ECR の変化を古い順に 1 件ずつ返し、
// 新しい記録が無ければ false を返す。溜まっている分を読み切り、最新の値を保持して使う
struct CanBusDiagReader {
  CAN_error_t latest;
  bool has_latest = false;

  template <CAN_DEV_TABLE _bus, FLEXCAN_RXQUEUE_TABLE _rxSize, FLEXCAN_TXQUEUE_TABLE _txSize>
  core_hardware::motor_diag::BusDiag read(FlexCAN_T4<_bus, _rxSize, _txSize> &can) {
    CAN_error_t err;
    for (int i = 0; i < CAN_BUS_DIAG_MAX_READ; ++i) {
      if (!can.error(err, false)) {
        break;
      }
      latest = err;
      has_latest = true;
    }
    if (!has_latest) {
      return core_hardware::motor_diag::BusDiag{};  // まだ記録が無い (valid = false)
    }
    return to_bus_diag(latest);
  }
};
