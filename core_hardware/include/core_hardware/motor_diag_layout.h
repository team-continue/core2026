#pragma once

// Teensy 層 (upper 基板ファーム) と ROS 層 (core_hardware) で共有する、
// モータ診断の PDO 上のビット配置。両方の層がこのヘッダの pack / unpack を使う。
//
// PDO (TxPDO) 上のオブジェクト:
//   0x6005 motor_diag[15]    uint32  bit 配置は下記 kMotor*
//   0x6006 motor_voltage[15] uint16  0.01 V 単位、0 = 取得できない
//   0x6009 bus_diag[2]       uint32  [0] = can3, [1] = can2。bit 配置は下記 kBus*
//
// このヘッダは Arduino / ROS のどちらにも依存しない (stdint.h のみ)。

#include <stdint.h>

namespace core_hardware {
namespace motor_diag {

// PDO の配置を変えたら esi.json の RevisionNumber と合わせて上げる。
// ROS 層は SII の RevisionNumber (SOEM の eep_rev) がこの値と一致しないと起動しない。
constexpr uint32_t kEsiRevision = 3;

constexpr uint8_t kNumMotors = 15;   // ID 0〜14 (ESC の ID 15 は対象外)
constexpr uint8_t kNumCanBuses = 2;  // [0] = can3, [1] = can2

// ---- motor_diag (uint32) ----
constexpr uint32_t kMotorConnectedBit = 1UL << 0;
constexpr uint32_t kMotorEnabledBit = 1UL << 1;
constexpr uint32_t kMotorFaultBit = 1UL << 2;
constexpr uint32_t kMotorSupportedBit = 1UL << 3;  // 診断対象か (0 なら他のビットは無効)
constexpr uint8_t kMotorErrorCodeShift = 8;         // bit 8〜15: ドライバ固有のエラーコード (生値)
constexpr uint8_t kMotorTempShift = 16;             // bit 16〜23: 主温度 [℃] (int8)
constexpr uint8_t kMotorTempRotorShift = 24;        // bit 24〜31: ローター温度 [℃] (int8)

constexpr int8_t kTempInvalid = -128;      // 温度が取得できない
constexpr uint16_t kVoltageInvalid = 0;    // 電圧が取得できない

struct MotorDiag {
  bool connected = false;   // 通信が生きているか
  bool enabled = false;     // 出力が有効か
  bool fault = false;       // 故障状態か
  bool supported = false;   // 診断対象か
  uint8_t error_code = 0;   // ドライバ固有の生値
  int8_t temp = kTempInvalid;        // 主温度 [℃]
  int8_t temp_rotor = kTempInvalid;  // ローター温度 [℃] (Damiao のみ)
  uint16_t voltage_cv = kVoltageInvalid;  // 電圧 [0.01 V]
};

inline uint32_t pack_motor_diag(const MotorDiag& diag) {
  uint32_t raw = 0;
  if (diag.connected) {raw |= kMotorConnectedBit;}
  if (diag.enabled) {raw |= kMotorEnabledBit;}
  if (diag.fault) {raw |= kMotorFaultBit;}
  if (diag.supported) {raw |= kMotorSupportedBit;}
  raw |= static_cast<uint32_t>(diag.error_code) << kMotorErrorCodeShift;
  raw |= static_cast<uint32_t>(static_cast<uint8_t>(diag.temp)) << kMotorTempShift;
  raw |= static_cast<uint32_t>(static_cast<uint8_t>(diag.temp_rotor)) << kMotorTempRotorShift;
  return raw;
}

inline MotorDiag unpack_motor_diag(const uint32_t raw, const uint16_t voltage_cv) {
  MotorDiag diag;
  diag.connected = (raw & kMotorConnectedBit) != 0U;
  diag.enabled = (raw & kMotorEnabledBit) != 0U;
  diag.fault = (raw & kMotorFaultBit) != 0U;
  diag.supported = (raw & kMotorSupportedBit) != 0U;
  diag.error_code = static_cast<uint8_t>((raw >> kMotorErrorCodeShift) & 0xFFU);
  diag.temp = static_cast<int8_t>(static_cast<uint8_t>((raw >> kMotorTempShift) & 0xFFU));
  diag.temp_rotor = static_cast<int8_t>(static_cast<uint8_t>((raw >> kMotorTempRotorShift) & 0xFFU));
  diag.voltage_cv = voltage_cv;
  return diag;
}

// ℃ を int8 に丸める。範囲外は飽和、NaN は無効値
inline int8_t to_temp_i8(const float celsius) {
  if (!(celsius == celsius)) {return kTempInvalid;}
  if (celsius >= 127.0f) {return 127;}
  if (celsius <= -127.0f) {return -127;}
  return static_cast<int8_t>(celsius >= 0.0f ? celsius + 0.5f : celsius - 0.5f);
}

// V を 0.01 V 単位の uint16 に丸める。0 以下と NaN は無効値
inline uint16_t to_voltage_cv(const float volt) {
  if (!(volt == volt) || volt <= 0.0f) {return kVoltageInvalid;}
  const float cv = volt * 100.0f + 0.5f;
  if (cv >= 65535.0f) {return 65535U;}
  return static_cast<uint16_t>(cv);
}

// ---- bus_diag (uint32) ----
constexpr uint32_t kBusOffBit = 1UL << 0;
constexpr uint32_t kBusErrorPassiveBit = 1UL << 1;
constexpr uint8_t kBusTxErrorCountShift = 8;   // bit 8〜15: 送信エラーカウンタ
constexpr uint8_t kBusRxErrorCountShift = 16;  // bit 16〜23: 受信エラーカウンタ
constexpr uint32_t kBusValidBit = 1UL << 31;   // 取得できているか (0 なら他のビットは無効)

struct BusDiag {
  bool valid = false;
  bool bus_off = false;
  bool error_passive = false;
  uint8_t tx_error_count = 0;
  uint8_t rx_error_count = 0;
};

inline uint32_t pack_bus_diag(const BusDiag& diag) {
  uint32_t raw = 0;
  if (diag.valid) {raw |= kBusValidBit;}
  if (diag.bus_off) {raw |= kBusOffBit;}
  if (diag.error_passive) {raw |= kBusErrorPassiveBit;}
  raw |= static_cast<uint32_t>(diag.tx_error_count) << kBusTxErrorCountShift;
  raw |= static_cast<uint32_t>(diag.rx_error_count) << kBusRxErrorCountShift;
  return raw;
}

inline BusDiag unpack_bus_diag(const uint32_t raw) {
  BusDiag diag;
  diag.valid = (raw & kBusValidBit) != 0U;
  diag.bus_off = (raw & kBusOffBit) != 0U;
  diag.error_passive = (raw & kBusErrorPassiveBit) != 0U;
  diag.tx_error_count = static_cast<uint8_t>((raw >> kBusTxErrorCountShift) & 0xFFU);
  diag.rx_error_count = static_cast<uint8_t>((raw >> kBusRxErrorCountShift) & 0xFFU);
  return diag;
}

}  // namespace motor_diag
}  // namespace core_hardware
