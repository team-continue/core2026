#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

namespace core_hardware {

// LED_TAPE の 1 スロット分: 下位 8 bit = パターン、上位 8 bit = 色
inline uint16_t pack_led_command(uint8_t pattern, uint8_t color) {
  return static_cast<uint16_t>(static_cast<uint16_t>(color) << 8U | pattern);
}

// 砲塔の区間 1 つ分の 1 byte: 下位 4 bit = パターン、bit 4〜6 = 色、bit 7 = 0。
// 収まらない値 (パターン > 15 または 色 > 7) は nullopt
inline constexpr uint8_t kTurretLedMaxPattern = 0x0FU;
inline constexpr uint8_t kTurretLedMaxColor = 0x07U;
inline std::optional<uint8_t> pack_turret_led_byte(uint8_t pattern, uint8_t color) {
  if (pattern > kTurretLedMaxPattern || color > kTurretLedMaxColor) {
    return std::nullopt;
  }
  return static_cast<uint8_t>(color << 4U | pattern);
}

// LED_TAPE[2]: 下位 8 bit = 左の砲塔、上位 8 bit = 右の砲塔
inline uint16_t pack_turret_led_pair(uint8_t left_byte, uint8_t right_byte) {
  return static_cast<uint16_t>(static_cast<uint16_t>(right_byte) << 8U | left_byte);
}

// 最後に受けた指令が timeout 以内ならその値を、受けていない・途絶えたなら 0 (指令なし) を返す。
// Teensy はパターン 0 を見て「EtherCAT は疎通、LED の指令なし」の表示を出す。
inline uint16_t led_tape_output(
    uint16_t last_value,
    const std::optional<std::chrono::steady_clock::time_point>& last_rx_tp,
    std::chrono::steady_clock::time_point now,
    std::chrono::milliseconds timeout) {
  if (!last_rx_tp.has_value() || now - *last_rx_tp > timeout) {
    return 0U;
  }
  return last_value;
}

}  // namespace core_hardware
