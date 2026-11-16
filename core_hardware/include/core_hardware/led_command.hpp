#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

namespace core_hardware {

// LED_TAPE の 1 スロット分: 下位 8 bit = パターン、上位 8 bit = 色
inline uint16_t pack_led_command(uint8_t pattern, uint8_t color) {
  return static_cast<uint16_t>(static_cast<uint16_t>(color) << 8U | pattern);
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
