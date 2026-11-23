#include <gtest/gtest.h>

#include <chrono>
#include <optional>

#include "core_hardware/led_command.hpp"

namespace {

using core_hardware::led_tape_output;
using core_hardware::pack_led_command;
using core_hardware::pack_turret_led_byte;
using core_hardware::pack_turret_led_pair;
using std::chrono::milliseconds;
using std::chrono::steady_clock;

TEST(PackLedCommand, PatternInLowByteColorInHighByte) {
  EXPECT_EQ(pack_led_command(14U, 4U, false), std::optional<uint16_t>(0x040EU));
  EXPECT_EQ(pack_led_command(2U, 0U, false), std::optional<uint16_t>(0x0002U));
}

TEST(PackLedCommand, ReverseInBit15) {
  EXPECT_EQ(pack_led_command(2U, 0U, true), std::optional<uint16_t>(0x8002U));
  EXPECT_EQ(pack_led_command(14U, 4U, true), std::optional<uint16_t>(0x840EU));
  EXPECT_EQ(pack_led_command(255U, 127U, true), std::optional<uint16_t>(0xFFFFU));
}

TEST(PackLedCommand, RejectsColorThatOverlapsReverseBit) {
  EXPECT_EQ(pack_led_command(2U, 128U, false), std::nullopt);
}

TEST(PackTurretLedByte, PatternInLowNibbleColorInBits4To6) {
  EXPECT_EQ(pack_turret_led_byte(8U, 2U, false), std::optional<uint8_t>(0x28U));
  EXPECT_EQ(pack_turret_led_byte(15U, 7U, false), std::optional<uint8_t>(0x7FU));
  EXPECT_EQ(pack_turret_led_byte(0U, 0U, false), std::optional<uint8_t>(0x00U));
}

TEST(PackTurretLedByte, ReverseInBit7) {
  EXPECT_EQ(pack_turret_led_byte(2U, 0U, true), std::optional<uint8_t>(0x82U));
  EXPECT_EQ(pack_turret_led_byte(15U, 7U, true), std::optional<uint8_t>(0xFFU));
}

TEST(PackTurretLedByte, RejectsValuesThatDoNotFit) {
  EXPECT_EQ(pack_turret_led_byte(16U, 0U, false), std::nullopt);
  EXPECT_EQ(pack_turret_led_byte(0U, 8U, true), std::nullopt);
}

TEST(PackTurretLedPair, LeftInLowByteRightInHighByte) {
  EXPECT_EQ(pack_turret_led_pair(0x28U, 0x02U), 0x0228U);
  EXPECT_EQ(pack_turret_led_pair(0x00U, 0x7FU), 0x7F00U);
}

TEST(LedTapeOutput, ReturnsZeroBeforeFirstCommand) {
  EXPECT_EQ(led_tape_output(0x0102U, std::nullopt, steady_clock::now(), milliseconds(1000)), 0U);
}

TEST(LedTapeOutput, ReturnsLastValueWithinTimeout) {
  const auto rx_tp = steady_clock::now();
  EXPECT_EQ(led_tape_output(0x0102U, rx_tp, rx_tp + milliseconds(1000), milliseconds(1000)), 0x0102U);
}

TEST(LedTapeOutput, ReturnsZeroAfterTimeout) {
  const auto rx_tp = steady_clock::now();
  EXPECT_EQ(led_tape_output(0x0102U, rx_tp, rx_tp + milliseconds(1001), milliseconds(1000)), 0U);
}

}  // namespace
