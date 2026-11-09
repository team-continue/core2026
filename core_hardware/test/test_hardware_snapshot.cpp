// hardware_snapshot.hpp (daemon と ROS ノードの間の IPC で使うスナップショット) の単体テスト
#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>

#include "core_hardware/hardware_snapshot.hpp"

namespace {

using core_hardware::HardwareSnapshot;
using core_hardware::SnapshotFieldId;

// すべてのフィールドに、要素ごとに違う 0 以外の値を入れる
HardwareSnapshot make_filled_snapshot() {
  HardwareSnapshot snapshot;
  for (std::size_t i = 0; i < snapshot.motor_state_pos.size(); ++i) {
    const auto n = static_cast<int>(i);
    snapshot.motor_state_pos[i] = 0.5f * static_cast<float>(n) - 3.25f;
    snapshot.motor_state_vel[i] = static_cast<int16_t>(100 - (n * 7));
    snapshot.motor_state_torque[i] = static_cast<int16_t>(-50 + (n * 3));
    snapshot.motor_diag[i] = 0xC5000009U | (static_cast<uint32_t>(i) << 8);
    snapshot.motor_voltage[i] = static_cast<uint16_t>(2400 + n);
  }
  snapshot.system_state = {1U, 1U};
  snapshot.core_state = {200U, 1U};
  snapshot.core_wireless = {11U, 12U, 13U, 14U, 15U, 16U, 17U};
  for (std::size_t i = 0; i < snapshot.motor_ref.size(); ++i) {
    snapshot.motor_ref[i] = 1.25f * static_cast<float>(i) + 0.5f;
  }
  snapshot.system_ref = {1U};
  snapshot.led_tape = {10U, 20U, 30U};
  snapshot.ethercat_connected = {1U};
  snapshot.last_ethercat_rx_ms = {1761100000123ULL};
  snapshot.bus_diag = {0x80050A02U, 0x80000001U};
  return snapshot;
}

void expect_snapshot_eq(const HardwareSnapshot& actual, const HardwareSnapshot& expected) {
  EXPECT_EQ(actual.motor_state_pos, expected.motor_state_pos);
  EXPECT_EQ(actual.motor_state_vel, expected.motor_state_vel);
  EXPECT_EQ(actual.motor_state_torque, expected.motor_state_torque);
  EXPECT_EQ(actual.system_state, expected.system_state);
  EXPECT_EQ(actual.core_state, expected.core_state);
  EXPECT_EQ(actual.core_wireless, expected.core_wireless);
  EXPECT_EQ(actual.motor_ref, expected.motor_ref);
  EXPECT_EQ(actual.system_ref, expected.system_ref);
  EXPECT_EQ(actual.led_tape, expected.led_tape);
  EXPECT_EQ(actual.ethercat_connected, expected.ethercat_connected);
  EXPECT_EQ(actual.last_ethercat_rx_ms, expected.last_ethercat_rx_ms);
  EXPECT_EQ(actual.motor_diag, expected.motor_diag);
  EXPECT_EQ(actual.motor_voltage, expected.motor_voltage);
  EXPECT_EQ(actual.bus_diag, expected.bus_diag);
}

// motor_diag 等を追加する前の daemon が送っていたペイロード (field 1〜11 のみ)
std::vector<uint8_t> encode_legacy_snapshot(const HardwareSnapshot& snapshot) {
  using core_hardware::detail::append_field;
  std::vector<uint8_t> buffer;
  append_field(buffer, SnapshotFieldId::kMotorStatePos, snapshot.motor_state_pos.data(), snapshot.motor_state_pos.size());
  append_field(buffer, SnapshotFieldId::kMotorStateVel, snapshot.motor_state_vel.data(), snapshot.motor_state_vel.size());
  append_field(buffer, SnapshotFieldId::kMotorStateTorque, snapshot.motor_state_torque.data(), snapshot.motor_state_torque.size());
  append_field(buffer, SnapshotFieldId::kSystemState, snapshot.system_state.data(), snapshot.system_state.size());
  append_field(buffer, SnapshotFieldId::kCoreState, snapshot.core_state.data(), snapshot.core_state.size());
  append_field(buffer, SnapshotFieldId::kCoreWireless, snapshot.core_wireless.data(), snapshot.core_wireless.size());
  append_field(buffer, SnapshotFieldId::kMotorRef, snapshot.motor_ref.data(), snapshot.motor_ref.size());
  append_field(buffer, SnapshotFieldId::kSystemRef, snapshot.system_ref.data(), snapshot.system_ref.size());
  append_field(buffer, SnapshotFieldId::kLedTape, snapshot.led_tape.data(), snapshot.led_tape.size());
  append_field(buffer, SnapshotFieldId::kEthercatConnected, snapshot.ethercat_connected.data(), snapshot.ethercat_connected.size());
  append_field(buffer, SnapshotFieldId::kLastEthercatRxMs, snapshot.last_ethercat_rx_ms.data(), snapshot.last_ethercat_rx_ms.size());
  return buffer;
}

TEST(HardwareSnapshotCodec, RoundTripIncludesDiagFields) {
  const HardwareSnapshot original = make_filled_snapshot();
  const HardwareSnapshot decoded = core_hardware::decode_snapshot(core_hardware::encode_snapshot(original));
  expect_snapshot_eq(decoded, original);
}

TEST(HardwareSnapshotCodec, DefaultSnapshotRoundTrip) {
  const HardwareSnapshot decoded = core_hardware::decode_snapshot(core_hardware::encode_snapshot(HardwareSnapshot{}));
  expect_snapshot_eq(decoded, HardwareSnapshot{});
}

TEST(HardwareSnapshotCodec, DecodesLegacyPayloadWithoutDiagFields) {
  const HardwareSnapshot original = make_filled_snapshot();
  const HardwareSnapshot decoded = core_hardware::decode_snapshot(encode_legacy_snapshot(original));

  HardwareSnapshot expected = original;
  expected.motor_diag = {};
  expected.motor_voltage = {};
  expected.bus_diag = {};
  expect_snapshot_eq(decoded, expected);
}

TEST(HardwareSnapshotCodec, SkipsUnknownField) {
  const HardwareSnapshot original = make_filled_snapshot();
  std::vector<uint8_t> payload = core_hardware::encode_snapshot(original);
  const std::vector<uint8_t> unknown = {1U, 2U, 3U};
  core_hardware::detail::append_field(payload, static_cast<SnapshotFieldId>(99), unknown.data(), unknown.size());

  const HardwareSnapshot decoded = core_hardware::decode_snapshot(payload);
  expect_snapshot_eq(decoded, original);
}

TEST(HardwareSnapshotCodec, RejectsDiagFieldSizeMismatch) {
  std::vector<uint8_t> payload = encode_legacy_snapshot(make_filled_snapshot());
  const std::vector<uint32_t> short_motor_diag(14U, 0U);
  core_hardware::detail::append_field(payload, SnapshotFieldId::kMotorDiag, short_motor_diag.data(), short_motor_diag.size());
  EXPECT_THROW(core_hardware::decode_snapshot(payload), std::runtime_error);
}

TEST(HardwareSnapshotCodec, RejectsTruncatedPayload) {
  std::vector<uint8_t> payload = core_hardware::encode_snapshot(make_filled_snapshot());
  payload.pop_back();
  EXPECT_THROW(core_hardware::decode_snapshot(payload), std::runtime_error);
}

TEST(HardwareSnapshotObjects, SnapshotTakesWirelessFromWirelessObject) {
  _Objects objects{};
  for (std::size_t i = 0; i < 15U; ++i) {
    objects.motor_state_torque[i] = static_cast<int16_t>(1000 + static_cast<int>(i));
    objects.motor_diag[i] = 0x00000008U | static_cast<uint32_t>(i << 16);
    objects.motor_voltage[i] = static_cast<uint16_t>(1200U + i);
  }
  for (std::size_t i = 0; i < 7U; ++i) {
    objects.wireless[i] = static_cast<uint8_t>(0xA0U + i);
  }
  objects.color[0] = 2U;
  objects.system_state[0] = 1U;
  objects.system_state[1] = 1U;
  objects.bus_diag[0] = 0x80000000U;
  objects.bus_diag[1] = 0x80000003U;

  const HardwareSnapshot snapshot = core_hardware::snapshot_from_objects(objects);

  const std::array<uint8_t, 7> expected_wireless = {0xA0U, 0xA1U, 0xA2U, 0xA3U, 0xA4U, 0xA5U, 0xA6U};
  EXPECT_EQ(snapshot.core_wireless, expected_wireless);
  // トルク欄は無線データに使われず、全 ID のトルクがそのまま入る
  for (std::size_t i = 0; i < 15U; ++i) {
    EXPECT_EQ(snapshot.motor_state_torque[i], objects.motor_state_torque[i]) << "id=" << i;
    EXPECT_EQ(snapshot.motor_diag[i], objects.motor_diag[i]) << "id=" << i;
    EXPECT_EQ(snapshot.motor_voltage[i], objects.motor_voltage[i]) << "id=" << i;
  }
  EXPECT_EQ(snapshot.system_state[1], 1U);
  EXPECT_EQ(snapshot.bus_diag[0], 0x80000000U);
  EXPECT_EQ(snapshot.bus_diag[1], 0x80000003U);
}

TEST(HardwareSnapshotObjects, ApplyWritesWirelessObjectAndKeepsTorque) {
  const HardwareSnapshot snapshot = make_filled_snapshot();
  _Objects objects{};
  core_hardware::apply_snapshot_to_objects(snapshot, objects);

  for (std::size_t i = 0; i < 7U; ++i) {
    EXPECT_EQ(objects.wireless[i], snapshot.core_wireless[i]) << "index=" << i;
  }
  for (std::size_t i = 0; i < 15U; ++i) {
    EXPECT_EQ(objects.motor_state_torque[i], snapshot.motor_state_torque[i]) << "id=" << i;
    EXPECT_EQ(objects.motor_diag[i], snapshot.motor_diag[i]) << "id=" << i;
    EXPECT_EQ(objects.motor_voltage[i], snapshot.motor_voltage[i]) << "id=" << i;
  }
  EXPECT_EQ(objects.bus_diag[0], snapshot.bus_diag[0]);
  EXPECT_EQ(objects.bus_diag[1], snapshot.bus_diag[1]);

  // _Objects に無い daemon の状態 (ethercat_connected / last_ethercat_rx_ms) 以外は往復で一致する
  HardwareSnapshot expected = snapshot;
  expected.ethercat_connected = {};
  expected.last_ethercat_rx_ms = {};
  expect_snapshot_eq(core_hardware::snapshot_from_objects(objects), expected);
}

TEST(EthercatLink, ConnectedWhileRxWithinTimeout) {
  HardwareSnapshot state;
  state.ethercat_connected = {1U};
  state.last_ethercat_rx_ms = {10000U};

  EXPECT_EQ(core_hardware::ethercat_rx_age_ms(state, 10250), std::optional<int64_t>(250));
  EXPECT_TRUE(core_hardware::is_ethercat_connected(state, 10000));
  EXPECT_TRUE(core_hardware::is_ethercat_connected(state, 10000 + core_hardware::kEthercatRxTimeoutMs));
  EXPECT_FALSE(core_hardware::is_ethercat_connected(state, 10000 + core_hardware::kEthercatRxTimeoutMs + 1));
}

TEST(EthercatLink, NotConnectedWhenDaemonReportsDisconnected) {
  HardwareSnapshot state;
  state.ethercat_connected = {0U};
  state.last_ethercat_rx_ms = {10000U};

  EXPECT_EQ(core_hardware::ethercat_rx_age_ms(state, 10100), std::optional<int64_t>(100));
  EXPECT_FALSE(core_hardware::is_ethercat_connected(state, 10100));
}

TEST(EthercatLink, UnknownRxTime) {
  HardwareSnapshot state;
  state.ethercat_connected = {1U};

  // 受信時刻 0 は daemon が一度も受信していない (または空のスナップショット)
  state.last_ethercat_rx_ms = {0U};
  EXPECT_FALSE(core_hardware::ethercat_rx_age_ms(state, 10000).has_value());
  EXPECT_FALSE(core_hardware::is_ethercat_connected(state, 10000));

  // 受信時刻が現在より未来なら経過時間は分からない
  state.last_ethercat_rx_ms = {10001U};
  EXPECT_FALSE(core_hardware::ethercat_rx_age_ms(state, 10000).has_value());
  EXPECT_FALSE(core_hardware::is_ethercat_connected(state, 10000));
}

}  // namespace
