#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <vector>

extern "C" {
#include "utypes.h"
}

namespace core_hardware {

enum class SnapshotFieldId : uint16_t {
  kMotorStatePos = 1,
  kMotorStateVel = 2,
  kMotorStateTorque = 3,
  kSystemState = 4,
  kCoreState = 5,
  kCoreWireless = 6,
  kMotorRef = 7,
  kSystemRef = 8,
  kLedTape = 9,
  kEthercatConnected = 10,
  kLastEthercatRxMs = 11,
  kMotorDiag = 12,
  kMotorVoltage = 13,
  kBusDiag = 14,
};

struct HardwareSnapshot {
  std::array<float, 15> motor_state_pos{};
  std::array<int16_t, 15> motor_state_vel{};
  std::array<int16_t, 15> motor_state_torque{};
  std::array<uint8_t, 2> system_state{};
  std::array<uint8_t, 2> core_state{};
  std::array<uint8_t, 7> core_wireless{};
  std::array<float, 16> motor_ref{};
  std::array<uint8_t, 1> system_ref{};
  std::array<uint16_t, 3> led_tape{};
  std::array<uint8_t, 1> ethercat_connected{};
  std::array<uint64_t, 1> last_ethercat_rx_ms{};
  std::array<uint32_t, 15> motor_diag{};     // ビット配置は motor_diag_layout.h
  std::array<uint16_t, 15> motor_voltage{};  // 0.01 V 単位、0 = 取得できない
  std::array<uint32_t, 2> bus_diag{};        // [0] = can3, [1] = can2
};

// _Objects (utypes.h) と memcpy でやり取りするので、大きさが食い違ったらビルドを止める
static_assert(sizeof(HardwareSnapshot::motor_state_pos) == sizeof(_Objects::motor_state_pos), "motor_state_pos size mismatch");
static_assert(sizeof(HardwareSnapshot::motor_state_vel) == sizeof(_Objects::motor_state_vel), "motor_state_vel size mismatch");
static_assert(sizeof(HardwareSnapshot::motor_state_torque) == sizeof(_Objects::motor_state_torque), "motor_state_torque size mismatch");
static_assert(sizeof(HardwareSnapshot::system_state) == sizeof(_Objects::system_state), "system_state size mismatch");
static_assert(sizeof(HardwareSnapshot::core_state) == sizeof(_Objects::core_state), "core_state size mismatch");
static_assert(sizeof(HardwareSnapshot::core_wireless) == sizeof(_Objects::wireless), "wireless size mismatch");
static_assert(sizeof(HardwareSnapshot::motor_ref) == sizeof(_Objects::motor_ref), "motor_ref size mismatch");
static_assert(sizeof(HardwareSnapshot::system_ref) == sizeof(_Objects::system_ref), "system_ref size mismatch");
static_assert(sizeof(HardwareSnapshot::led_tape) == sizeof(_Objects::LED_TAPE), "LED_TAPE size mismatch");
static_assert(sizeof(HardwareSnapshot::motor_diag) == sizeof(_Objects::motor_diag), "motor_diag size mismatch");
static_assert(sizeof(HardwareSnapshot::motor_voltage) == sizeof(_Objects::motor_voltage), "motor_voltage size mismatch");
static_assert(sizeof(HardwareSnapshot::bus_diag) == sizeof(_Objects::bus_diag), "bus_diag size mismatch");

inline HardwareSnapshot snapshot_from_objects(const _Objects& objects) {
  HardwareSnapshot snapshot;
  std::memcpy(snapshot.motor_state_pos.data(), objects.motor_state_pos, sizeof(objects.motor_state_pos));
  std::memcpy(snapshot.motor_state_vel.data(), objects.motor_state_vel, sizeof(objects.motor_state_vel));
  std::memcpy(snapshot.motor_state_torque.data(), objects.motor_state_torque, sizeof(objects.motor_state_torque));
  std::memcpy(snapshot.system_state.data(), objects.system_state, sizeof(objects.system_state));
  std::memcpy(snapshot.core_state.data(), objects.core_state, sizeof(objects.core_state));
  std::memcpy(snapshot.core_wireless.data(), objects.wireless, sizeof(objects.wireless));
  std::memcpy(snapshot.motor_ref.data(), objects.motor_ref, sizeof(objects.motor_ref));
  std::memcpy(snapshot.system_ref.data(), objects.system_ref, sizeof(objects.system_ref));
  std::memcpy(snapshot.led_tape.data(), objects.LED_TAPE, sizeof(objects.LED_TAPE));
  std::memcpy(snapshot.motor_diag.data(), objects.motor_diag, sizeof(objects.motor_diag));
  std::memcpy(snapshot.motor_voltage.data(), objects.motor_voltage, sizeof(objects.motor_voltage));
  std::memcpy(snapshot.bus_diag.data(), objects.bus_diag, sizeof(objects.bus_diag));
  return snapshot;
}

inline void apply_snapshot_to_objects(const HardwareSnapshot& snapshot, _Objects& objects) {
  std::memcpy(objects.motor_state_pos, snapshot.motor_state_pos.data(), sizeof(objects.motor_state_pos));
  std::memcpy(objects.motor_state_vel, snapshot.motor_state_vel.data(), sizeof(objects.motor_state_vel));
  std::memcpy(objects.motor_state_torque, snapshot.motor_state_torque.data(), sizeof(objects.motor_state_torque));
  std::memcpy(objects.system_state, snapshot.system_state.data(), sizeof(objects.system_state));
  std::memcpy(objects.core_state, snapshot.core_state.data(), sizeof(objects.core_state));
  std::memcpy(objects.wireless, snapshot.core_wireless.data(), sizeof(objects.wireless));
  std::memcpy(objects.motor_ref, snapshot.motor_ref.data(), sizeof(objects.motor_ref));
  std::memcpy(objects.system_ref, snapshot.system_ref.data(), sizeof(objects.system_ref));
  std::memcpy(objects.LED_TAPE, snapshot.led_tape.data(), sizeof(objects.LED_TAPE));
  std::memcpy(objects.motor_diag, snapshot.motor_diag.data(), sizeof(objects.motor_diag));
  std::memcpy(objects.motor_voltage, snapshot.motor_voltage.data(), sizeof(objects.motor_voltage));
  std::memcpy(objects.bus_diag, snapshot.bus_diag.data(), sizeof(objects.bus_diag));
}

// EtherCAT の最終受信からこの時間を超えたら通信断とみなす [ms]
constexpr int64_t kEthercatRxTimeoutMs = 1000;

// EtherCAT の最終受信からの経過時間 [ms]。受信時刻が不明 (0) か now_ms より未来なら nullopt
inline std::optional<int64_t> ethercat_rx_age_ms(const HardwareSnapshot& state, const int64_t now_ms) {
  const uint64_t last_rx_ms = state.last_ethercat_rx_ms[0];
  if (last_rx_ms == 0U || now_ms < static_cast<int64_t>(last_rx_ms)) {
    return std::nullopt;
  }
  return now_ms - static_cast<int64_t>(last_rx_ms);
}

// daemon が EtherCAT 接続中と報告していて、最終受信が kEthercatRxTimeoutMs 以内なら通信中とみなす。
// now_ms は daemon が last_ethercat_rx_ms に使う時計 (system_clock の epoch ms) で渡す
inline bool is_ethercat_connected(const HardwareSnapshot& state, const int64_t now_ms) {
  const std::optional<int64_t> age_ms = ethercat_rx_age_ms(state, now_ms);
  return state.ethercat_connected[0] != 0U && age_ms.has_value() && *age_ms <= kEthercatRxTimeoutMs;
}

struct SnapshotFieldHeader {
  uint16_t field_id;
  uint16_t element_size;
  uint32_t element_count;
};

namespace detail {

template <typename T>
inline void append_field(std::vector<uint8_t>& buffer, SnapshotFieldId field_id, const T* data, std::size_t count) {
  const SnapshotFieldHeader header{
      static_cast<uint16_t>(field_id),
      static_cast<uint16_t>(sizeof(T)),
      static_cast<uint32_t>(count),
  };
  const auto* header_ptr = reinterpret_cast<const uint8_t*>(&header);
  buffer.insert(buffer.end(), header_ptr, header_ptr + sizeof(header));
  const auto* bytes = reinterpret_cast<const uint8_t*>(data);
  buffer.insert(buffer.end(), bytes, bytes + (sizeof(T) * count));
}

template <typename T, std::size_t N>
inline void read_field(const uint8_t* payload, std::size_t bytes, std::array<T, N>& destination) {
  if (bytes != sizeof(T) * N) {
    throw std::runtime_error("Snapshot field size mismatch");
  }
  std::memcpy(destination.data(), payload, bytes);
}

}  // namespace detail

inline std::vector<uint8_t> encode_snapshot(const HardwareSnapshot& snapshot) {
  std::vector<uint8_t> buffer;
  buffer.reserve(512);
  detail::append_field(buffer, SnapshotFieldId::kMotorStatePos, snapshot.motor_state_pos.data(), snapshot.motor_state_pos.size());
  detail::append_field(buffer, SnapshotFieldId::kMotorStateVel, snapshot.motor_state_vel.data(), snapshot.motor_state_vel.size());
  detail::append_field(buffer, SnapshotFieldId::kMotorStateTorque, snapshot.motor_state_torque.data(), snapshot.motor_state_torque.size());
  detail::append_field(buffer, SnapshotFieldId::kSystemState, snapshot.system_state.data(), snapshot.system_state.size());
  detail::append_field(buffer, SnapshotFieldId::kCoreState, snapshot.core_state.data(), snapshot.core_state.size());
  detail::append_field(buffer, SnapshotFieldId::kCoreWireless, snapshot.core_wireless.data(), snapshot.core_wireless.size());
  detail::append_field(buffer, SnapshotFieldId::kMotorRef, snapshot.motor_ref.data(), snapshot.motor_ref.size());
  detail::append_field(buffer, SnapshotFieldId::kSystemRef, snapshot.system_ref.data(), snapshot.system_ref.size());
  detail::append_field(buffer, SnapshotFieldId::kLedTape, snapshot.led_tape.data(), snapshot.led_tape.size());
  detail::append_field(buffer, SnapshotFieldId::kEthercatConnected, snapshot.ethercat_connected.data(), snapshot.ethercat_connected.size());
  detail::append_field(buffer, SnapshotFieldId::kLastEthercatRxMs, snapshot.last_ethercat_rx_ms.data(), snapshot.last_ethercat_rx_ms.size());
  detail::append_field(buffer, SnapshotFieldId::kMotorDiag, snapshot.motor_diag.data(), snapshot.motor_diag.size());
  detail::append_field(buffer, SnapshotFieldId::kMotorVoltage, snapshot.motor_voltage.data(), snapshot.motor_voltage.size());
  detail::append_field(buffer, SnapshotFieldId::kBusDiag, snapshot.bus_diag.data(), snapshot.bus_diag.size());
  return buffer;
}

inline HardwareSnapshot decode_snapshot(const std::vector<uint8_t>& payload) {
  HardwareSnapshot snapshot;
  std::size_t offset = 0;
  while ((offset + sizeof(SnapshotFieldHeader)) <= payload.size()) {
    SnapshotFieldHeader header{};
    std::memcpy(&header, payload.data() + offset, sizeof(header));
    offset += sizeof(header);
    const std::size_t field_bytes = static_cast<std::size_t>(header.element_size) * header.element_count;
    if ((offset + field_bytes) > payload.size()) {
      throw std::runtime_error("Truncated snapshot payload");
    }
    const uint8_t* field_payload = payload.data() + offset;
    switch (static_cast<SnapshotFieldId>(header.field_id)) {
      case SnapshotFieldId::kMotorStatePos:
        detail::read_field(field_payload, field_bytes, snapshot.motor_state_pos);
        break;
      case SnapshotFieldId::kMotorStateVel:
        detail::read_field(field_payload, field_bytes, snapshot.motor_state_vel);
        break;
      case SnapshotFieldId::kMotorStateTorque:
        detail::read_field(field_payload, field_bytes, snapshot.motor_state_torque);
        break;
      case SnapshotFieldId::kSystemState:
        detail::read_field(field_payload, field_bytes, snapshot.system_state);
        break;
      case SnapshotFieldId::kCoreState:
        detail::read_field(field_payload, field_bytes, snapshot.core_state);
        break;
      case SnapshotFieldId::kCoreWireless:
        detail::read_field(field_payload, field_bytes, snapshot.core_wireless);
        break;
      case SnapshotFieldId::kMotorRef:
        detail::read_field(field_payload, field_bytes, snapshot.motor_ref);
        break;
      case SnapshotFieldId::kSystemRef:
        detail::read_field(field_payload, field_bytes, snapshot.system_ref);
        break;
      case SnapshotFieldId::kLedTape:
        detail::read_field(field_payload, field_bytes, snapshot.led_tape);
        break;
      case SnapshotFieldId::kEthercatConnected:
        detail::read_field(field_payload, field_bytes, snapshot.ethercat_connected);
        break;
      case SnapshotFieldId::kLastEthercatRxMs:
        detail::read_field(field_payload, field_bytes, snapshot.last_ethercat_rx_ms);
        break;
      case SnapshotFieldId::kMotorDiag:
        detail::read_field(field_payload, field_bytes, snapshot.motor_diag);
        break;
      case SnapshotFieldId::kMotorVoltage:
        detail::read_field(field_payload, field_bytes, snapshot.motor_voltage);
        break;
      case SnapshotFieldId::kBusDiag:
        detail::read_field(field_payload, field_bytes, snapshot.bus_diag);
        break;
      default:
        // 知らない field は読み飛ばす (daemon と ROS ノードの版が違っても読めるように)
        break;
    }
    offset += field_bytes;
  }
  if (offset != payload.size()) {
    throw std::runtime_error("Trailing bytes in snapshot payload");
  }
  return snapshot;
}

}  // namespace core_hardware
