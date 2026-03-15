#pragma once

#include <array>
#include <cstdint>
#include <cstring>
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
};

inline HardwareSnapshot snapshot_from_objects(const _Objects& objects) {
  HardwareSnapshot snapshot;
  std::memcpy(snapshot.motor_state_pos.data(), objects.motor_state_pos, sizeof(objects.motor_state_pos));
  std::memcpy(snapshot.motor_state_vel.data(), objects.motor_state_vel, sizeof(objects.motor_state_vel));
  std::memcpy(snapshot.motor_state_torque.data(), objects.motor_state_torque, sizeof(objects.motor_state_torque));
  std::memcpy(snapshot.system_state.data(), objects.system_state, sizeof(objects.system_state));
  std::memcpy(snapshot.core_state.data(), objects.core_state, sizeof(objects.core_state));
  const auto* torque_bytes = reinterpret_cast<const uint8_t*>(objects.motor_state_torque);
  std::memcpy(snapshot.core_wireless.data(), torque_bytes, snapshot.core_wireless.size());
  std::memcpy(snapshot.motor_ref.data(), objects.motor_ref, sizeof(objects.motor_ref));
  std::memcpy(snapshot.system_ref.data(), objects.system_ref, sizeof(objects.system_ref));
  std::memcpy(snapshot.led_tape.data(), objects.LED_TAPE, sizeof(objects.LED_TAPE));
  return snapshot;
}

inline void apply_snapshot_to_objects(const HardwareSnapshot& snapshot, _Objects& objects) {
  std::memcpy(objects.motor_state_pos, snapshot.motor_state_pos.data(), sizeof(objects.motor_state_pos));
  std::memcpy(objects.motor_state_vel, snapshot.motor_state_vel.data(), sizeof(objects.motor_state_vel));
  std::memcpy(objects.motor_state_torque, snapshot.motor_state_torque.data(), sizeof(objects.motor_state_torque));
  std::memcpy(objects.system_state, snapshot.system_state.data(), sizeof(objects.system_state));
  std::memcpy(objects.core_state, snapshot.core_state.data(), sizeof(objects.core_state));
  auto* torque_bytes = reinterpret_cast<uint8_t*>(objects.motor_state_torque);
  std::memcpy(torque_bytes, snapshot.core_wireless.data(), snapshot.core_wireless.size());
  std::memcpy(objects.motor_ref, snapshot.motor_ref.data(), sizeof(objects.motor_ref));
  std::memcpy(objects.system_ref, snapshot.system_ref.data(), sizeof(objects.system_ref));
  std::memcpy(objects.LED_TAPE, snapshot.led_tape.data(), sizeof(objects.LED_TAPE));
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
  buffer.reserve(256);
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
      default:
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
