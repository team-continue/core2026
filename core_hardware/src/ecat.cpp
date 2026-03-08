#include "core_hardware/ecat.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <stdexcept>

extern "C" {
#include "utypes.h"
}

namespace {
constexpr std::size_t kOutputMotorRefBits = 16U * 32U;
constexpr std::size_t kOutputSystemRefBit = kOutputMotorRefBits;
constexpr std::size_t kOutputLedTapeBit = kOutputSystemRefBit + 1U;
constexpr std::size_t kOutputRequiredBytes = (kOutputLedTapeBit + (3U * 16U) + 7U) / 8U;

constexpr std::size_t kInputMotorPosBits = 15U * 32U;
constexpr std::size_t kInputMotorVelBit = kInputMotorPosBits;
constexpr std::size_t kInputMotorTorqueBit = kInputMotorVelBit + (15U * 16U);
constexpr std::size_t kInputSystemStateBit = kInputMotorTorqueBit + (15U * 16U);
constexpr std::size_t kInputCoreStateBit = kInputSystemStateBit + 2U;
constexpr std::size_t kInputRequiredBytes = (kInputCoreStateBit + (2U * 8U) + 7U) / 8U;

constexpr std::array<uint8_t, 15> kPacketJointIds = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};

void throw_state_error(const char* prefix, const ecx_contextt& ctx) {
  std::ostringstream oss;
  oss << prefix;
  ecx_readstate(const_cast<ecx_contextt*>(&ctx));
  for (int slave = 1; slave <= ctx.slavecount; ++slave) {
    const ec_slavet& entry = ctx.slavelist[slave];
    oss << " slave=" << slave << " state=0x" << std::hex << static_cast<int>(entry.state)
        << " al=0x" << entry.ALstatuscode << "(" << ec_ALstatuscode2string(entry.ALstatuscode) << ")";
  }
  throw std::runtime_error(oss.str());
}

float read_float_unaligned(const uint8_t* buffer, std::size_t offset) {
  float value = 0.0f;
  std::memcpy(&value, buffer + offset, sizeof(value));
  return value;
}

int16_t read_int16_bits(const uint8_t* buffer, std::size_t bit_offset) {
  uint16_t raw = 0;
  for (std::size_t bit = 0; bit < 16U; ++bit) {
    const std::size_t absolute_bit = bit_offset + bit;
    const uint8_t bit_value = (buffer[absolute_bit / 8U] >> (absolute_bit % 8U)) & 0x01U;
    raw |= static_cast<uint16_t>(bit_value) << bit;
  }
  return static_cast<int16_t>(raw);
}

void write_float_unaligned(uint8_t* buffer, std::size_t offset, float value) {
  std::memcpy(buffer + offset, &value, sizeof(value));
}
}  // namespace

extern "C" {
_Objects Obj = {};
}

Ecat::Ecat() = default;

Ecat::~Ecat() {
  close();
}

void Ecat::connect(const char* ifname) {
  close();
  ifname_ = ifname != nullptr ? ifname : "";
  if (ifname_.empty()) {
    throw std::runtime_error("EtherCAT interface name is empty");
  }

  if (!ecx_init(&ctx_, const_cast<char*>(ifname_.c_str()))) {
    throw std::runtime_error("ecx_init failed");
  }
  port_open_ = true;

  if (ecx_config_init(&ctx_) <= 0) {
    close();
    throw std::runtime_error("No EtherCAT slaves found");
  }
  if (ctx_.slavecount < kSlaveIndex) {
    close();
    throw std::runtime_error("Expected at least one EtherCAT slave");
  }

  ecx_config_map_group(&ctx_, iomap_, 0);
  output_bytes_ = ctx_.slavelist[kSlaveIndex].Obytes;
  input_bytes_ = ctx_.slavelist[kSlaveIndex].Ibytes;
  if (output_bytes_ < static_cast<int>(kOutputRequiredBytes) || input_bytes_ < static_cast<int>(kInputRequiredBytes)) {
    std::ostringstream oss;
    oss << "PDO size mismatch Obytes=" << output_bytes_ << " Ibytes=" << input_bytes_;
    close();
    throw std::runtime_error(oss.str());
  }

  expected_wkc_ = (ctx_.grouplist[0].outputsWKC * 2) + ctx_.grouplist[0].inputsWKC;
  wait_for_safe_op();
  write_outputs();
  request_operational();
  connected_ = true;
  tx_in_flight_ = false;
}

void Ecat::close() {
  if (port_open_) {
    ctx_.slavelist[0].state = EC_STATE_INIT;
    ecx_writestate(&ctx_, 0);
    ecx_close(&ctx_);
  }
  connected_ = false;
  port_open_ = false;
  tx_in_flight_ = false;
  expected_wkc_ = 0;
  output_bytes_ = 0;
  input_bytes_ = 0;
  std::memset(&ctx_, 0, sizeof(ctx_));
  std::memset(iomap_, 0, sizeof(iomap_));
}

void Ecat::send(const uint8_t* buffer, std::size_t size) {
  ensure_connected();
  if (buffer == nullptr && size != 0U) {
    throw std::runtime_error("send buffer is null");
  }

  std::size_t offset = 0;
  while (offset < size) {
    if ((size - offset) < 2U) {
      throw std::runtime_error("Truncated packet header");
    }
    const uint8_t id = buffer[offset];
    const uint8_t payload_size = buffer[offset + 1U];
    offset += 2U;
    if ((size - offset) < payload_size) {
      throw std::runtime_error("Truncated packet payload");
    }
    apply_tx_record(id, buffer + offset, payload_size);
    offset += payload_size;
  }

  write_outputs();
  ecx_send_processdata(&ctx_);
  tx_in_flight_ = true;
}

int Ecat::read(uint8_t* decode_buffer) {
  int rx_len = 0;
  int wkc = 0;
  if (try_read(decode_buffer, EC_TIMEOUTRET, rx_len, wkc)) {
    return rx_len;
  }
  if (wkc <= 0) {
    throw std::runtime_error("Failed to receive process data");
  }
  std::ostringstream oss;
  oss << "Process data WKC mismatch wkc=" << wkc << " expected>=" << expected_wkc_;
  throw std::runtime_error(oss.str());
}

bool Ecat::try_read(uint8_t* decode_buffer, int timeout_us, int& rx_len, int& wkc) {
  ensure_connected();
  if (decode_buffer == nullptr) {
    throw std::runtime_error("read buffer is null");
  }

  if (!tx_in_flight_) {
    write_outputs();
    ecx_send_processdata(&ctx_);
    tx_in_flight_ = true;
  }

  wkc = ecx_receive_processdata(&ctx_, timeout_us);
  tx_in_flight_ = false;
  if (wkc <= 0 || wkc < expected_wkc_) {
    rx_len = 0;
    return false;
  }
  rx_len = decode_rx_records(decode_buffer);
  return true;
}

void Ecat::ensure_connected() const {
  if (!connected_) {
    throw std::runtime_error("EtherCAT master is not connected");
  }
}

void Ecat::wait_for_safe_op() {
  ctx_.slavelist[0].state = EC_STATE_SAFE_OP;
  ecx_writestate(&ctx_, 0);
  ecx_statecheck(&ctx_, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 3);
  ecx_readstate(&ctx_);
  if ((ctx_.slavelist[0].state & EC_STATE_SAFE_OP) == 0) {
    throw_state_error("Failed to reach SAFE-OP", ctx_);
  }
}

void Ecat::request_operational() {
  for (int cycle = 0; cycle < 20; ++cycle) {
    ecx_send_processdata(&ctx_);
    ecx_receive_processdata(&ctx_, EC_TIMEOUTRET);
    osal_usleep(1000);
  }

  for (int slave = 1; slave <= ctx_.slavecount; ++slave) {
    ctx_.slavelist[slave].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ctx_, slave);
  }
  ecx_statecheck(&ctx_, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE * 5);
  ecx_readstate(&ctx_);
  if (ctx_.slavelist[0].state != EC_STATE_OPERATIONAL) {
    throw_state_error("Failed to reach OP", ctx_);
  }
}

void Ecat::write_outputs() {
  auto* outputs = reinterpret_cast<uint8_t*>(ctx_.slavelist[kSlaveIndex].outputs);
  if (outputs == nullptr) {
    throw std::runtime_error("EtherCAT outputs PDO is null");
  }

  std::memset(outputs, 0, static_cast<std::size_t>(output_bytes_));
  for (std::size_t index = 0; index < 16U; ++index) {
    write_float_unaligned(outputs, index * sizeof(float), output_cache_.motor_ref[index]);
    Obj.motor_ref[index] = output_cache_.motor_ref[index];
  }
  pack_bits(outputs, kOutputSystemRefBit, 1U, output_cache_.system_ref ? 1U : 0U);
  Obj.system_ref[0] = output_cache_.system_ref ? 1U : 0U;
  for (std::size_t index = 0; index < 3U; ++index) {
    pack_bits(outputs, kOutputLedTapeBit + (index * 16U), 16U, output_cache_.led_tape[index]);
    Obj.LED_TAPE[index] = output_cache_.led_tape[index];
  }
}

void Ecat::apply_tx_record(uint8_t id, const uint8_t* payload, uint8_t payload_size) {
  static_assert(std::numeric_limits<int16_t>::max() >= 512, "int16_t must represent velocity range");
  static_assert(std::numeric_limits<int16_t>::max() >= 100, "int16_t must represent torque range");
  auto read_last_float = [payload, payload_size]() {
    float value = 0.0f;
    if (payload_size >= sizeof(float)) {
      std::memcpy(&value, payload + (payload_size - sizeof(float)), sizeof(value));
    }
    return value;
  };

  if (id <= 15U) {
    output_cache_.motor_ref[id] = (payload_size == 0U) ? 0.0f : read_last_float();
    return;
  }
  if (id == 17U) {
    if (payload_size == 1U) {
      output_cache_.system_ref = payload[0] != 0U;
    } else {
      output_cache_.system_ref = read_last_float() != 0.0f;
    }
    return;
  }
}

int Ecat::decode_rx_records(uint8_t* decode_buffer) const {
  const auto* inputs = reinterpret_cast<const uint8_t*>(ctx_.slavelist[kSlaveIndex].inputs);
  if (inputs == nullptr) {
    throw std::runtime_error("EtherCAT inputs PDO is null");
  }

  int offset = 0;
  auto append_uint8 = [&](uint8_t id, uint8_t value) {
    if ((offset + 3) > static_cast<int>(ECAT_BUFFER_SIZE)) {
      throw std::runtime_error("RX packet buffer overflow");
    }
    decode_buffer[offset++] = id;
    decode_buffer[offset++] = 1U;
    decode_buffer[offset++] = value;
  };
  auto append_float_array = [&](uint8_t id, const std::array<float, 6>& values) {
    const uint8_t payload_size = static_cast<uint8_t>(sizeof(float) * values.size());
    if ((offset + 2 + payload_size) > static_cast<int>(ECAT_BUFFER_SIZE)) {
      throw std::runtime_error("RX packet buffer overflow");
    }
    decode_buffer[offset++] = id;
    decode_buffer[offset++] = payload_size;
    std::memcpy(decode_buffer + offset, values.data(), payload_size);
    offset += payload_size;
  };

  for (uint8_t id : kPacketJointIds) {
    const int position_index = position_index_from_packet_id(id);
    const int16_t velocity = read_int16_bits(inputs, kInputMotorVelBit + (static_cast<std::size_t>(id) * 16U));
    const int16_t torque = read_int16_bits(inputs, kInputMotorTorqueBit + (static_cast<std::size_t>(id) * 16U));

    float position = 0.0f;
    if (position_index >= 0) {
      position = read_float_unaligned(inputs, static_cast<std::size_t>(position_index) * sizeof(float));
      Obj.motor_state_pos[position_index] = position;
    }
    Obj.motor_state_vel[id] = velocity;
    Obj.motor_state_torque[id] = torque;

    std::array<float, 6> frame = {0.0f, static_cast<float>(torque), 0.0f, static_cast<float>(velocity), 0.0f, position};
    if (id <= 4U) {
      frame[2] = output_cache_.motor_ref[id];
    } else {
      frame[4] = output_cache_.motor_ref[id];
    }
    append_float_array(id, frame);
  }

  const uint8_t system_upper = static_cast<uint8_t>(unpack_bits(inputs, kInputSystemStateBit, 1U));
  const uint8_t system_bottom = static_cast<uint8_t>(unpack_bits(inputs, kInputSystemStateBit + 1U, 1U));
  const uint8_t core_damage = static_cast<uint8_t>(unpack_bits(inputs, kInputCoreStateBit, 8U));
  const uint8_t core_destroy = static_cast<uint8_t>(unpack_bits(inputs, kInputCoreStateBit + 8U, 8U));

  Obj.system_state[0] = system_upper;
  Obj.system_state[1] = system_bottom;
  Obj.core_state[0] = core_damage;
  Obj.core_state[1] = core_destroy;

  append_uint8(100U, core_damage);
  append_uint8(101U, core_destroy);
  append_uint8(104U, system_upper);
  return offset;
}

int Ecat::position_index_from_packet_id(uint8_t id) {
  if (id <= 14U) {
    return static_cast<int>(id);
  }
  return -1;
}

uint32_t Ecat::unpack_bits(const uint8_t* buffer, std::size_t bit_offset, std::size_t bit_length) {
  uint32_t value = 0;
  for (std::size_t bit = 0; bit < bit_length; ++bit) {
    const std::size_t absolute_bit = bit_offset + bit;
    const uint8_t bit_value = (buffer[absolute_bit / 8U] >> (absolute_bit % 8U)) & 0x01U;
    value |= static_cast<uint32_t>(bit_value) << bit;
  }
  return value;
}

void Ecat::pack_bits(uint8_t* buffer, std::size_t bit_offset, std::size_t bit_length, uint32_t value) {
  for (std::size_t bit = 0; bit < bit_length; ++bit) {
    const std::size_t absolute_bit = bit_offset + bit;
    const uint8_t mask = static_cast<uint8_t>(1U << (absolute_bit % 8U));
    if (((value >> bit) & 0x01U) != 0U) {
      buffer[absolute_bit / 8U] |= mask;
    } else {
      buffer[absolute_bit / 8U] &= static_cast<uint8_t>(~mask);
    }
  }
}
