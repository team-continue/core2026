#include "core_hardware/ecat.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "core_hardware/motor_diag_layout.h"

extern "C" {
#include "utypes.h"
}

namespace {
constexpr std::size_t kOutputMotorRefOffset = 0U;
constexpr std::size_t kOutputMotorRefBytes = 16U * sizeof(float);
constexpr std::size_t kOutputSystemRefBitOffset = kOutputMotorRefBytes * 8U;
constexpr std::size_t kOutputLedTape0BitOffset = kOutputSystemRefBitOffset + 1U;
constexpr std::size_t kOutputLedTape1BitOffset = kOutputLedTape0BitOffset + 16U;
constexpr std::size_t kOutputLedTape2BitOffset = kOutputLedTape1BitOffset + 16U;
constexpr std::size_t kOutputRequiredBytes = (kOutputLedTape2BitOffset + 16U + 7U) / 8U;

// TxPDO (upper 基板 → PC) の配置。esi.json / objectlist.c / utypes.h と同じ順に並べる
constexpr std::size_t kInputMotorPosOffset = 0U;
constexpr std::size_t kInputMotorPosBytes = 15U * sizeof(float);
constexpr std::size_t kInputMotorVelOffset = kInputMotorPosOffset + kInputMotorPosBytes;
constexpr std::size_t kInputMotorVelBytes = 15U * sizeof(int16_t);
constexpr std::size_t kInputMotorTorqueOffset = kInputMotorVelOffset + kInputMotorVelBytes;
constexpr std::size_t kInputMotorTorqueBytes = 15U * sizeof(int16_t);
constexpr std::size_t kInputSystemStateOffset = kInputMotorTorqueOffset + kInputMotorTorqueBytes;
constexpr std::size_t kInputSystemStateBytes = 2U * sizeof(uint8_t);
constexpr std::size_t kInputCoreStateOffset = kInputSystemStateOffset + kInputSystemStateBytes;
constexpr std::size_t kInputCoreStateBytes = 2U * sizeof(uint8_t);
constexpr std::size_t kInputMotorDiagOffset = kInputCoreStateOffset + kInputCoreStateBytes;
constexpr std::size_t kInputMotorDiagBytes = 15U * sizeof(uint32_t);
constexpr std::size_t kInputMotorVoltageOffset = kInputMotorDiagOffset + kInputMotorDiagBytes;
constexpr std::size_t kInputMotorVoltageBytes = 15U * sizeof(uint16_t);
constexpr std::size_t kInputWirelessOffset = kInputMotorVoltageOffset + kInputMotorVoltageBytes;
constexpr std::size_t kInputWirelessBytes = 7U * sizeof(uint8_t);
constexpr std::size_t kInputColorOffset = kInputWirelessOffset + kInputWirelessBytes;
constexpr std::size_t kInputColorBytes = 1U * sizeof(uint8_t);
constexpr std::size_t kInputBusDiagOffset = kInputColorOffset + kInputColorBytes;
constexpr std::size_t kInputBusDiagBytes = 2U * sizeof(uint32_t);
constexpr std::size_t kInputRequiredBytes = kInputBusDiagOffset + kInputBusDiagBytes;

// ESI RevisionNumber 3 (core_hardware::motor_diag::kEsiRevision) の TxPDO 配置
static_assert(kInputSystemStateOffset == 120U, "system_state must start at byte 120");
static_assert(kInputCoreStateOffset == 122U, "core_state must start at byte 122");
static_assert(kInputMotorDiagOffset == 124U, "motor_diag must start at byte 124");
static_assert(kInputMotorVoltageOffset == 184U, "motor_voltage must start at byte 184");
static_assert(kInputWirelessOffset == 214U, "wireless must start at byte 214");
static_assert(kInputColorOffset == 221U, "color must start at byte 221");
static_assert(kInputBusDiagOffset == 222U, "bus_diag must start at byte 222");
static_assert(kInputRequiredBytes == 230U, "TxPDO must be 230 bytes");
static_assert(sizeof(_Objects::system_state) == kInputSystemStateBytes, "utypes.h system_state size mismatch");
static_assert(sizeof(_Objects::core_state) == kInputCoreStateBytes, "utypes.h core_state size mismatch");
static_assert(sizeof(_Objects::motor_diag) == kInputMotorDiagBytes, "utypes.h motor_diag size mismatch");
static_assert(sizeof(_Objects::motor_voltage) == kInputMotorVoltageBytes, "utypes.h motor_voltage size mismatch");
static_assert(sizeof(_Objects::wireless) == kInputWirelessBytes, "utypes.h wireless size mismatch");
static_assert(sizeof(_Objects::color) == kInputColorBytes, "utypes.h color size mismatch");
static_assert(sizeof(_Objects::bus_diag) == kInputBusDiagBytes, "utypes.h bus_diag size mismatch");

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

int16_t read_int16_unaligned(const uint8_t* buffer, std::size_t offset) {
  int16_t value = 0;
  std::memcpy(&value, buffer + offset, sizeof(value));
  return value;
}

uint16_t read_uint16_unaligned(const uint8_t* buffer, std::size_t offset) {
  uint16_t value = 0;
  std::memcpy(&value, buffer + offset, sizeof(value));
  return value;
}

uint32_t read_uint32_unaligned(const uint8_t* buffer, std::size_t offset) {
  uint32_t value = 0;
  std::memcpy(&value, buffer + offset, sizeof(value));
  return value;
}

float read_float_unaligned(const uint8_t* buffer, std::size_t offset) {
  float value = 0.0f;
  std::memcpy(&value, buffer + offset, sizeof(value));
  return value;
}

void write_float_unaligned(uint8_t* buffer, std::size_t offset, float value) {
  std::memcpy(buffer + offset, &value, sizeof(value));
}

void write_uint16_unaligned(uint8_t* buffer, std::size_t offset, uint16_t value) {
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

  // PDO 配置の版を 2 段で確かめ、どちらかが違えば値を読み違えるので通信を始めない。
  // - EEPROM (SII) の RevisionNumber (eep_rev): esi.json から書いた EEPROM が今の配置の版か
  // - 下の Ibytes: SOEM は CoE の SDO でファームの PDO 構成を読んでサイズを決めるので、ファームが今の配置か
  const uint32_t eep_rev = ctx_.slavelist[kSlaveIndex].eep_rev;
  if (eep_rev != core_hardware::motor_diag::kEsiRevision) {
    std::ostringstream oss;
    oss << "EtherCAT slave revision mismatch: expected RevisionNumber=0x" << std::hex
        << core_hardware::motor_diag::kEsiRevision << " but slave " << std::dec << kSlaveIndex
        << " reports eep_rev=0x" << std::hex << eep_rev << std::dec
        << " (from its EEPROM/SII). The upper board was set up for a different PDO layout than this core_hardware build."
        << " Rewrite the slave EEPROM from the current esi.json and flash the matching upper firmware.";
    close();
    throw std::runtime_error(oss.str());
  }

  ecx_config_map_group(&ctx_, iomap_, 0);
  output_bytes_ = ctx_.slavelist[kSlaveIndex].Obytes;
  input_bytes_ = ctx_.slavelist[kSlaveIndex].Ibytes;
  if (output_bytes_ < static_cast<int>(kOutputRequiredBytes) || input_bytes_ < static_cast<int>(kInputRequiredBytes)) {
    std::ostringstream oss;
    oss << "PDO size mismatch Obytes=" << output_bytes_ << " (expected >= " << kOutputRequiredBytes << ")"
        << " Ibytes=" << input_bytes_ << " (expected >= " << kInputRequiredBytes << ")."
        << " The PDO mapping read from the upper firmware (CoE) does not match this core_hardware build;"
        << " flash the matching upper firmware.";
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

void Ecat::set_float_packet_callback(FloatPacketCallback callback) {
  float_packet_callback_ = std::move(callback);
}

void Ecat::set_uint8_packet_callback(Uint8PacketCallback callback) {
  uint8_packet_callback_ = std::move(callback);
}

void Ecat::set_float(uint8_t id, const std::vector<float>& data) {
  apply_tx_record(id, data);
}

void Ecat::set_system_ref(bool enabled) {
  output_cache_.system_ref = enabled;
  Obj.system_ref[0] = enabled ? 1U : 0U;
}

void Ecat::set_led_tape(const std::array<uint16_t, 3>& led_tape) {
  for (std::size_t i = 0; i < led_tape.size(); ++i) {
    output_cache_.led_tape[i] = led_tape[i];
    Obj.LED_TAPE[i] = led_tape[i];
  }
}

void Ecat::clear_tx_packets() {
  std::memset(output_cache_.motor_ref, 0, sizeof(output_cache_.motor_ref));
  output_cache_.system_ref = false;
  std::memset(output_cache_.led_tape, 0, sizeof(output_cache_.led_tape));
  std::memset(Obj.motor_ref, 0, sizeof(Obj.motor_ref));
  std::memset(Obj.system_ref, 0, sizeof(Obj.system_ref));
  std::memset(Obj.LED_TAPE, 0, sizeof(Obj.LED_TAPE));
}

bool Ecat::cycle(int timeout_us, int& wkc) {
  ensure_connected();
  if (!tx_in_flight_) {
    write_outputs();
    ecx_send_processdata(&ctx_);
    tx_in_flight_ = true;
  }

  wkc = ecx_receive_processdata(&ctx_, timeout_us);
  tx_in_flight_ = false;
  if (wkc <= 0 || wkc < expected_wkc_) {
    return false;
  }
  emit_rx_packets();
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
    write_float_unaligned(outputs, kOutputMotorRefOffset + (index * sizeof(float)), output_cache_.motor_ref[index]);
    Obj.motor_ref[index] = output_cache_.motor_ref[index];
  }
  pack_bits(outputs, kOutputSystemRefBitOffset, 1U, output_cache_.system_ref ? 1U : 0U);
  Obj.system_ref[0] = output_cache_.system_ref ? 1U : 0U;
  pack_bits(outputs, kOutputLedTape0BitOffset, 16U, output_cache_.led_tape[0]);
  pack_bits(outputs, kOutputLedTape1BitOffset, 16U, output_cache_.led_tape[1]);
  pack_bits(outputs, kOutputLedTape2BitOffset, 16U, output_cache_.led_tape[2]);
  for (std::size_t index = 0; index < 3U; ++index) {
    Obj.LED_TAPE[index] = output_cache_.led_tape[index];
  }
}

void Ecat::apply_tx_record(uint8_t id, const std::vector<float>& payload) {
  static_assert(std::numeric_limits<int16_t>::max() >= 512, "int16_t must represent velocity range");
  static_assert(std::numeric_limits<int16_t>::max() >= 100, "int16_t must represent torque range");
  const float value = payload.empty() ? 0.0f : payload.back();

  if (id <= 15U) {
    output_cache_.motor_ref[id] = value;
    Obj.motor_ref[id] = value;
    return;
  }
  if (id == 17U) {
    output_cache_.system_ref = (value != 0.0f);
    Obj.system_ref[0] = output_cache_.system_ref ? 1U : 0U;
  }
}

void Ecat::emit_rx_packets() const {
  const auto* inputs = reinterpret_cast<const uint8_t*>(ctx_.slavelist[kSlaveIndex].inputs);
  if (inputs == nullptr) {
    throw std::runtime_error("EtherCAT inputs PDO is null");
  }

  for (uint8_t id : kPacketJointIds) {
    const int16_t velocity = read_int16_unaligned(inputs, kInputMotorVelOffset + (static_cast<std::size_t>(id) * sizeof(int16_t)));
    const int16_t torque = read_int16_unaligned(inputs, kInputMotorTorqueOffset + (static_cast<std::size_t>(id) * sizeof(int16_t)));
    const float position = read_float_unaligned(inputs, kInputMotorPosOffset + (static_cast<std::size_t>(id) * sizeof(float)));
    Obj.motor_state_pos[id] = position;
    Obj.motor_state_vel[id] = velocity;
    Obj.motor_state_torque[id] = torque;

    if (float_packet_callback_) {
      std::vector<float> frame = {0.0f, static_cast<float>(torque), 0.0f, static_cast<float>(velocity), 0.0f, position};
      if (id <= 4U) {
        frame[2] = output_cache_.motor_ref[id];
      } else {
        frame[4] = output_cache_.motor_ref[id];
      }
      float_packet_callback_(id, frame);
    }
  }

  // system_state: [0] = hardware_enable (1:通常, 0:非常停止), [1] = bottom 基板との CAN 接続 (1:接続)
  for (std::size_t i = 0; i < std::size(Obj.system_state); ++i) {
    Obj.system_state[i] = inputs[kInputSystemStateOffset + i];
  }
  for (std::size_t i = 0; i < std::size(Obj.core_state); ++i) {
    Obj.core_state[i] = inputs[kInputCoreStateOffset + i];
  }
  for (std::size_t i = 0; i < std::size(Obj.motor_diag); ++i) {
    Obj.motor_diag[i] = read_uint32_unaligned(inputs, kInputMotorDiagOffset + (i * sizeof(uint32_t)));
  }
  for (std::size_t i = 0; i < std::size(Obj.motor_voltage); ++i) {
    Obj.motor_voltage[i] = read_uint16_unaligned(inputs, kInputMotorVoltageOffset + (i * sizeof(uint16_t)));
  }
  std::memcpy(Obj.wireless, inputs + kInputWirelessOffset, sizeof(Obj.wireless));
  std::memcpy(Obj.color, inputs + kInputColorOffset, sizeof(Obj.color));
  for (std::size_t i = 0; i < std::size(Obj.bus_diag); ++i) {
    Obj.bus_diag[i] = read_uint32_unaligned(inputs, kInputBusDiagOffset + (i * sizeof(uint32_t)));
  }

  if (uint8_packet_callback_) {
    uint8_packet_callback_(100U, {Obj.core_state[0]});
    uint8_packet_callback_(101U, {Obj.core_state[1]});
    uint8_packet_callback_(102U, std::vector<uint8_t>(std::begin(Obj.wireless), std::end(Obj.wireless)));
    uint8_packet_callback_(103U, {Obj.color[0]});
    uint8_packet_callback_(104U, {Obj.system_state[0]});
  }
}

int Ecat::position_index_from_packet_id(uint8_t id) {
  (void)id;
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
