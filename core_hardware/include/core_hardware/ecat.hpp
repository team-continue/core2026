#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

extern "C" {
#include <soem/soem.h>
}

class Ecat {
 public:
  using FloatPacketCallback = std::function<void(uint8_t, const std::vector<float>&)>;
  using Uint8PacketCallback = std::function<void(uint8_t, const std::vector<uint8_t>&)>;

  Ecat();
  ~Ecat();

  void connect(const char* ifname);
  void close();
  void set_float_packet_callback(FloatPacketCallback callback);
  void set_uint8_packet_callback(Uint8PacketCallback callback);
  void set_float(uint8_t id, const std::vector<float>& data);
  void set_system_ref(bool enabled);
  void set_led_tape(const std::array<uint16_t, 3>& led_tape);
  void clear_tx_packets();
  bool cycle(int timeout_us, int& wkc);

 private:
  static constexpr int kSlaveIndex = 1;
  static constexpr std::size_t kIoMapSize = 4096;

  ecx_contextt ctx_{};
  char iomap_[kIoMapSize]{};
  bool connected_ = false;
  bool port_open_ = false;
  bool tx_in_flight_ = false;
  int expected_wkc_ = 0;
  int output_bytes_ = 0;
  int input_bytes_ = 0;
  std::string ifname_;

  struct OutputCache {
    float motor_ref[16]{};
    bool system_ref = false;
    uint16_t led_tape[3]{};
  } output_cache_;

  FloatPacketCallback float_packet_callback_;
  Uint8PacketCallback uint8_packet_callback_;

  void ensure_connected() const;
  void wait_for_safe_op();
  void request_operational();
  void write_outputs();
  void apply_tx_record(uint8_t id, const std::vector<float>& payload);
  void emit_rx_packets() const;

  static int position_index_from_packet_id(uint8_t id);
  static uint32_t unpack_bits(const uint8_t* buffer, std::size_t bit_offset, std::size_t bit_length);
  static void pack_bits(uint8_t* buffer, std::size_t bit_offset, std::size_t bit_length, uint32_t value);
};
