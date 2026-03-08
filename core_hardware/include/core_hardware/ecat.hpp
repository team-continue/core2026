#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

extern "C" {
#include <soem/soem.h>
}

constexpr std::size_t ECAT_BUFFER_SIZE = 32768;

class Ecat {
 public:
  Ecat();
  ~Ecat();

  void connect(const char* ifname);
  void close();
  void send(const uint8_t* buffer, std::size_t size);
  int read(uint8_t* decode_buffer);
  bool try_read(uint8_t* decode_buffer, int timeout_us, int& rx_len, int& wkc);

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

  void ensure_connected() const;
  void wait_for_safe_op();
  void request_operational();
  void write_outputs();
  void apply_tx_record(uint8_t id, const uint8_t* payload, uint8_t payload_size);
  int decode_rx_records(uint8_t* decode_buffer) const;

  static int position_index_from_packet_id(uint8_t id);
  static uint32_t unpack_bits(const uint8_t* buffer, std::size_t bit_offset, std::size_t bit_length);
  static void pack_bits(uint8_t* buffer, std::size_t bit_offset, std::size_t bit_length, uint32_t value);
};
