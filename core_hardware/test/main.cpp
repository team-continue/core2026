#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <cstring>
#include <vector>

#include "core_hardware/ecat.hpp"

namespace {

void append_float_packet(std::vector<uint8_t>& buffer, uint8_t id, float value) {
  const auto* raw = reinterpret_cast<const uint8_t*>(&value);
  buffer.push_back(id);
  buffer.push_back(static_cast<uint8_t>(sizeof(value)));
  buffer.insert(buffer.end(), raw, raw + sizeof(value));
}

void append_uint8_packet(std::vector<uint8_t>& buffer, uint8_t id, uint8_t value) {
  buffer.push_back(id);
  buffer.push_back(1U);
  buffer.push_back(value);
}

void print_float_record(uint8_t id, const uint8_t* payload, uint8_t size) {
  const std::size_t count = static_cast<std::size_t>(size) / sizeof(float);
  std::cout << "/can/rx id=" << static_cast<int>(id) << " data=[";
  for (std::size_t i = 0; i < count; ++i) {
    float value = 0.0f;
    std::memcpy(&value, payload + (i * sizeof(float)), sizeof(float));
    if (i != 0U) {
      std::cout << ", ";
    }
    std::cout << std::fixed << std::setprecision(3) << value;
  }
  std::cout << "]" << std::endl;
}

void print_u8_record(uint8_t id, const uint8_t* payload, uint8_t size) {
  std::cout << "/can/rx id=" << static_cast<int>(id) << " data=[";
  for (uint8_t i = 0; i < size; ++i) {
    if (i != 0U) {
      std::cout << ", ";
    }
    std::cout << static_cast<float>(payload[i]);
  }
  std::cout << "]" << std::endl;
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <ifname>" << std::endl;
    return 1;
  }

  try {
    Ecat ecat;
    ecat.connect(argv[1]);

    std::vector<uint8_t> tx_buffer;
    tx_buffer.reserve(18U * 6U);
    for (uint8_t id = 0; id < 16U; ++id) {
      append_float_packet(tx_buffer, id, 0.0f);
    }
    append_uint8_packet(tx_buffer, 17U, 0U);

    std::vector<uint8_t> rx_buffer(ECAT_BUFFER_SIZE, 0U);
    ecat.send(tx_buffer.data(), tx_buffer.size());
    const int rx_len = ecat.read(rx_buffer.data());

    int offset = 0;
    while (offset < rx_len) {
      const uint8_t id = rx_buffer[static_cast<std::size_t>(offset)];
      const uint8_t size = rx_buffer[static_cast<std::size_t>(offset + 1)];
      const uint8_t* payload = rx_buffer.data() + offset + 2;
      if ((offset + 2 + size) > rx_len) {
        throw std::runtime_error("Received truncated packet");
      }

      if ((size % sizeof(float)) == 0U && id < 100U) {
        print_float_record(id, payload, size);
      } else {
        print_u8_record(id, payload, size);
      }
      offset += 2 + size;
    }
  } catch (const std::exception& ex) {
    std::cerr << "Test failed: " << ex.what() << std::endl;
    return 1;
  }

  return 0;
}
