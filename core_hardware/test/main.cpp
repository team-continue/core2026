#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "core_hardware/ecat.hpp"

namespace {

constexpr int kReceiveTimeoutUs = 3000;
constexpr int kWarmupCycles = 5;

void print_float_packet(uint8_t id, const std::vector<float>& data) {
  std::cout << "/can/rx id=" << static_cast<int>(id) << " data=[";
  for (std::size_t i = 0; i < data.size(); ++i) {
    if (i != 0U) {
      std::cout << ", ";
    }
    std::cout << std::fixed << std::setprecision(3) << data[i];
  }
  std::cout << "]" << std::endl;
}

void print_uint8_packet(uint8_t id, const std::vector<uint8_t>& data) {
  std::cout << "/can/rx id=" << static_cast<int>(id) << " data=[";
  for (std::size_t i = 0; i < data.size(); ++i) {
    if (i != 0U) {
      std::cout << ", ";
    }
    std::cout << static_cast<unsigned int>(data[i]);
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
    ecat.set_float_packet_callback(print_float_packet);
    ecat.set_uint8_packet_callback(print_uint8_packet);
    ecat.connect(argv[1]);

    for (uint8_t id = 0; id < 16U; ++id) {
      ecat.set_float(id, {0.0f});
    }
    ecat.set_system_ref(false);
    ecat.set_led_tape({0U, 0U, 0U});

    bool received_any_packet = false;
    for (int cycle_index = 0; cycle_index < kWarmupCycles; ++cycle_index) {
      int wkc = 0;
      if (!ecat.cycle(kReceiveTimeoutUs, wkc)) {
        std::cerr << "Cycle failed, wkc=" << wkc << std::endl;
        continue;
      }
      received_any_packet = true;
      std::cout << "Cycle " << cycle_index << " succeeded, wkc=" << wkc << std::endl;
    }

    if (!received_any_packet) {
      throw std::runtime_error("No successful EtherCAT cycles");
    }
  } catch (const std::exception& ex) {
    std::cerr << "Test failed: " << ex.what() << std::endl;
    return 1;
  }

  return 0;
}
