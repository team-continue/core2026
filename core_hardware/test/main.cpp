#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "core_hardware/ecat.hpp"
#include "core_hardware/motor_diag_layout.h"

extern "C" {
#include "utypes.h"
}

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

void print_temp(const char* label, int8_t temp) {
  std::cout << " " << label << "=";
  if (temp == core_hardware::motor_diag::kTempInvalid) {
    std::cout << "n/a";
  } else {
    std::cout << static_cast<int>(temp);
  }
}

void print_hex32(uint32_t value) {
  std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0') << value << std::dec << std::setfill(' ');
}

// cycle() が Obj に書き込んだ motor_diag / motor_voltage / bus_diag / system_state を表示する
void print_diag_objects() {
  namespace md = core_hardware::motor_diag;
  std::cout << "system_state upper=" << static_cast<unsigned int>(Obj.system_state[0])
            << " bottom=" << static_cast<unsigned int>(Obj.system_state[1]) << std::endl;
  for (uint8_t id = 0; id < md::kNumMotors; ++id) {
    const md::MotorDiag diag = md::unpack_motor_diag(Obj.motor_diag[id], Obj.motor_voltage[id]);
    std::cout << "motor_diag id=" << static_cast<int>(id) << " raw=";
    print_hex32(Obj.motor_diag[id]);
    std::cout << " supported=" << diag.supported << " connected=" << diag.connected << " enabled=" << diag.enabled
              << " fault=" << diag.fault << " error_code=0x" << std::hex << static_cast<unsigned int>(diag.error_code)
              << std::dec;
    print_temp("temp", diag.temp);
    print_temp("temp_rotor", diag.temp_rotor);
    std::cout << " voltage=";
    if (diag.voltage_cv == md::kVoltageInvalid) {
      std::cout << "n/a";
    } else {
      std::cout << std::fixed << std::setprecision(2) << (static_cast<double>(diag.voltage_cv) * 0.01) << "V";
    }
    std::cout << std::endl;
  }
  const char* const bus_names[md::kNumCanBuses] = {"can3", "can2"};
  for (uint8_t index = 0; index < md::kNumCanBuses; ++index) {
    const md::BusDiag diag = md::unpack_bus_diag(Obj.bus_diag[index]);
    std::cout << "bus_diag " << bus_names[index] << " raw=";
    print_hex32(Obj.bus_diag[index]);
    std::cout << " valid=" << diag.valid << " bus_off=" << diag.bus_off << " error_passive=" << diag.error_passive
              << " tec=" << static_cast<unsigned int>(diag.tx_error_count)
              << " rec=" << static_cast<unsigned int>(diag.rx_error_count) << std::endl;
  }
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
      print_diag_objects();
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
