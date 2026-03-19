#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "core_hardware/ecat.hpp"
#include "core_hardware/hardware_snapshot.hpp"
#include "core_hardware/ipc_protocol.hpp"

namespace {
constexpr int kReceiveTimeoutUs = 3000;
constexpr int kMaxMissedCycles = 10;
constexpr auto kCyclePeriod = std::chrono::milliseconds(10);
constexpr auto kCommandTimeout = std::chrono::seconds(1);

uint64_t current_system_time_ms() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                   std::chrono::system_clock::now().time_since_epoch())
                                   .count());
}

volatile std::sig_atomic_t g_should_stop = 0;

void signal_handler(int) {
  g_should_stop = 1;
}

struct Options {
  std::string if_name = "enp2s0";
  std::string socket_path = std::string(core_hardware::kDefaultSocketPath);
};

Options parse_args(int argc, char* argv[]) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--if-name" && (i + 1) < argc) {
      options.if_name = argv[++i];
      continue;
    }
    if (arg == "--socket-path" && (i + 1) < argc) {
      options.socket_path = argv[++i];
      continue;
    }
    throw std::runtime_error("Unknown argument: " + arg);
  }
  return options;
}

class HardwareDaemon {
 public:
  explicit HardwareDaemon(const Options& options)
      : if_name_(options.if_name), socket_path_(options.socket_path) {
    ecat_.set_float_packet_callback([this](uint8_t id, const std::vector<float>& data) {
      pending_float_packets_.emplace_back(id, data);
    });
    ecat_.set_uint8_packet_callback([this](uint8_t id, const std::vector<uint8_t>& data) {
      pending_uint8_packets_.emplace_back(id, data);
    });
  }

  int run() {
    server_.listen(socket_path_);
    std::cout << "Hardware daemon listening on " << socket_path_ << std::endl;

    auto next_tick = std::chrono::steady_clock::now();
    while (!g_should_stop) {
      next_tick += kCyclePeriod;

      try {
        server_.accept_if_ready();
        drain_commands();
        update_command_timeout();
      } catch (const std::exception& ex) {
        std::cerr << "Hardware daemon IPC error: " << ex.what() << std::endl;
        handle_client_failure();
      }

      if (!command_active_) {
        std::this_thread::sleep_until(next_tick);
        continue;
      }

      try {
        ensure_ecat_connected();
        apply_command_snapshot();
        pending_float_packets_.clear();
        pending_uint8_packets_.clear();

        int wkc = 0;
        if (!ecat_.cycle(kReceiveTimeoutUs, wkc)) {
          ++missed_cycles_;
          if (missed_cycles_ >= kMaxMissedCycles) {
            std::cerr << "EtherCAT reconnect after missed cycles, wkc=" << wkc << std::endl;
            ethercat_connected_for_state_ = false;
            ecat_.close();
            ecat_connected_ = false;
            missed_cycles_ = 0;
          }
        } else {
          missed_cycles_ = 0;
          ethercat_connected_for_state_ = true;
          last_ethercat_rx_ms_ = current_system_time_ms();
          try {
            publish_state();
          } catch (const std::exception& ex) {
            std::cerr << "Hardware daemon IPC error: " << ex.what() << std::endl;
            handle_client_failure();
          }
        }
      } catch (const std::exception& ex) {
        std::cerr << "Hardware daemon error: " << ex.what() << std::endl;
        handle_ecat_failure();
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }

      std::this_thread::sleep_until(next_tick);
    }

    ecat_.close();
    server_.close();
    return 0;
  }

 private:
  void drain_commands() {
    if (!server_.has_client()) {
      return;
    }
    while (server_.poll_once([this](core_hardware::IpcMessageType type, uint32_t, const std::vector<uint8_t>& payload) {
      if (type == core_hardware::IpcMessageType::kCommandSnapshot) {
        latest_command_ = core_hardware::decode_snapshot(payload);
        last_command_rx_tp_ = std::chrono::steady_clock::now();
        command_active_ = true;
      }
    })) {
    }
  }

  void update_command_timeout() {
    if (!command_active_) {
      return;
    }
    if ((std::chrono::steady_clock::now() - last_command_rx_tp_) <= kCommandTimeout) {
      return;
    }

    command_active_ = false;
    if (ecat_connected_) {
      std::cerr << "EtherCAT closed after command timeout" << std::endl;
      ethercat_connected_for_state_ = false;
      ecat_.close();
      ecat_connected_ = false;
    }
    missed_cycles_ = 0;
    pending_float_packets_.clear();
    pending_uint8_packets_.clear();
  }

  void ensure_ecat_connected() {
    if (ecat_connected_) {
      return;
    }
    ecat_.connect(if_name_.c_str());
    ecat_connected_ = true;
    missed_cycles_ = 0;
    std::cout << "Connected to EtherCAT via " << if_name_ << std::endl;
  }

  void apply_command_snapshot() {
    for (uint8_t id = 0; id < latest_command_.motor_ref.size(); ++id) {
      ecat_.set_float(id, {latest_command_.motor_ref[id]});
    }
    ecat_.set_system_ref(latest_command_.system_ref[0] != 0U);
    ecat_.set_led_tape(latest_command_.led_tape);
  }

  void publish_state() {
    if (!server_.has_client()) {
      return;
    }
    auto snapshot = core_hardware::snapshot_from_objects(Obj);
    snapshot.ethercat_connected[0] = ethercat_connected_for_state_ ? 1U : 0U;
    snapshot.last_ethercat_rx_ms[0] = last_ethercat_rx_ms_;
    server_.send(core_hardware::IpcMessageType::kStateSnapshot, sequence_++, core_hardware::encode_snapshot(snapshot));
    for (const auto& packet : pending_float_packets_) {
      server_.send(core_hardware::IpcMessageType::kFloatPacket, sequence_++, core_hardware::encode_packet_message(packet.first, packet.second));
    }
    for (const auto& packet : pending_uint8_packets_) {
      server_.send(core_hardware::IpcMessageType::kUint8Packet, sequence_++, core_hardware::encode_uint8_packet_message(packet.first, packet.second));
    }
  }

  void handle_client_failure() {
    if (server_.has_client()) {
      server_.disconnect_client();
    }
    command_active_ = false;
    if (ecat_connected_) {
      ethercat_connected_for_state_ = false;
      ecat_.close();
      ecat_connected_ = false;
    }
    missed_cycles_ = 0;
    pending_float_packets_.clear();
    pending_uint8_packets_.clear();
  }

  void handle_ecat_failure() {
    latest_command_ = {};
    ecat_.clear_tx_packets();
    if (server_.has_client()) {
      publish_empty_state();
    }
    ethercat_connected_for_state_ = false;
    ecat_.close();
    ecat_connected_ = false;
    missed_cycles_ = 0;
    pending_float_packets_.clear();
    pending_uint8_packets_.clear();
  }

  void publish_empty_state() {
    try {
      server_.send(
          core_hardware::IpcMessageType::kStateSnapshot,
          sequence_++,
          core_hardware::encode_snapshot(core_hardware::HardwareSnapshot{}));
    } catch (const std::exception&) {
      latest_command_ = {};
      handle_client_failure();
    }
  }

  Ecat ecat_;
  core_hardware::IpcServer server_;
  core_hardware::HardwareSnapshot latest_command_{};
  std::vector<std::pair<uint8_t, std::vector<float>>> pending_float_packets_;
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> pending_uint8_packets_;
  std::string if_name_;
  std::string socket_path_;
  bool ecat_connected_ = false;
  bool command_active_ = false;
  bool ethercat_connected_for_state_ = false;
  int missed_cycles_ = 0;
  uint32_t sequence_ = 0;
  uint64_t last_ethercat_rx_ms_ = 0;
  std::chrono::steady_clock::time_point last_command_rx_tp_{};
};

}  // namespace

int main(int argc, char* argv[]) {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  try {
    const Options options = parse_args(argc, argv);
    HardwareDaemon daemon(options);
    return daemon.run();
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    return 1;
  }
}
