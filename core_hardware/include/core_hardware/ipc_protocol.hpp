#pragma once

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <sys/poll.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <vector>

namespace core_hardware {

constexpr uint32_t kIpcMagic = 0x43483236;  // CH26
constexpr uint16_t kIpcVersion = 1;
constexpr std::size_t kMaxIpcMessageSize = 8192;
constexpr std::string_view kDefaultSocketPath = "/tmp/core_hardware.sock";

enum class IpcMessageType : uint16_t {
  kCommandSnapshot = 1,
  kStateSnapshot = 2,
  kFloatPacket = 3,
  kUint8Packet = 4,
  kHeartbeat = 5,
  kError = 6,
};

struct IpcMessageHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t type;
  uint32_t sequence;
  uint32_t payload_size;
};

struct PacketMessageHeader {
  uint8_t id;
  uint8_t reserved[3]{};
  uint32_t float_count;
};

struct Uint8PacketMessageHeader {
  uint8_t id;
  uint8_t reserved[3]{};
  uint32_t byte_count;
};

class SeqPacketSocket {
 public:
  SeqPacketSocket() = default;
  explicit SeqPacketSocket(int fd) : fd_(fd) {}

  SeqPacketSocket(const SeqPacketSocket&) = delete;
  SeqPacketSocket& operator=(const SeqPacketSocket&) = delete;

  SeqPacketSocket(SeqPacketSocket&& other) noexcept : fd_(other.fd_) { other.fd_ = -1; }

  SeqPacketSocket& operator=(SeqPacketSocket&& other) noexcept {
    if (this != &other) {
      close();
      fd_ = other.fd_;
      other.fd_ = -1;
    }
    return *this;
  }

  ~SeqPacketSocket() { close(); }

  bool is_open() const { return fd_ >= 0; }
  int fd() const { return fd_; }

  void reset(int fd) {
    close();
    fd_ = fd;
  }

  void close() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  void send_message(IpcMessageType type, uint32_t sequence, const std::vector<uint8_t>& payload) const {
    if (payload.size() > (kMaxIpcMessageSize - sizeof(IpcMessageHeader))) {
      throw std::runtime_error("IPC payload too large");
    }
    std::vector<uint8_t> buffer(sizeof(IpcMessageHeader) + payload.size());
    const IpcMessageHeader header{
        kIpcMagic,
        kIpcVersion,
        static_cast<uint16_t>(type),
        sequence,
        static_cast<uint32_t>(payload.size()),
    };
    std::memcpy(buffer.data(), &header, sizeof(header));
    if (!payload.empty()) {
      std::memcpy(buffer.data() + sizeof(header), payload.data(), payload.size());
    }
    const ssize_t sent = ::send(fd_, buffer.data(), buffer.size(), MSG_NOSIGNAL);
    if (sent < 0) {
      throw std::runtime_error(std::string("IPC send failed: ") + std::strerror(errno));
    }
    if (static_cast<std::size_t>(sent) != buffer.size()) {
      throw std::runtime_error("IPC partial send");
    }
  }

  bool receive_message(IpcMessageType& type, uint32_t& sequence, std::vector<uint8_t>& payload) const {
    std::vector<uint8_t> buffer(kMaxIpcMessageSize);
    const ssize_t received = ::recv(fd_, buffer.data(), buffer.size(), MSG_DONTWAIT);
    if (received < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return false;
      }
      throw std::runtime_error(std::string("IPC recv failed: ") + std::strerror(errno));
    }
    if (received == 0) {
      throw std::runtime_error("IPC peer disconnected");
    }
    if (static_cast<std::size_t>(received) < sizeof(IpcMessageHeader)) {
      throw std::runtime_error("IPC header truncated");
    }
    IpcMessageHeader header{};
    std::memcpy(&header, buffer.data(), sizeof(header));
    if (header.magic != kIpcMagic || header.version != kIpcVersion) {
      throw std::runtime_error("IPC protocol mismatch");
    }
    const std::size_t total_size = sizeof(IpcMessageHeader) + header.payload_size;
    if (static_cast<std::size_t>(received) != total_size) {
      throw std::runtime_error("IPC message size mismatch");
    }
    type = static_cast<IpcMessageType>(header.type);
    sequence = header.sequence;
    payload.assign(buffer.begin() + static_cast<std::ptrdiff_t>(sizeof(IpcMessageHeader)),
                   buffer.begin() + static_cast<std::ptrdiff_t>(total_size));
    return true;
  }

 private:
  int fd_ = -1;
};

inline sockaddr_un make_sockaddr(const std::string& path) {
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  if (path.size() >= sizeof(addr.sun_path)) {
    throw std::runtime_error("Socket path too long");
  }
  std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path.c_str());
  return addr;
}

inline int create_seqpacket_socket() {
  const int fd = ::socket(AF_UNIX, SOCK_SEQPACKET, 0);
  if (fd < 0) {
    throw std::runtime_error(std::string("socket() failed: ") + std::strerror(errno));
  }
  return fd;
}

class IpcClient {
 public:
  void connect(const std::string& path) {
    socket_.close();
    const int fd = create_seqpacket_socket();
    const sockaddr_un addr = make_sockaddr(path);
    if (::connect(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
      const std::string error = std::strerror(errno);
      ::close(fd);
      throw std::runtime_error("IPC connect failed: " + error);
    }
    socket_.reset(fd);
  }

  bool is_connected() const { return socket_.is_open(); }
  void close() { socket_.close(); }

  void send(IpcMessageType type, uint32_t sequence, const std::vector<uint8_t>& payload) const {
    socket_.send_message(type, sequence, payload);
  }

  bool poll_once(const std::function<void(IpcMessageType, uint32_t, const std::vector<uint8_t>&)>& handler) const {
    IpcMessageType type{};
    uint32_t sequence = 0;
    std::vector<uint8_t> payload;
    if (!socket_.receive_message(type, sequence, payload)) {
      return false;
    }
    handler(type, sequence, payload);
    return true;
  }

 private:
  SeqPacketSocket socket_;
};

class IpcServer {
 public:
  ~IpcServer() { close(); }

  void listen(const std::string& path) {
    close();
    path_ = path;
    listener_fd_ = create_seqpacket_socket();
    const sockaddr_un addr = make_sockaddr(path_);
    ::unlink(path_.c_str());
    if (::bind(listener_fd_, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
      const std::string error = std::strerror(errno);
      close();
      throw std::runtime_error("IPC bind failed: " + error);
    }
    if (::chmod(path_.c_str(), 0666) != 0) {
      const std::string error = std::strerror(errno);
      close();
      throw std::runtime_error("IPC chmod failed: " + error);
    }
    if (::listen(listener_fd_, 1) != 0) {
      const std::string error = std::strerror(errno);
      close();
      throw std::runtime_error("IPC listen failed: " + error);
    }
  }

  void close() {
    client_.close();
    if (listener_fd_ >= 0) {
      ::close(listener_fd_);
      listener_fd_ = -1;
    }
    if (!path_.empty()) {
      ::unlink(path_.c_str());
      path_.clear();
    }
  }

  bool has_client() const { return client_.is_open(); }

  void accept_if_ready() {
    if (listener_fd_ < 0 || client_.is_open()) {
      return;
    }
    pollfd pfd{listener_fd_, POLLIN, 0};
    const int result = ::poll(&pfd, 1, 0);
    if (result <= 0 || (pfd.revents & POLLIN) == 0) {
      return;
    }
    const int client_fd = ::accept(listener_fd_, nullptr, nullptr);
    if (client_fd < 0) {
      throw std::runtime_error(std::string("IPC accept failed: ") + std::strerror(errno));
    }
    client_.reset(client_fd);
  }

  void disconnect_client() { client_.close(); }

  void send(IpcMessageType type, uint32_t sequence, const std::vector<uint8_t>& payload) const {
    client_.send_message(type, sequence, payload);
  }

  bool poll_once(const std::function<void(IpcMessageType, uint32_t, const std::vector<uint8_t>&)>& handler) const {
    IpcMessageType type{};
    uint32_t sequence = 0;
    std::vector<uint8_t> payload;
    if (!client_.receive_message(type, sequence, payload)) {
      return false;
    }
    handler(type, sequence, payload);
    return true;
  }

 private:
  int listener_fd_ = -1;
  std::string path_;
  SeqPacketSocket client_;
};

inline std::vector<uint8_t> encode_packet_message(uint8_t id, const std::vector<float>& data) {
  PacketMessageHeader header{};
  header.id = id;
  header.float_count = static_cast<uint32_t>(data.size());
  std::vector<uint8_t> payload(sizeof(header) + (sizeof(float) * data.size()));
  std::memcpy(payload.data(), &header, sizeof(header));
  if (!data.empty()) {
    std::memcpy(payload.data() + sizeof(header), data.data(), sizeof(float) * data.size());
  }
  return payload;
}

inline std::pair<uint8_t, std::vector<float>> decode_packet_message(const std::vector<uint8_t>& payload) {
  if (payload.size() < sizeof(PacketMessageHeader)) {
    throw std::runtime_error("Packet payload truncated");
  }
  PacketMessageHeader header{};
  std::memcpy(&header, payload.data(), sizeof(header));
  const std::size_t expected_size = sizeof(header) + (sizeof(float) * header.float_count);
  if (payload.size() != expected_size) {
    throw std::runtime_error("Packet payload size mismatch");
  }
  std::vector<float> data(header.float_count, 0.0f);
  if (!data.empty()) {
    std::memcpy(data.data(), payload.data() + sizeof(header), sizeof(float) * data.size());
  }
  return {header.id, data};
}

inline std::vector<uint8_t> encode_uint8_packet_message(uint8_t id, const std::vector<uint8_t>& data) {
  Uint8PacketMessageHeader header{};
  header.id = id;
  header.byte_count = static_cast<uint32_t>(data.size());
  std::vector<uint8_t> payload(sizeof(header) + data.size());
  std::memcpy(payload.data(), &header, sizeof(header));
  if (!data.empty()) {
    std::memcpy(payload.data() + sizeof(header), data.data(), data.size());
  }
  return payload;
}

inline std::pair<uint8_t, std::vector<uint8_t>> decode_uint8_packet_message(const std::vector<uint8_t>& payload) {
  if (payload.size() < sizeof(Uint8PacketMessageHeader)) {
    throw std::runtime_error("Uint8 packet payload truncated");
  }
  Uint8PacketMessageHeader header{};
  std::memcpy(&header, payload.data(), sizeof(header));
  const std::size_t expected_size = sizeof(header) + header.byte_count;
  if (payload.size() != expected_size) {
    throw std::runtime_error("Uint8 packet payload size mismatch");
  }
  std::vector<uint8_t> data(header.byte_count, 0U);
  if (!data.empty()) {
    std::memcpy(data.data(), payload.data() + sizeof(header), data.size());
  }
  return {header.id, data};
}

}  // namespace core_hardware
