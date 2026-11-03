#pragma once

// Ported unchanged from codex/speedl_xbox_teleop/xbox/speedl_ipc_client.hpp
// (2026-08-12 SpeedL port). Only addition: <sys/time.h> for struct timeval.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>

namespace codex_speedl {

// Persistent newline-JSON connection for Xbox -> ArmGateway.
// One connection is reused for owner acquisition and all velocity updates.
class SpeedLIpcClient {
 public:
  SpeedLIpcClient(std::string host = "127.0.0.1", int port = 6200,
                  std::string owner = "xbox")
      : host_(std::move(host)), port_(port), owner_(std::move(owner)) {}
  ~SpeedLIpcClient() { close_connection(); }

  bool connect_and_claim(std::string* response = nullptr) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (!ensure_socket()) return false;
    std::string reply;
    if (!transact_locked("{\"cmd\":\"set_owner\",\"owner\":\"" + owner_ + "\"}", reply))
      return false;
    if (response) *response = reply;
    claimed_ = reply.find("\"ok\":true") != std::string::npos;
    return claimed_;
  }

  bool update(const std::array<double, 6>& velocity,
              int coordinate,
              bool deadman,
              std::string* response = nullptr) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (!ensure_socket()) return false;
    if (!claimed_) {
      std::string owner_reply;
      if (!transact_locked("{\"cmd\":\"set_owner\",\"owner\":\"" + owner_ + "\"}", owner_reply) ||
          owner_reply.find("\"ok\":true") == std::string::npos)
        return false;
      claimed_ = true;
    }
    std::ostringstream request;
    request << "{\"cmd\":\"speedl_update\",\"owner\":\"" << owner_
            << "\",\"seq\":" << ++sequence_
            << ",\"coordinate\":" << coordinate
            << ",\"deadman\":" << (deadman ? "true" : "false")
            << ",\"vel\":[" << values(velocity) << "]}";
    std::string reply;
    if (!transact_locked(request.str(), reply)) return false;
    if (response) *response = reply;
    return reply.find("\"ok\":true") != std::string::npos;
  }

  bool zero(int coordinate) {
    std::array<double, 6> zero{};
    return update(zero, coordinate, false, nullptr);
  }

  void release() {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (fd_ >= 0 && claimed_) {
      std::string ignored;
      transact_locked("{\"cmd\":\"release_owner\",\"owner\":\"" + owner_ + "\"}", ignored);
    }
    claimed_ = false;
    close_locked();
  }

  bool standby(std::string* response = nullptr) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (!ensure_socket()) return false;
    std::string reply;
    const bool ok = transact_locked("{\"cmd\":\"standby\",\"owner\":\"" + owner_ + "\"}", reply) &&
                    reply.find("\"ok\":true") != std::string::npos;
    if (response) *response = reply;
    claimed_ = false;
    close_locked();
    return ok;
  }

  bool resume_gateway(std::string* response = nullptr) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (!ensure_socket()) return false;
    std::string reply;
    const bool ok = transact_locked("{\"cmd\":\"resume_gateway\"}", reply) &&
                    reply.find("\"ok\":true") != std::string::npos;
    if (response) *response = reply;
    claimed_ = false;
    close_locked();
    return ok;
  }

 private:
  bool ensure_socket() {
    if (fd_ >= 0) return true;
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return false;
    timeval timeout{1, 0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(port_));
    if (inet_pton(AF_INET, host_.c_str(), &address.sin_addr) != 1 ||
        connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
      close(fd);
      return false;
    }
    fd_ = fd;
    return true;
  }

  bool transact_locked(const std::string& request, std::string& response) {
    const std::string wire = request + "\n";
    size_t offset = 0;
    while (offset < wire.size()) {
      const ssize_t n = send(fd_, wire.data() + offset, wire.size() - offset, MSG_NOSIGNAL);
      if (n <= 0) { close_locked(); return false; }
      offset += static_cast<size_t>(n);
    }
    response.clear();
    char ch = 0;
    while (response.size() < 65536) {
      const ssize_t n = recv(fd_, &ch, 1, 0);
      if (n <= 0) { close_locked(); return false; }
      if (ch == '\n') return true;
      response.push_back(ch);
    }
    close_locked();
    return false;
  }

  static std::string values(const std::array<double, 6>& value) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < value.size(); ++i) {
      if (i) out << ',';
      out << value[i];
    }
    return out.str();
  }
  void close_connection() { std::lock_guard<std::mutex> lock(io_mutex_); close_locked(); }
  void close_locked() {
    if (fd_ >= 0) { shutdown(fd_, SHUT_RDWR); close(fd_); fd_ = -1; }
    claimed_ = false;
  }

  std::string host_;
  int port_;
  std::string owner_;
  int fd_{-1};
  bool claimed_{false};
  uint64_t sequence_{0};
  std::mutex io_mutex_;
};

}  // namespace codex_speedl
