#pragma once

// Ported from codex/speedl_xbox_teleop/gateway/speedl_runtime.hpp
// (2026-08-12 SpeedL port). Only addition over the reference: needs_quiet(),
// used by Gateway's normal-command mutual exclusion (docs step 6).

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>

namespace codex_speedl {

// Thread-safe latest-value store for Xbox SpeedL teleoperation.
// It is deliberately NOT a FIFO: stale joystick velocities must never queue.
class SpeedLRuntime {
 public:
  using Clock = std::chrono::steady_clock;

  struct Config {
    int send_period_ms{50};       // Start at 20 Hz; use 100 ms (10 Hz) for first bench test.
    int watchdog_ms{200};         // No fresh Xbox update -> emit zero velocity.
    int last_count{100};          // Controller cycles, NOT milliseconds. Verify on the real controller.
    int zero_frames{3};           // Repeat zero velocity before becoming idle.
    int coordinate_base{0};
    int coordinate_tool{1};
    double max_linear_mps{0.080};
    double max_angular_radps{0.436332}; // 25 deg/s.
    std::array<double, 6> acceleration{{0.4, 0.4, 0.4, 1.0, 1.0, 1.0}};
    std::array<double, 6> jerk{{2.0, 2.0, 2.0, 5.0, 5.0, 5.0}};
  };

  struct Frame {
    std::array<double, 6> velocity{};
    int coordinate{0};
    int last_count{100};
    bool zero{true};
    uint64_t input_sequence{0};
  };

  SpeedLRuntime() = default;
  explicit SpeedLRuntime(const Config& config) : config_(config) {}

  // Called by the IPC handler after Gateway has checked owner and READY state.
  // deadman=false is always converted to a zero target.
  bool update(uint64_t sequence,
              int coordinate,
              const std::array<double, 6>& requested,
              bool deadman,
              Clock::time_point now = Clock::now()) {
    if (coordinate != config_.coordinate_base && coordinate != config_.coordinate_tool)
      return false;
    for (double value : requested)
      if (!std::isfinite(value)) return false;

    std::lock_guard<std::mutex> lock(mutex_);
    if (have_sequence_ && sequence <= last_input_sequence_) return false;
    have_sequence_ = true;
    last_input_sequence_ = sequence;
    latest_.fill(0.0);
    if (deadman) {
      for (size_t i = 0; i < latest_.size(); ++i) {
        const double limit = i < 3 ? config_.max_linear_mps : config_.max_angular_radps;
        latest_[i] = std::clamp(requested[i], -limit, limit);
      }
    }
    coordinate_ = coordinate;
    last_update_ = now;
    active_ = deadman && !is_zero(latest_);
    watchdog_latched_ = false;
    zero_frames_left_ = config_.zero_frames;
    return true;
  }

  // Used for LB release, IPC stop, owner release and joystick disconnect.
  void request_zero(Clock::time_point now = Clock::now()) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_.fill(0.0);
    last_update_ = now;
    have_sequence_ = true;
    active_ = false;
    watchdog_latched_ = false;
    zero_frames_left_ = config_.zero_frames;
  }

  // Used on reconnect: old input is forgotten and is never replayed.
  void reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_.fill(0.0);
    active_ = false;
    zero_frames_left_ = 0;
    have_sequence_ = false;
    watchdog_latched_ = false;
    next_send_ = Clock::time_point{};
  }

  // True while a non-zero target is active, zero frames still need sending, or
  // a fresh input may still be within the watchdog window. Normal commands
  // (MoveAbsJ / MoveBlend / gripper) must wait for this to become false after
  // request_zero() before the async SpeedL pipeline is drained (docs step 6).
  bool needs_quiet(Clock::time_point now = Clock::now()) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_) return true;
    if (zero_frames_left_ > 0) return true;
    if (have_sequence_ &&
        (now - last_update_ <= std::chrono::milliseconds(config_.watchdog_ms)))
      return true;
    return false;
  }

  // Call only from Gateway's single controller worker.
  std::optional<Frame> tick(Clock::time_point now = Clock::now()) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (now < next_send_) return std::nullopt;
    next_send_ = now + std::chrono::milliseconds(config_.send_period_ms);
    if (!have_sequence_) return std::nullopt;

    const bool stale = now - last_update_ > std::chrono::milliseconds(config_.watchdog_ms);
    if (stale && !watchdog_latched_) {
      active_ = false;
      latest_.fill(0.0);
      zero_frames_left_ = config_.zero_frames;
      watchdog_latched_ = true;
    }

    Frame frame;
    frame.coordinate = coordinate_;
    frame.last_count = config_.last_count;
    frame.input_sequence = last_input_sequence_;

    if (active_ && !stale) {
      frame.velocity = latest_;
      frame.zero = false;
      return frame;
    }
    if (zero_frames_left_ > 0) {
      frame.velocity.fill(0.0);
      frame.zero = true;
      --zero_frames_left_;
      return frame;
    }
    return std::nullopt;
  }

  std::string command(const Frame& frame, const std::string& arm_id) const {
    std::ostringstream inner;
    inner << "SpeedL --vel={" << values(frame.velocity)
          << "} --acc={" << values(config_.acceleration)
          << "} --jerk={" << values(config_.jerk)
          << "} --last_count=" << frame.last_count
          << " --coordinate=" << frame.coordinate;
    if (arm_id == "single") return "{" + inner.str() + "}";
    return arm_id == "left" ? "{" + inner.str() + "||NotRunExecute}"
                            : "{NotRunExecute||" + inner.str() + "}";
  }

  int send_period_ms() const { return config_.send_period_ms; }
  int watchdog_ms() const { return config_.watchdog_ms; }

 private:
  static bool is_zero(const std::array<double, 6>& value) {
    for (double x : value) if (std::abs(x) > 1e-9) return false;
    return true;
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

  Config config_;
  mutable std::mutex mutex_;
  std::array<double, 6> latest_{};
  int coordinate_{0};
  bool active_{false};
  bool have_sequence_{false};
  bool watchdog_latched_{false};
  int zero_frames_left_{0};
  uint64_t last_input_sequence_{0};
  Clock::time_point last_update_{};
  Clock::time_point next_send_{};
};

}  // namespace codex_speedl
