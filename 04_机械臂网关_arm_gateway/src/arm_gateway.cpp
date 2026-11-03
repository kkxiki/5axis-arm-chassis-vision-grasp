#include "cpp_rpc.hpp"
#include "resp_dto.h"
#include "json.hpp"
#include "speedl_runtime.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cctype>
#include <cstring>
#include <deque>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <set>

using json = nlohmann::json;
using Clock = std::chrono::steady_clock;

namespace
{
  std::atomic<bool> g_exit{false};
  void on_signal(int) { g_exit.store(true); }
  std::string trim(std::string s)
  {
    const auto b = s.find_first_not_of(" \t\r\n");
    return b == std::string::npos ? "" : s.substr(b, s.find_last_not_of(" \t\r\n") - b + 1);
  }
  std::string unquote(std::string s)
  {
    s = trim(std::move(s));
    if (s.size() >= 2 && ((s.front() == '"' && s.back() == '"') || (s.front() == '\'' && s.back() == '\'')))
      return s.substr(1, s.size() - 2);
    return s;
  }
  std::unordered_map<std::string, std::string> load_yaml(const std::string &path)
  {
    std::ifstream in(path);
    if (!in)
      throw std::runtime_error("cannot open config: " + path);
    std::unordered_map<std::string, std::string> out;
    std::string line;
    while (std::getline(in, line))
    {
      auto h = line.find('#');
      if (h != std::string::npos)
        line.erase(h);
      auto c = line.find(':');
      if (c == std::string::npos)
        continue;
      auto k = trim(line.substr(0, c));
      if (!k.empty())
        out[k] = unquote(line.substr(c + 1));
    }
    return out;
  }
  template <class T>
  T getv(const std::unordered_map<std::string, std::string> &, const std::string &, T);
  template <>
  int getv(const std::unordered_map<std::string, std::string> &m, const std::string &k, int d)
  {
    auto i = m.find(k);
    return i == m.end() || i->second.empty() ? d : std::stoi(i->second);
  }
  template <>
  double getv(const std::unordered_map<std::string, std::string> &m, const std::string &k, double d)
  {
    auto i = m.find(k);
    return i == m.end() || i->second.empty() ? d : std::stod(i->second);
  }
  template <>
  bool getv(const std::unordered_map<std::string, std::string> &m, const std::string &k, bool d)
  {
    auto i = m.find(k);
    if (i == m.end() || i->second.empty())
      return d;
    auto s = i->second;
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c)
                   { return std::tolower(c); });
    if (s == "true" || s == "yes" || s == "on" || s == "1")
      return true;
    if (s == "false" || s == "no" || s == "off" || s == "0")
      return false;
    throw std::runtime_error("invalid bool: " + k);
  }
  template <>
  std::string getv(const std::unordered_map<std::string, std::string> &m, const std::string &k, std::string d)
  {
    auto i = m.find(k);
    return i == m.end() ? d : i->second;
  }

  struct Config
  {
    std::string controller_ip{"192.168.11.11"}, listen_ip{"127.0.0.1"}, heartbeat_file{"/tmp/move_actual.txt"};
    int controller_port{5868}, listen_port{6200}, probe_timeout_ms{1000}, reconnect_backoff_ms{30000}, rpc_timeout_ms{30000};
    int heartbeat_period_ms{500}, heartbeat_fail_limit{3}, heartbeat_stale_ms{2000}, fake_motion_timeout_ms{3000}, max_line_bytes{262144};
    bool heartbeat_required{true};
    // This controller keeps its enable state across a gateway-side transport
    // drop, so a reconnect must NOT power-cycle the servos ({Disable} is only
    // for a true cold start).  Keep true unless a real cold-start init is
    // wanted at boot as well.
    bool reattach_enable{true};
    double fake_motion_epsilon_m{0.0005};
    // SpeedL continuous-teleop (ported from codex/speedl_xbox_teleop, 2026-08-12).
    // last_count is controller cycles, NOT milliseconds; confirm its duration
    // on the real controller before changing send_hz (see README step 6).
    std::string arm_id{"single"};
    int speedl_send_hz{10};
    int speedl_watchdog_ms{250};
    int speedl_last_count{150};
    int speedl_zero_frames{3};
    double speedl_max_linear_mps{0.080};
    double speedl_max_angular_radps{0.436332}; // 25 deg/s
    static Config from_file(const std::string &p)
    {
      auto m = load_yaml(p);
      Config c;
#define R(T, x) c.x = getv<T>(m, #x, c.x)
      R(std::string, controller_ip);
      R(int, controller_port);
      R(std::string, listen_ip);
      R(int, listen_port);
      R(int, probe_timeout_ms);
      R(int, reconnect_backoff_ms);
      R(int, rpc_timeout_ms);
      R(int, heartbeat_period_ms);
      R(int, heartbeat_fail_limit);
      R(bool, heartbeat_required);
      R(bool, reattach_enable);
      R(std::string, heartbeat_file);
      R(int, heartbeat_stale_ms);
      R(int, fake_motion_timeout_ms);
      R(double, fake_motion_epsilon_m);
      R(int, max_line_bytes);
      R(std::string, arm_id);
      R(int, speedl_send_hz);
      R(int, speedl_watchdog_ms);
      R(int, speedl_last_count);
      R(int, speedl_zero_frames);
      R(double, speedl_max_linear_mps);
      R(double, speedl_max_angular_radps);
#undef R
      if (c.controller_port < 1 || c.controller_port > 65535 || c.listen_port < 1 || c.listen_port > 65535)
        throw std::runtime_error("port out of range");
      if (c.probe_timeout_ms < 100 || c.probe_timeout_ms > 1000)
        throw std::runtime_error("probe_timeout_ms must be 100..1000");
      if (c.reconnect_backoff_ms < 30000)
        throw std::runtime_error("reconnect_backoff_ms must be >=30000 (P36)");
      if (c.heartbeat_period_ms != 500 || c.heartbeat_fail_limit < 3)
        throw std::runtime_error("heartbeat must be 500ms and failures >=3");
      if (c.heartbeat_required && (c.heartbeat_file.empty() || c.heartbeat_file.front() != '/'))
        throw std::runtime_error("heartbeat_file must be absolute");
      if (c.arm_id != "left" && c.arm_id != "right" && c.arm_id != "single")
        throw std::runtime_error("arm_id must be left, right, or single");
      if (c.speedl_send_hz < 1 || c.speedl_send_hz > 50 || c.speedl_watchdog_ms < 50 || c.speedl_watchdog_ms > 5000 ||
          c.speedl_last_count < 1 || c.speedl_last_count > 1000000 || c.speedl_zero_frames < 1 || c.speedl_zero_frames > 20)
        throw std::runtime_error("unsafe speedl parameters");
      return c;
    }
  };
  // Build the SpeedLRuntime config from gateway Config (must stay in sync
  // with the xbox_arm_teleop speedl_* values).
  codex_speedl::SpeedLRuntime::Config speedl_runtime_config(const Config &c)
  {
    codex_speedl::SpeedLRuntime::Config cfg;
    cfg.send_period_ms = 1000 / std::max(1, c.speedl_send_hz);
    cfg.watchdog_ms = c.speedl_watchdog_ms;
    cfg.last_count = c.speedl_last_count;
    cfg.zero_frames = c.speedl_zero_frames;
    cfg.max_linear_mps = c.speedl_max_linear_mps;
    cfg.max_angular_radps = c.speedl_max_angular_radps;
    return cfg;
  }
  enum class State
  {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    READY,
    STANDBY,
    RECONNECTING
  };
  const char *sn(State s)
  {
    switch (s)
    {
    case State::DISCONNECTED:
      return "DISCONNECTED";
    case State::CONNECTING:
      return "CONNECTING";
    case State::CONNECTED:
      return "CONNECTED";
    case State::READY:
      return "READY";
    case State::STANDBY:
      return "STANDBY";
    case State::RECONNECTING:
      return "RECONNECTING";
    }
    return "DISCONNECTED";
  }
  std::string owner_name(std::string s)
  {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c)
                   { return std::toupper(c); });
    if (s == "XBOX" || s == "MANUAL" || s == "MANUAL_XBOX")
      return "MANUAL_XBOX";
    if (s == "AUTO" || s == "AUTO_TASK" || s == "YOLO")
      return "AUTO_TASK";
    if (s == "MOVEIT" || s == "MOVE_IT")
      return "MOVEIT";
    if (s == "IDLE")
      return "IDLE";
    return {};
  }
  bool stop_forbidden(const std::string &r)
  {
    std::string s;
    for (unsigned char c : r)
      if (!std::isspace(c))
        s.push_back(std::tolower(c));
    return s.find("stop") != std::string::npos;
  }
  bool is_motion(const std::string &r) { return r.find("MoveAbsJ") != std::string::npos || (r.find("MoveBlend") != std::string::npos && r.find("type=start") != std::string::npos); }
  struct Pose
  {
    double p[3]{}, q[4]{};
  };
  std::optional<Pose> read_pose(const std::string &p)
  {
    std::ifstream f(p);
    Pose x;
    if (!(f >> x.p[0] >> x.p[1] >> x.p[2] >> x.q[0] >> x.q[1] >> x.q[2] >> x.q[3]))
      return {};
    return x;
  }
  bool heartbeat_ok(const std::string &p, int stale)
  {
    struct stat st{};
    if (stat(p.c_str(), &st))
      return false;
    timespec n{};
    clock_gettime(CLOCK_REALTIME, &n);
    auto age = (n.tv_sec - st.st_mtim.tv_sec) * 1000LL + (n.tv_nsec - st.st_mtim.tv_nsec) / 1000000LL;
    return age >= 0 && age <= stale && read_pose(p).has_value();
  }
  bool probe(const std::string &ip, int port, int timeout)
  {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
      return false;
    int fl = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, fl | O_NONBLOCK);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &a.sin_addr) != 1)
    {
      close(fd);
      return false;
    }
    int r = connect(fd, (sockaddr *)&a, sizeof(a));
    if (r && errno == EINPROGRESS)
    {
      pollfd p{fd, POLLOUT, 0};
      r = poll(&p, 1, timeout);
      int e = 0;
      socklen_t n = sizeof(e);
      if (r > 0)
        getsockopt(fd, SOL_SOCKET, SO_ERROR, &e, &n);
      r = (r > 0 && !e) ? 0 : -1;
    }
    shutdown(fd, SHUT_RDWR);
    close(fd);
    return r == 0;
  }
  struct Result
  {
    bool ok{false};
    int rc{-1};
    std::string error;
  };
  enum class Kind
  {
    STOP,
    STANDBY,
    ENABLE,
    DISABLE,
    SEND
  };
  struct Work
  {
    Kind kind;
    std::string owner, raw;
    std::promise<Result> done;
  };

  class Gateway
  {
  public:
    explicit Gateway(Config c) : c_(std::move(c)), speedl_(speedl_runtime_config(c_)) {}
    ~Gateway() { stop(); }
    void start()
    {
      worker_ = std::thread(&Gateway::worker, this);
      server_ = std::thread(&Gateway::server, this);
    }
    void stop()
    {
      if (stopping_.exchange(true))
        return;
      cancel_all("gateway_stopping");
      cv_.notify_all();
      int fd = listen_.exchange(-1);
      if (fd >= 0)
      {
        shutdown(fd, SHUT_RDWR);
        close(fd);
      }
      if (server_.joinable())
        server_.join();
      {
        std::lock_guard<std::mutex> g(clients_m_);
        for (int c : client_fds_)
          shutdown(c, SHUT_RDWR);
      }
      if (worker_.joinable())
        worker_.join();
      for (auto &t : clients_)
        if (t.joinable())
          t.join();
    }

  private:
    void log(const char *l, const std::string &t)
    {
      auto now = std::chrono::system_clock::now();
      auto tt = std::chrono::system_clock::to_time_t(now);
      std::tm tm{};
      localtime_r(&tt, &tm);
      std::lock_guard<std::mutex> g(log_m_);
      std::cout << std::put_time(&tm, "%F %T") << " [" << l << "] " << t << std::endl;
    }
    void state(State s, bool conn, bool en)
    {
      std::lock_guard<std::mutex> g(state_m_);
      state_ = s;
      connected_ = conn;
      enabled_ = en;
    }
    size_t queued()
    {
      std::lock_guard<std::mutex> g(queue_m_);
      return high_.size() + normal_.size();
    }
    json status()
    {
      State s;
      bool c, e;
      std::string o;
      {
        std::lock_guard<std::mutex> g(state_m_);
        s = state_;
        c = connected_;
        e = enabled_;
        o = owner_;
      }
      auto now = Clock::now();
      size_t rate;
      {
        std::lock_guard<std::mutex> g(rate_m_);
        while (!sent_.empty() && now - sent_.front() > std::chrono::seconds(1))
          sent_.pop_front();
        rate = sent_.size();
      }
      return {{"ok", true}, {"state", sn(s)}, {"owner", o}, {"connected", c}, {"arm_enabled", e}, {"command_rate_hz", rate}, {"queued", queued()}};
    }
    json set_owner(const json &r)
    {
      auto w = owner_name(r.value("owner", ""));
      if (w.empty() || w == "IDLE")
        return {{"ok", false}, {"error", "invalid_owner"}};
      std::lock_guard<std::mutex> g(state_m_);
      if (owner_ != "IDLE" && owner_ != w)
        return {{"ok", false}, {"error", "owner_busy"}, {"owner", owner_}};
      // New owner, or the SAME owner re-claiming after a client restart
      // (xbox restarted without releasing, e.g. pkill): always treat as a
      // fresh session. Otherwise the restarted client's sequence numbers
      // (which restart at 0) are rejected as stale by last_input_sequence_
      // and its stick velocities never reach the controller.
      if (owner_ == "IDLE" || owner_ == w)
      {
        speedl_.reset();
        speedl_seen_.store(false);
      }
      owner_ = w;
      return {{"ok", true}, {"owner", owner_}};
    }
    json release(const json &r)
    {
      auto w = owner_name(r.value("owner", ""));
      std::lock_guard<std::mutex> g(state_m_);
      if (w.empty() || owner_ != w)
        return {{"ok", false}, {"error", "no_control"}, {"owner", owner_}};
      speedl_.request_zero();
      owner_ = "IDLE";
      return {{"ok", true}, {"owner", owner_}};
    }
    json standby(const json &r)
    {
      auto w = owner_name(r.value("owner", ""));
      {
        std::lock_guard<std::mutex> g(state_m_);
        if (w.empty() || owner_ != w)
          return {{"ok", false}, {"error", "no_control"}, {"owner", owner_}};
      }
      return enqueue(Kind::STANDBY, w, "");
    }
    json resume_gateway()
    {
      standby_.store(false);
      {
        std::lock_guard<std::mutex> g(state_m_);
        if (state_ == State::STANDBY)
        {
          state_ = State::DISCONNECTED;
          connected_ = false;
          enabled_ = false;
        }
      }
      next_connect_ = Clock::now();
      cv_.notify_one();
      return {{"ok", true}, {"state", "DISCONNECTED"}};
    }
    json enqueue(Kind k, std::string o, std::string raw)
    {
      if (k == Kind::SEND)
      {
        if (stop_forbidden(raw))
          return {{"ok", false}, {"error", "forbidden_stop_P37"}};
        if (raw.empty() || raw.front() != '{' || raw.back() != '}')
          return {{"ok", false}, {"error", "invalid_raw"}};
        o = owner_name(o);
        std::lock_guard<std::mutex> g(state_m_);
        if (o.empty() || owner_ != o)
          return {{"ok", false}, {"error", "no_control"}, {"owner", owner_}};
        if (state_ != State::READY)
          return {{"ok", false}, {"error", "not_ready"}, {"state", sn(state_)}};
      }
      auto w = std::make_shared<Work>();
      w->kind = k;
      w->owner = o;
      w->raw = std::move(raw);
      auto f = w->done.get_future();
      {
        std::lock_guard<std::mutex> g(queue_m_);
        (k == Kind::SEND ? normal_ : high_).push_back(w);
      }
      cv_.notify_one();
      if (f.wait_for(std::chrono::milliseconds(c_.rpc_timeout_ms + 5000)) != std::future_status::ready)
        return {{"ok", false}, {"error", "gateway_timeout"}};
      auto x = f.get();
      json j{{"ok", x.ok}, {"rc", x.rc}};
      if (!x.error.empty())
        j["error"] = x.error;
      {
        std::lock_guard<std::mutex> g(state_m_);
        j["state"] = sn(state_);
      }
      return j;
    }
    json handle(const json &r)
    {
      if (!r.is_object())
        return {{"ok", false}, {"error", "invalid_request"}};
      auto c = r.value("cmd", "");
      if (c == "state")
        return status();
      if (c == "set_owner")
        return set_owner(r);
      if (c == "release_owner")
        return release(r);
      if (c == "standby")
        return standby(r);
      if (c == "resume_gateway")
        return resume_gateway();
      if (c == "stop")
        return enqueue(Kind::STOP, "", "");
      if (c == "enable")
        return enqueue(Kind::ENABLE, "", "");
      if (c == "clear_error")
        return enqueue(Kind::ENABLE, "", "");
      if (c == "disable")
        return enqueue(Kind::DISABLE, "", "");
      if (c == "send")
        return enqueue(Kind::SEND, r.value("owner", ""), r.value("raw", ""));
      if (c == "speedl_update")
      {
        // Latest-value speed slot (NOT a FIFO). Only touches memory, never
        // waits for the controller; the worker's CallAsync loop sends it.
        const std::string owner = owner_name(r.value("owner", ""));
        {
          std::lock_guard<std::mutex> g(state_m_);
          if (state_ != State::READY)
            return {{"ok", false}, {"error", "not_ready"}, {"state", sn(state_)}};
          if (owner.empty() || owner_ != owner)
            return {{"ok", false}, {"error", "no_control"}, {"owner", owner_}};
        }
        if (!r.contains("vel") || !r["vel"].is_array() || r["vel"].size() != 6)
          return {{"ok", false}, {"error", "invalid_velocity"}};
        std::array<double, 6> velocity{};
        for (size_t i = 0; i < 6; ++i)
        {
          if (!r["vel"][i].is_number())
            return {{"ok", false}, {"error", "invalid_velocity"}};
          velocity[i] = r["vel"][i].get<double>();
        }
        const uint64_t seq = r.value("seq", uint64_t{0});
        const int coordinate = r.value("coordinate", 0);
        const bool deadman = r.value("deadman", false);
        if (!speedl_.update(seq, coordinate, velocity, deadman))
          return {{"ok", false}, {"error", "invalid_or_stale_speedl"}};
        speedl_seen_.store(true);
        return {{"ok", true}, {"accepted_seq", seq}, {"state", "READY"}, {"speedl_active", deadman}};
      }
      return {{"ok", false}, {"error", "unknown_command"}};
    }
    void client(int fd)
    {
      std::string buf;
      char b[4096];
      while (!stopping_ && !g_exit)
      {
        auto n = recv(fd, b, sizeof(b), 0);
        if (n <= 0)
          break;
        buf.append(b, n);
        if (buf.size() > (size_t)c_.max_line_bytes)
          break;
        size_t p;
        while ((p = buf.find('\n')) != std::string::npos)
        {
          auto line = buf.substr(0, p);
          buf.erase(0, p + 1);
          json out;
          try
          {
            out = handle(json::parse(line));
          }
          catch (const std::exception &e)
          {
            out = {{"ok", false}, {"error", "bad_json"}, {"detail", e.what()}};
          }
          auto s = out.dump() + "\n";
          size_t off = 0;
          while (off < s.size())
          {
            auto n2 = send(fd, s.data() + off, s.size() - off, MSG_NOSIGNAL);
            if (n2 <= 0)
              break;
            off += n2;
          }
        }
      }
      shutdown(fd, SHUT_RDWR);
      close(fd);
      // An owner's IPC link dying must not leave a stale speed target: the
      // watchdog/last_count would eventually stop it, but zero now is safer.
      speedl_.request_zero();
      {
        std::lock_guard<std::mutex> g(clients_m_);
        client_fds_.erase(fd);
      }
    }
    void server()
    {
      int fd = socket(AF_INET, SOCK_STREAM, 0);
      if (fd < 0)
      {
        log("ERROR", "IPC socket failed");
        return;
      }
      listen_ = fd;
      int one = 1;
      setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
      sockaddr_in a{};
      a.sin_family = AF_INET;
      a.sin_port = htons(c_.listen_port);
      if (inet_pton(AF_INET, c_.listen_ip.c_str(), &a.sin_addr) != 1 || bind(fd, (sockaddr *)&a, sizeof(a)) || listen(fd, 32))
      {
        log("ERROR", "IPC bind failed: " + std::string(strerror(errno)));
        return;
      }
      log("INFO", "IPC listening " + c_.listen_ip + ":" + std::to_string(c_.listen_port));
      while (!stopping_ && !g_exit)
      {
        pollfd p{fd, POLLIN, 0};
        if (poll(&p, 1, 500) <= 0)
          continue;
        int c = accept(fd, nullptr, nullptr);
        if (c >= 0)
        {
          std::lock_guard<std::mutex> g(clients_m_);
          client_fds_.insert(c);
          clients_.emplace_back(&Gateway::client, this, c);
        }
      }
    }
    std::shared_ptr<Work> pop()
    {
      std::lock_guard<std::mutex> g(queue_m_);
      std::shared_ptr<Work> w;
      if (!high_.empty())
      {
        w = high_.front();
        high_.pop_front();
      }
      else if (!normal_.empty())
      {
        w = normal_.front();
        normal_.pop_front();
      }
      return w;
    }
    void cancel(const std::string &e)
    {
      std::deque<std::shared_ptr<Work>> q;
      {
        std::lock_guard<std::mutex> g(queue_m_);
        q.swap(normal_);
      }
      for (auto &w : q)
        try
        {
          w->done.set_value({false, -1, e});
        }
        catch (...)
        {
        }
    }
    void cancel_all(const std::string &e)
    {
      std::deque<std::shared_ptr<Work>> q;
      {
        std::lock_guard<std::mutex> g(queue_m_);
        q.swap(normal_);
        q.insert(q.end(), high_.begin(), high_.end());
        high_.clear();
      }
      for (auto &w : q)
        try
        {
          w->done.set_value({false, -1, e});
        }
        catch (...)
        {
        }
    }
    Result call(const std::string &raw, int timeout, bool init = false)
    {
      if (!rpc_)
        return {false, -1, "not_connected"};
      try
      {
        core::Msg m(raw);
        m.setMsgID(10001);
        m.setMsgSeqID(seq_++);
        auto r = rpc_->CallAwait<RespDemo>(m, timeout);
        if (r.first || r.second.empty())
          return {false, r.first, "transport_error"};
        for (auto &i : r.second)
          if (i.return_code != 0)
          {
            if (init)
            {
              log("WARN", raw + " rc=" + std::to_string(i.return_code) + " " + i.return_message);
              continue;
            }
            log("WARN", raw + " rc=" + std::to_string(i.return_code) + " " + i.return_message);
            return {false, i.return_code, "controller_rejected"};
          }
        {
          std::lock_guard<std::mutex> g(rate_m_);
          sent_.push_back(Clock::now());
        }
        return {true, 0, ""};
      }
      catch (const std::exception &e)
      {
        log("ERROR", std::string("RPC exception: ") + e.what());
        return {false, -1, "transport_exception"};
      }
      catch (...)
      {
        return {false, -1, "transport_exception"};
      }
    }
    bool init(bool /*reattach*/)
    {
      // The controller keeps the servo enable state across gateway start and
      // transport reconnects. Recovery only clears the controller error and
      // restores session parameters. Never send {Disable}/{Enable} here.
      static const std::vector<std::string> c = {"{Clear}", "{Mode}", "{SetMaxToq}", "{Recover}", "{SetRate}", "{Var --clear}", "{Recover}", "{SetUsingSP --state=on}", "{SetSPFeedbacKp --kp={0.1,0.1,0.1,0.1,0.1,0.1}}"};
      // Init commands are millisecond-scale replies; do NOT inherit the (now
      // much larger) motion rpc_timeout_ms, or a hung init stalls for minutes.
      const int init_timeout = 30000;
      for (auto &s : c)
      {
        auto r = call(s, init_timeout, true);
        if (!r.ok)
          return false;
      }
      return true;
    }
    void reconnect(const std::string &why)
    {
      log("WARN", "RECONNECTING: " + why);
      state(State::RECONNECTING, false, false);
      cancel("cancelled_on_reconnect");
      // Never replay a pre-disconnect speed target on the new connection.
      speedl_.reset();
      speedl_seen_.store(false);
      watch_.reset();
      rpc_.reset();
      next_connect_ = Clock::now() + std::chrono::milliseconds(c_.reconnect_backoff_ms);
    }
    bool connect()
    {
      state(State::CONNECTING, false, false);
      if (!probe(c_.controller_ip, c_.controller_port, c_.probe_timeout_ms))
      {
        log("WARN", "TCP probe failed; 30s backoff");
        state(State::DISCONNECTED, false, false);
        next_connect_ = Clock::now() + std::chrono::milliseconds(c_.reconnect_backoff_ms);
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      try
      {
        rpc_ = std::make_unique<cpp_rpc::CPPClient>(c_.controller_ip, c_.controller_port);
      }
      catch (const std::exception &e)
      {
        log("ERROR", e.what());
      }
      if (!rpc_)
      {
        state(State::DISCONNECTED, false, false);
        next_connect_ = Clock::now() + std::chrono::milliseconds(c_.reconnect_backoff_ms);
        return false;
      }
      state(State::CONNECTED, true, false);
      if (!init(c_.reattach_enable))
      {
        reconnect("init failed");
        return false;
      }
      hb_fail_ = 0;
      next_hb_ = Clock::now() + std::chrono::milliseconds(c_.heartbeat_period_ms);
      speedl_.reset();
      speedl_seen_.store(false);
      state(State::READY, true, true);
      log("INFO", "READY (controller error cleared; enable state left unchanged)");
      return true;
    }
    struct Watch
    {
      Pose before;
      Clock::time_point deadline;
    };
    void heartbeat()
    {
      auto now = Clock::now();
      if (now < next_hb_)
        return;
      next_hb_ = now + std::chrono::milliseconds(c_.heartbeat_period_ms);
      if (c_.heartbeat_required)
      {
        if (heartbeat_ok(c_.heartbeat_file, c_.heartbeat_stale_ms))
          hb_fail_ = 0;
        else if (++hb_fail_ >= c_.heartbeat_fail_limit)
        {
          reconnect("heartbeat failed 3 times");
          return;
        }
      }
      if (watch_)
      {
        auto p = read_pose(c_.heartbeat_file);
        if (p)
        {
          double x = p->p[0] - watch_->before.p[0], y = p->p[1] - watch_->before.p[1], z = p->p[2] - watch_->before.p[2];
          double dot = std::abs(p->q[0] * watch_->before.q[0] + p->q[1] * watch_->before.q[1] + p->q[2] * watch_->before.q[2] + p->q[3] * watch_->before.q[3]);
          if (std::sqrt(x * x + y * y + z * z) >= c_.fake_motion_epsilon_m || dot < 0.999999)
          {
            watch_.reset();
            return;
          }
        }
        if (now >= watch_->deadline)
          reconnect("fake connection: no actual pose/orientation change");
      }
    }
    // CallAsync is the only allowed transport for continuous SpeedL frames:
    // the controller treats it as a persistent command and CallAwait would
    // block the whole worker. The callback only sets atomic fault flags; all
    // reconnect work stays in the single worker thread (never touch CPPClient
    // from the SDK callback thread).
    bool submit_speedl_async(const std::string &raw, bool is_zero_frame)
    {
      if (!rpc_)
        return false;
      // Never build an unbounded queue behind the controller. A small in-flight
      // window keeps SpeedL refreshed smoothly even when the SDK reports late
      // or missing replies, while still dropping stale joystick samples before
      // they can become delayed motion.
      constexpr int kMaxSpeedlInflight = 3;
      if (speedl_async_inflight_.load() >= kMaxSpeedlInflight)
        return true;
      try
      {
        core::Msg message(raw);
        message.setMsgID(10001);
        message.setMsgSeqID(seq_++);
        speedl_async_inflight_.fetch_add(1);
        const bool submitted = rpc_->CallAsync<RespDemo>(
            message, 250,
            [this, is_zero_frame](int transport_rc, const std::vector<RespDemo> &responses)
            {
              speedl_async_inflight_.fetch_sub(1);
              if (transport_rc < 0)
              {
                // The controller silently absorbs all-zero-velocity SpeedL
                // frames (bench probe: zero frame -> timeout, non-zero frame
                // -> rc=1). A zero-frame timeout is therefore NOT a transport
                // fault and must not trigger reconnect; only a non-zero frame
                // timing out means the controller stopped answering.
                (void)is_zero_frame;
                return;
              }
              for (const auto &response : responses)
                if (response.return_code < 0)
                  speedl_async_controller_fault_.store(true);
            });
        if (!submitted)
        {
          speedl_async_inflight_.fetch_sub(1);
          // CallAsync returns false on has_err (SDK latched a transport error)
          // or a dead socket: that is a real transport fault, reconnect.
          speedl_async_transport_fault_.store(true);
        }
        return submitted;
      }
      catch (...)
      {
        speedl_async_inflight_.fetch_sub(1);
        speedl_async_transport_fault_.store(true);
        return false;
      }
    }
    void execute(const std::shared_ptr<Work> &w)
    {
      // A normal RPC command (MoveAbsJ/MoveBlend/{Var}...) occupies the same
      // connection and the vendor stops answering SpeedL frames for its
      // duration. A non-zero frame left over from teleop can then time out and
      // be misread as a transport fault, forcing an avoidable reconnect that
      // costs ~30 s (vendor backend recovery, P36). Suppress that while the
      // command runs; a real link loss is still caught by the command's own
      // transport_error/transport_exception check below.
      struct QuietGuard
      {
        Gateway &g;
        explicit QuietGuard(Gateway &g_) : g(g_) { g.suppress_speedl_fault_.store(true); }
        ~QuietGuard()
        {
          g.speedl_async_transport_fault_.store(false);
          g.suppress_speedl_fault_.store(false);
        }
      } quiet(*this);
      if (w->kind == Kind::STOP)
      {
        cancel("cancelled_by_stop");
        speedl_.request_zero();
        w->done.set_value({true, 0, ""});
        log("INFO", "soft stop: queue discarded, SpeedL zeroed, no {Stop} sent");
        return;
      }
      if (w->kind == Kind::STANDBY)
      {
        cancel("cancelled_by_standby");
        speedl_.request_zero();
        std::this_thread::sleep_for(std::chrono::milliseconds(
            speedl_.watchdog_ms() + 3 * speedl_.send_period_ms()));
        speedl_.reset();
        speedl_seen_.store(false);
        speedl_async_transport_fault_.store(false);
        speedl_async_controller_fault_.store(false);
        {
          std::lock_guard<std::mutex> g(state_m_);
          if (owner_ == w->owner)
            owner_ = "IDLE";
        }
        standby_.store(true);
        watch_.reset();
        rpc_.reset();
        state(State::STANDBY, false, false);
        w->done.set_value({true, 0, ""});
        log("INFO", "standby: SpeedL zeroed, owner released, controller socket closed");
        return;
      }
      if (w->kind == Kind::ENABLE)
      {
        State s;
        bool en;
        {
          std::lock_guard<std::mutex> g(state_m_);
          s = state_;
          en = enabled_;
        }
        if (s != State::READY && s != State::CONNECTED)
        {
          w->done.set_value({false, -1, "not_connected"});
          return;
        }
        // Both legacy "enable" and "clear_error" requests arrive here.
        // Recovery is deliberately {Clear}-only and never power-cycles the arm.
        (void)en;
        auto r = call("{Clear}", 30000, true);
        w->done.set_value(r);
        if (!r.ok)
          reconnect("clear error failed");
        else
        {
          state(State::READY, true, true);
          log("INFO", "controller error cleared; enable state unchanged");
        }
        return;
      }
      if (w->kind == Kind::DISABLE)
      {
        speedl_.request_zero();
        std::this_thread::sleep_for(std::chrono::milliseconds(
            speedl_.watchdog_ms() + 3 * speedl_.send_period_ms()));
        auto r = call("{Disable}", 30000, true);
        w->done.set_value(r);
        if (!r.ok)
          reconnect("manual disable failed");
        else
          state(State::READY, true, false);
        return;
      }
      {
        std::lock_guard<std::mutex> g(state_m_);
        if (state_ != State::READY)
        {
          w->done.set_value({false, -1, "not_ready"});
          return;
        }
        if (owner_ != w->owner)
        {
          w->done.set_value({false, -1, "no_control"});
          return;
        }
      }
      // SpeedL and normal commands share the one RPC connection. If a speed
      // target is still live (or zero frames / watchdog window are pending),
      // stop it and let the async pipeline drain before the CallAwait below,
      // so the vendor never sees SpeedL and MoveBlend/MoveAbsJ interleaved.
      if (speedl_.needs_quiet())
      {
        speedl_.request_zero();
        const auto drain_ms = speedl_.watchdog_ms() + 3 * speedl_.send_period_ms();
        std::this_thread::sleep_for(std::chrono::milliseconds(drain_ms));
      }
      std::optional<Pose> before;
      if (is_motion(w->raw))
        before = read_pose(c_.heartbeat_file);
      auto r = call(w->raw, c_.rpc_timeout_ms);
      w->done.set_value(r);
      if (!r.ok && (r.error == "transport_error" || r.error == "transport_exception"))
      {
        // The vendor SDK can report a transport-style error even though the
        // controller is still online and merely has a latched command fault.
        // First clear that fault on the existing session. Reconnect only when
        // even {Clear} cannot be delivered. Neither path re-enables the arm.
        auto cleared = call("{Clear}", 30000, true);
        if (cleared.ok)
        {
          state(State::READY, true, true);
          log("WARN", "RPC error recovered with {Clear}; no reconnect/re-enable");
          return;
        }
        reconnect("RPC transport failure and {Clear} failed");
        return;
      }
      if (r.ok && before)
        watch_ = Watch{*before, Clock::now() + std::chrono::milliseconds(c_.fake_motion_timeout_ms)};
    }
    void worker()
    {
      next_connect_ = Clock::now();
      while (!stopping_ && !g_exit)
      {
        State s;
        {
          std::lock_guard<std::mutex> g(state_m_);
          s = state_;
        }
        if (!standby_.load() && s != State::READY && Clock::now() >= next_connect_)
          connect();
        {
          std::lock_guard<std::mutex> g(state_m_);
          s = state_;
        }
        if (s == State::READY)
        {
          heartbeat();
          {
            std::lock_guard<std::mutex> g(state_m_);
            s = state_;
          }
          if (s != State::READY)
            continue;
          // SpeedL async pipeline. A transport fault means the RPC link is
          // really gone: reset and reconnect. A controller fault (rc<0) means
          // the backend is alive but rejected the frame: zero and retry there
          // without reconnecting, because the reconnect cycle itself can kill
          // the backend (P36).
          if (speedl_async_transport_fault_.exchange(false) && !suppress_speedl_fault_.load())
          {
            speedl_.reset();
            auto cleared = call("{Clear}", 30000, true);
            if (cleared.ok)
            {
              state(State::READY, true, true);
              log("WARN", "SpeedL error recovered with {Clear}; no reconnect/re-enable");
            }
            else
              reconnect("SpeedL async transport failure and {Clear} failed");
            continue;
          }
          if (speedl_async_controller_fault_.exchange(false))
          {
            speedl_.request_zero();
            log("WARN", "SpeedL controller rejected (rc<0); zeroing, no reconnect (P36)");
          }
          if (auto frame = speedl_.tick())
          {
            const std::string raw = speedl_.command(*frame, c_.arm_id);
            if (!submit_speedl_async(raw, frame->zero))
            {
              // Transport died. Busy samples are dropped inside
              // submit_speedl_async() and are never converted into delayed
              // commands or queued for later execution.
              speedl_.request_zero();
            }
          }
        }
        if (auto w = pop())
          execute(w);
        else
        {
          std::unique_lock<std::mutex> g(queue_m_);
          // 10 Hz SpeedL needs a ~10 ms tick so send deadlines do not jitter
          // beyond a whole 100 ms period; otherwise idle waits stay coarse.
          const int wait_ms = (s == State::READY) ? 10 : 50;
          cv_.wait_for(g, std::chrono::milliseconds(wait_ms));
        }
      }
      cancel_all("gateway_stopping");
      rpc_.reset();
      state(State::DISCONNECTED, false, false);
    }
    Config c_;
    codex_speedl::SpeedLRuntime speedl_;
    std::atomic<int> speedl_async_inflight_{0};
    std::atomic<bool> speedl_async_transport_fault_{false};
    std::atomic<bool> speedl_async_controller_fault_{false};
    // While a normal RPC command (MoveAbsJ/MoveBlend) is running the vendor
    // stops answering SpeedL frames; a non-zero frame left over from teleop can
    // then time out and be misread as a transport fault. execute() sets this so
    // such stale timeouts are dropped instead of triggering a reconnect.
    std::atomic<bool> suppress_speedl_fault_{false};
    std::atomic<bool> speedl_seen_{false};
    std::atomic<bool> stopping_{false};
    std::atomic<bool> standby_{false};
    std::atomic<int> listen_{-1};
    std::thread worker_, server_;
    std::vector<std::thread> clients_;
    std::set<int> client_fds_;
    std::mutex log_m_, state_m_, queue_m_, rate_m_, clients_m_;
    std::condition_variable cv_;
    State state_{State::DISCONNECTED};
    bool connected_{false}, enabled_{false};
    std::string owner_{"IDLE"};
    std::deque<std::shared_ptr<Work>> high_, normal_;
    std::deque<Clock::time_point> sent_;
    std::unique_ptr<cpp_rpc::CPPClient> rpc_;
    int64_t seq_{1};
    Clock::time_point next_connect_{}, next_hb_{};
    int hb_fail_{0};
    std::optional<Watch> watch_;
  };
}
int main(int argc, char **argv)
{
  std::string config = "config/arm_gateway.yaml";
  for (int i = 1; i < argc; ++i)
  {
    std::string a = argv[i];
    if (a == "--config" && i + 1 < argc)
      config = argv[++i];
    else if (a == "-h" || a == "--help")
    {
      std::cout << "Usage: " << argv[0] << " [--config path]\n";
      return 0;
    }
    else
      return 64;
  }
  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  signal(SIGPIPE, SIG_IGN);
  try
  {
    Gateway g(Config::from_file(config));
    g.start();
    while (!g_exit)
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    g.stop();
    return 0;
  }
  catch (const std::exception &e)
  {
    std::cerr << "FATAL: " << e.what() << '\n';
    return 1;
  }
}
