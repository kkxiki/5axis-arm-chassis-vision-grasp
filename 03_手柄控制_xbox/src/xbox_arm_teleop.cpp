// Xbox teleoperation for the Zhenshi controller (SpeedL continuous).
// Uses ArmGateway IPC; this process never connects to controller port 5868.
//
// SpeedL port (2026-08-12, from codex/speedl_xbox_teleop): the joystick hot
// path no longer issues MoveBlend steps. The joystick thread only updates a
// latest-velocity slot; a dedicated sender thread pushes it to the gateway as
// SpeedL at speedl_send_hz. Motion is continuous instead of step-and-stop.

#include <linux/joystick.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include <limits>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "speedl_ipc_client.hpp"
#include "velocity_mapper.hpp"

namespace
{
  std::atomic<bool> g_exit{false};
  void on_signal(int) { g_exit.store(true); }

  std::string trim(std::string s)
  {
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos)
      return "";
    return s.substr(b, s.find_last_not_of(" \t\r\n") - b + 1);
  }
  std::string unquote(std::string s)
  {
    s = trim(std::move(s));
    if (s.size() >= 2 && ((s.front() == '"' && s.back() == '"') ||
                          (s.front() == '\'' && s.back() == '\'')))
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
      if (line.size() >= 3 && static_cast<unsigned char>(line[0]) == 0xef &&
          static_cast<unsigned char>(line[1]) == 0xbb && static_cast<unsigned char>(line[2]) == 0xbf)
        line.erase(0, 3);
      const auto hash = line.find('#');
      if (hash != std::string::npos)
        line.erase(hash);
      const auto colon = line.find(':');
      if (colon == std::string::npos)
        continue;
      const auto key = trim(line.substr(0, colon));
      if (!key.empty())
        out[key] = unquote(line.substr(colon + 1));
    }
    return out;
  }
  bool as_bool(std::string s)
  {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c)
                   { return std::tolower(c); });
    if (s == "true" || s == "1" || s == "yes" || s == "on")
      return true;
    if (s == "false" || s == "0" || s == "no" || s == "off")
      return false;
    throw std::runtime_error("invalid boolean: " + s);
  }

  std::vector<double> parse_number_list(const std::string &text)
  {
    std::vector<double> values;
    std::istringstream input(text);
    std::string token;
    while (std::getline(input, token, ','))
    {
      token = trim(token);
      if (token.empty())
        throw std::runtime_error("empty value in comma-separated number list");
      size_t used = 0;
      const double value = std::stod(token, &used);
      if (used != token.size() || !std::isfinite(value))
        throw std::runtime_error("invalid numeric value: " + token);
      values.push_back(value);
    }
    return values;
  }

  std::string format_number_list(const std::vector<double> &values)
  {
    std::ostringstream output;
    output.setf(std::ios::fixed);
    output.precision(8);
    for (size_t i = 0; i < values.size(); ++i)
    {
      if (i)
        output << ',';
      output << values[i];
    }
    return output.str();
  }

  // Read the ACTUAL right-arm pose (x y z qx qy qz qw) written continuously by
  // report_actual_pos.py (ZMQ 19091 subscriber) to /tmp/move_actual.txt. Only
  // used for the arrival wait of the View-button home MoveBlend now.
  bool read_actual_7(const std::string &path, double *out7)
  {
    std::ifstream f(path);
    if (!f)
      return false;
    for (int i = 0; i < 7; ++i)
      f >> out7[i];
    return f.good();
  }

  template <typename T>
  T get(const std::unordered_map<std::string, std::string> &, const std::string &, const T &);
  template <>
  int get(const std::unordered_map<std::string, std::string> &m, const std::string &k, const int &d)
  {
    const auto it = m.find(k);
    return it == m.end() || it->second.empty() ? d : std::stoi(it->second);
  }
  template <>
  double get(const std::unordered_map<std::string, std::string> &m, const std::string &k, const double &d)
  {
    const auto it = m.find(k);
    return it == m.end() || it->second.empty() ? d : std::stod(it->second);
  }
  template <>
  bool get(const std::unordered_map<std::string, std::string> &m, const std::string &k, const bool &d)
  {
    const auto it = m.find(k);
    return it == m.end() || it->second.empty() ? d : as_bool(it->second);
  }
  template <>
  std::string get(const std::unordered_map<std::string, std::string> &m, const std::string &k, const std::string &d)
  {
    const auto it = m.find(k);
    return it == m.end() ? d : it->second;
  }

  struct Config
  {
    std::string joystick_device{"/dev/input/js0"}, expected_joystick_name, lock_path{"/tmp/xbox_arm_teleop.lock"};
    std::string gateway_ip{"127.0.0.1"}, gateway_owner{"xbox"}, arm_id{"single"};
    int gateway_port{6200};
    bool allow_motion{false};
    int base_coordinate{0}, tool_coordinate{1};
    int cart_axis_x{0}, cart_axis_y{1}, cart_axis_z{2}, cart_axis_roll{3}, cart_axis_pitch{4}, cart_axis_yaw{5};
    int joint_count{5};
    // Controller v-values, used only by the Safe Home / View MoveAbsJ commands.
    int speed_fine{5}, speed_normal{25}, speed_fast{50};
    bool fast_enabled{false};
    // SpeedL continuous teleop. Keep the first bench test at 10 Hz / 10 mm/s /
    // deadman on (see codex/speedl_xbox_teleop/README step 6). Units m/s and
    // rad/s must be confirmed on the real controller before raising them.
    int speedl_send_hz{10};
    double fine_linear_mps{0.010};
    double fine_angular_radps{0.052360}; // 3 deg/s
    double normal_linear_mps{0.030};
    double normal_angular_radps{0.139626}; // 8 deg/s
    double fast_linear_mps{0.060};
    double fast_angular_radps{0.261799}; // 15 deg/s
    // Legacy MoveBlend-stepping fields kept for config compatibility; no longer
    // used for motion.
    double step_mm_fine{6}, step_mm_normal{16}, step_mm_fast{30};
    double step_deg_fine{1.5}, step_deg_normal{4}, step_deg_fast{8};
    double move_timeout_sec{6.0}, reach_epsilon_m{0.003};
    std::string actual_file{"/tmp/move_actual.txt"};
    bool deadman_required{true};
    // LB toggle-enable: each LB press flips motion_enabled_ (start disabled for
    // safety). Independent of the deadman path; keep deadman_required=false
    // when this is enabled.
    bool lb_toggle_enable{false};
    int repeat_hz{12}, jog_increase_count{1}, deadman_max_hold_ms{0};
    double deadzone{0.10};
    double diag_ratio{1.0};
    int button_a{0}, button_b{1}, button_x{2}, button_y{3}, button_lb{4}, button_view{6}, button_menu{7}, button_capture{-1}, button_pause{2};
    int axis_left_x{0}, axis_left_y{1}, axis_lt{2}, axis_right_x{3}, axis_right_y{4}, axis_rt{5}, axis_dpad_x{6}, axis_dpad_y{7};
    int sign_left_x{1}, sign_left_y{-1}, sign_right_x{1}, sign_right_y{-1}, sign_dpad_y{-1}, sign_dpad_x{1};
    bool triggers_are_signed{true};
    double trigger_threshold{0.35};
    bool gripper_enabled{false};
    int gripper_hand_id{-1}, gripper_open_state{2}, gripper_close_state{0};
    int gripper_pre_state{1};
    std::string gripper_command{"/home/nvidia/calibration/ctag2f120_control.py"};
    bool safe_home_enabled{false};
    std::string safe_home_joints_rad;
    int safe_home_joint_count{7}, safe_home_speed{5};
    double safe_home_hold_seconds{2.0};
    // View-button MoveAbsJ return is disabled until the controller backend
    // issue behind the rc=1 MoveAbsJ failures is resolved (2026-08-13).
    bool view_home_enabled{false};
    std::string home_pose;
    std::vector<double> home_pose_parsed;
    std::string home_joints;
    std::vector<double> home_joints_parsed;

    static Config from_file(const std::string &path)
    {
      const auto v = load_yaml(path);
      Config c;
#define GET(T, x) c.x = get<T>(v, #x, c.x)
      GET(std::string, joystick_device);
      GET(std::string, expected_joystick_name);
      GET(std::string, lock_path);
      GET(std::string, gateway_ip);
      GET(int, gateway_port);
      GET(std::string, gateway_owner);
      GET(std::string, arm_id);
      GET(bool, allow_motion);
      GET(int, base_coordinate);
      GET(int, tool_coordinate);
      GET(int, cart_axis_x);
      GET(int, cart_axis_y);
      GET(int, cart_axis_z);
      GET(int, cart_axis_roll);
      GET(int, cart_axis_pitch);
      GET(int, cart_axis_yaw);
      GET(int, joint_count);
      GET(int, speed_fine);
      GET(int, speed_normal);
      GET(int, speed_fast);
      GET(bool, fast_enabled);
      GET(int, speedl_send_hz);
      GET(double, fine_linear_mps);
      GET(double, fine_angular_radps);
      GET(double, normal_linear_mps);
      GET(double, normal_angular_radps);
      GET(double, fast_linear_mps);
      GET(double, fast_angular_radps);
      GET(double, step_mm_fine);
      GET(double, step_mm_normal);
      GET(double, step_mm_fast);
      GET(double, step_deg_fine);
      GET(double, step_deg_normal);
      GET(double, step_deg_fast);
      GET(double, move_timeout_sec);
      GET(double, reach_epsilon_m);
      GET(std::string, actual_file);
      GET(int, repeat_hz);
      GET(int, jog_increase_count);
      GET(bool, deadman_required);
      GET(bool, lb_toggle_enable);
      GET(int, deadman_max_hold_ms);
      GET(double, deadzone);
      GET(double, diag_ratio);
      GET(int, button_a);
      GET(int, button_b);
      GET(int, button_x);
      GET(int, button_y);
      GET(int, button_lb);
      GET(int, button_view);
      GET(int, button_menu);
      GET(int, button_capture);
      GET(int, button_pause);
      GET(int, axis_left_x);
      GET(int, axis_left_y);
      GET(int, axis_lt);
      GET(int, axis_right_x);
      GET(int, axis_right_y);
      GET(int, axis_rt);
      GET(int, axis_dpad_x);
      GET(int, axis_dpad_y);
      GET(int, sign_left_x);
      GET(int, sign_left_y);
      GET(int, sign_right_x);
      GET(int, sign_right_y);
      GET(int, sign_dpad_y);
      GET(int, sign_dpad_x);
      GET(bool, triggers_are_signed);
      GET(double, trigger_threshold);
      GET(bool, gripper_enabled);
      GET(int, gripper_hand_id);
      GET(int, gripper_open_state);
      GET(int, gripper_close_state);
      GET(int, gripper_pre_state);
      GET(std::string, gripper_command);
      GET(bool, safe_home_enabled);
      GET(bool, view_home_enabled);
      GET(std::string, safe_home_joints_rad);
      GET(int, safe_home_joint_count);
      GET(int, safe_home_speed);
      GET(double, safe_home_hold_seconds);
      GET(std::string, home_pose);
      GET(std::string, home_joints);
#undef GET
      c.validate();
      if (!c.home_pose.empty())
        c.home_pose_parsed = parse_number_list(c.home_pose);
      if (!c.home_joints.empty())
      {
        auto joints = parse_number_list(c.home_joints);
        if (joints.size() != 7 && joints.size() != 10)
          throw std::runtime_error("home_joints must contain 7 or 10 joint values");
        if (joints.size() == 7)
          joints.resize(10, 0.0);
        c.home_joints = format_number_list(joints);
        c.home_joints_parsed = joints;
      }
      // The verified calibration code writes 10 controller slots. A seven-joint
      // Safe Home is therefore padded with three fixed zero slots here.
      if (c.safe_home_enabled)
      {
        auto joints = parse_number_list(c.safe_home_joints_rad);
        if (joints.size() == 7)
          joints.resize(10, 0.0);
        c.safe_home_joints_rad = format_number_list(joints);
      }
      return c;
    }
    void validate() const
    {
      if (arm_id != "left" && arm_id != "right" && arm_id != "single")
        throw std::runtime_error("arm_id must be left, right, or single");
      if (gateway_port <= 0 || gateway_port > 65535)
        throw std::runtime_error("gateway_port out of range");
      if (gateway_ip != "127.0.0.1" && gateway_ip != "localhost")
        throw std::runtime_error("gateway_ip must stay local (127.0.0.1/localhost)");
      if (joint_count < 1 || joint_count > 10)
        throw std::runtime_error("joint_count must be 1..10");
      if (deadman_max_hold_ms != 0 && (deadman_max_hold_ms < 500 || deadman_max_hold_ms > 30000))
        throw std::runtime_error("deadman_max_hold_ms must be 0 or 500..30000");
      if (deadzone < 0.0 || deadzone >= 0.8)
        throw std::runtime_error("deadzone must be in [0, 0.8)");
      if (speedl_send_hz < 1 || speedl_send_hz > 50)
        throw std::runtime_error("speedl_send_hz must be 1..50");
      for (double x : {fine_linear_mps, normal_linear_mps, fast_linear_mps})
        if (x <= 0.0 || x > 0.2)
          throw std::runtime_error("speedl linear speed must be in (0, 0.2] m/s");
      for (double x : {fine_angular_radps, normal_angular_radps, fast_angular_radps})
        if (x <= 0.0 || x > 1.0)
          throw std::runtime_error("speedl angular speed must be in (0, 1.0] rad/s");
      if (move_timeout_sec < 1.0 || move_timeout_sec > 60.0)
        throw std::runtime_error("move_timeout_sec must be in [1, 60]");
      if (reach_epsilon_m < 0.0001 || reach_epsilon_m > 0.05)
        throw std::runtime_error("reach_epsilon_m must be in [0.0001, 0.05]");
      if (actual_file.empty() || actual_file.front() != '/')
        throw std::runtime_error("actual_file must be an absolute path");
      for (int x : {cart_axis_x, cart_axis_y, cart_axis_z, cart_axis_roll, cart_axis_pitch, cart_axis_yaw})
        if (x < 0 || x > 5)
          throw std::runtime_error("cart axis must be 0..5");
      if (gripper_enabled && gripper_command.empty())
        throw std::runtime_error("gripper_enabled needs gripper_command");
      if (lock_path.empty() || lock_path.front() != '/')
        throw std::runtime_error("lock_path must be an absolute path");
      if (safe_home_enabled)
      {
        const auto values = parse_number_list(safe_home_joints_rad);
        if ((safe_home_joint_count != 7 && safe_home_joint_count != 10) || values.size() != static_cast<size_t>(safe_home_joint_count))
          throw std::runtime_error("Safe Home must contain exactly 7 or 10 verified joint values");
      }
      if (!home_pose.empty() && parse_number_list(home_pose).size() != 7)
        throw std::runtime_error("home_pose must contain exactly 7 values: x,y,z,qx,qy,qz,qw");
    }
  };

  class Controller
  {
  public:
    explicit Controller(const Config &c) : c_(c) {}
    ~Controller()
    {
      std::string ignored;
      request("{\"cmd\":\"release_owner\",\"owner\":\"" + json_escape(c_.gateway_owner) + "\"}", ignored, 1000);
    }
    bool faulted() const { return faulted_; }
    // Eager connect+init at startup; retries internally every 5 s while the
    // controller backend is down (self-heals after a controller restart).
    void ensure_connected() { ensure(); }
    bool set_arm_enabled(bool enabled)
    {
      std::string response;
      const std::string command = enabled ? "{\"cmd\":\"enable\"}" : "{\"cmd\":\"disable\"}";
      const bool exchanged = request(command, response, 35000);
      const bool ok = exchanged && response.find("\"ok\":true") != std::string::npos;
      if (!ok)
        std::cerr << "[GATEWAY] arm " << (enabled ? "enable" : "disable") << " failed: " << response << '\n';
      return ok;
    }
    // Send one absolute-pose MoveBlend path on the existing connection and wait
    // for arrival (position AND orientation), so consecutive paths never
    // overlap on the controller. Used only by the View-button home now.
    bool move_to_pose(const double PT[3], const double QT[4], const std::string &label)
    {
      std::ostringstream value;
      value << std::fixed << std::setprecision(8)
            << PT[0] << ',' << PT[1] << ',' << PT[2] << ','
            << QT[0] << ',' << QT[1] << ',' << QT[2] << ',' << QT[3];
      // Slotted MoveBlend sequence, matching the verified move_pose_tool_rot
      // shape. A stale xbox_step can survive a hard kill of a previous client
      // (e.g. pkill mid-sequence) and would make the Var below return rc=-2
      // "variable already exists", stalling every subsequent step; clear it
      // first. The leading DeleteVar is tolerated because the first-ever use
      // returns rc=-2 "variable not exists" (send_tolerant ignores the rc).
      const std::string name = "xbox_step";
      send_tolerant(slot("{DeleteVar --name=" + name + "}"), "clean stale step var");
      const std::vector<std::string> cmds = {
          slot("{Var --type=robottarget --name=" + name + " --value={" + value.str() + "}}"),
          slot("{MoveBlend --type=first_insert}"),
          slot("{MoveBlend --type=insert_line --robottarget_var=" + name + " --zone={0.0,0.0}}"),
          slot("{MoveBlend --type=start}"),
          slot("{DeleteVar --name=" + name + "}"),
      };
      for (const auto &cmd : cmds)
        if (!send(cmd, label, 30000))
          return false;
      // Synchronous arrival wait (like wait_reach in move_pose_tool_rot).
      const auto wait_start = std::chrono::steady_clock::now();
      bool arrived = false;
      double last_dist_m = 1e9;
      while (std::chrono::duration<double>(std::chrono::steady_clock::now() - wait_start)
                 .count() < c_.move_timeout_sec)
      {
        double b7[7];
        if (read_actual_7(c_.actual_file, b7))
        {
          last_dist_m = std::sqrt(std::pow(b7[0] - PT[0], 2) + std::pow(b7[1] - PT[1], 2) +
                                  std::pow(b7[2] - PT[2], 2));
          const double dot = std::fabs(b7[3] * QT[0] + b7[4] * QT[1] + b7[5] * QT[2] + b7[6] * QT[3]);
          if (last_dist_m < c_.reach_epsilon_m && dot > 0.999)
          {
            arrived = true;
            break;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if (!arrived)
      {
        // The RPC accepted the MoveBlend but the arm did not reach the target:
        // the backend is in a "half-alive" state (e.g. lost OPERATION_ENABLE,
        // P19). A live socket but no motion must be treated as a fault, else
        // every subsequent step just times out and the operator sees no
        // response. Invalidating forces a full Clear+Enable re-init through the
        // 30 s probed reconnect, which restores motion.
        std::cerr << "[RPC] " << label << ": not reached in " << c_.move_timeout_sec
                  << " s (dist=" << last_dist_m * 1000.0 << " mm); re-initialising RPC (arm may be in fault state)\n";
        invalidate();
      }
      return arrived;
    }
    // Return to the configured home_pose (View button), same single-client path.
    bool go_home()
    {
      if (c_.home_pose_parsed.size() != 7)
      {
        std::cerr << "[SAFE] go home: home_pose not configured\n";
        return false;
      }
      const double PT[3] = {c_.home_pose_parsed[0], c_.home_pose_parsed[1], c_.home_pose_parsed[2]};
      const double QT[4] = {c_.home_pose_parsed[3], c_.home_pose_parsed[4], c_.home_pose_parsed[5], c_.home_pose_parsed[6]};
      std::cout << "[SAFE] returning to home pose\n";
      return move_to_pose(PT, QT, "go home");
    }
    bool gripper(int state, const std::string &label)
    {
      return send(slot("{SetHandState --state=" + std::to_string(state) + " --hand_id=" + std::to_string(c_.gripper_hand_id) + "}"), label);
    }
    bool home(const std::string &joints, int speed)
    {
      // Like the grasp program's send_rpcsy: rc<0 here can be normal warnings
      // (rc=-2 variable not exists, rc=-40 prepare cancelled), so only a
      // TRANSPORT failure counts as a fault. Treating these rc codes as faults
      // parked the client and the reconnect cycle killed the controller backend
      // (P36). See yolo_result_subscriber.cpp init_pos_cmds / send_rpcsy.
      const std::string name = "xbox_safe_home_" + std::to_string(++home_sequence_);
      if (!send_tolerant("{Var --type=jointtarget --name=" + name + " --value={" + joints + "}}", "create home target"))
        return false;
      if (!send_tolerant(slot("{MoveAbsJ --jointtarget_var=" + name + " --speed=v" + std::to_string(speed) + "}"), "MoveAbsJ safe home"))
        return false;
      return send_tolerant("{DeleteVar --name=" + name + "}", "delete home target");
    }
    // Send and only fail on transport errors. Controller rc codes are printed
    // but ignored, matching the proven grasp program path.
    bool send_tolerant(const std::string &command, const std::string &label, int timeout_ms = 30000)
    {
      return send(command, label, timeout_ms);
    }
    // Park the current client and schedule a probed reconnect. Used when the
    // backend accepts RPC but the arm does not move (move_to_pose timeout).
    void invalidate()
    {
      faulted_ = true;
      std::string ignored;
      request("{\"cmd\":\"stop\"}", ignored, 5000);
    }

  private:
    std::string slot(const std::string &command) const
    {
      if (c_.arm_id == "single") return command;
      const std::string inner = command.substr(1, command.size() - 2);
      return c_.arm_id == "left" ? "{" + inner + "||NotRunExecute}" : "{NotRunExecute||" + inner + "}";
    }
    bool ensure()
    {
      std::string state;
      if (!request("{\"cmd\":\"state\"}", state, 2000) || state.find("\"state\":\"READY\"") == std::string::npos)
      {
        faulted_ = true;
        return false;
      }
      std::string owner;
      const std::string body = "{\"cmd\":\"set_owner\",\"owner\":\"" + json_escape(c_.gateway_owner) + "\"}";
      if (!request(body, owner, 2000) || owner.find("\"ok\":true") == std::string::npos)
      {
        faulted_ = true;
        return false;
      }
      faulted_ = false;
      return true;
    }
    bool send(const std::string &command, const std::string &label, int timeout_ms = 1500)
    {
      // Only re-run the state+set_owner handshake while faulted.
      if (faulted_ && !ensure())
        return false;
      return send_existing(command, label, timeout_ms);
    }
    bool send_existing(const std::string &command, const std::string &label, int timeout_ms = 1500)
    {
      std::string response;
      const std::string body = "{\"cmd\":\"send\",\"owner\":\"" + json_escape(c_.gateway_owner) +
                               "\",\"raw\":\"" + json_escape(command) + "\"}";
      const bool exchanged = request(body, response, timeout_ms + 5000);
      const bool ok = exchanged && response.find("\"ok\":true") != std::string::npos;
      if (!ok)
      {
        std::cerr << "[GATEWAY] " << label << " failed: " << response << '\n';
        if (!exchanged || response.find("transport_") != std::string::npos || response.find("not_ready") != std::string::npos)
          faulted_ = true;
      }
      else
        faulted_ = false;
      return ok;
    }
    static std::string json_escape(const std::string &value)
    {
      std::ostringstream out;
      for (unsigned char c : value)
      {
        if (c == '"' || c == '\\')
          out << '\\' << c;
        else if (c == '\n')
          out << "\\n";
        else if (c == '\r')
          out << "\\r";
        else if (c == '\t')
          out << "\\t";
        else if (c >= 0x20)
          out << c;
      }
      return out.str();
    }
    bool request(const std::string &line, std::string &response, int timeout_ms)
    {
      response.clear();
      int fd = socket(AF_INET, SOCK_STREAM, 0);
      if (fd < 0)
        return false;
      timeval tv{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
      setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
      setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
      sockaddr_in address{};
      address.sin_family = AF_INET;
      address.sin_port = htons(static_cast<uint16_t>(c_.gateway_port));
      if (inet_pton(AF_INET, c_.gateway_ip == "localhost" ? "127.0.0.1" : c_.gateway_ip.c_str(), &address.sin_addr) != 1 ||
          connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0)
      {
        close(fd);
        return false;
      }
      const std::string wire = line + "\n";
      size_t offset = 0;
      while (offset < wire.size())
      {
        const ssize_t n = ::send(fd, wire.data() + offset, wire.size() - offset, MSG_NOSIGNAL);
        if (n <= 0)
        {
          close(fd);
          return false;
        }
        offset += static_cast<size_t>(n);
      }
      char ch;
      while (response.size() < 262144)
      {
        const ssize_t n = recv(fd, &ch, 1, 0);
        if (n <= 0)
        {
          close(fd);
          return false;
        }
        if (ch == '\n')
          break;
        response.push_back(ch);
      }
      close(fd);
      return !response.empty();
    }
    const Config &c_;
    bool faulted_{false};
    uint64_t home_sequence_{0};
  };

  class ProcessLock
  {
  public:
    explicit ProcessLock(const std::string &path)
    {
      fd_ = open(path.c_str(), O_RDWR | O_CREAT, 0600);
      if (fd_ < 0)
        throw std::runtime_error("cannot open lock file: " + path);
      if (flock(fd_, LOCK_EX | LOCK_NB) != 0)
      {
        close(fd_);
        throw std::runtime_error("another xbox_arm_teleop process already owns " + path);
      }
    }
    ~ProcessLock()
    {
      if (fd_ >= 0)
      {
        flock(fd_, LOCK_UN);
        close(fd_);
      }
    }
    ProcessLock(const ProcessLock &) = delete;
    ProcessLock &operator=(const ProcessLock &) = delete;

  private:
    int fd_{-1};
  };

  class Teleop
  {
  public:
    Teleop(Config c, bool dry)
        : c_(std::move(c)), dry_(dry), rpc_(c_),
          speedl_client_(c_.gateway_ip == "localhost" ? "127.0.0.1" : c_.gateway_ip,
                         c_.gateway_port, c_.gateway_owner)
    {
      buttons_.fill(0);
      axes_.fill(0);
      motion_enabled_.store(!c_.lb_toggle_enable);
    }
    int run()
    {
      banner();
      ProcessLock lock(c_.lock_path);
      int fd = open(c_.joystick_device.c_str(), O_RDONLY | O_NONBLOCK);
      if (fd < 0)
        throw std::runtime_error("cannot open " + c_.joystick_device + "; check /dev/input/js* permissions");
      char name[128] = "unknown";
      ioctl(fd, JSIOCGNAME(sizeof(name)), name);
      std::cout << "[JOY] connected: " << name << " (" << c_.joystick_device << ")\n";
      if (!c_.expected_joystick_name.empty() && std::string(name).find(c_.expected_joystick_name) == std::string::npos)
      {
        close(fd);
        throw std::runtime_error("joystick name does not contain expected_joystick_name: " + c_.expected_joystick_name);
      }
      // Eagerly try the RPC+init now (instead of waiting for the first stick
      // push) so the log shows the true motion state right away; ensure()
      // keeps retrying every 5 s when the controller backend is down.
      rpc_.ensure_connected();
      // SpeedL persistent IPC + one-time owner claim. Not fatal: the sender
      // thread re-claims inside update() when the gateway becomes available.
      std::string reply;
      if (!speedl_client_.connect_and_claim(&reply))
        std::cout << "[SPEEDL] gateway owner unavailable: " << trim(reply) << "\n";
      if (!dry_ && c_.lb_toggle_enable)
      {
        const bool disabled = rpc_.set_arm_enabled(false);
        motion_enabled_.store(false);
        std::cout << "[ENABLE] startup hardware DISABLE "
                  << (disabled ? "OK" : "FAILED") << "\n";
      }
      speed_sender_ = std::thread([this]
                                  { speed_sender_loop(); });
      pollfd p{};
      p.fd = fd;
      p.events = POLLIN;
      while (!g_exit.load())
      {
        const int r = poll(&p, 1, 40);
        if (r < 0)
        {
          if (errno == EINTR)
            continue;
          stop("joystick poll error");
          close(fd);
          throw std::runtime_error("poll failed");
        }
        if (p.revents & (POLLERR | POLLHUP | POLLNVAL))
        {
          // The 2.4 GHz receiver drops the radio link when the gamepad sleeps
          // or is switched off; js0 disappears and xpad re-creates it when the
          // pad is woken/re-paired. Do NOT fatal-exit: wait for the device to
          // reappear, keeping the RPC connection alive, and resume on
          // reconnect. The sender keeps pushing zero in the meantime.
          close(fd);
          stop("joystick disconnected");
          // Push a zero frame immediately so no stale non-zero SpeedL frame is
          // left in-flight to trip the gateway's async transport watchdog while
          // the pad is gone (keeps the arm still, not just the local target).
          if (!manual_standby_.load())
            speedl_client_.zero(tool_ ? c_.tool_coordinate : c_.base_coordinate);
          std::cerr << "[JOY] disconnected; waiting for " << c_.joystick_device
                    << " to reappear (wake / re-pair the gamepad)\n";
          while (!g_exit.load())
          {
            const int nfd = open(c_.joystick_device.c_str(), O_RDONLY | O_NONBLOCK);
            if (nfd >= 0)
            {
              char n2[128] = "unknown";
              ioctl(nfd, JSIOCGNAME(sizeof(n2)), n2);
              fd = nfd;
              p.fd = fd;
              buttons_.fill(0);
              axes_.fill(0);
              std::cout << "[JOY] reconnected: " << n2 << "\n";
              break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
          }
          if (g_exit.load())
            break;
          continue;
        }
        if (p.revents & POLLIN)
        {
          js_event e{};
          while (read(fd, &e, sizeof(e)) == sizeof(e))
            event(e);
        }
        tick();
      }
      // Exit: zero the target, stop the sender, send one final zero over the
      // persistent IPC, then release the owner. No {Stop} is ever sent (P37);
      // the gateway's watchdog + short last_count backstop the stop.
      stop("program exit");
      speed_sender_exit_.store(true);
      if (speed_sender_.joinable())
        speed_sender_.join();
      if (!manual_standby_.load())
      {
        speedl_client_.zero(tool_ ? c_.tool_coordinate : c_.base_coordinate);
        speedl_client_.release();
      }
      close(fd);
      return rpc_.faulted() ? 2 : 0;
    }

  private:
    static constexpr size_t N = 32;
    bool pressed(int n) const { return n >= 0 && n < static_cast<int>(N) && buttons_[n]; }
    int16_t axis(int n) const { return n >= 0 && n < static_cast<int>(N) ? axes_[n] : 0; }
    double stick(int n, int sign = 1) const { return std::clamp(axis(n) / 32767.0 * sign, -1.0, 1.0); }
    double trigger(int n) const
    {
      const double raw = axis(n);
      return std::clamp(c_.triggers_are_signed ? (raw + 32767.0) / 65534.0 : raw / 32767.0, 0.0, 1.0);
    }
    // Signed deflection past the deadzone, linearly rescaled to [0,1].
    double deflection(int n, int sign) const
    {
      const double v = stick(n, sign);
      if (std::abs(v) <= c_.deadzone)
        return 0.0;
      const double s = v >= 0.0 ? 1.0 : -1.0;
      return s * (std::abs(v) - c_.deadzone) / (1.0 - c_.deadzone);
    }
    double selected_linear_mps() const
    {
      return mode_speed_ == 0 ? c_.fine_linear_mps : mode_speed_ == 1 ? c_.normal_linear_mps
                                                                      : c_.fast_linear_mps;
    }
    double selected_angular_radps() const
    {
      return mode_speed_ == 0 ? c_.fine_angular_radps : mode_speed_ == 1 ? c_.normal_angular_radps
                                                                         : c_.fast_angular_radps;
    }
    // Map the current sticks into a six-velocity target [vx,vy,vz,wx,wy,wz].
    // Preserves the project's customised mapping, signs, tool-frame swap and
    // the diagonal discrimination for BOTH sticks (user request 2026-08-12).
    std::array<double, 6> desired_velocity() const
    {
      std::array<double, 6> v{};
      const double lin = selected_linear_mps();
      const double ang = selected_angular_radps();
      const auto shape = [&](int axis, int sign)
      {
        return codex_speedl::shape_axis(stick(axis, sign), c_.deadzone);
      };
      // The six Cartesian directions share one selected coordinate frame.
      const double lx = shape(c_.axis_left_x, c_.sign_left_x);
      const double ly = shape(c_.axis_left_y, c_.sign_left_y);
      const double rx = shape(c_.axis_right_x, c_.sign_right_x);
      const double ry = shape(c_.axis_right_y, c_.sign_right_y);
      // Diagonal discrimination: a corner push keeps only the dominant axis.
      double lx_eff = lx, ly_eff = ly;
      if (std::fabs(ly_eff) > c_.diag_ratio * std::fabs(lx_eff))
        lx_eff = 0.0;
      else if (std::fabs(lx_eff) > c_.diag_ratio * std::fabs(ly_eff))
        ly_eff = 0.0;
      double rx_eff = rx, ry_eff = ry;
      if (std::fabs(ry_eff) > c_.diag_ratio * std::fabs(rx_eff))
        rx_eff = 0.0;
      else if (std::fabs(rx_eff) > c_.diag_ratio * std::fabs(ry_eff))
        ry_eff = 0.0;
      // D-pad: same mapping as the approved stepping layout.
      const double dpad_y = shape(c_.axis_dpad_y, c_.sign_dpad_y);
      const double dpad_x = shape(c_.axis_dpad_x, -c_.sign_dpad_x); // reversed in both frames
      v[0] = ly_eff * lin; // X
      v[1] = ry_eff * lin; // Y
      v[2] = lx_eff * lin; // Z
      v[3] = rx_eff * ang; // RX
      v[4] = dpad_x * ang; // RY
      v[5] = dpad_y * ang; // RZ
      return v;
    }
    // Rotate a 6-DoF velocity expressed in the TOOL frame into the BASE frame
    // using the live TCP quaternion from /tmp/move_actual.txt. SpeedL
    // --coordinate=1 is deliberately NOT used: the controller's tool
    // definition is not populated yet (hand-eye calibration pending), so the
    // controller treats TOOL as BASE; the old MoveBlend teleop applied the same
    // host-side transform (move_pose_tool_rot.cpp P33).
    static void angular_tool_to_base(const double q[4], std::array<double, 6> &v)
    {
      double x = q[0], y = q[1], z = q[2], w = q[3];
      const double n = std::sqrt(x * x + y * y + z * z + w * w);
      if (n > 1e-9)
      {
        x /= n;
        y /= n;
        z /= n;
        w /= n;
      }
      const double xx = x * x, yy = y * y, zz = z * z;
      const double xy = x * y, xz = x * z, yz = y * z;
      const double wx = w * x, wy = w * y, wz = w * z;
      const double ang0 = v[3], ang1 = v[4], ang2 = v[5];
      // Keep v[0..2] in BASE; rotate only TOOL angular velocity into BASE.
      v[3] = (1 - 2 * (yy + zz)) * ang0 + 2 * (xy - wz) * ang1 + 2 * (xz + wy) * ang2;
      v[4] = 2 * (xy + wz) * ang0 + (1 - 2 * (xx + zz)) * ang1 + 2 * (yz - wx) * ang2;
      v[5] = 2 * (xz - wy) * ang0 + 2 * (yz + wx) * ang1 + (1 - 2 * (xx + yy)) * ang2;
    }

    // Dedicated 10 Hz sender: the only thread that talks SpeedL IPC. It never
    // blocks on the joystick loop and never queues old velocities.
    void speed_sender_loop()
    {
      const auto period = std::chrono::milliseconds(1000 / c_.speedl_send_hz);
      while (!speed_sender_exit_.load())
      {
        if (!dry_)
        {
          if (manual_standby_.load())
          {
            std::this_thread::sleep_for(period);
            continue;
          }
          if (!motion_enabled_.load())
          {
            if (speedl_active_.exchange(false))
              speedl_client_.zero(tool_atomic_.load() ? c_.tool_coordinate : c_.base_coordinate);
            std::this_thread::sleep_for(period);
            continue;
          }
          std::array<double, 6> velocity;
          bool deadman;
          {
            std::lock_guard<std::mutex> lock(velocity_mutex_);
            velocity = latest_velocity_;
            deadman = deadman_atomic_.load();
          }
          const int coordinate = tool_atomic_.load() ? c_.tool_coordinate : c_.base_coordinate;
          const bool nonzero = velocity != std::array<double, 6>{};
          const bool active = deadman && nonzero && !paused_.load();
          if (!active)
          {
            if (speedl_active_.exchange(false))
              speedl_client_.zero(coordinate);
            std::this_thread::sleep_for(period);
            continue;
          }
          speedl_active_.store(true);
          if (!speedl_client_.update(velocity, coordinate, true))
          {
            // Gateway unreachable / owner busy: local zero. The gateway side
            // watchdog + short last_count backstop whatever the arm is doing.
            const auto now = std::chrono::steady_clock::now();
            if (now - last_speedl_fail_log_ >= std::chrono::seconds(1))
            {
              std::cout << "[SPEEDL] update failed; local velocity zeroed\n";
              last_speedl_fail_log_ = now;
            }
            std::lock_guard<std::mutex> lock(velocity_mutex_);
            latest_velocity_.fill(0.0);
          }
        }
        std::this_thread::sleep_for(period);
      }
    }
    void event(const js_event &e)
    {
      const bool initial = e.type & JS_EVENT_INIT;
      const uint8_t type = e.type & ~JS_EVENT_INIT;
      if (type == JS_EVENT_BUTTON && e.number < N)
      {
        const bool was = buttons_[e.number];
        const bool now = e.value != 0;
        buttons_[e.number] = now;
        if (!initial && e.number == c_.button_lb && now && !was && c_.deadman_required)
        {
          deadman_since_ = std::chrono::steady_clock::now();
          deadman_timed_out_ = false;
        }
        if (!initial && now && !was)
          button(e.number);
        if (!initial && e.number == c_.button_lb && was && !now && c_.deadman_required)
        {
          stop("LB deadman released");
          deadman_since_.reset();
          deadman_timed_out_ = false;
          deadman_ready_ = true;
        }
        return;
      }
      if (type == JS_EVENT_AXIS && e.number < N)
      {
        axes_[e.number] = e.value;
        if (initial && e.number == c_.axis_lt)
          lt_prev_ = deflection(c_.axis_lt, -1.0) > c_.trigger_threshold;
      }
    }
    void button(int b)
    {
      if (b == c_.button_view)
      {
        if (!c_.view_home_enabled)
        {
          std::cout << "[SAFE] View home disabled (view_home_enabled=false): MoveAbsJ return not sent\n";
          return;
        }
        if (!c_.home_joints_parsed.empty())
        {
          if (blocked())
          {
            std::cout << "[DRY] go home ignored: motion disabled\n";
            return;
          }
          std::cout << "[SAFE] returning to bottle-grasp initial point (MoveAbsJ p_init)\n";
          stop("before View home");
          speedl_client_.zero(tool_ ? c_.tool_coordinate : c_.base_coordinate);
          rpc_.home(c_.home_joints, c_.safe_home_speed);
          return;
        }
        if (!c_.home_pose_parsed.empty())
        {
          if (blocked())
          {
            std::cout << "[DRY] go home ignored: motion disabled\n";
            return;
          }
          stop("before View home");
          speedl_client_.zero(tool_ ? c_.tool_coordinate : c_.base_coordinate);
          rpc_.go_home();
          return;
        }
        std::cout << "[INFO] joint mode unsupported: JogJoint/JogCartesian return rc=1 on this controller; cartesian MoveBlend only\n";
        return;
      }
      else if (b == c_.button_x)
      {
        tool_ = !tool_;
        tool_atomic_.store(tool_);
        status("frame switched");
      }
      else if (b == c_.button_y)
      {
        mode_speed_ = (mode_speed_ + 1) % (c_.fast_enabled ? 3 : 2);
        status("speed switched");
      }
      else if (b == c_.button_lb && c_.lb_toggle_enable)
      {
        const bool enabled = !motion_enabled_.load();
        if (!enabled)
        {
          motion_enabled_.store(false);
          stop("before LB hardware disable");
          if (speedl_active_.exchange(false))
            speedl_client_.zero(tool_ ? c_.tool_coordinate : c_.base_coordinate);
          std::this_thread::sleep_for(std::chrono::milliseconds(1200));
        }
        const bool ok = rpc_.set_arm_enabled(enabled);
        if (enabled && ok)
          motion_enabled_.store(enabled);
        std::cout << "[ENABLE] arm " << (enabled ? "ENABLE" : "DISABLE")
                  << (ok ? " OK" : " FAILED") << "\n";
      }
      else if (b == c_.button_lb && !c_.deadman_required && !c_.lb_toggle_enable)
      {
        toggle_manual_standby();
      }
      else if (c_.button_pause >= 0 && b == c_.button_pause)
      {
        const bool paused = !paused_.load();
        paused_.store(paused);
        std::cout << "[PAUSE] motion " << (paused ? "PAUSED" : "RESUMED") << "\n";
        if (paused)
          stop("pause button toggled on");
      }
      else if (b == c_.button_a)
        grip(c_.gripper_close_state, "GRAB/CLOSE");
      else if (b == c_.button_b)
        grip(c_.gripper_open_state, "RELEASE/OPEN");
      else if (c_.button_capture >= 0 && b == c_.button_capture)
      {
        // Hand-eye calibration capture (RB): fire-and-forget ROS2 service
        // call so the 1-3 s CLI startup never stalls the joystick loop.
        std::cout << "[CAPTURE] requesting hand-eye sample (keep arm still)...\n";
        std::thread([]
                    {
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          const int rc = std::system(
              "bash -lc 'source /opt/ros/humble/setup.bash && "
              "timeout 12 ros2 service call /right_handeye/capture "
              "std_srvs/srv/Trigger \"{}\" > /tmp/xbox_capture.log 2>&1'");
          std::cout << "[CAPTURE] rc=" << rc
                    << " (result in /tmp/xbox_capture.log)\n"; })
            .detach();
      }
    }
    bool blocked() const { return dry_ || !c_.allow_motion || rpc_.faulted(); }
    void toggle_manual_standby()
    {
      const bool suspend = !manual_standby_.load();
      if (suspend)
      {
        stop("LB standby");
        speedl_active_.store(false);
        manual_standby_.store(true);
        std::string reply;
        const bool ok = speedl_client_.standby(&reply);
        motion_enabled_.store(false);
        std::cout << "[STANDBY] Xbox manual channel "
                  << (ok ? "STOPPED; ArmGateway owner released and controller socket closed"
                         : "STOP request failed")
                  << " reply=" << trim(reply) << "\n";
        return;
      }

      std::string reply;
      const bool ok = speedl_client_.resume_gateway(&reply);
      manual_standby_.store(false);
      motion_enabled_.store(true);
      rpc_.ensure_connected();
      std::cout << "[STANDBY] Xbox manual channel "
                << (ok ? "RESUMING; gateway reconnect requested" : "RESUME request failed")
                << " reply=" << trim(reply) << "\n";
    }
    void grip(int state, const char *what)
    {
      if (!c_.gripper_enabled)
      {
        std::cout << "[DRY] " << what << " ignored: gripper_enabled=false\n";
        return;
      }
      std::string action = "pre";
      if (state == c_.gripper_close_state) action = "close";
      else if (state == c_.gripper_open_state) action = "open";
      const std::string cmd = "python3 " + c_.gripper_command + " --action " + action +
                              " >/tmp/xbox_gripper.log 2>&1";
      std::thread([cmd, what] {
        const int rc = std::system(cmd.c_str());
        std::cout << "[GRIPPER] " << what << " rc=" << rc << "\n";
      }).detach();
    }
    void tick()
    {
      const auto now = std::chrono::steady_clock::now();
      // Background reconnect: when the controller backend comes back (P36),
      // re-run connect+init here so the teleop self-heals without input.
      if (!manual_standby_.load() && rpc_.faulted())
        rpc_.ensure_connected();
      // LT can be mapped as an optional analog pause trigger, but production
      // uses a button pause because some receivers report noisy trigger edges.
      const bool lt_down = c_.axis_lt >= 0 && deflection(c_.axis_lt, -1.0) > c_.trigger_threshold;
      if (lt_down && !lt_prev_)
      {
        const bool paused = !paused_.load();
        paused_.store(paused);
        std::cout << "[PAUSE] motion " << (paused ? "PAUSED" : "RESUMED") << "\n";
        if (paused)
          stop("LT pause toggled on");
      }
      lt_prev_ = lt_down;
      home(now);
      if (c_.deadman_required)
      {
        if (!pressed(c_.button_lb))
        {
          deadman_ready_ = true;
          deadman_timed_out_ = false;
          deadman_since_.reset();
        }
        if (pressed(c_.button_lb) && !deadman_since_)
          deadman_since_ = now;
        if (c_.deadman_max_hold_ms > 0 && pressed(c_.button_lb) && deadman_ready_ && !deadman_timed_out_ &&
            std::chrono::duration_cast<std::chrono::milliseconds>(now - *deadman_since_).count() >= c_.deadman_max_hold_ms)
        {
          deadman_timed_out_ = true;
          std::cout << "[SAFE] LB maximum hold time reached; release and press LB again to continue\n";
          stop("LB deadman maximum hold time");
        }
        if (pressed(c_.button_lb) && !deadman_ready_ && !deadman_wait_logged_)
        {
          std::cout << "[SAFE] release LB once after startup before motion is accepted\n";
          deadman_wait_logged_ = true;
        }
      }
      const bool deadman_valid = c_.deadman_required ? (pressed(c_.button_lb) && deadman_ready_ && !deadman_timed_out_) : true;
      // Menu blocks motion (it is the Safe Home chord); LB release or a
      // centred stick both simply zero the velocity target.
      const bool deadman = deadman_valid && !paused_.load();
      deadman_atomic_.store(deadman);
      const auto raw_v = desired_velocity();
      auto v = raw_v;
      if (!deadman || !motion_enabled_.load() || paused_.load() || manual_standby_.load())
        v.fill(0.0);
      {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        latest_velocity_ = v;
      }
    }
    static std::string format_velocity(const std::array<double, 6> &v)
    {
      std::ostringstream out;
      out.setf(std::ios::fixed);
      out.precision(4);
      out << "{";
      for (size_t i = 0; i < v.size(); ++i)
      {
        if (i)
          out << ",";
        out << v[i];
      }
      out << "}";
      return out.str();
    }
    // Safe Home: LB + Menu held for safe_home_hold_seconds, then MoveAbsJ.
    void home(std::chrono::steady_clock::time_point now)
    {
      const bool dm_ok = c_.deadman_required ? (deadman_ready_ && !deadman_timed_out_) : true;
      const bool requested = pressed(c_.button_lb) && pressed(c_.button_menu) && dm_ok;
      if (!requested)
      {
        home_start_.reset();
        home_done_ = false;
        return;
      }
      if (!home_start_)
      {
        home_start_ = now;
        std::cout << "[SAFE] safe home requires LB + Menu held for " << c_.safe_home_hold_seconds << " s\n";
        return;
      }
      if (home_done_ || std::chrono::duration<double>(now - *home_start_).count() < c_.safe_home_hold_seconds)
        return;
      home_done_ = true;
      if (!c_.safe_home_enabled)
      {
        std::cout << "[SAFE] safe home ignored: safe_home_enabled=false\n";
        return;
      }
      if (blocked())
      {
        std::cout << "[DRY] safe home ignored: motion disabled\n";
        return;
      }
      // End SpeedL first: zero the target and push one final zero frame so the
      // gateway drains the async pipeline before the MoveAbsJ below.
      stop("before safe home");
      speedl_client_.zero(tool_ ? c_.tool_coordinate : c_.base_coordinate);
      rpc_.home(c_.safe_home_joints_rad, c_.safe_home_speed);
    }
    // Stop the velocity target immediately. The sender thread pushes the zero
    // to the gateway; no RPC {Stop} is ever sent (P37).
    void stop(const std::string &reason)
    {
      {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        latest_velocity_.fill(0.0);
      }
      deadman_atomic_.store(false);
      std::cout << "[SAFE] stop: " << reason << '\n';
    }
    void banner() const
    {
      std::cout << "\nXbox arm teleop (SpeedL continuous velocity)\n  frame=BASE speed=FINE "
                << "linear=" << c_.fine_linear_mps * 1000.0 << "mm/s\n  "
                << (c_.lb_toggle_enable
                        ? "LB: toggle enable (press to enable/disable motion)"
                    : c_.deadman_required ? "LB: deadman (hold to move; release stops)"
                                          : "LB: standby/resume ArmGateway, release stick stops")
                << "\n  X: BASE/TOOL (all XYZ/RXYZ), pause disabled\n"
                << "  Y: speed, A: grab, B: release\n"
                << "  speedl send: " << c_.speedl_send_hz << " Hz\n"
                << "  motion_enabled=" << (motion_enabled_.load() ? "YES" : "NO")
                << "  robot=" << (blocked() ? "DRY-RUN (no robot RPC)" : "ARMED") << "\n\n";
    }
    void status(const std::string &why) const
    {
      static const std::array<const char *, 3> s{"FINE", "NORMAL", "FAST"};
      std::cout << "[STATE] " << why << ": FRAME=" << (tool_ ? "TOOL" : "BASE") << " SPEED=" << s[mode_speed_]
                << " LINEAR=" << (selected_linear_mps() * 1000.0) << "mm/s\n";
    }
    Config c_;
    bool dry_;
    Controller rpc_;
    codex_speedl::SpeedLIpcClient speedl_client_;
    std::mutex velocity_mutex_;
    std::array<double, 6> latest_velocity_{};
    std::atomic<bool> deadman_atomic_{false};
    std::atomic<bool> motion_enabled_{false};
    std::atomic<bool> tool_atomic_{false};
    std::atomic<bool> paused_{false};
    std::atomic<bool> speed_sender_exit_{false};
    std::atomic<bool> speedl_active_{false};
    std::atomic<bool> manual_standby_{false};
    std::thread speed_sender_;
    std::array<uint8_t, N> buttons_{};
    std::array<int16_t, N> axes_{};
    bool tool_{false}, home_done_{false}, deadman_ready_{false}, deadman_timed_out_{false}, deadman_wait_logged_{false};
    bool last_velocity_nonzero_{false};
    bool lt_prev_{false}; // LT axis rising edge -> pre-grasp
    int mode_speed_{0};
    std::optional<std::chrono::steady_clock::time_point> home_start_, deadman_since_;
    std::chrono::steady_clock::time_point last_velocity_log_{};
    std::chrono::steady_clock::time_point last_axis_log_{};
    std::chrono::steady_clock::time_point last_speedl_fail_log_{};
  };

  void usage(const char *p) { std::cout << "Usage: " << p << " [--config path] [--dry-run] [--print-mapping]\n"; }
  void mapping()
  {
    std::cout << "LB: deadman enable (hold to move, release to stop); X: pause/resume\n"
              << "Left stick X/Y; right stick Y=Z; right stick X=Roll; D-pad Y=Yaw; D-pad X=Pitch; LT/RT=Roll(Tool)\n"
              << "A=grab/close, B=release/open, LT=pre-grasp; Y=speed; BASE position + TOOL rotation\n";
  }
} // namespace

int main(int argc, char **argv)
{
  std::string config = "config/xbox_mapping.yaml";
  bool dry = false;
  for (int i = 1; i < argc; ++i)
  {
    const std::string a = argv[i];
    if (a == "--config" && i + 1 < argc)
      config = argv[++i];
    else if (a == "--dry-run")
      dry = true;
    else if (a == "--print-mapping")
    {
      mapping();
      return 0;
    }
    else if (a == "--help" || a == "-h")
    {
      usage(argv[0]);
      return 0;
    }
    else
    {
      usage(argv[0]);
      return 64;
    }
  }
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  // The controller RPC backend resets connections while it is still
  // restarting (P36). Writing to that socket raises SIGPIPE, whose default
  // action silently kills the process (observed 2026-08-12: process vanished
  // with no log after "errno:104" from the SDK recv thread). Ignore it so the
  // send() error paths handle the dead connection instead.
  std::signal(SIGPIPE, SIG_IGN);
  try
  {
    return Teleop(Config::from_file(config), dry).run();
  }
  catch (const std::exception &e)
  {
    std::cerr << "FATAL: " << e.what() << '\n';
    return 1;
  }
}
