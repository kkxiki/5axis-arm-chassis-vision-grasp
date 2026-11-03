#pragma once

// Ported unchanged from codex/speedl_xbox_teleop/xbox/velocity_mapper.hpp
// (2026-08-12 SpeedL port). shape_axis() provides the deadzone response curve;
// the current project's axis/sign/diag-ratio mapping is applied in
// xbox_arm_teleop.cpp (desired_velocity) on top of shape_axis().

#include <algorithm>
#include <array>
#include <cmath>

namespace codex_speedl {

struct VelocityLimits {
  double linear_mps{0.020};
  double angular_radps{0.139626}; // 8 deg/s.
  double deadzone{0.10};
};

inline double shape_axis(double value, double deadzone) {
  value = std::clamp(value, -1.0, 1.0);
  if (std::abs(value) <= deadzone) return 0.0;
  const double normalized = (std::abs(value) - deadzone) / (1.0 - deadzone);
  return std::copysign(normalized, value);
}

// Matches the current project's customized mapping:
// left Y=X, left X=Z, right Y=Y, right X=Roll, D-pad X=Pitch, D-pad Y=Yaw.
inline std::array<double, 6> map_velocity(double left_x,
                                          double left_y,
                                          double right_x,
                                          double right_y,
                                          double dpad_x,
                                          double dpad_y,
                                          const VelocityLimits& limit) {
  std::array<double, 6> velocity{};
  velocity[0] = shape_axis(left_y, limit.deadzone) * limit.linear_mps;
  velocity[1] = shape_axis(right_y, limit.deadzone) * limit.linear_mps;
  velocity[2] = shape_axis(left_x, limit.deadzone) * limit.linear_mps;
  velocity[3] = shape_axis(right_x, limit.deadzone) * limit.angular_radps;
  velocity[4] = shape_axis(dpad_x, limit.deadzone) * limit.angular_radps;
  velocity[5] = shape_axis(dpad_y, limit.deadzone) * limit.angular_radps;
  return velocity;
}

}  // namespace codex_speedl
