#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace arm_control {
struct CanAxis {
  uint16_t motor_id{};
  double gear_ratio{1}, zero_rad{0}, kp{20}, kd{0.5};
  uint16_t master_id{};
  double p_max{12.5}, v_max{20}, t_max{10}, max_temperature{80};
};
struct RoboMasterAxis {
  unsigned channel{};
  double startup_position{}, position_kp{5}, velocity_kp{3}, velocity_ki{1}, current_limit{3};
  double max_temperature{80};
};
struct Axis {
  std::string name;
  double lower{}, upper{}, velocity{}, counts_per_rad{}, zero{}, direction{1};
  uint16_t alias{}, position{}, rx_pdo{0x1600}, tx_pdo{0x1a00};
  uint32_t vendor{}, product{};
  CanAxis can;
  unsigned sdk_index{};
  RoboMasterAxis rm;
};
inline void validate(const Axis & a) {
  if (a.name.empty() || !std::isfinite(a.lower) || !std::isfinite(a.upper) ||
      !std::isfinite(a.velocity) || !std::isfinite(a.counts_per_rad) ||
      !std::isfinite(a.zero) || a.lower >= a.upper || a.velocity <= 0 ||
      a.counts_per_rad <= 0 || (a.direction != 1 && a.direction != -1))
    throw std::invalid_argument("Invalid joint configuration: " + a.name);
  for (double q : {a.lower, a.upper}) {
    const double raw = a.zero + a.direction * q * a.counts_per_rad;
    if (raw < std::numeric_limits<int32_t>::min() || raw > std::numeric_limits<int32_t>::max())
      throw std::invalid_argument("Encoder range overflow: " + a.name);
  }
}
inline double limit(double requested, double previous, double dt, const Axis & a) {
  if (!std::isfinite(requested) || !std::isfinite(previous) || !std::isfinite(dt) || dt <= 0)
    throw std::invalid_argument("Non-finite command or invalid control period");
  const double bounded = std::clamp(requested, a.lower, a.upper);
  return std::clamp(bounded, previous - a.velocity * dt, previous + a.velocity * dt);
}
// Match the C620 feedback guard: allow tracking transients up to 150%.
inline bool feedback_speed_valid(double velocity, const Axis & axis) {
  return std::isfinite(velocity) && std::abs(velocity) <= 1.5 * axis.velocity;
}
inline bool enabled(uint16_t status) { return (status & 0x006f) == 0x0027; }
inline uint16_t enable_word(uint16_t status) {
  if (status & 0x0008) return 0; // Fault reset must be explicitly requested.
  switch (status & 0x006f) {
    case 0x0040: return 0x0006;
    case 0x0021: return 0x0007;
    case 0x0023: case 0x0027: return 0x000f;
    default: return 0;
  }
}
struct Feedback {
  double position{}, velocity{}; uint16_t status{}, error{}; int8_t mode{};
  bool ready{}, fault{}; // Transport-specific readiness; never infer CiA402 state for CAN.
};
} // namespace arm_control
