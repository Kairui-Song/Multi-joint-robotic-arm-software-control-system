#pragma once
#include <array>
#include <cstdint>
#include <cmath>
#include <stdexcept>

namespace arm_control::can {
struct Frame {
  uint32_t id{};
  uint8_t size{};
  bool extended{}, remote{}, error{};
  std::array<uint8_t, 8> data{};
};
// Wire format ported from this repository's send_motor_ctrl_cmd and ACK type 1.
// This is NOT the RoboMaster C620 protocol or a generic MIT protocol identifier.
struct Sample { double position{}, velocity{}, current{}, temperature{}; uint8_t error{}, ack{}; };
inline uint32_t pack(double v, double lo, double hi, unsigned bits) {
  if (!std::isfinite(v) || v < lo || v > hi) throw std::invalid_argument("RV field outside protocol range");
  return static_cast<uint32_t>((v - lo) * ((1u << bits) - 1u) / (hi - lo));
}
inline double unpack(uint32_t v, double lo, double hi, unsigned bits) {
  return lo + v * (hi - lo) / ((1u << bits) - 1u);
}
inline void check_id(uint16_t id) {
  if (id == 0 || id >= 0x7ff) throw std::invalid_argument("Motor ID must be 1..2046");
}
inline Frame position_command(uint16_t id, double kp, double kd, double position) {
  check_id(id);
  const auto p = pack(position, -12.5, 12.5, 16);
  const auto k = pack(kp, 0, 500, 12), d = pack(kd, 0, 5, 9);
  const auto v = pack(0, -18, 18, 12), t = pack(0, -30, 30, 12);
  Frame f; f.id = id; f.size = 8;
  f.data = {static_cast<uint8_t>(k >> 7), static_cast<uint8_t>((k << 1) | (d >> 8)),
    static_cast<uint8_t>(d), static_cast<uint8_t>(p >> 8), static_cast<uint8_t>(p),
    static_cast<uint8_t>(v >> 4), static_cast<uint8_t>((v << 4) | (t >> 8)), static_cast<uint8_t>(t)};
  return f;
}
inline Frame zero_current(uint16_t id) {
  check_id(id); Frame f; f.id = id; f.size = 3;
  f.data[0] = 0x61; // set_motor_cur_tor(id, 0, ctrl_status=0, ack_status=1)
  return f;
}
inline bool decode(const Frame & f, Sample & s) noexcept {
  if (f.extended || f.remote || f.error || f.id == 0 || f.id >= 0x7ff ||
      (f.size != 7 && f.size != 8) || (f.data[0] >> 5) != 1) return false;
  s.position = unpack((uint32_t(f.data[1]) << 8) | f.data[2], -12.5, 12.5, 16);
  s.velocity = unpack((uint32_t(f.data[3]) << 4) | (f.data[4] >> 4), -18, 18, 12);
  s.current = unpack((uint32_t(f.data[4] & 15) << 8) | f.data[5], -30, 30, 12);
  s.temperature = (static_cast<int>(f.data[6]) - 50) / 2.0;
  s.error = f.data[0] & 31; s.ack = 1;
  return true;
}
} // namespace arm_control::can
