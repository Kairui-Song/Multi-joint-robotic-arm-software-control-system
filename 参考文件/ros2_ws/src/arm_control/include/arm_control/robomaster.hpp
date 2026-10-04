#pragma once
#include "arm_control/can_driver.hpp"

namespace arm_control::rm {
constexpr double pi = 3.14159265358979323846;
struct Sample { uint16_t encoder{}; int16_t rpm{}, current{}; uint8_t temperature{}; };
inline int16_t signed16(uint8_t hi, uint8_t lo) {
  const int v = (int(hi) << 8) | lo;
  return static_cast<int16_t>(v >= 32768 ? v - 65536 : v);
}
inline bool decode(const can::Frame & f, Sample & s) {
  if (f.extended || f.remote || f.error || f.id < 0x201 || f.id > 0x208 || f.size != 8) return false;
  s.encoder = static_cast<uint16_t>((uint16_t(f.data[0]) << 8) | f.data[1]);
  if (s.encoder >= 8192) return false;
  s.rpm = signed16(f.data[2],f.data[3]); s.current = signed16(f.data[4],f.data[5]); s.temperature = f.data[6];
  return true;
}
inline can::Frame group(uint32_t id, const std::array<int16_t,4> & currents) {
  can::Frame f; f.id=id; f.size=8;
  for (size_t i=0;i<4;++i) {
    const auto value = static_cast<uint16_t>(std::clamp<int>(currents[i],-16384,16384));
    f.data[2*i]=static_cast<uint8_t>(value>>8); f.data[2*i+1]=static_cast<uint8_t>(value);
  }
  return f;
}
class Encoder {
  int previous_{}; int64_t accumulated_{}; bool seen_{false};
public:
  void reset() { seen_=false; accumulated_=0; }
  double update(uint16_t raw, const Axis & a) {
    if (!seen_) { previous_=raw; seen_=true; }
    int delta=int(raw)-previous_; if(delta>4096) delta-=8192; if(delta<-4096) delta+=8192;
    accumulated_+=delta; previous_=raw;
    return a.rm.startup_position + a.direction * accumulated_ * (2*pi)/(8192*a.can.gear_ratio);
  }
};
class CascadeController {
  double integral_{0};
public:
  void reset() { integral_=0; }
  double current(double target, double position, double velocity, double dt, const Axis & a) {
    const double desired_velocity=std::clamp(a.rm.position_kp*(target-position),-a.velocity,a.velocity);
    const double e=desired_velocity-velocity;
    const double candidate=std::clamp(integral_+a.rm.velocity_ki*e*dt,-a.rm.current_limit,a.rm.current_limit);
    const double u=a.rm.velocity_kp*e+candidate;
    if (std::abs(u)<=a.rm.current_limit || u*e<0) integral_=candidate;
    return std::clamp(a.rm.velocity_kp*e+integral_,-a.rm.current_limit,a.rm.current_limit);
  }
};
std::unique_ptr<Bus> make_bus(const std::string & backend);
} // namespace arm_control::rm
