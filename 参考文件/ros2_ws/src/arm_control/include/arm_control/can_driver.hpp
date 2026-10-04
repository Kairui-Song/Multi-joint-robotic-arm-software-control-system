#pragma once
#include "arm_control/bus.hpp"
#include "arm_control/can_protocol.hpp"
#include <chrono>

namespace arm_control::can {
inline uint64_t monotonic_ns() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
}
class Driver {
public:
  virtual ~Driver() = default;
  virtual void open(const CanConfig &, const std::vector<Axis> &) = 0;
  virtual bool transmit(const Frame * frames, size_t count) = 0;
  // Nonblocking. Negative means driver/bus error or RX overrun. Must not report own TX echoes.
  virtual int receive(Frame * frames, size_t capacity) = 0;
  virtual void close() noexcept = 0;
  virtual void inject_fault(bool) {}
};
std::unique_ptr<Driver> make_driver(const std::string & backend);
} // namespace arm_control::can
