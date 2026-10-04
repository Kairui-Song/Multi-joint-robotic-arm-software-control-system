#pragma once
#include "arm_control/can_driver.hpp"
#include <atomic>
#include <functional>

namespace arm_control {
class CanBus final : public Bus {
public:
  explicit CanBus(std::unique_ptr<can::Driver> driver,
    std::function<uint64_t()> clock = can::monotonic_ns);
  ~CanBus() override { close(); }
  void open(const std::vector<Axis> &, const BusConfig &) override;
  bool receive(std::vector<Feedback> &, double dt) override;
  const char * control_mode() const noexcept override { return "motor RV position/Kp/Kd"; }
  bool send(const std::vector<double> &, bool enable, bool reset = false) override;
  void close() noexcept override;
  void inject_fault(bool v) override { driver_->inject_fault(v); }
  const char * fault_reason() const noexcept override;
private:
  enum Error { NONE, DRIVER, STALE, MALFORMED, MOTOR, COMMAND, OVERSPEED };
  bool fail(Error error);
  std::unique_ptr<can::Driver> driver_;
  std::function<uint64_t()> clock_;
  std::vector<Axis> axes_;
  std::vector<Feedback> feedback_;
  std::vector<uint64_t> received_at_;
  std::vector<bool> seen_, armed_feedback_;
  std::vector<can::Frame> tx_, stop_;
  std::array<can::Frame, 256> rx_{};
  std::array<int, 2048> index_{};
  uint64_t timeout_ns_{};
  bool open_{false}, armed_{false};
  std::atomic<int> error_{NONE};
};
} // namespace arm_control
