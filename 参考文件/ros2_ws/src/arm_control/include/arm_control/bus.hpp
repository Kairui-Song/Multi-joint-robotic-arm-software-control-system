#pragma once
#include "arm_control/core.hpp"
#include <memory>
#include <vector>

namespace arm_control {
struct CanConfig {
  std::string protocol{"rv_packed_v1"}, interface{"can0"};
  unsigned bitrate{1000000};
  double feedback_timeout_s{0.05};
  bool protocol_confirmed{false}, stop_confirmed{false}, motor_watchdog_confirmed{false};
  unsigned device_type{}, device_index{}, channel{}, timing0{}, timing1{};
  std::string bitrate_property; // Optional SDK property, e.g. "0/baud_rate", per adapter manual.
  bool fd_adapter{false};
  unsigned data_bitrate{5000000}, channel_count{1}, feedback_hz{1000};
  std::string second_interface{"can1"};
  bool startup_reference_confirmed{false}, group_exclusive_confirmed{false};
  double command_timeout_s{0.05};
};
struct BusConfig {
  std::string backend{"sim"};
  unsigned master{}, cycle_ns{1000000}, watchdog_divider{2498}, watchdog_intervals{100};
  bool commissioned{false}, reset_faults{false};
  uint16_t dc_assign{};
  CanConfig can;
  std::string sdk_xml;
  double sdk_command_timeout_s{0.05};
  unsigned sdk_cpu{};
};
enum class ActivationState { Waiting, Ready, Fault };
// Also used when HardwareInfo is supplied directly, bypassing the YAML launcher.
template<class Parameters>
void validate_control_parameters(const std::string & protocol, const Parameters & parameters) {
  const auto reject = [&](const char * key) {
    if (parameters.count(key)) throw std::invalid_argument("Parameter " + std::string(key) +
      " does not belong to " + protocol);
  };
  if (protocol == "robomaster_c620") {
    for (const char * key : {"kp", "kd", "master_id", "p_max", "v_max", "t_max", "motor_zero_rad"}) reject(key);
  } else if (protocol == "mit_shared_v1" || protocol == "rv_packed_v1") {
    for (const char * key : {"position_kp", "velocity_kp", "velocity_ki", "current_limit_a", "startup_position", "can_channel"}) reject(key);
    if (protocol == "rv_packed_v1")
      for (const char * key : {"master_id", "p_max", "v_max", "t_max"}) reject(key);
  }
}
class Bus {
public:
  virtual ~Bus() = default;
  virtual void open(const std::vector<Axis> &, const BusConfig &) = 0;
  virtual bool receive(std::vector<Feedback> &, double dt) = 0;
  virtual bool prepare_activate() { return true; }
  // valid means all feedback is fresh and passes the hardware layer's position/speed limits.
  // Called only during lifecycle activation, never in the real-time write loop.
  virtual ActivationState activation_step(const std::vector<Feedback> &, bool valid, bool reset);
  virtual const char * control_mode() const noexcept { return "backend position control"; }
  virtual bool send(const std::vector<double> &, bool enable, bool reset = false) = 0;
  virtual void close() noexcept = 0;
  virtual void inject_fault(bool) {}
  virtual const char * fault_reason() const noexcept { return ""; }
};
std::unique_ptr<Bus> make_bus(const std::string & backend);
std::unique_ptr<Bus> make_bus(const BusConfig & config);
} // namespace arm_control
