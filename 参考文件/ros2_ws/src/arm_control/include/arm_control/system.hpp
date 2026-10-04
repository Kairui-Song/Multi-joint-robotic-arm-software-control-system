#pragma once
#include "arm_control/bus.hpp"
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <array>
#include <atomic>
#include <thread>

namespace arm_control {
class ArmSystem final : public hardware_interface::SystemInterface {
public:
  using CallbackReturn = hardware_interface::CallbackReturn;
  ~ArmSystem() override;
  CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
private:
  enum Fault { NONE, BUS, DRIVE, LIMIT, WATCHDOG, COMMAND };
  void stop() noexcept;
  hardware_interface::return_type fail(Fault);
  bool feedback_valid(bool require_enabled) const;
  void publish_diagnostics();
  std::vector<Axis> axes_;
  BusConfig config_;
  std::unique_ptr<Bus> bus_;
  std::vector<Feedback> feedback_;
  std::vector<double> position_, velocity_, command_, sent_;
  struct AxisHealth {
    std::atomic<double> position{0}, velocity{0};
    std::atomic<unsigned> status{0}, error{0};
    std::atomic<int> mode{0};
  };
  std::unique_ptr<AxisHealth[]> axis_health_;
  std::atomic<bool> active_{false}, configured_{false}, inject_{false};
  std::atomic<int> fault_{NONE};
  std::atomic<uint64_t> cycles_{0}, overruns_{0}, clamps_{0};
  std::atomic<double> period_us_{0}, jitter_max_us_{0}, read_us_{0}, write_us_{0};
  std::atomic<int64_t> last_cycle_ns_{0};
  std::array<std::array<double, 5>, 4096> timing_ring_{};
  std::atomic<size_t> timing_head_{0}, timing_tail_{0};
  std::atomic<uint64_t> timing_dropped_{0};
  std::chrono::steady_clock::time_point previous_{};
  double watchdog_s_{0.02}, activation_s_{5.0};
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread thread_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr timing_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr injection_service_;
};
} // namespace arm_control
