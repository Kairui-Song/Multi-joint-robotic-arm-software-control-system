#include "arm_control/bus.hpp"
#include "arm_control/can_bus.hpp"
#include "arm_control/mit_bus.hpp"
#include "arm_control/robomaster.hpp"

namespace arm_control {
ActivationState Bus::activation_step(const std::vector<Feedback> & feedback, bool valid, bool reset) {
  if (*fault_reason()) return ActivationState::Fault;
  bool ready = valid && !feedback.empty(), fault = false;
  std::vector<double> hold(feedback.size(), 0.0);
  for (size_t i = 0; i < feedback.size(); ++i) {
    if (valid) hold[i] = feedback[i].position;
    fault = fault || feedback[i].fault || feedback[i].error;
    ready = ready && feedback[i].ready && !feedback[i].fault && !feedback[i].error;
  }
  if (ready) return ActivationState::Ready;
  if (!send(hold, valid && !fault, valid && reset)) return ActivationState::Fault;
  return ActivationState::Waiting;
}
class SimBus final : public Bus {
  std::vector<Axis> axes_;
  std::vector<double> q_, target_;
  bool active_{false}, fault_{false};
public:
  void open(const std::vector<Axis> & axes, const BusConfig &) override {
    axes_ = axes; q_.resize(axes.size()); target_.resize(axes.size());
    for (size_t i = 0; i < axes.size(); ++i) q_[i] = target_[i] = std::clamp(0.0, axes[i].lower, axes[i].upper);
    active_ = false; fault_ = false;
  }
  bool receive(std::vector<Feedback> & out, double dt) override {
    if (out.size() != q_.size() || dt <= 0) return false;
    for (size_t i = 0; i < q_.size(); ++i) {
      const double old = q_[i];
      if (active_ && !fault_) q_[i] = limit(target_[i], q_[i], dt, axes_[i]);
      out[i] = {q_[i], (q_[i] - old) / dt,
        static_cast<uint16_t>(fault_ ? 0x0008 : active_ ? 0x0027 : 0x0040),
        static_cast<uint16_t>(fault_ ? 0x2310 : 0), 8, active_ && !fault_, fault_};
    }
    return true;
  }
  bool send(const std::vector<double> & q, bool enable, bool reset) override {
    if (q.size() != q_.size()) return false;
    if (reset) fault_ = false;
    target_ = q; active_ = enable && !fault_;
    return true;
  }
  void inject_fault(bool value) override { fault_ = value; }
  void close() noexcept override { active_ = false; }
};
#ifdef ARM_WITH_IGH
std::unique_ptr<Bus> make_igh_bus();
#endif
#ifdef ARM_WITH_LOONG
std::unique_ptr<Bus> make_loong_bus();
#endif
std::unique_ptr<Bus> make_bus(const std::string & backend) {
  BusConfig config; config.backend = backend;
  if (backend == "rm_mock" || backend == "rm_socketcan" || backend == "rm_zcan")
    config.can.protocol = "robomaster_c620";
  return make_bus(config);
}
std::unique_ptr<Bus> make_bus(const BusConfig & config) {
  const auto & backend = config.backend;
  if (backend == "sim") return std::make_unique<SimBus>();
  if (backend == "rm_mock" || backend == "rm_socketcan" || backend == "rm_zcan") {
    if (config.can.protocol != "robomaster_c620") throw std::invalid_argument("RM backend requires robomaster_c620");
    return rm::make_bus(backend);
  }
  if (backend == "socketcan" || backend == "zcan" || backend == "can_mock") {
    if (config.can.protocol == "mit_shared_v1") return std::make_unique<MitBus>(can::make_driver(backend));
    if (config.can.protocol != "rv_packed_v1") throw std::invalid_argument("CAN backend requires rv_packed_v1 or mit_shared_v1");
    return std::make_unique<CanBus>(can::make_driver(backend));
  }
#ifdef ARM_WITH_LOONG
  if (backend == "loong") return make_loong_bus();
#endif
#ifdef ARM_WITH_IGH
  if (backend == "igh") return make_igh_bus();
#endif
  throw std::invalid_argument("Unsupported backend or IgH not built: " + backend);
}
} // namespace arm_control
