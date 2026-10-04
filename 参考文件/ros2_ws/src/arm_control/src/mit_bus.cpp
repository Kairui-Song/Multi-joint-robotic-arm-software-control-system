#include "arm_control/mit_bus.hpp"
#include <set>
#include "arm_control/mit_protocol.hpp"

namespace arm_control {
MitBus::MitBus(std::unique_ptr<can::Driver> driver, std::function<uint64_t()> clock)
: driver_(std::move(driver)), clock_(std::move(clock)) {
  if (!driver_) throw std::invalid_argument("CAN driver missing");
}
void MitBus::open(const std::vector<Axis> & axes, const BusConfig & cfg) {
  close();
  const bool mock = cfg.backend == "can_mock";
  if (cfg.can.protocol != "mit_shared_v1") throw std::invalid_argument("Unsupported motor wire protocol");
  if (!mock && (!cfg.commissioned || !cfg.can.protocol_confirmed || !cfg.can.stop_confirmed ||
      !cfg.can.motor_watchdog_confirmed)) throw std::invalid_argument("Confirm real CAN protocol, stop behavior and motor watchdog before commissioning");
  if (axes.empty() || axes.size() > 128 || cfg.can.bitrate == 0 || cfg.cycle_ns == 0 ||
      !std::isfinite(cfg.can.feedback_timeout_s) || cfg.can.feedback_timeout_s <= cfg.cycle_ns * 1e-9 ||
      cfg.can.feedback_timeout_s > 5) throw std::invalid_argument("Invalid CAN size/rate/timeout");
  // Conservative full 8-byte command + feedback, including worst-case stuffing/intermission.
  if (axes.size() * 2.0 * 135 * 1e9 / cfg.cycle_ns / cfg.can.bitrate > 0.70)
    throw std::invalid_argument("CAN estimated utilization exceeds 70%; lower control rate or use more buses");
  axes_ = axes; index_.fill(-1);
  std::set<uint16_t> slave_ids;
  for (size_t i = 0; i < axes.size(); ++i) {
    const auto & a = axes[i]; const auto & c = a.can; validate(a); can::check_id(c.motor_id);
    const auto feedback_id = c.master_id;
    if (c.master_id == 0 || c.master_id > 255 ||
        !std::isfinite(c.p_max) || c.p_max <= 0 || !std::isfinite(c.v_max) || c.v_max <= 0 ||
        !std::isfinite(c.t_max) || c.t_max <= 0 || !std::isfinite(c.max_temperature) || c.max_temperature <= 0)
      throw std::invalid_argument("Invalid MIT ranges/master ID/temperature limit");
    if (!slave_ids.insert(c.motor_id).second || index_[feedback_id] != -1) throw std::invalid_argument("Duplicate CAN motor ID");
    if (!std::isfinite(c.gear_ratio) || c.gear_ratio <= 0 || !std::isfinite(c.zero_rad) ||
        !std::isfinite(c.kp) || c.kp <= 0 || c.kp > 500 || !std::isfinite(c.kd) || c.kd < 0 || c.kd > 5 ||
        a.velocity * c.gear_ratio > c.v_max) throw std::invalid_argument("Invalid CAN ratio/gains/velocity");
    for (double q : {a.lower, a.upper})
      if (std::abs(c.zero_rad + a.direction * c.gear_ratio * q) > c.p_max)
        throw std::invalid_argument("Joint range exceeds motor protocol position range");
    index_[feedback_id] = static_cast<int>(i);
  }
  feedback_.assign(axes.size(), {}); received_at_.assign(axes.size(), 0);
  seen_.assign(axes.size(), false); armed_feedback_.assign(axes.size(), false);
  tx_.resize(axes.size()); stop_.resize(axes.size());
  for (size_t i = 0; i < axes.size(); ++i) stop_[i] = mit::mode(axes[i].can.motor_id, false);
  timeout_ns_ = static_cast<uint64_t>(cfg.can.feedback_timeout_s * 1e9);
  error_ = NONE; armed_ = false;
  try { driver_->open(cfg.can, axes); open_ = true; }
  catch (...) { driver_->close(); throw; }
  // Start disabled. No position motion at open.
  if (!driver_->transmit(stop_.data(), stop_.size())) {
    close(); throw std::runtime_error("CAN initial stop/probe failed");
  }
}
bool MitBus::prepare_activate() {
  if (!open_ || error_ != NONE) return false;
  // Discard disabled-state feedback; activation must acquire a new sample per axis.
  if (driver_->receive(rx_.data(), rx_.size()) < 0) return fail(DRIVER);
  armed_ = false;
  std::fill(armed_feedback_.begin(), armed_feedback_.end(), false);
  std::fill(seen_.begin(), seen_.end(), false);
  for (size_t i=0; i<axes_.size(); ++i) {
    // Clear any previous position gains before entering motor mode.
    tx_[i]=mit::command(axes_[i].can, 0, false);
  }
  if (!driver_->transmit(tx_.data(),tx_.size())) return fail(DRIVER);
  for (size_t i=0; i<axes_.size(); ++i) tx_[i]=mit::mode(axes_[i].can.motor_id,true);
  if (!driver_->transmit(tx_.data(),tx_.size())) return fail(DRIVER);
  mit_enabled_ = true;
  return true;
}
ActivationState MitBus::activation_step(const std::vector<Feedback> & feedback, bool valid, bool reset) {
  if (!open_ || error_ != NONE || !mit_enabled_) return ActivationState::Fault;
  // Preserve enable while waiting for fresh feedback after prepare_activate().
  if (!valid) return ActivationState::Waiting;
  return Bus::activation_step(feedback, valid, reset);
}
bool MitBus::fail(Error error) {
  error_ = error; armed_ = false; mit_enabled_ = false;
  if (open_) driver_->transmit(stop_.data(), stop_.size());
  return false;
}
bool MitBus::receive(std::vector<Feedback> & out, double) {
  if (!open_ || out.size() != axes_.size()) return false;
  if (error_ != NONE) return false; // Explicit close/open required after a fault.
  const int count = driver_->receive(rx_.data(), rx_.size());
  if (count < 0 || static_cast<size_t>(count) > rx_.size()) return fail(DRIVER);
  const auto now = clock_();
  for (int k = 0; k < count; ++k) {
    const auto & frame = rx_[k];
    if (frame.error) return fail(DRIVER);
    if (frame.extended || frame.remote) continue;
    if (frame.id != 0 || frame.size == 0) continue;
    const auto key = frame.data[0];
    if (key >= index_.size() || index_[key] < 0) continue;
    const auto i = static_cast<size_t>(index_[key]);
    can::Sample sample;
    if (!mit::decode(frame, axes_[i].can, sample)) return fail(MALFORMED);
    if (sample.temperature >= axes_[i].can.max_temperature) return fail(MOTOR);
    const auto & a = axes_[i]; const double scale = a.direction * a.can.gear_ratio;
    if (!feedback_speed_valid(sample.velocity / scale, a)) return fail(OVERSPEED);
    feedback_[i] = {(sample.position - a.can.zero_rad) / scale, sample.velocity / scale,
      frame.data[0], sample.error, -1, false, sample.error != 0};
    received_at_[i] = now; seen_[i] = true;
    if (armed_) armed_feedback_[i] = true;
    if (sample.error) { out = feedback_; return fail(MOTOR); }
  }
  bool all_seen = true;
  for (size_t i = 0; i < axes_.size(); ++i) {
    if (!seen_[i]) { all_seen = false; continue; }
    if (now < received_at_[i] || now - received_at_[i] > timeout_ns_) {
      if (!armed_) { all_seen = false; continue; }
      return fail(STALE);
    }
    feedback_[i].ready = armed_ && armed_feedback_[i];
  }
  out = feedback_;
  return all_seen;
}
bool MitBus::send(const std::vector<double> & q, bool enable, bool reset) {
  if (!open_) return false;
  if (!enable) {
    mit_enabled_ = false;
    armed_ = false; std::fill(armed_feedback_.begin(), armed_feedback_.end(), false);
    if (!driver_->transmit(stop_.data(), stop_.size())) return fail(DRIVER);
    return true;
  }
  if (error_ != NONE || reset || q.size() != axes_.size()) return false;
  if (!mit_enabled_) return false;
  const auto now = clock_();
  for (size_t i = 0; i < axes_.size(); ++i) {
    const auto & a = axes_[i];
    if (!seen_[i] || now < received_at_[i] || now - received_at_[i] > timeout_ns_) return fail(STALE);
    if (!std::isfinite(q[i]) || q[i] < a.lower || q[i] > a.upper) return fail(COMMAND);
    const double target = a.can.zero_rad + a.direction * a.can.gear_ratio * q[i];
    tx_[i] = mit::command(a.can, target);
  }
  if (!driver_->transmit(tx_.data(), tx_.size())) return fail(DRIVER);
  if (!armed_) std::fill(armed_feedback_.begin(), armed_feedback_.end(), false);
  armed_ = true;
  return true;
}
void MitBus::close() noexcept {
  if (open_) driver_->transmit(stop_.data(), stop_.size());
  driver_->close(); open_ = false; armed_ = false; mit_enabled_ = false;
}
const char * MitBus::fault_reason() const noexcept {
  switch (error_.load()) {
    case DRIVER: return "CAN transmit/receive error or RX overflow";
    case STALE: return "A configured motor has stale/missing feedback";
    case MALFORMED: return "Malformed feedback from a configured motor";
    case MOTOR: return "MIT motor temperature limit exceeded";
    case COMMAND: return "Invalid joint target";
    case OVERSPEED: return "Measured joint speed exceeds 150% of configured velocity limit";
    default: return "";
  }
}
} // namespace arm_control
