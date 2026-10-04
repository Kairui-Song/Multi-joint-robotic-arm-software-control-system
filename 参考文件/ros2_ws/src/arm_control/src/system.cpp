#include "arm_control/system.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <set>
#include <limits>

namespace arm_control {
namespace {
using Clock = std::chrono::steady_clock;
double number(const std::string & s) {
  size_t used{}; double v = std::stod(s, &used);
  if (used != s.size() || !std::isfinite(v)) throw std::invalid_argument("Invalid number: " + s);
  return v;
}
uint32_t integer(const std::string & s, uint32_t max = UINT32_MAX) {
  size_t used{}; auto v = std::stoull(s, &used, 0);
  if (used != s.size() || v > max || s[0] == '-') throw std::invalid_argument("Invalid integer: " + s);
  return static_cast<uint32_t>(v);
}
double elapsed_us(Clock::time_point start) {
  return std::chrono::duration<double, std::micro>(Clock::now() - start).count();
}
}
ArmSystem::~ArmSystem() {
  executor_.cancel(); if (thread_.joinable()) thread_.join(); stop();
}
ArmSystem::CallbackReturn ArmSystem::on_init(const hardware_interface::HardwareInfo & info) {
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
  try {
    const auto & p = info.hardware_parameters;
    auto option = [&](const std::string & key, const std::string & fallback) {
      auto it = p.find(key); return it == p.end() ? fallback : it->second;
    };
    config_.backend = p.at("backend");
    const bool can_backend = config_.backend == "socketcan" || config_.backend == "zcan" || config_.backend == "can_mock";
    const bool rm_backend = config_.backend == "rm_socketcan" || config_.backend == "rm_zcan" || config_.backend == "rm_mock";
    config_.commissioned = option("commissioned", "false") == "true";
    if (config_.backend == "loong") {
      config_.sdk_xml = p.at("sdk_xml");
      config_.sdk_command_timeout_s = number(p.at("command_timeout_s"));
      config_.can.feedback_timeout_s = number(p.at("feedback_timeout_s"));
      config_.can.motor_watchdog_confirmed = option("motor_watchdog_confirmed", "false") == "true";
      config_.sdk_cpu = integer(p.at("sdk_cpu"), UINT16_MAX);
    }
    if (can_backend || rm_backend) {
      config_.can.protocol = p.at("can_protocol"); config_.can.interface = option("can_interface", "can0");
      config_.can.bitrate = integer(p.at("can_bitrate"));
      config_.can.feedback_timeout_s = number(p.at("feedback_timeout_s"));
      config_.can.protocol_confirmed = option("protocol_confirmed", "false") == "true";
      config_.can.stop_confirmed = option("stop_confirmed", "false") == "true";
      config_.can.motor_watchdog_confirmed = option("motor_watchdog_confirmed", "false") == "true";
      if (rm_backend) {
        config_.can.channel_count = integer(p.at("can_channels"), 2);
        config_.can.second_interface = option("can_second_interface", "can1");
        config_.can.feedback_hz = integer(p.at("can_feedback_hz"));
        config_.can.command_timeout_s = number(p.at("command_timeout_s"));
        config_.can.startup_reference_confirmed = option("startup_reference_confirmed", "false") == "true";
        config_.can.group_exclusive_confirmed = option("group_exclusive_confirmed", "false") == "true";
        if (config_.can.protocol != "robomaster_c620") throw std::invalid_argument("RM backend requires C620 protocol");
      }
      if (config_.backend == "zcan" || config_.backend == "rm_zcan") {
        config_.can.device_type = integer(p.at("zcan_device_type"));
        config_.can.device_index = integer(p.at("zcan_device_index"));
        config_.can.channel = integer(p.at("zcan_channel"));
        config_.can.timing0 = integer(p.at("zcan_timing0"), 255);
        config_.can.timing1 = integer(p.at("zcan_timing1"), 255);
        config_.can.bitrate_property = option("zcan_bitrate_property", "");
        config_.can.fd_adapter = option("zcan_fd_adapter", "false") == "true";
        config_.can.data_bitrate = integer(option("zcan_data_bitrate", "5000000"));
      }
    }
    config_.cycle_ns = integer(p.at("cycle_ns"));
    watchdog_s_ = number(p.at("watchdog_s")); activation_s_ = number(p.at("activation_timeout_s"));
    if (config_.cycle_ns < 100000 || config_.cycle_ns > 100000000 ||
        watchdog_s_ <= config_.cycle_ns * 1e-9 || activation_s_ <= 0 || activation_s_ > 60)
      throw std::invalid_argument("Invalid timing parameters");
    if (config_.backend == "igh") {
      config_.commissioned = p.at("commissioned") == "true";
      config_.reset_faults = p.at("reset_faults_on_activate") == "true";
      config_.master = integer(p.at("master_index"));
      config_.dc_assign = integer(p.at("dc_assign"), UINT16_MAX);
      config_.watchdog_divider = integer(p.at("watchdog_divider"), UINT16_MAX);
      config_.watchdog_intervals = integer(p.at("watchdog_intervals"), UINT16_MAX);
      if (!config_.watchdog_intervals) throw std::invalid_argument("Drive watchdog must be enabled");
    }
    std::set<std::string> names;
    std::set<std::pair<uint16_t, uint16_t>> addresses;
    std::set<uint16_t> motor_ids;
    for (const auto & j : info.joints) {
      std::set<std::string> state_names;
      for (const auto & interface : j.state_interfaces) state_names.insert(interface.name);
      if (j.command_interfaces.size() != 1 || j.command_interfaces[0].name != "position" ||
          j.state_interfaces.size() != 2 || state_names != std::set<std::string>{"position", "velocity"})
        throw std::invalid_argument("Expected position command and position/velocity states");
      const auto & jp = j.parameters;
      if (can_backend || rm_backend) validate_control_parameters(config_.can.protocol, jp);
      Axis a; a.name = j.name;
      a.lower = number(jp.at("lower")); a.upper = number(jp.at("upper"));
      a.velocity = number(jp.at("max_velocity")); a.direction = number(jp.at("direction"));
      a.counts_per_rad = 1;
      if (rm_backend) {
        a.can.motor_id = integer(jp.at("motor_id"), 8);
        a.can.gear_ratio = number(jp.at("gear_ratio")); a.rm.channel = integer(jp.at("can_channel"), 1);
        a.rm.startup_position = number(jp.at("startup_position"));
        a.rm.position_kp = number(jp.at("position_kp")); a.rm.velocity_kp = number(jp.at("velocity_kp"));
        a.rm.velocity_ki = number(jp.at("velocity_ki")); a.rm.current_limit = number(jp.at("current_limit_a"));
        a.rm.max_temperature = number(jp.at("max_temperature_c"));
      } else if (can_backend) {
        a.can.motor_id = integer(jp.at("motor_id"), 2046);
        a.can.gear_ratio = number(jp.at("gear_ratio")); a.can.zero_rad = number(jp.at("motor_zero_rad"));
        a.can.kp = number(jp.at("kp")); a.can.kd = number(jp.at("kd"));
        if (config_.can.protocol == "mit_shared_v1") {
          a.can.master_id = integer(jp.at("master_id"), 255);
          a.can.p_max = number(jp.at("p_max")); a.can.v_max = number(jp.at("v_max"));
          a.can.t_max = number(jp.at("t_max")); a.can.max_temperature = number(jp.at("max_temperature_c"));
        }
        if (!motor_ids.insert(a.can.motor_id).second) throw std::invalid_argument("Duplicate motor ID");
      } else if (config_.backend == "loong") {
        a.sdk_index = integer(jp.at("sdk_index"));
        a.can.kp = number(jp.at("kp")); a.can.kd = number(jp.at("kd"));
      } else {
        a.counts_per_rad = number(jp.at("counts_per_rad")); a.zero = number(jp.at("zero_counts"));
        a.alias = integer(jp.at("alias"), UINT16_MAX); a.position = integer(jp.at("position"), UINT16_MAX);
        a.vendor = integer(jp.at("vendor_id")); a.product = integer(jp.at("product_code"));
        a.rx_pdo = integer(jp.at("rx_pdo"), UINT16_MAX); a.tx_pdo = integer(jp.at("tx_pdo"), UINT16_MAX);
        if (!addresses.emplace(a.alias, a.position).second) throw std::invalid_argument("Duplicate EtherCAT address");
      }
      validate(a);
      if (!names.insert(a.name).second) throw std::invalid_argument("Duplicate joint name");
      axes_.push_back(a);
    }
    if (axes_.empty()) throw std::invalid_argument("No joints configured");
    const size_t n = axes_.size(); feedback_.resize(n); position_.resize(n); velocity_.resize(n);
    command_.resize(n); sent_.resize(n);
    std::fill(position_.begin(), position_.end(), std::numeric_limits<double>::quiet_NaN());
    std::fill(velocity_.begin(), velocity_.end(), std::numeric_limits<double>::quiet_NaN());
    axis_health_ = std::make_unique<AxisHealth[]>(n);
    bus_ = make_bus(config_);
    node_ = std::make_shared<rclcpp::Node>("arm_hardware_monitor", rclcpp::NodeOptions().use_global_arguments(false));
    RCLCPP_INFO(node_->get_logger(), "backend=%s protocol=%s axes=%zu cycle_ns=%u control=%s",
      config_.backend.c_str(), (can_backend || rm_backend) ? config_.can.protocol.c_str() : "backend-defined",
      axes_.size(), config_.cycle_ns, bus_->control_mode());
    diagnostics_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    timing_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_hardware/timing", 10);
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), [this] { publish_diagnostics(); });
    if (config_.backend == "sim" || config_.backend == "can_mock" || config_.backend == "rm_mock") {
      injection_service_ = node_->create_service<std_srvs::srv::Trigger>("/arm_hardware/inject_fault",
        [this](std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          if (!active_.load()) { res->success = false; res->message = "Hardware is not active"; return; }
          inject_.store(true); res->success = true; res->message = "Fault queued for next read cycle";
        });
    }
    executor_.add_node(node_); thread_ = std::thread([this] { executor_.spin(); });
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("arm_control"), "%s", e.what()); return CallbackReturn::ERROR;
  }
}
std::vector<hardware_interface::StateInterface> ArmSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> out;
  for (size_t i = 0; i < axes_.size(); ++i) {
    out.emplace_back(axes_[i].name, "position", &position_[i]);
    out.emplace_back(axes_[i].name, "velocity", &velocity_[i]);
  }
  return out;
}
std::vector<hardware_interface::CommandInterface> ArmSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> out;
  for (size_t i = 0; i < axes_.size(); ++i) out.emplace_back(axes_[i].name, "position", &command_[i]);
  return out;
}
ArmSystem::CallbackReturn ArmSystem::on_configure(const rclcpp_lifecycle::State &) {
  if (configured_) return CallbackReturn::ERROR;
  try {
    bus_->open(axes_, config_);
    for (size_t i = 0; i < axes_.size(); ++i) sent_[i] = command_[i] = std::clamp(0.0, axes_[i].lower, axes_[i].upper);
    configured_ = true; inject_ = false; return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    // open() may have acquired resources before throwing; configured_ is still false.
    bus_->close();
    stop(); RCLCPP_ERROR(node_->get_logger(), "%s", e.what()); return CallbackReturn::ERROR;
  }
}
bool ArmSystem::feedback_valid(bool require_enabled) const {
  for (size_t i = 0; i < axes_.size(); ++i) {
    const auto & f = feedback_[i]; const auto & a = axes_[i];
    if (!std::isfinite(f.position) || !feedback_speed_valid(f.velocity, a) ||
        f.position < a.lower || f.position > a.upper || f.error || f.fault ||
        (require_enabled && !f.ready)) return false;
  }
  return true;
}
ArmSystem::CallbackReturn ArmSystem::on_activate(const rclcpp_lifecycle::State &) {
  if (!configured_) return CallbackReturn::ERROR;
  if (!bus_->prepare_activate()) { fault_ = BUS; stop(); return CallbackReturn::ERROR; }
  const auto deadline = Clock::now() + std::chrono::duration<double>(activation_s_);
  bool reset_sent = false;
  while (Clock::now() < deadline) {
    const bool valid = bus_->receive(feedback_, config_.cycle_ns * 1e-9);
    bool positions_ok = valid;
    for (size_t i = 0; i < axes_.size(); ++i) {
      const double q = feedback_[i].position;
      positions_ok = positions_ok && std::isfinite(q) && q >= axes_[i].lower && q <= axes_[i].upper &&
        feedback_speed_valid(feedback_[i].velocity, axes_[i]);
    }
    bool has_fault = false;
    for (const auto & f : feedback_) has_fault = has_fault || f.fault || f.error;
    const bool reset = positions_ok && has_fault && config_.reset_faults && !reset_sent;
    const auto activation = bus_->activation_step(feedback_, positions_ok, reset);
    reset_sent = reset_sent || reset;
    if (activation == ActivationState::Fault) { fault_ = BUS; stop(); return CallbackReturn::ERROR; }
    if (activation == ActivationState::Ready && positions_ok && feedback_valid(true)) {
      for (size_t i = 0; i < axes_.size(); ++i) command_[i] = sent_[i] = position_[i] = feedback_[i].position;
      for (size_t i = 0; i < axes_.size(); ++i) velocity_[i] = feedback_[i].velocity;
      period_us_ = config_.cycle_ns * 1e-3;
      fault_ = NONE; active_ = true; previous_ = Clock::now();
      last_cycle_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(previous_.time_since_epoch()).count();
      return CallbackReturn::SUCCESS;
    }
    std::this_thread::sleep_for(std::chrono::nanoseconds(config_.cycle_ns));
  }
  fault_ = DRIVE; stop();
  RCLCPP_ERROR(node_->get_logger(), "Activation timeout: check bus, all motor feedback, limits and readiness");
  return CallbackReturn::ERROR;
}
void ArmSystem::stop() noexcept {
  active_ = false;
  if (bus_ && configured_.exchange(false)) {
    try { if (!bus_->send(sent_, false)) fault_ = BUS; }
    catch (...) { fault_ = BUS; }
    bus_->close();
  }
  std::fill(position_.begin(), position_.end(), std::numeric_limits<double>::quiet_NaN());
  std::fill(velocity_.begin(), velocity_.end(), std::numeric_limits<double>::quiet_NaN());
}
ArmSystem::CallbackReturn ArmSystem::on_deactivate(const rclcpp_lifecycle::State &) {
  active_ = false;
  if (configured_) {
    try {
      if (!bus_->send(sent_, false)) { fault_ = BUS; return CallbackReturn::ERROR; }
    } catch (...) { fault_ = BUS; return CallbackReturn::ERROR; }
  }
  return CallbackReturn::SUCCESS;
}
ArmSystem::CallbackReturn ArmSystem::on_cleanup(const rclcpp_lifecycle::State &) { stop(); return CallbackReturn::SUCCESS; }
ArmSystem::CallbackReturn ArmSystem::on_shutdown(const rclcpp_lifecycle::State &) { stop(); return CallbackReturn::SUCCESS; }
ArmSystem::CallbackReturn ArmSystem::on_error(const rclcpp_lifecycle::State &) { stop(); return CallbackReturn::SUCCESS; }
hardware_interface::return_type ArmSystem::fail(Fault code) {
  fault_ = code; active_ = false;
  if (configured_) bus_->send(sent_, false);
  return hardware_interface::return_type::ERROR;
}
hardware_interface::return_type ArmSystem::read(const rclcpp::Time &, const rclcpp::Duration &) {
  if (!configured_) return hardware_interface::return_type::OK;
  const auto start = Clock::now();
  const double dt = active_ ? std::chrono::duration<double>(start - previous_).count() : config_.cycle_ns * 1e-9;
  previous_ = start;
  if (active_ && (dt <= 0 || dt > watchdog_s_)) return fail(WATCHDOG);
  if (inject_.exchange(false)) bus_->inject_fault(true);
  if (!bus_->receive(feedback_, dt)) {
    if (active_) return fail(BUS);
    std::fill(position_.begin(), position_.end(), std::numeric_limits<double>::quiet_NaN());
    std::fill(velocity_.begin(), velocity_.end(), std::numeric_limits<double>::quiet_NaN());
    bus_->send(sent_, false); return hardware_interface::return_type::OK;
  }
  for (size_t i = 0; i < axes_.size(); ++i) {
    axis_health_[i].position = feedback_[i].position; axis_health_[i].velocity = feedback_[i].velocity;
    axis_health_[i].status = feedback_[i].status; axis_health_[i].error = feedback_[i].error;
    axis_health_[i].mode = feedback_[i].mode;
  }
  if (active_ && !feedback_valid(true)) return fail(DRIVE);
  for (size_t i = 0; i < axes_.size(); ++i) { position_[i] = feedback_[i].position; velocity_[i] = feedback_[i].velocity; }
  if (!active_) bus_->send(sent_, false);
  if (active_) {
    const double us = dt * 1e6; period_us_ = us;
    const double jitter = std::abs(us - config_.cycle_ns * 1e-3);
    jitter_max_us_ = std::max(jitter_max_us_.load(), jitter);
    if (dt > 1.5 * config_.cycle_ns * 1e-9) ++overruns_;
    ++cycles_;
    last_cycle_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(start.time_since_epoch()).count();
  }
  read_us_ = elapsed_us(start); return hardware_interface::return_type::OK;
}
hardware_interface::return_type ArmSystem::write(const rclcpp::Time &, const rclcpp::Duration &) {
  if (!active_) return hardware_interface::return_type::OK;
  const auto start = Clock::now();
  if (std::chrono::duration<double>(start - previous_).count() > watchdog_s_) return fail(WATCHDOG);
  const double dt = std::min(period_us_.load() * 1e-6, config_.cycle_ns * 1e-9);
  for (double q : command_) if (!std::isfinite(q)) return fail(COMMAND);
  for (size_t i = 0; i < axes_.size(); ++i) {
    const double q = limit(command_[i], sent_[i], std::max(dt, 1e-9), axes_[i]);
    if (std::abs(q - command_[i]) > 1e-12) ++clamps_;
    sent_[i] = q;
  }
  if (!bus_->send(sent_, true)) return fail(BUS);
  write_us_ = elapsed_us(start);
  const auto head = timing_head_.load(std::memory_order_relaxed);
  const auto next = (head + 1) % timing_ring_.size();
  if (next != timing_tail_.load(std::memory_order_acquire)) {
    timing_ring_[head] = {static_cast<double>(cycles_.load()), period_us_.load(), read_us_.load(), write_us_.load(), config_.cycle_ns * 1e-3};
    timing_head_.store(next, std::memory_order_release);
  } else ++timing_dropped_;
  return hardware_interface::return_type::OK;
}
void ArmSystem::publish_diagnostics() {
  diagnostic_msgs::msg::DiagnosticArray array; array.header.stamp = node_->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "arm_control/hardware"; status.hardware_id = config_.backend;
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
  const bool stale = active_ && (now_ns - last_cycle_ns_.load()) * 1e-9 > watchdog_s_;
  status.level = fault_ != NONE || stale ? 2 : active_ ? 0 : 1;
  status.message = stale ? "Control loop stalled; drive watchdog must stop hardware" : fault_ != NONE ? "Fault latched; explicit lifecycle recovery required" : active_ ? "Active" : "Inactive";
  auto add = [&](const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue kv; kv.key = key; kv.value = value; status.values.push_back(kv);
  };
  add("fault_code", std::to_string(fault_.load())); add("cycles", std::to_string(cycles_.load()));
  add("bus_fault_reason", bus_->fault_reason());
  add("backend", config_.backend);
  const bool wire_backend = config_.backend == "can_mock" || config_.backend == "socketcan" ||
    config_.backend == "zcan" || config_.backend.rfind("rm_", 0) == 0;
  add("protocol", wire_backend ? config_.can.protocol : "backend-defined");
  add("control_mode", bus_->control_mode());
  add("joint_count", std::to_string(axes_.size()));
  add("configured_cycle_ns", std::to_string(config_.cycle_ns));
  add("period_us", std::to_string(period_us_.load())); add("max_abs_jitter_us", std::to_string(jitter_max_us_.load()));
  add("read_us", std::to_string(read_us_.load())); add("write_us", std::to_string(write_us_.load()));
  add("overruns", std::to_string(overruns_.load())); add("limited_commands", std::to_string(clamps_.load()));
  add("timing_dropped", std::to_string(timing_dropped_.load()));
  array.status.push_back(status);
  for (size_t i = 0; i < axes_.size(); ++i) {
    diagnostic_msgs::msg::DiagnosticStatus joint;
    joint.name = "arm_control/" + axes_[i].name; joint.hardware_id = config_.backend;
    const auto sw = axis_health_[i].status.load(); const auto error = axis_health_[i].error.load();
    joint.level = error ? 2 : !active_ || stale ? 1 : 0;
    joint.message = error ? "Drive fault" : !active_ || stale ? "Inactive or stale feedback" : "Feedback available";
    auto value = [&](const std::string & key, const std::string & val) {
      diagnostic_msgs::msg::KeyValue kv; kv.key = key; kv.value = val; joint.values.push_back(kv);
    };
    value("position_rad", std::to_string(axis_health_[i].position.load()));
    value("velocity_rad_s", std::to_string(axis_health_[i].velocity.load()));
    value("statusword", std::to_string(sw)); value("error_code", std::to_string(error));
    value("mode_display", std::to_string(axis_health_[i].mode.load()));
    array.status.push_back(joint);
  }
  diagnostics_->publish(array);
  std_msgs::msg::Float64MultiArray timing;
  auto tail = timing_tail_.load(std::memory_order_relaxed);
  const auto head = timing_head_.load(std::memory_order_acquire);
  while (tail != head) {
    const auto & row = timing_ring_[tail]; timing.data.insert(timing.data.end(), row.begin(), row.end());
    tail = (tail + 1) % timing_ring_.size();
  }
  timing_tail_.store(tail, std::memory_order_release);
  if (!timing.data.empty()) timing_pub_->publish(timing);
}
} // namespace arm_control
PLUGINLIB_EXPORT_CLASS(arm_control::ArmSystem, hardware_interface::SystemInterface)
