#include "arm_control/bus.hpp"
#include <loong_driver_sdk.h>
#include <tinyxml2.h>
#include <atomic>
#include <chrono>
#include <mutex>
#include <set>
#include <thread>
#include <cctype>
#include <sstream>

namespace arm_control {
namespace {
using Clock = std::chrono::steady_clock;
using SDK = DriverSDK::DriverSDK;
std::atomic<bool> sdk_owner{false};
bool initialized = false, attempted = false;
std::string initialized_xml;
// Validate the subset this adapter supports BEFORE the SDK parses or starts interfaces.
void validate_xml(const std::string & path, const std::vector<Axis> & axes) {
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) throw std::invalid_argument("Cannot read Loong XML");
  auto * root = doc.FirstChildElement("Config");
  if (!root) throw std::invalid_argument("Missing SDK Config element");
  for (const char * section : {"ECAT", "CAN", "RS485", "RS485Emu"}) {
    auto * bus = root->FirstChildElement(section);
    if (!bus || !bus->FirstChildElement("Devices") || !bus->FirstChildElement("Masters") || !bus->FirstChildElement("Slaves") || !bus->FirstChildElement("Categories") || !bus->FirstChildElement("Domains"))
      throw std::invalid_argument("SDK XML requires Devices/Masters/Slaves sections");
    for (auto * s = bus->FirstChildElement("Slaves")->FirstChildElement("Slave"); s; s = s->NextSiblingElement("Slave"))
      if (std::string(section) != "CAN" && s->IntText() != 0)
        throw std::invalid_argument("Loong arm adapter requires a CAN-only XML; disable other slaves");
  }
  auto * imu = root->FirstChildElement("IMU");
  if (!imu || !imu->Attribute("device") || std::string(imu->Attribute("device")) != "" || !imu->Attribute("type"))
    throw std::invalid_argument("Loong CAN-only adapter requires disabled IMU");
  auto * motors = root->FirstChildElement("Motors");
  if (!motors) throw std::invalid_argument("Missing SDK motors");
  std::set<int> aliases;
  for (auto * m = motors->FirstChildElement("Motor"); m; m = m->NextSiblingElement("Motor")) {
    const int alias = m->IntAttribute("alias");
    if (alias <= 0 || !aliases.insert(alias).second || m->IntAttribute("limb") < 0 || m->IntAttribute("limb") > 5 || m->IntAttribute("motor") < 0)
      throw std::invalid_argument("Invalid SDK motor alias/limb");
    for (const char * field : {"Polarity", "CountBias", "EncoderResolution", "GearRatioTor", "GearRatioPosVel", "RatedCurrent", "TorqueConstant", "GearEfficiency", "RatedTorque", "MaximumTorque", "MinimumPosition", "MaximumPosition"}) {
      auto * value = m->FirstChildElement(field); double x{};
      if (!value || value->QueryDoubleText(&x) != tinyxml2::XML_SUCCESS || !std::isfinite(x))
        throw std::invalid_argument("Missing/nonfinite SDK motor parameter");
    }
    for (const auto & a : axes) if (a.sdk_index + 1 == static_cast<unsigned>(alias))
      if (a.lower < m->FirstChildElement("MinimumPosition")->DoubleText() || a.upper > m->FirstChildElement("MaximumPosition")->DoubleText())
        throw std::invalid_argument("ROS limits exceed SDK limits");
  }
  if (aliases.empty() || *aliases.rbegin() != static_cast<int>(aliases.size()))
    throw std::invalid_argument("SDK aliases must cover 1..total without gaps");
  auto * can = root->FirstChildElement("CAN");
  std::set<int> masters;
  for (auto * m = can->FirstChildElement("Masters")->FirstChildElement("Master"); m; m = m->NextSiblingElement("Master")) {
    int order = m->IntAttribute("order"); const char * device = m->Attribute("device");
    if (order < 0 || order >= 8 || !masters.insert(order).second || !device || !*device || std::string(device).size()>15 || m->BoolAttribute("canfd") || m->BoolAttribute("canhal") || m->IntAttribute("division") <= 0)
      throw std::invalid_argument("Adapter supports classic SocketCAN masters only");
    for (const char * p = device; *p; ++p) if (!std::isalnum(static_cast<unsigned char>(*p)) && *p != '_')
      throw std::invalid_argument("Invalid CAN interface name");
  }
  std::set<int> active, wanted;
  std::set<std::pair<int,int>> rx_ids, tx_ids;
  for (const auto & a : axes) wanted.insert(static_cast<int>(a.sdk_index + 1));
  for (auto * s = can->FirstChildElement("Slaves")->FirstChildElement("Slave"); s; s = s->NextSiblingElement("Slave")) {
    if (s->IntText() == 0) continue;
    const int alias = s->IntAttribute("alias"), master = s->IntAttribute("master"), tx = s->IntAttribute("slave_id");
    const char * type = s->Attribute("type"), * ids = s->Attribute("master_ids");
    if (!aliases.count(alias) || !active.insert(alias).second || !masters.count(master) || !type || !ids ||
        tx < 1 || tx > 15 || !tx_ids.emplace(master,tx).second)
      throw std::invalid_argument("Invalid/duplicate SDK CAN slave");
    if (std::string(type).rfind("Encos",0) != 0 && std::string(type).rfind("Damiao",0) != 0)
      throw std::invalid_argument("This Loong adapter validates Encos/Damiao profiles only");
    std::istringstream stream(ids); int id{}, count{};
    while (stream >> id) { if (id < 1 || id >= 2048 || !rx_ids.emplace(master,id).second) throw std::invalid_argument("Invalid/duplicate RX ID"); ++count; }
    if (!count || !stream.eof()) throw std::invalid_argument("Invalid master_ids");
    tinyxml2::XMLElement * profile = nullptr;
    for (auto * d = can->FirstChildElement("Devices")->FirstChildElement("Device"); d; d = d->NextSiblingElement("Device"))
      if (d->Attribute("type") && std::string(d->Attribute("type")) == type) profile = d;
    if (!profile) throw std::invalid_argument("Missing CAN device profile");
    for (const auto & a : axes) if (a.sdk_index + 1 == static_cast<unsigned>(alias)) {
      for (const char * field : {"MinP","MaxP","MinV","MaxV","MinKp","MaxKp","MinKd","MaxKd","MinT","MaxT"})
        if (!profile->FirstChildElement(field)) throw std::invalid_argument("Incomplete CAN device profile");
      if (a.can.kp <= 0 || a.can.kp < profile->FirstChildElement("MinKp")->DoubleText() || a.can.kp > profile->FirstChildElement("MaxKp")->DoubleText() ||
          a.can.kd < profile->FirstChildElement("MinKd")->DoubleText() || a.can.kd > profile->FirstChildElement("MaxKd")->DoubleText())
        throw std::invalid_argument("SDK gain outside device range");
    }
  }
  if (active != wanted) throw std::invalid_argument("ROS joint mapping must cover exactly all active SDK CAN motors");
}
}
class LoongBus final : public Bus {
  std::vector<Axis> axes_;
  std::vector<DriverSDK::motorTargetStruct> target_;
  std::vector<DriverSDK::motorActualStruct> actual_;
  std::vector<double> ages_, requested_;
  std::vector<Feedback> snapshot_;
  std::mutex mutex_;
  std::thread worker_;
  std::atomic<bool> running_{false};
  std::atomic<int> fault_{0};
  bool owner_{false}, enable_{false}, valid_{false};
  Clock::time_point updated_{};
  BusConfig cfg_;
  void loop() {
    auto next = Clock::now();
    while (running_) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        auto & sdk = SDK::instance();
        bool fresh = sdk.getMotorActual(actual_) == 0 && sdk.getCANFeedbackAge(ages_) == 0;
        for (const auto & a : axes_) fresh = fresh && std::isfinite(ages_[a.sdk_index]) && ages_[a.sdk_index] <= cfg_.can.feedback_timeout_s;
        if (enable_ && (!fresh || std::chrono::duration<double>(Clock::now() - updated_).count() > cfg_.sdk_command_timeout_s)) fault_ = 1;
        for (size_t i = 0; i < axes_.size(); ++i) {
          const auto k = axes_[i].sdk_index; const auto & f = actual_[k];
          const bool bad = f.errorCode || (f.statusWord != 0xffff && (f.statusWord & 8)) || !std::isfinite(f.pos) || !std::isfinite(f.vel);
          if (enable_ && bad) fault_ = 2;
          snapshot_[i] = {f.pos, f.vel, f.statusWord, f.errorCode, -1,
            fresh && enable_ && !fault_ && ((f.statusWord & 0x7f) == 0x37), bad};
          auto & t = target_[k];
          t.enabled = enable_ && fresh && !fault_ ? 1 : 0;
          t.pos = static_cast<float>(requested_[i]); t.vel = 0; t.tor = 0;
          t.kp = t.enabled ? static_cast<float>(axes_[i].can.kp) : 0;
          t.kd = t.enabled ? static_cast<float>(axes_[i].can.kd) : 0;
        }
        valid_ = fresh;
        if (sdk.setMotorTarget(target_) != 0) fault_ = 3;
      }
      next += std::chrono::nanoseconds(cfg_.cycle_ns);
      if (next < Clock::now()) next = Clock::now();
      std::this_thread::sleep_until(next);
    }
    for (auto & t : target_) { t.enabled = 0; t.kp = t.kd = t.tor = 0; }
    SDK::instance().setMotorTarget(target_);
  }
public:
  ~LoongBus() override { close(); }
  void open(const std::vector<Axis> & axes, const BusConfig & cfg) override {
    close();
    if (!cfg.commissioned || !cfg.can.motor_watchdog_confirmed || cfg.sdk_xml.empty() ||
        cfg.sdk_command_timeout_s <= cfg.cycle_ns * 1e-9 || cfg.can.feedback_timeout_s <= cfg.cycle_ns * 1e-9)
      throw std::invalid_argument("Loong commissioning/XML/watchdog configuration missing");
    validate_xml(cfg.sdk_xml, axes);
    bool expected = false;
    if (!sdk_owner.compare_exchange_strong(expected, true)) throw std::runtime_error("Loong SDK is process-global; only one ArmSystem may own it");
    owner_ = true;
    try {
      auto & sdk = SDK::instance();
      if (!initialized) {
        if (attempted) throw std::runtime_error("SDK init previously failed; restart the process");
        attempted = true;
        SDK::InitConfig setup; setup.cpusCAN.assign(3, cfg.sdk_cpu);
        if (sdk.init(cfg.sdk_xml.c_str(), setup) != 0) throw std::runtime_error("Loong SDK init failed");
        initialized = true; initialized_xml = cfg.sdk_xml;
      } else if (initialized_xml != cfg.sdk_xml) throw std::runtime_error("Restart to change Loong XML");
      const auto total = sdk.getTotalMotorNr();
      if (total <= 0) throw std::runtime_error("No SDK motors");
      for (const auto & a : axes) if (a.sdk_index >= static_cast<unsigned>(total) || a.direction != 1) throw std::invalid_argument("Invalid SDK index; polarity is handled by SDK XML");
      const auto active = sdk.getActiveMotors(); std::set<unsigned> active_ids(active.begin(),active.end()), wanted;
      for (const auto & a : axes) wanted.insert(a.sdk_index);
      if (wanted != active_ids || wanted.size() != axes.size()) throw std::invalid_argument("SDK active motors differ from ROS mapping");
      axes_ = axes; cfg_ = cfg;
      target_.assign(total, {}); actual_.resize(total); ages_.resize(total);
      requested_.assign(axes.size(),0); snapshot_.assign(axes.size(),{});
      enable_ = valid_ = false; fault_ = 0; updated_ = Clock::now(); running_ = true;
      worker_ = std::thread([this] { loop(); });
    } catch (...) { close(); throw; }
  }
  bool receive(std::vector<Feedback> & out, double) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || out.size() != snapshot_.size()) return false;
    out = snapshot_; return valid_ && !fault_;
  }
  bool send(const std::vector<double> & q, bool enable, bool reset) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || q.size() != axes_.size() || reset) return false;
    if (!enable) { enable_ = false; updated_ = Clock::now(); return true; }
    if (enable && fault_) return false;
    for (size_t i = 0; i < q.size(); ++i) if (!std::isfinite(q[i]) || q[i] < axes_[i].lower || q[i] > axes_[i].upper) return false;
    requested_ = q; enable_ = enable; updated_ = Clock::now(); return true;
  }
  void close() noexcept override {
    running_ = false; if (worker_.joinable()) worker_.join();
    if (owner_) { sdk_owner = false; owner_ = false; }
  }
  const char * fault_reason() const noexcept override {
    return fault_ == 1 ? "Loong feedback/command timeout" : fault_ == 2 ? "Loong motor fault" : fault_ == 3 ? "Loong setMotorTarget failed" : "";
  }
};
std::unique_ptr<Bus> make_loong_bus() { return std::make_unique<LoongBus>(); }
} // namespace arm_control
