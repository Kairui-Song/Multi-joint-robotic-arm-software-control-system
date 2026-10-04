#include "arm_control/bus.hpp"
#include <ecrt.h>
#include <chrono>
#include <type_traits>

namespace arm_control {
// Older IgH headers expose several operations as void; newer 1.6 headers return errors.
template<class F> bool call_ok(F && fn) {
  if constexpr (std::is_void_v<std::invoke_result_t<F>>) { fn(); return true; }
  else return fn() >= 0;
}
class IghBus final : public Bus {
  struct Offsets { unsigned control{}, target{}, mode{}, status{}, actual{}, velocity{}, display{}, error{}; };
  ec_master_t * master_{nullptr};
  ec_domain_t * domain_{nullptr};
  uint8_t * data_{nullptr};
  std::vector<Axis> axes_;
  std::vector<Offsets> offsets_;
  std::vector<ec_slave_config_t *> slaves_;
  bool dc_{false};
public:
  ~IghBus() override { close(); }
  void open(const std::vector<Axis> & axes, const BusConfig & cfg) override {
    close();
    if (!cfg.commissioned) throw std::runtime_error("Real hardware requires commissioned=true");
    axes_ = axes; offsets_.resize(axes.size()); slaves_.clear(); dc_ = cfg.dc_assign != 0;
    master_ = ecrt_request_master(cfg.master);
    if (!master_) throw std::runtime_error("Cannot request IgH master");
    try {
      domain_ = ecrt_master_create_domain(master_);
      if (!domain_) throw std::runtime_error("Cannot create EtherCAT domain");
      for (size_t i = 0; i < axes.size(); ++i) {
        const auto & a = axes[i];
        if (!a.vendor || !a.product) throw std::runtime_error("Missing slave identity");
        auto * sc = ecrt_master_slave_config(master_, a.alias, a.position, a.vendor, a.product);
        if (!sc) throw std::runtime_error("Slave identity/configuration failed");
        slaves_.push_back(sc);
        // Explicit supported PDO layout. Verify against ESI before commissioning.
        ec_pdo_entry_info_t rx[] = {{0x6040, 0, 16}, {0x607a, 0, 32}, {0x6060, 0, 8}};
        ec_pdo_entry_info_t tx[] = {{0x6041, 0, 16}, {0x6064, 0, 32}, {0x606c, 0, 32}, {0x6061, 0, 8}, {0x603f, 0, 16}};
        ec_pdo_info_t pdos[] = {{a.rx_pdo, 3, rx}, {a.tx_pdo, 5, tx}};
        ec_sync_info_t syncs[] = {{0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
          {1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE},
          {2, EC_DIR_OUTPUT, 1, &pdos[0], EC_WD_ENABLE},
          {3, EC_DIR_INPUT, 1, &pdos[1], EC_WD_DISABLE}, {0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DEFAULT}};
        if (ecrt_slave_config_pdos(sc, EC_END, syncs)) throw std::runtime_error("PDO assignment failed");
        if (!call_ok([&] { return ecrt_slave_config_watchdog(sc, cfg.watchdog_divider, cfg.watchdog_intervals); }))
          throw std::runtime_error("Watchdog configuration failed");
        if (dc_ && !call_ok([&] { return ecrt_slave_config_dc(sc, cfg.dc_assign, cfg.cycle_ns, 0, 0, 0); }))
          throw std::runtime_error("DC configuration failed");
        auto reg = [&](uint16_t index) {
          unsigned bit = 0;
          const int offset = ecrt_slave_config_reg_pdo_entry(sc, index, 0, domain_, &bit);
          if (offset < 0 || bit != 0) throw std::runtime_error("PDO registration/alignment failed");
          return static_cast<unsigned>(offset);
        };
        auto & o = offsets_[i];
        o.control = reg(0x6040); o.target = reg(0x607a); o.mode = reg(0x6060);
        o.status = reg(0x6041); o.actual = reg(0x6064); o.velocity = reg(0x606c);
        o.display = reg(0x6061); o.error = reg(0x603f);
      }
      if (ecrt_master_activate(master_)) throw std::runtime_error("Master activation failed");
      data_ = ecrt_domain_data(domain_);
      if (!data_) throw std::runtime_error("Missing process data");
    } catch (...) { close(); throw; }
  }
  bool receive(std::vector<Feedback> & out, double) override {
    if (!master_ || out.size() != axes_.size()) return false;
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    if (!call_ok([&] { return ecrt_master_application_time(master_, static_cast<uint64_t>(ns)); }) ||
        !call_ok([&] { return ecrt_master_receive(master_); }) ||
        !call_ok([&] { return ecrt_domain_process(domain_); })) return false;
    ec_domain_state_t ds{}; ec_master_state_t ms{};
    if (!call_ok([&] { return ecrt_domain_state(domain_, &ds); }) ||
        !call_ok([&] { return ecrt_master_state(master_, &ms); })) return false;
    bool valid = ds.wc_state == EC_WC_COMPLETE && ms.link_up;
    for (size_t i = 0; i < axes_.size(); ++i) {
      ec_slave_config_state_t ss{};
      if (!call_ok([&] { return ecrt_slave_config_state(slaves_[i], &ss); })) return false;
      valid = valid && ss.online && ss.operational;
      const auto & a = axes_[i]; const auto & o = offsets_[i];
      out[i] = {(EC_READ_S32(data_ + o.actual) - a.zero) / (a.direction * a.counts_per_rad),
        EC_READ_S32(data_ + o.velocity) / (a.direction * a.counts_per_rad),
        EC_READ_U16(data_ + o.status), EC_READ_U16(data_ + o.error), EC_READ_S8(data_ + o.display)};
      out[i].ready = enabled(out[i].status) && out[i].mode == 8;
      out[i].fault = (out[i].status & 8) || out[i].error;
    }
    return valid;
  }
  bool send(const std::vector<double> & q, bool enable, bool reset) override {
    if (!data_ || q.size() != axes_.size()) return false;
    for (size_t i = 0; i < axes_.size(); ++i) {
      const auto & a = axes_[i]; const auto & o = offsets_[i];
      const auto sw = EC_READ_U16(data_ + o.status);
      const double raw = a.zero + a.direction * q[i] * a.counts_per_rad;
      EC_WRITE_S32(data_ + o.target, static_cast<int32_t>(std::llround(raw)));
      EC_WRITE_S8(data_ + o.mode, 8);
      EC_WRITE_U16(data_ + o.control, reset ? 0x0080 : enable ? enable_word(sw) : 0x0000);
    }
    bool okay = true;
    if (dc_) {
      okay = call_ok([&] { return ecrt_master_sync_reference_clock(master_); });
      okay = call_ok([&] { return ecrt_master_sync_slave_clocks(master_); }) && okay;
    }
    // Attempt the output frame even if DC synchronization failed, including disable commands.
    okay = call_ok([&] { return ecrt_domain_queue(domain_); }) && okay;
    return call_ok([&] { return ecrt_master_send(master_); }) && okay;
  }
  void close() noexcept override {
    if (master_) ecrt_release_master(master_);
    master_ = nullptr; domain_ = nullptr; data_ = nullptr;
  }
};
std::unique_ptr<Bus> make_igh_bus() { return std::make_unique<IghBus>(); }
} // namespace arm_control
