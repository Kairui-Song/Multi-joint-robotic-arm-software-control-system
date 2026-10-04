#include "arm_control/can_driver.hpp"
#include "arm_control/robomaster.hpp"

namespace arm_control::can {
// Frame-level emulator: exercises the same CanBus encoder, driver API and feedback decoder.
// No vendor SDK or hardware is used by this explicitly selected backend.
class MockDriver final : public Driver {
  std::vector<Axis> axes_;
  std::vector<double> position_;
  std::array<Frame, 256> queue_{};
  size_t count_{0}; bool fault_{false};
  bool c620_{false}, mit_{false}; uint64_t previous_ns_{};
  std::vector<double> speed_, current_;
public:
  void open(const CanConfig & cfg, const std::vector<Axis> & axes) override {
    axes_ = axes; position_.resize(axes.size()); count_ = 0; fault_ = false;
    c620_=cfg.protocol=="robomaster_c620"; previous_ns_=monotonic_ns();
    mit_=cfg.protocol=="mit_shared_v1";
    speed_.assign(axes.size(),0); current_.assign(axes.size(),0);
    for (size_t i = 0; i < axes.size(); ++i)
      position_[i] = c620_ ? axes[i].rm.startup_position : axes[i].can.zero_rad + axes[i].direction * axes[i].can.gear_ratio * std::clamp(0., axes[i].lower, axes[i].upper);
  }
  bool transmit(const Frame * frames, size_t count) override {
    if(c620_) {
      for(size_t k=0;k<count;++k) for(size_t i=0;i<axes_.size();++i) {
        const unsigned id=axes_[i].can.motor_id; const auto & f=frames[k];
        if(f.size!=8) return false;
        if(f.id==(id<=4?0x200u:0x1ffu)) {
          const unsigned slot=(id-1)%4;
          current_[i]=axes_[i].direction*rm::signed16(f.data[2*slot],f.data[2*slot+1])*20.0/16384;
        }
      }
      return true;
    }
    for (size_t k = 0; k < count; ++k) {
      for (size_t i = 0; i < axes_.size(); ++i) {
        const auto & f = frames[k]; if (f.id != axes_[i].can.motor_id) continue;
        if (count_ == queue_.size()) return false;
        if (mit_) {
          const auto & a=axes_[i].can;
          const bool special=std::all_of(f.data.begin(),f.data.begin()+7,[](uint8_t b){return b==255;});
          if(f.size!=8) return false;
          const unsigned kp=((f.data[3]&15)<<8)|f.data[4];
          if(!fault_ && !special && kp) position_[i]=unpack((uint32_t(f.data[0])<<8)|f.data[1],-a.p_max,a.p_max,16);
          const auto p=pack(position_[i],-a.p_max,a.p_max,16);
          const auto v=pack(0,-a.v_max,a.v_max,12), t=pack(0,-a.t_max,a.t_max,12);
          Frame r; r.id=0; r.size=8;
          r.data={uint8_t(a.master_id),uint8_t(p>>8),uint8_t(p),uint8_t(v>>4),
            uint8_t((v<<4)|(t>>8)),uint8_t(t),uint8_t(fault_?100:25),25};
          queue_[count_++]=r;
          continue;
        }
        if (!fault_ && f.size == 8 && (f.data[0] >> 5) == 0)
          position_[i] = unpack((uint32_t(f.data[3]) << 8) | f.data[4], -12.5, 12.5, 16);
        const auto p = pack(position_[i], -12.5, 12.5, 16);
        const auto v = pack(0, -18, 18, 12), t = pack(0, -30, 30, 12);
        Frame r; r.id = f.id; r.size = 8;
        r.data = {static_cast<uint8_t>(fault_ ? 0x28 : 0x20), static_cast<uint8_t>(p >> 8),
          static_cast<uint8_t>(p), static_cast<uint8_t>(v >> 4),
          static_cast<uint8_t>((v << 4) | (t >> 8)), static_cast<uint8_t>(t), 100, 0};
        queue_[count_++] = r;
      }
    }
    return true;
  }
  int receive(Frame * frames, size_t capacity) override {
    if(c620_) {
      if(capacity<axes_.size()) return -1;
      const auto now=monotonic_ns(); const double dt=std::min((now-previous_ns_)*1e-9,.02); previous_ns_=now;
      for(size_t i=0;i<axes_.size();++i) {
        // A deliberately simple test plant, not a physical M3508 dynamics model.
        speed_[i]+=(8*current_[i]-3*speed_[i])*dt; position_[i]+=speed_[i]*dt;
        auto ticks=static_cast<int64_t>(std::llround(axes_[i].direction*position_[i]*axes_[i].can.gear_ratio*8192/(2*rm::pi)));
        const uint16_t encoder=static_cast<uint16_t>((ticks%8192+8192)%8192);
        const auto rpm=static_cast<uint16_t>(static_cast<int16_t>(std::lround(axes_[i].direction*speed_[i]*axes_[i].can.gear_ratio*60/(2*rm::pi))));
        const auto amps=static_cast<uint16_t>(static_cast<int16_t>(std::lround(axes_[i].direction*current_[i]*16384/20)));
        auto & f=frames[i]; f={}; f.id=0x200+axes_[i].can.motor_id; f.size=8;
        f.data={static_cast<uint8_t>(encoder>>8),static_cast<uint8_t>(encoder),static_cast<uint8_t>(rpm>>8),static_cast<uint8_t>(rpm),
          static_cast<uint8_t>(amps>>8),static_cast<uint8_t>(amps),static_cast<uint8_t>(fault_?100:30),0};
      }
      return static_cast<int>(axes_.size());
    }
    if (count_ > capacity) return -1;
    std::copy_n(queue_.data(), count_, frames); const auto n = count_; count_ = 0;
    return static_cast<int>(n);
  }
  void close() noexcept override { count_ = 0; }
  void inject_fault(bool v) override { fault_ = v; }
};
#ifdef __linux__
std::unique_ptr<Driver> make_socketcan_driver();
#endif
#ifdef ARM_WITH_ZCAN
std::unique_ptr<Driver> make_zcan_driver();
#endif
std::unique_ptr<Driver> make_driver(const std::string & backend) {
  if (backend == "can_mock") return std::make_unique<MockDriver>();
#ifdef __linux__
  if (backend == "socketcan") return make_socketcan_driver();
#endif
#ifdef ARM_WITH_ZCAN
  if (backend == "zcan") return make_zcan_driver();
#endif
  throw std::invalid_argument("CAN driver not built or unsupported on this OS: " + backend);
}
} // namespace arm_control::can
