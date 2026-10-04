#include "arm_control/robomaster.hpp"
#include <atomic>
#include <mutex>
#include <thread>
#include <set>

namespace arm_control::rm {
class RoboMasterBus final : public Bus {
  using Clock=std::chrono::steady_clock;
  struct Channel {
    std::unique_ptr<can::Driver> driver;
    std::array<int,9> mapping{};
    std::array<can::Frame,256> rx{};
    std::array<can::Frame,2> tx{};
    bool groups[2]{false,false};
  };
  std::string backend_;
  std::vector<Channel> channels_;
  std::vector<Axis> axes_;
  std::vector<Feedback> state_;
  std::vector<Encoder> encoders_;
  std::vector<CascadeController> controllers_;
  std::vector<Clock::time_point> last_rx_;
  std::vector<bool> seen_;
  std::vector<double> targets_;
  BusConfig cfg_;
  std::mutex mutex_;
  std::thread worker_;
  std::atomic<bool> running_{false}, injected_{false};
  std::atomic<int> fault_{0};
  bool enabled_{false}, valid_{false}, reference_lost_{false};
  Clock::time_point last_command_{};
  void zero() {
    const std::array<int16_t,4> zero{};
    for(auto & c:channels_) {
      size_t n=0;
      if(c.groups[0]) c.tx[n++]=group(0x200,zero);
      if(c.groups[1]) c.tx[n++]=group(0x1ff,zero);
      if(!c.driver->transmit(c.tx.data(),n)) fault_=2;
    }
    for(auto & controller:controllers_) controller.reset();
  }
  void loop() {
    auto previous=Clock::now(), next=previous;
    while(running_) {
      const auto now=Clock::now();
      const double dt=std::chrono::duration<double>(now-previous).count(); previous=now;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if(enabled_ && (dt>cfg_.can.command_timeout_s ||
          std::chrono::duration<double>(now-last_command_).count()>cfg_.can.command_timeout_s)) fault_=1;
        for(auto & c:channels_) {
          if(injected_.load()) c.driver->inject_fault(true);
          int n=c.driver->receive(c.rx.data(),c.rx.size());
          if(n<0 || static_cast<size_t>(n)>c.rx.size()) { fault_=2; continue; }
          for(int k=0;k<n;++k) {
            const auto & f=c.rx[k];
            if(f.error) {fault_=2; continue;}
            if(f.extended || f.remote || f.id<0x201 || f.id>0x208) continue;
            const int mapped=c.mapping[f.id-0x200]; if(mapped<0) continue;
            const auto i=static_cast<size_t>(mapped); Sample sample;
            if(!decode(f,sample)) {fault_=3; continue;}
            const auto & a=axes_[i];
            const double position=encoders_[i].update(sample.encoder,a);
            const double velocity=a.direction*sample.rpm*(2*pi)/(60*a.can.gear_ratio);
            const bool bad=sample.temperature>a.rm.max_temperature || std::abs(velocity)>a.velocity*1.5 ||
              position<a.lower || position>a.upper;
            state_[i]={position,velocity,0,static_cast<uint16_t>(bad?0x1001:0),-1,false,bad};
            seen_[i]=true; last_rx_[i]=now; if(bad) fault_=4;
          }
        }
        valid_=true;
        for(size_t i=0;i<axes_.size();++i) {
          if(!seen_[i]) valid_=false;
          else if(std::chrono::duration<double>(now-last_rx_[i]).count()>cfg_.can.feedback_timeout_s) { valid_=false; fault_=5; }
          state_[i].ready=enabled_ && valid_ && !fault_;
        }
        if(fault_ || !enabled_ || !valid_) zero();
        else {
          for(auto & c:channels_) {
            std::array<std::array<int16_t,4>,2> commands{};
            for(unsigned id=1;id<=8;++id) {
              int mapped=c.mapping[id]; if(mapped<0) continue; size_t i=static_cast<size_t>(mapped);
              const double amp=controllers_[i].current(targets_[i],state_[i].position,state_[i].velocity,
                std::max(dt,1e-6),axes_[i]);
              commands[(id-1)/4][(id-1)%4]=static_cast<int16_t>(std::lround(axes_[i].direction*amp*16384/20));
            }
            size_t n=0; if(c.groups[0]) c.tx[n++]=group(0x200,commands[0]); if(c.groups[1]) c.tx[n++]=group(0x1ff,commands[1]);
            if(!c.driver->transmit(c.tx.data(),n)) fault_=2;
          }
          if(fault_) zero();
        }
        if(fault_) reference_lost_=true;
      }
      next+=std::chrono::nanoseconds(cfg_.cycle_ns);
      if(next<Clock::now()) next=Clock::now();
      std::this_thread::sleep_until(next);
    }
    zero();
  }
public:
  const char * control_mode() const noexcept override { return "host position P + velocity PI -> C620 current"; }
  explicit RoboMasterBus(std::string backend):backend_(std::move(backend)){}
  ~RoboMasterBus() override {close();}
  bool prepare_activate() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || fault_) return false;
    enabled_ = false;
    for (auto & controller : controllers_) controller.reset();
    for (auto & feedback : state_) feedback.ready = false;
    return true;
  }
  void open(const std::vector<Axis> & axes,const BusConfig & cfg) override {
    close();
    const bool mock=backend_=="rm_mock";
    if(cfg.can.protocol!="robomaster_c620") throw std::invalid_argument("RM backend requires robomaster_c620");
    if(!mock && (!cfg.commissioned || !cfg.can.motor_watchdog_confirmed ||
      !cfg.can.startup_reference_confirmed || !cfg.can.group_exclusive_confirmed))
      throw std::invalid_argument("C620 requires commissioning, startup reference, dedicated current groups and motor watchdog confirmation");
    if(!mock && reference_lost_) throw std::runtime_error("Encoder reference lost: place arm at confirmed startup reference and restart; blind recovery is prohibited");
    if(axes.empty() || cfg.can.channel_count<1 || cfg.can.channel_count>2 || cfg.can.bitrate!=1000000 ||
      cfg.cycle_ns<1000000 || cfg.cycle_ns>20000000 || cfg.can.feedback_timeout_s<=cfg.cycle_ns*1e-9 ||
      !std::isfinite(cfg.can.feedback_timeout_s) || !std::isfinite(cfg.can.command_timeout_s) ||
      cfg.can.command_timeout_s<=cfg.cycle_ns*1e-9 || cfg.can.command_timeout_s>1 || cfg.can.feedback_hz!=1000)
      throw std::invalid_argument("Invalid C620 bus rate/channel/timeout configuration");
    cfg_=cfg; axes_=axes; channels_.resize(cfg.can.channel_count);
    for(auto & c:channels_) c.mapping.fill(-1);
    for(size_t i=0;i<axes.size();++i) {
      const auto & a=axes[i]; validate(a);
      if(a.rm.channel>=channels_.size() || a.can.motor_id<1 || a.can.motor_id>8 ||
         !std::isfinite(a.can.gear_ratio) || a.can.gear_ratio<=0 || !std::isfinite(a.rm.startup_position) ||
         a.rm.startup_position<a.lower || a.rm.startup_position>a.upper ||
         !std::isfinite(a.rm.position_kp) || a.rm.position_kp<=0 || !std::isfinite(a.rm.velocity_kp) || a.rm.velocity_kp<=0 ||
         !std::isfinite(a.rm.velocity_ki) || a.rm.velocity_ki<0 || !std::isfinite(a.rm.current_limit) || a.rm.current_limit<=0 || a.rm.current_limit>20 ||
         !std::isfinite(a.rm.max_temperature) || a.rm.max_temperature<=0 ||
         a.velocity*1.5*a.can.gear_ratio*cfg.can.feedback_timeout_s>=pi)
        throw std::invalid_argument("Invalid C620 axis/gains/reference or encoder unwrap interval");
      auto & c=channels_[a.rm.channel];
      if(c.mapping[a.can.motor_id]>=0) throw std::invalid_argument("Duplicate C620 ID on channel");
      c.mapping[a.can.motor_id]=static_cast<int>(i); c.groups[(a.can.motor_id-1)/4]=true;
    }
    for(unsigned ch=0;ch<channels_.size();++ch) {
      auto & c=channels_[ch]; std::vector<Axis> selected;
      for(const auto & a:axes) if(a.rm.channel==ch) selected.push_back(a);
      if(selected.empty()) throw std::invalid_argument("Empty C620 channel");
      const double frames=selected.size()*cfg.can.feedback_hz+(int(c.groups[0])+int(c.groups[1]))*1e9/cfg.cycle_ns;
      if(frames*135/cfg.can.bitrate>.70) throw std::invalid_argument("C620 feedback bandwidth exceeds 70%; split axes across CAN channels");
      CanConfig dc=cfg.can; dc.channel+=ch; dc.protocol="robomaster_c620";
      dc.interface=ch==0?cfg.can.interface:cfg.can.second_interface;
      c.driver=can::make_driver(mock?"can_mock":backend_=="rm_zcan"?"zcan":"socketcan");
      try { c.driver->open(dc,selected); } catch(...) { for(auto & opened:channels_) if(opened.driver) opened.driver->close(); throw; }
    }
    state_.assign(axes.size(),{}); encoders_.assign(axes.size(),{}); controllers_.assign(axes.size(),{});
    targets_.resize(axes.size()); for(size_t i=0;i<axes.size();++i) targets_[i]=axes[i].rm.startup_position;
    seen_.assign(axes.size(),false); last_rx_.assign(axes.size(),{});
    enabled_=valid_=false; fault_=0; reference_lost_=false; injected_=false; last_command_=Clock::now();
    running_=true; worker_=std::thread([this]{loop();});
  }
  bool receive(std::vector<Feedback> & out,double) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!running_ || out.size()!=state_.size()) return false;
    out=state_; return valid_ && !fault_;
  }
  bool send(const std::vector<double> & q,bool enable,bool reset) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!running_ || reset || q.size()!=axes_.size() || (enable && (!valid_ || fault_))) return false;
    if(!enable) {enabled_=false; last_command_=Clock::now(); return true;}
    for(size_t i=0;i<q.size();++i) if(!std::isfinite(q[i]) || q[i]<axes_[i].lower || q[i]>axes_[i].upper) {fault_=6; return false;}
    targets_=q; enabled_=enable; last_command_=Clock::now(); return true;
  }
  void close() noexcept override {
    running_=false;
    if(worker_.joinable()) {
      worker_.join();
      // No multi-turn absolute reference survives a stopped receive worker.
      if(backend_!="rm_mock") reference_lost_=true;
    }
    for(auto & c:channels_) if(c.driver) c.driver->close();
    channels_.clear();
  }
  void inject_fault(bool v) override {injected_=v;}
  const char * fault_reason() const noexcept override {
    switch(fault_.load()) {
      case 1:return "C620 command/worker watchdog"; case 2:return "CAN driver error";
      case 3:return "Malformed C620 feedback"; case 4:return "C620 temperature, speed or joint limit exceeded";
      case 5:return "C620 motor feedback timed out"; case 6:return "Invalid C620 joint target"; default:return "";
    }
  }
};
std::unique_ptr<Bus> make_bus(const std::string & backend) {return std::make_unique<RoboMasterBus>(backend);}
} // namespace arm_control::rm
