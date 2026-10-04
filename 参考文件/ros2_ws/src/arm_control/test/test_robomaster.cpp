#include "arm_control/robomaster.hpp"
#include <thread>
#include <iostream>
#include <cstdlib>
#define CHECK(x) do { if(!(x)) {std::cerr<<__LINE__<<": "<<#x<<'\n'; std::exit(1);} } while(false)
using namespace arm_control;
int main() {
  auto f=rm::group(0x200,{16384,-16384,1,-1});
  const std::array<uint8_t,8> expected{64,0,192,0,0,1,255,255}; CHECK(f.data==expected);
  f.id=0x201; f.data={31,255,255,156,0,1,30,0}; rm::Sample sample;
  CHECK(rm::decode(f,sample)); CHECK(sample.encoder==8191 && sample.rpm==-100);
  f.data[0]=32; CHECK(!rm::decode(f,sample));
  Axis a; a.name="joint"; a.lower=-2; a.upper=2; a.velocity=1; a.counts_per_rad=1;
  a.can.gear_ratio=19.203208556; a.can.motor_id=1;
  rm::Encoder encoder; CHECK(encoder.update(8190,a)==0);
  CHECK(std::abs(encoder.update(2,a)-4*2*rm::pi/(8192*a.can.gear_ratio))<1e-12);
  CHECK(std::abs(encoder.update(8190,a))<1e-12);
  rm::CascadeController control; CHECK(control.current(2,0,0,.005,a)<=a.rm.current_limit);
  BusConfig cfg; cfg.backend="rm_mock"; cfg.cycle_ns=5000000; cfg.can.feedback_timeout_s=.02;
  cfg.can.protocol="robomaster_c620";
  // Broad scheduler allowance on Windows; production YAML retains a 50 ms watchdog.
  cfg.can.command_timeout_s=.3;
  auto bus=make_bus("rm_mock"); bus->open({a},cfg); std::vector<Feedback> state(1);
  auto pause=[] {std::this_thread::sleep_for(std::chrono::milliseconds(5));};
  bool ready=false; for(int k=0;k<100 && !ready;++k){pause();ready=bus->receive(state,.005);} CHECK(ready);
  auto activate=[&] {
    CHECK(bus->prepare_activate());
    ActivationState result=ActivationState::Waiting;
    for(int k=0;k<100 && result==ActivationState::Waiting;++k) {
      const bool valid=bus->receive(state,.005);
      result=bus->activation_step(state,valid,false); pause();
    }
    CHECK(result==ActivationState::Ready);
  };
  activate();
  for(int k=0;k<500;++k) {CHECK(bus->send({.1},true)); pause(); CHECK(bus->receive(state,.005));}
  CHECK(state[0].ready); CHECK(std::abs(state[0].position-.1)<.02);
  CHECK(bus->send({.9},false)); activate();
  CHECK(std::abs(state[0].position-.1)<.02); // Reactivation holds feedback, not an old target.
  std::this_thread::sleep_for(std::chrono::milliseconds(350)); CHECK(!bus->receive(state,.005));
  CHECK(!bus->send({0},true)); CHECK(std::string(bus->fault_reason()).find("watchdog")!=std::string::npos);
  bus->close(); bus->open({a},cfg); ready=false;
  for(int k=0;k<100 && !ready;++k){pause();ready=bus->receive(state,.005);} CHECK(ready);
  bus->inject_fault(true); pause(); pause(); CHECK(!bus->receive(state,.005)); bus->close();
  std::vector<Axis> six(6,a); for(unsigned k=0;k<6;++k) six[k].can.motor_id=k+1;
  bool rejected=false; try {bus->open(six,cfg);} catch(const std::invalid_argument &){rejected=true;} CHECK(rejected);
  cfg.can.channel_count=2;
  for(unsigned k=0;k<6;++k){six[k].can.motor_id=k%3+1;six[k].rm.channel=k/3;}
  bus->open(six,cfg); state.resize(6); ready=false;
  for(int k=0;k<100 && !ready;++k){pause();ready=bus->receive(state,.005);} CHECK(ready); bus->close();
  std::cout<<"PASS: C620 wire frames, encoder wrap, current limit, closed-loop motion, watchdog, temperature fault, dual-channel mapping/bandwidth\n";
}
