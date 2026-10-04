#include "arm_control/mit_bus.hpp"
#include "arm_control/can_bus.hpp"
#include "arm_control/mit_protocol.hpp"
#include <iostream>
#include <cstdlib>
#include <map>
#define CHECK(x) do {if(!(x)){std::cerr<<__LINE__<<": "<<#x<<'\n';std::exit(1);}}while(false)
using namespace arm_control;
struct Driver : can::Driver {
  std::vector<can::Frame> rx, tx;
  bool broken=false;
  void open(const CanConfig &,const std::vector<Axis> &) override {tx.clear();rx.clear();}
  bool transmit(const can::Frame *f,size_t n) override {tx.insert(tx.end(),f,f+n);return !broken;}
  int receive(can::Frame *f,size_t n) override {if(rx.size()>n)return -1;std::copy(rx.begin(),rx.end(),f);int k=rx.size();rx.clear();return k;}
  void close() noexcept override {}
};
can::Frame feedback(unsigned id) {
  can::Frame f;f.id=0;f.size=8;f.data={uint8_t(id),0x7f,0xff,0x7f,0xf7,0xff,25,26};return f;
}
int main() {
  Axis a;a.name="joint1";a.lower=-1;a.upper=1;a.velocity=.3;a.counts_per_rad=1;
  a.can.motor_id=1;a.can.master_id=17;a.can.p_max=12.566;a.can.v_max=20;a.can.t_max=120;a.can.kp=10;a.can.kd=1;
  // Independent golden vector from reference Python integer packing.
  const std::array<uint8_t,8> golden{127,255,127,240,81,51,55,255};
  CHECK(mit::command(a.can,0).data==golden);
  CHECK(mit::mode(1,true).data[7]==0xfc);CHECK(mit::mode(1,false).data[7]==0xfd);
  can::Sample s;auto f=feedback(17);CHECK(mit::decode(f,a.can,s));CHECK(s.temperature==26);
  f.id=17;CHECK(!mit::decode(f,a.can,s));f=feedback(17);f.size=7;CHECK(!mit::decode(f,a.can,s));
  auto driver=std::make_unique<Driver>();auto *raw=driver.get();uint64_t now=1000000000;
  MitBus bus(std::move(driver),[&]{return now;});BusConfig cfg;cfg.backend="can_mock";cfg.can.protocol="mit_shared_v1";cfg.cycle_ns=10000000;
  bus.open({a},cfg);CHECK(raw->tx.back().data[7]==0xfd);
  CHECK(bus.prepare_activate());CHECK(raw->tx.back().data[7]==0xfc);
  CHECK((raw->tx[1].data[3]&15)==0 && raw->tx[1].data[4]==0);
  std::vector<Feedback> out(1);raw->rx={feedback(18)};CHECK(!bus.receive(out,.01));
  raw->rx={feedback(17)};CHECK(bus.receive(out,.01));CHECK(!out[0].ready);
  CHECK(bus.send({.1},true));raw->rx={feedback(17)};CHECK(bus.receive(out,.01));CHECK(out[0].ready);
  now+=60000000;CHECK(!bus.receive(out,.01));CHECK(raw->tx.back().data[7]==0xfd);CHECK(!bus.send({.1},true));
  bus.open({a},cfg);CHECK(bus.prepare_activate());f=feedback(17);f.data[7]=90;raw->rx={f};CHECK(!bus.receive(out,.01));CHECK(raw->tx.back().data[7]==0xfd);
  bus.open({a},cfg);f=feedback(17);f.size=6;raw->rx={f};CHECK(!bus.receive(out,.01));
  // Both speed directions trip even with an in-range position, and stay latched.
  for (unsigned packed : {0u, 4095u}) {
    bus.open({a},cfg); CHECK(bus.prepare_activate());
    f=feedback(17); f.data[3]=packed>>4; f.data[4]=(packed<<4)|(f.data[4]&15);
    raw->rx={f}; CHECK(!bus.receive(out,.01));
    CHECK(raw->tx.back().data[7]==0xfd); CHECK(!bus.send({0},true));
    raw->rx={feedback(17)}; CHECK(!bus.receive(out,.01));
  }
  auto duplicate=a;duplicate.can.motor_id=2;
  bool rejected=false;try{bus.open({a,duplicate},cfg);}catch(const std::invalid_argument&){rejected=true;}CHECK(rejected);
  // Lifecycle handshake: waiting must not send FD or use disabled-state feedback.
  bus.open({a},cfg); raw->rx={feedback(17)}; CHECK(bus.prepare_activate());
  const auto waiting_tx=raw->tx.size();
  CHECK(!bus.receive(out,.01));
  CHECK(bus.activation_step(out,false,false)==ActivationState::Waiting);
  CHECK(raw->tx.size()==waiting_tx);
  f=feedback(17); f.data[1]=0x80; f.data[2]=0xff; raw->rx={f};
  CHECK(bus.receive(out,.01));
  CHECK(bus.activation_step(out,true,false)==ActivationState::Waiting);
  CHECK(raw->tx.back().data==mit::command(a.can,out[0].position).data);
  raw->rx={f}; CHECK(bus.receive(out,.01));
  CHECK(bus.activation_step(out,true,false)==ActivationState::Ready);
  CHECK(bus.send({.8},false)); CHECK(raw->tx.back().data[7]==0xfd);
  raw->rx={f}; CHECK(bus.prepare_activate()); CHECK(!bus.receive(out,.01));
  CHECK(bus.activation_step(out,false,false)==ActivationState::Waiting);
  f=feedback(17); raw->rx={f}; CHECK(bus.receive(out,.01));
  CHECK(bus.activation_step(out,true,false)==ActivationState::Waiting);
  CHECK(raw->tx.back().data==mit::command(a.can,out[0].position).data);
  raw->rx={f}; CHECK(bus.receive(out,.01));
  CHECK(bus.activation_step(out,true,false)==ActivationState::Ready);
  now+=60000000; CHECK(!bus.receive(out,.01));
  CHECK(bus.activation_step(out,false,false)==ActivationState::Fault);
  CHECK(raw->tx.back().data[7]==0xfd);
  bus.open({a},cfg); raw->broken=true; CHECK(!bus.prepare_activate());
  CHECK(bus.activation_step(out,false,false)==ActivationState::Fault); raw->broken=false;
  // Factory and direct backend construction both reject the wrong wire profile.
  auto bad=cfg; bad.backend="rm_mock";
  rejected=false;try{make_bus(bad);}catch(const std::invalid_argument&){rejected=true;}CHECK(rejected);
  bad=cfg; bad.can.protocol="robomaster_c620";
  rejected=false;try{make_bus(bad);}catch(const std::invalid_argument&){rejected=true;}CHECK(rejected);
  CanBus rv(std::make_unique<Driver>());
  rejected=false;try{rv.open({a},cfg);}catch(const std::invalid_argument&){rejected=true;}CHECK(rejected);
  bad=cfg; bad.can.protocol="rv_packed_v1";
  CHECK(dynamic_cast<CanBus *>(make_bus(bad).get()));
  rejected=false;try{bus.open({a},bad);}catch(const std::invalid_argument&){rejected=true;}CHECK(rejected);
  for (const auto & entry : std::vector<std::pair<std::string,std::string>>{
      {"mit_shared_v1","velocity_ki"},{"robomaster_c620","kp"},{"rv_packed_v1","master_id"}}) {
    std::map<std::string,std::string> params{{entry.second,"1"}};
    rejected=false;try{validate_control_parameters(entry.first,params);}catch(const std::invalid_argument&){rejected=true;}CHECK(rejected);
  }
  // Seven axes through the production factory and frame emulator, including Bezier samples.
  std::vector<Axis> axes(7,a);for(size_t i=0;i<7;++i){axes[i].name="j"+std::to_string(i);axes[i].can.motor_id=i+1;axes[i].can.master_id=i+17;}
  auto wire=make_bus(cfg);CHECK(dynamic_cast<MitBus *>(wire.get()));wire->open(axes,cfg);CHECK(wire->prepare_activate());out.resize(7);CHECK(wire->receive(out,.01));
  std::vector<double> q(7);
  for(int step=0;step<=100;++step){double u=step/100.;for(size_t i=0;i<7;++i)q[i]=.1*(3*u*u-2*u*u*u)*(i%2?-1:1);
    CHECK(wire->send(q,true));CHECK(wire->receive(out,.01));for(size_t i=0;i<7;++i)CHECK(std::abs(q[i]-out[i].position)<.001);}
  wire->inject_fault(true);CHECK(wire->send(q,true));CHECK(!wire->receive(out,.01));
  std::cout<<"MIT protocol, startup, mapping, timeout, temperature and seven-axis Bezier tests passed\n";
}
