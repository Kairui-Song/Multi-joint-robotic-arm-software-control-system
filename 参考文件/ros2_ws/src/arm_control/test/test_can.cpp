#include "arm_control/can_bus.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>

#define CHECK(expr) do { if (!(expr)) { std::cerr << "line " << __LINE__ << ": " << #expr << '\n'; std::exit(1); } } while (false)
template<class F> void rejects(F f) { bool threw=false; try { f(); } catch(const std::exception &) { threw=true; } CHECK(threw); }
using namespace arm_control;
class TestDriver : public can::Driver {
public:
  std::vector<can::Frame> incoming, sent; bool fail_tx=false, fail_rx=false;
  void open(const CanConfig &, const std::vector<Axis> &) override { incoming.clear(); sent.clear(); }
  bool transmit(const can::Frame * data, size_t n) override { sent.assign(data,data+n); return !fail_tx; }
  int receive(can::Frame * data, size_t capacity) override {
    if(fail_rx || incoming.size()>capacity) return -1;
    std::copy(incoming.begin(),incoming.end(),data); int n=static_cast<int>(incoming.size()); incoming.clear(); return n;
  }
  void close() noexcept override {}
};
can::Frame feedback(uint16_t id) {
  can::Frame f; f.id=id; f.size=8; f.data={0x20,0x7f,0xff,0x7f,0xf7,0xff,100,0}; return f;
}
int main() {
  const auto golden=can::position_command(7,500,5,12.5);
  CHECK(golden.id==7 && golden.size==8);
  const std::array<uint8_t,8> bytes{31,255,255,255,255,127,247,255}; CHECK(golden.data==bytes);
  const auto stop=can::zero_current(5); CHECK(stop.size==3 && stop.data[0]==0x61 && stop.data[1]==0 && stop.data[2]==0);
  rejects([]{can::position_command(0,1,1,0);}); rejects([]{can::position_command(2047,1,1,0);});
  rejects([]{can::position_command(1,1,1,std::numeric_limits<double>::quiet_NaN());});
  rejects([]{can::position_command(1,501,1,0);});
  can::Sample sample; auto f=feedback(500); CHECK(can::decode(f,sample)); CHECK(std::abs(sample.position)<.0004);
  f.data={0x20,255,255,255,255,255,100,0}; CHECK(can::decode(f,sample));
  CHECK(sample.position==12.5 && sample.velocity==18 && sample.current==30 && sample.temperature==25);
  for(unsigned n=0;n<7;++n){f.size=n; CHECK(!can::decode(f,sample));}
  f.size=8; f.extended=true; CHECK(!can::decode(f,sample));
  f.extended=false; f.remote=true; CHECK(!can::decode(f,sample));
  f.remote=false; f.error=true; CHECK(!can::decode(f,sample));
  Axis a; a.name="a"; a.lower=-2; a.upper=2; a.velocity=1; a.counts_per_rad=1; a.can.motor_id=3;
  Axis b=a; b.name="b"; b.can.motor_id=600; b.direction=-1; b.can.gear_ratio=2;
  std::vector<Axis> axes{a,b}; BusConfig cfg; cfg.backend="can_mock"; cfg.cycle_ns=5000000;
  auto driver=std::make_unique<TestDriver>(); auto * raw=driver.get(); uint64_t now=1000000000;
  CanBus bus(std::move(driver),[&]{return now;}); std::vector<Feedback> state(2);
  bus.open(axes,cfg); CHECK(raw->sent[0].data[0]==0x61); CHECK(!bus.receive(state,.005));
  raw->incoming={feedback(600),feedback(3),feedback(42)}; CHECK(bus.receive(state,.005)); CHECK(!state[0].ready);
  CHECK(state[0].position<0 && state[1].position>0); // ID mapping and direction/ratio.
  CHECK(bus.send({.1,-.1},true)); CHECK(raw->sent[1].id==600);
  raw->incoming={feedback(600),feedback(3)}; CHECK(bus.receive(state,.005)); CHECK(state[0].ready && state[1].ready);
  now+=60000000; raw->incoming={feedback(3)}; CHECK(!bus.receive(state,.005)); // one stale axis trips all.
  CHECK(raw->sent[0].data[0]==0x61); CHECK(!bus.send({0,0},true));
  bus.close(); bus.open(axes,cfg); raw->incoming={feedback(3),feedback(600)}; CHECK(bus.receive(state,.005));
  CHECK(!bus.send({std::numeric_limits<double>::infinity(),0},true));
  bus.open(axes,cfg); auto bad=feedback(3); bad.size=2; raw->incoming={bad}; CHECK(!bus.receive(state,.005));
  bus.open(axes,cfg); bad=feedback(3); bad.data[0]=0x28; raw->incoming={bad}; CHECK(!bus.receive(state,.005));
  bus.open(axes,cfg); raw->incoming={feedback(3),feedback(600)}; CHECK(bus.receive(state,.005));
  raw->fail_tx=true; CHECK(!bus.send({0,0},true)); raw->fail_tx=false;
  for (unsigned packed : {0u, 4095u}) {
    bus.open(axes,cfg); auto fast=feedback(600);
    fast.data[3]=packed>>4; fast.data[4]=(packed<<4)|(fast.data[4]&15);
    raw->incoming={feedback(3),fast}; CHECK(!bus.receive(state,.005));
    CHECK(raw->sent[0].data[0]==0x61); CHECK(!bus.send({0,0},true));
    raw->incoming={feedback(3),feedback(600)}; CHECK(!bus.receive(state,.005));
  }
  auto overloaded=cfg; overloaded.cycle_ns=100000; rejects([&]{bus.open(axes,overloaded);});
  auto physical=cfg; physical.backend="socketcan"; rejects([&]{bus.open(axes,physical);});
  auto duplicate=axes; duplicate[1].can.motor_id=3; rejects([&]{bus.open(duplicate,cfg);});
  // Same Bus factory used by ArmSystem, through wire encoding/decoding and an emulated network.
  auto wire=make_bus("can_mock"); wire->open(axes,cfg); CHECK(wire->receive(state,.005));
  CHECK(wire->send({.2,-.2},true)); CHECK(wire->receive(state,.005));
  CHECK(state[0].ready && state[1].ready); CHECK(std::abs(state[0].position-.2)<.001);
  CHECK(std::abs(state[1].position+.2)<.001);
  wire->inject_fault(true); CHECK(wire->send({.3,-.3},true)); CHECK(!wire->receive(state,.005));
  CHECK(!wire->send({0,0},true)); wire->close(); wire->open(axes,cfg); CHECK(wire->receive(state,.005));
  std::cout << "PASS: wire golden vectors, ID mapping, units, stale axis, malformed frame, motor fault, partial-send failure, recovery\n";
}
