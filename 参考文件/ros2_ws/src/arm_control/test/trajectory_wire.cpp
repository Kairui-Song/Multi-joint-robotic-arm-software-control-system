// Offline integration harness: the production Bus factory, C620 codec and controller.
#include "arm_control/robomaster.hpp"
#include <fstream>
#include <iostream>
#include <thread>
using namespace arm_control;
int main(int argc,char **argv) {
  if(argc!=2) return 2;
  std::ifstream input(argv[1]); std::vector<std::vector<double>> points;
  while(input) {std::vector<double> row(6); for(auto & q:row) input>>q; if(input) points.push_back(row);}
  if(points.empty()) return 2;
  std::vector<Axis> axes(6);
  for(unsigned i=0;i<6;++i) {
    auto & a=axes[i]; a.name="joint"+std::to_string(i+1); a.lower=-2.8; a.upper=2.8;
    a.velocity=1; a.counts_per_rad=1; a.can.gear_ratio=19.203208556; a.can.motor_id=i%3+1; a.rm.channel=i/3;
  }
  BusConfig cfg; cfg.backend="rm_mock"; cfg.cycle_ns=5000000; cfg.can.channel_count=2;
  cfg.can.protocol="robomaster_c620";
  cfg.can.feedback_timeout_s=.02; cfg.can.command_timeout_s=.3;
  auto bus=make_bus(cfg.backend); bus->open(axes,cfg); std::vector<Feedback> feedback(6);
  auto pause=[] {std::this_thread::sleep_for(std::chrono::milliseconds(5));};
  bool ready=false; for(int k=0;k<100 && !ready;++k){pause();ready=bus->receive(feedback,.005);}
  if(!ready) return 3;
  auto next=std::chrono::steady_clock::now();
  for(size_t k=0;k<points.size()+400;++k) {
    if(!bus->send(points[std::min(k,points.size()-1)],true)) return 4;
    next+=std::chrono::milliseconds(5); std::this_thread::sleep_until(next);
    if(!bus->receive(feedback,.005)){std::cerr<<bus->fault_reason();return 5;}
  }
  for(const auto & f:feedback) std::cout<<f.position<<' ';
  std::cout<<'\n'; bus->close();
}
