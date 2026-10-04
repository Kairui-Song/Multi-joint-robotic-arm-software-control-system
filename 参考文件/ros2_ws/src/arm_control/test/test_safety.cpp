#include "arm_control/bus.hpp"
#include <iostream>
#include <limits>
#include <cstdlib>

#define CHECK(expr) do { if (!(expr)) { std::cerr << "Failed at line " << __LINE__ << ": " << #expr << '\n'; std::exit(1); } } while (false)
template<class F> void rejects(F f) { bool thrown = false; try { f(); } catch (const std::exception &) { thrown = true; } CHECK(thrown); }
int main() {
  using namespace arm_control;
  Axis a; a.name = "joint1"; a.lower = -1; a.upper = 1; a.velocity = 2; a.counts_per_rad = 1000;
  validate(a);
  CHECK(feedback_speed_valid(3., a)); CHECK(feedback_speed_valid(-3., a));
  CHECK(!feedback_speed_valid(3.01, a)); CHECK(!feedback_speed_valid(-3.01, a));
  CHECK(!feedback_speed_valid(std::numeric_limits<double>::quiet_NaN(), a));
  CHECK(!feedback_speed_valid(std::numeric_limits<double>::infinity(), a));
  CHECK(std::abs(limit(3, 0, .01, a) - .02) < 1e-12);
  CHECK(std::abs(limit(-3, 0, .01, a) + .02) < 1e-12);
  CHECK(limit(3, .99, .01, a) == 1);
  CHECK(limit(-3, -.99, .01, a) == -1);
  CHECK(limit(0, 0, .01, a) == 0);
  rejects([&] { limit(std::numeric_limits<double>::quiet_NaN(), 0, .01, a); });
  rejects([&] { limit(std::numeric_limits<double>::infinity(), 0, .01, a); });
  rejects([&] { limit(0, 0, 0, a); });
  rejects([&] { limit(0, 0, -.1, a); });
  auto bad = a; bad.direction = 0; rejects([&] { validate(bad); });
  bad = a; bad.upper = bad.lower; rejects([&] { validate(bad); });
  bad = a; bad.counts_per_rad = 1e30; rejects([&] { validate(bad); });
  CHECK(enable_word(0x0040) == 6); CHECK(enable_word(0x0021) == 7);
  CHECK(enable_word(0x0023) == 15); CHECK(enable_word(0x0027) == 15);
  CHECK(enable_word(0x0008) == 0); CHECK(enable_word(0x000f) == 0);
  CHECK(!enabled(0x0040)); CHECK(enabled(0x1227));
  auto bus = make_bus("sim");
  std::vector<Axis> axes{a, a}; axes[1].name = "joint2";
  std::vector<Feedback> states(2); std::vector<double> targets{.5, -.5};
  bus->open(axes, BusConfig{}); CHECK(bus->receive(states, .01));
  CHECK(!enabled(states[0].status)); CHECK(states[0].position == 0);
  bus->send(targets, true); CHECK(bus->receive(states, .01));
  CHECK(enabled(states[0].status)); CHECK(states[0].position > 0); CHECK(states[1].position < 0);
  for (int i = 0; i < 50; ++i) CHECK(bus->receive(states, .01));
  CHECK(states[0].position == .5); CHECK(states[1].position == -.5);
  bus->inject_fault(true); bus->send({1, 1}, true); CHECK(bus->receive(states, .01));
  CHECK(states[0].status & 8); CHECK(states[0].position == .5);
  bus->send({0, 0}, true); CHECK(bus->receive(states, .01)); CHECK(states[0].status & 8);
  bus->send({.5, -.5}, false, true); CHECK(bus->receive(states, .01)); CHECK(!(states[0].status & 8));
  bus->send({.5, -.5}, true); CHECK(bus->receive(states, .01)); CHECK(enabled(states[0].status));
  bus->send({0, 0}, false); CHECK(bus->receive(states, .01)); CHECK(states[0].position == .5);
  bus->close(); rejects([] { make_bus("unknown"); });
  std::cout << "Safety core and simulated multi-axis fault recovery passed\n";
}
