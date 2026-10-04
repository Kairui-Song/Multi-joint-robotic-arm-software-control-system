#include "arm_control/system.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>

#define CHECK(expr) do { if (!(expr)) { std::cerr << "Failed at line " << __LINE__ << ": " << #expr << '\n'; std::exit(1); } } while (false)
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  {
    arm_control::ArmSystem system;
    hardware_interface::HardwareInfo info; info.name = "TestArm"; info.type = "system";
    info.hardware_parameters = {{"backend", "sim"}, {"cycle_ns", "1000000"},
      {"watchdog_s", "0.02"}, {"activation_timeout_s", "1"}};
    hardware_interface::ComponentInfo joint; joint.name = "joint1"; joint.type = "joint";
    hardware_interface::InterfaceInfo position; position.name = "position";
    hardware_interface::InterfaceInfo velocity; velocity.name = "velocity";
    joint.command_interfaces = {position}; joint.state_interfaces = {velocity, position};
    joint.parameters = {{"lower", "-1"}, {"upper", "1"}, {"max_velocity", "2"},
      {"counts_per_rad", "1000"}, {"zero_counts", "0"}, {"direction", "1"},
      {"alias", "0"}, {"position", "0"}, {"vendor_id", "0"}, {"product_code", "0"},
      {"rx_pdo", "0x1600"}, {"tx_pdo", "0x1a00"}};
    info.joints = {joint};
    const rclcpp_lifecycle::State state;
    const auto ok = hardware_interface::CallbackReturn::SUCCESS;
    const auto read_ok = hardware_interface::return_type::OK;
    const auto error = hardware_interface::return_type::ERROR;
    const rclcpp::Time time(0); const auto period = rclcpp::Duration::from_seconds(.001);
    CHECK(system.on_init(info) == ok);
    auto commands = system.export_command_interfaces(); auto states = system.export_state_interfaces();
    CHECK(std::isnan(states[0].get_value())); // No synthetic encoder feedback before configuration.
    CHECK(system.on_configure(state) == ok); CHECK(system.on_activate(state) == ok);
    CHECK(commands[0].get_value() == 0);
    CHECK(system.read(time, period) == read_ok);
    // Jazzy deduces the handle data type from the argument; position is double.
    (void)commands[0].set_value(99.0); CHECK(system.write(time, period) == read_ok);
    CHECK(system.read(time, period) == read_ok); CHECK(states[0].get_value() <= .002);
    (void)commands[0].set_value(std::numeric_limits<double>::quiet_NaN());
    CHECK(system.write(time, period) == error);
    CHECK(system.on_error(state) == ok);
    CHECK(system.on_configure(state) == ok); CHECK(system.on_activate(state) == ok);
    CHECK(commands[0].get_value() == 0); // Old goal must not survive recovery.
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    CHECK(system.read(time, period) == error); CHECK(system.on_error(state) == ok);
    CHECK(system.on_configure(state) == ok); CHECK(system.on_activate(state) == ok);
    CHECK(system.on_deactivate(state) == ok);
    (void)commands[0].set_value(.7);
    CHECK(system.on_activate(state) == ok);
    CHECK(commands[0].get_value() == states[0].get_value()); // Reactivation discards stale commands.
    CHECK(system.on_deactivate(state) == ok); CHECK(system.on_cleanup(state) == ok);
    CHECK(std::isnan(states[0].get_value()));
    CHECK(system.on_shutdown(state) == ok);
  }
  // Production hardware layer with both independent protocol backends.
  for (bool mit : {false, true}) {
    arm_control::ArmSystem system;
    hardware_interface::HardwareInfo info; info.name="ProtocolArm"; info.type="system";
    info.hardware_parameters={{"backend",mit?"can_mock":"rm_mock"},
      {"can_protocol",mit?"mit_shared_v1":"robomaster_c620"},
      {"cycle_ns",mit?"10000000":"5000000"}, {"watchdog_s","0.5"}, {"activation_timeout_s","2"},
      {"can_bitrate","1000000"}, {"feedback_timeout_s","0.1"}, {"can_channels","2"},
      {"can_feedback_hz","1000"}, {"command_timeout_s","0.5"}};
    for (int i=0;i<(mit?7:6);++i) {
      hardware_interface::ComponentInfo joint; joint.name="joint"+std::to_string(i+1); joint.type="joint";
      hardware_interface::InterfaceInfo position; position.name="position";
      hardware_interface::InterfaceInfo velocity; velocity.name="velocity";
      joint.command_interfaces={position}; joint.state_interfaces={position,velocity};
      joint.parameters={{"lower","-1"},{"upper","1"},{"max_velocity","0.3"},{"direction","1"},
        {"motor_id",std::to_string(mit?i+1:i%3+1)},{"gear_ratio",mit?"1":"19.203208556"},
        {"max_temperature_c","80"}};
      if(mit) {
        joint.parameters.insert({{"master_id",std::to_string(i+17)},{"kp","10"},{"kd","1"},
          {"p_max","12.5"},{"v_max","20"},{"t_max","10"},{"motor_zero_rad","0"}});
      } else {
        joint.parameters.insert({{"can_channel",std::to_string(i/3)},{"startup_position","0"},
          {"position_kp","5"},{"velocity_kp","3"},{"velocity_ki","1"},{"current_limit_a","3"}});
      }
      info.joints.push_back(joint);
    }
    { // Direct HardwareInfo cannot bypass mixed-parameter validation.
      arm_control::ArmSystem invalid;
      auto mixed=info; mixed.joints[0].parameters[mit?"velocity_ki":"kp"]="1";
      CHECK(invalid.on_init(mixed)==hardware_interface::CallbackReturn::ERROR);
    }
    const rclcpp_lifecycle::State state;
    const auto ok=hardware_interface::CallbackReturn::SUCCESS;
    const auto io_ok=hardware_interface::return_type::OK;
    const rclcpp::Time time(0); const auto period=rclcpp::Duration::from_seconds(.01);
    CHECK(system.on_init(info)==ok);
    auto commands=system.export_command_interfaces(); auto states=system.export_state_interfaces();
    CHECK(system.on_configure(state)==ok); CHECK(system.on_activate(state)==ok);
    CHECK(system.read(time,period)==io_ok); CHECK(system.write(time,period)==io_ok);
    CHECK(system.on_deactivate(state)==ok);
    for(auto & command:commands) (void)command.set_value(.8);
    CHECK(system.on_activate(state)==ok);
    for(size_t i=0;i<commands.size();++i) CHECK(commands[i].get_value()==states[2*i].get_value());
    CHECK(system.read(time,period)==io_ok); CHECK(system.write(time,period)==io_ok);
    CHECK(system.on_deactivate(state)==ok); CHECK(system.on_cleanup(state)==ok);
    CHECK(std::isnan(states[0].get_value()));
  }
  rclcpp::shutdown(); std::cout << "Lifecycle, protocol isolation, reactivation, command limit, NaN rejection, watchdog and recovery passed\n";
}
