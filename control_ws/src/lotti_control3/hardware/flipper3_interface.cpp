#include "lotti_control3/flipper3_interface.hpp"

#include <algorithm>
#include <exception>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

rclcpp::Clock & throttle_clock(){
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

int clamp_flipper_command(const double raw_command){
  const auto rounded = static_cast<int>(std::lround(raw_command));
  return std::max(-1, std::min(rounded, 1));
}

}  // namespace

namespace flipper3_interface{
  CallbackReturn FlipperInterface::on_init(const hardware_interface::HardwareInfo &info){
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS){
      return CallbackReturn::ERROR;
    }

    // Get the configured serial device from the ros2_control file.
    device_ = info_.hardware_parameters["device"];
    const auto feedback_mode = info_.hardware_parameters.find("feedback_mode");
    if (feedback_mode != info_.hardware_parameters.end() && !feedback_mode->second.empty()) {
      feedback_mode_ = feedback_mode->second;
    }

    // robot has 4 joints, 2 interfaces
    joint_positions_.assign(4, 0);
    joint_velocities_command_.assign(4, 0);

    for (const auto &joint : info_.joints){
      for (const auto &interface : joint.state_interfaces){
        joint_interfaces[interface.name].push_back(joint.name);
      }
    }
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> FlipperInterface::export_state_interfaces(){
    std::vector<hardware_interface::StateInterface> state_interfaces;

    int ind = 0;
    for (const auto &joint_name : joint_interfaces["position"]){
      state_interfaces.emplace_back(joint_name, "position", &joint_positions_[ind++]);
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> FlipperInterface::export_command_interfaces(){
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (int ind = 0; ind < 4; ind++){
      command_interfaces.emplace_back(info_.joints[ind].name, "velocity", &joint_velocities_command_[ind]);
    }
    return command_interfaces;
  }

  hardware_interface::CallbackReturn FlipperInterface::on_configure(const rclcpp_lifecycle::State &previous_state){
    RCLCPP_INFO(rclcpp::get_logger("FlipperInterface"), "Configuring ...please wait...");

    flipper_transport_ready_ = false;

    try {
      if (flipper_comms_.connected()){
        flipper_comms_.disconnect();
      }
      flipper_comms_.connect(device_);
      flipper_transport_ready_ = flipper_comms_.connected();
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        rclcpp::get_logger("FlipperInterface"),
        "Failed to initialize the flipper serial transport on %s: %s",
        device_.c_str(),
        exception.what()
      );
    } catch (...) {
      RCLCPP_ERROR(
        rclcpp::get_logger("FlipperInterface"),
        "Failed to initialize the flipper serial transport on %s due to an unknown error",
        device_.c_str()
      );
    }
 
    RCLCPP_INFO(rclcpp::get_logger("FlipperInterface"), "Successfully configured");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn FlipperInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state){
    RCLCPP_INFO(rclcpp::get_logger("FlipperInterface"), "Cleaning up ...please wait...");

    if (flipper_comms_.connected()){
      flipper_comms_.disconnect();
    }
    flipper_transport_ready_ = false;

    RCLCPP_INFO(rclcpp::get_logger("FlipperInterface"), "Successfully cleaned up!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn FlipperInterface::on_activate(const rclcpp_lifecycle::State &previous_state){
    RCLCPP_INFO(rclcpp::get_logger("FlipperInterface"), "Activating ...please wait...");

    flipper_transport_ready_ = flipper_comms_.connected();

    if (!flipper_transport_ready_){
      RCLCPP_WARN(
        rclcpp::get_logger("FlipperInterface"),
        "FlipperInterface is active without a serial connection. Leave the flipper controller disabled until the Arduino transport is available."
      );
    } else if (feedback_mode_ == "write_only") {
      RCLCPP_WARN(
        rclcpp::get_logger("FlipperInterface"),
        "Flipper transport is connected in write-only mode. Joint states remain latched because the current Arduino firmware does not publish encoder feedback."
      );
    }

    RCLCPP_INFO(rclcpp::get_logger("FlipperInterface"), "Successfully activated");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn FlipperInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state){
    RCLCPP_INFO(rclcpp::get_logger("FlipperInterface"), "Deactivating ...please wait...");

    joint_velocities_command_.assign(4, 0.0);
    write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0));

    RCLCPP_INFO(rclcpp::get_logger("FlipperInterface"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  return_type FlipperInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
    if (!flipper_transport_ready_) {
      return return_type::OK;
    }

    if (feedback_mode_ == "write_only") {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("FlipperInterface"),
        throttle_clock(),
        5000,
        "FlipperInterface is running with write-only Arduino firmware. Joint positions stay at their last reported values until real feedback is added."
      );
      return return_type::OK;
    }
     
    return return_type::OK;
  }

  return_type FlipperInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration &){
    if (!flipper_transport_ready_) {
      return return_type::OK;
    }

    // The controller already presents commands in logical FL/FR/RL/RR order.
    flipper_comms_.set_flipper_values(
      clamp_flipper_command(joint_velocities_command_[0]),
      clamp_flipper_command(joint_velocities_command_[1]),
      clamp_flipper_command(joint_velocities_command_[2]),
      clamp_flipper_command(joint_velocities_command_[3])
    );

    return return_type::OK;
  }

}  // namespace flipper3_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  flipper3_interface::FlipperInterface, hardware_interface::SystemInterface)
