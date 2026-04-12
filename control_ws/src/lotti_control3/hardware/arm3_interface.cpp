#include "lotti_control3/arm3_interface.hpp"

#include <cstdio>
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

}  // namespace

namespace arm3_interface{
  CallbackReturn ArmInterface::on_init(const hardware_interface::HardwareInfo & info){
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS){
      return CallbackReturn::ERROR;
    }
    
    //get the Arduino ID from the ros2_control file
    device_ = info_.hardware_parameters["device"];

    // robot has 5 joints, 1 interfaces
    joint_positions_.assign(5, 0);
    joint_positions_command_.assign(5, 0);

    for (const auto & joint : info_.joints){
      for (const auto & interface : joint.state_interfaces){
        joint_interfaces[interface.name].push_back(joint.name);
      }
    }

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> ArmInterface::export_state_interfaces(){
    std::vector<hardware_interface::StateInterface> state_interfaces;

    int ind = 0;
    for (const auto &joint_name : joint_interfaces["position"]){
      state_interfaces.emplace_back(joint_name, "position", &joint_positions_[ind++]);
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> ArmInterface::export_command_interfaces(){
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    int ind = 0;
    for (const auto &joint_name : joint_interfaces["position"]){
      command_interfaces.emplace_back(joint_name, "position", &joint_positions_command_[ind++]);
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn ArmInterface::on_configure(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Configuring ...please wait...");

    if (arm_comms_.connected()){
      arm_comms_.disconnect();
    }
    arm_comms_.connect(device_); 

    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Successfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

   hardware_interface::CallbackReturn ArmInterface::on_cleanup(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Cleaning up ...please wait...");

    if (arm_comms_.connected()){
      arm_comms_.disconnect();
    } 

    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Successfully cleaned up!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ArmInterface::on_activate(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Activating ...please wait...");

    if (!arm_comms_.connected()){
      RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"), "Arduino not connected");
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ArmInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Deactivating ...please wait...");

    for (auto i = 0ul; i < joint_positions_command_.size(); i++){
      com_pos[i] = 0.01;
    }
    arm_comms_.set_arm_values(com_pos);

    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  return_type ArmInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period){
    if (!arm_comms_.connected()){
      return hardware_interface::return_type::ERROR;
    }

    std::string arm_answer_ = arm_comms_.read_msg();
    if (arm_answer_.empty()) {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("ArmInterface"),
        throttle_clock(),
        2000,
        "Arm serial transport timed out without a feedback line. Keeping the last known joint positions."
      );
      return return_type::OK;
    }

    const int parsed_values = std::sscanf(
      arm_answer_.c_str(),
      "%i:%i:%i:%i:%i",
      &state_pos_[0],
      &state_pos_[1],
      &state_pos_[2],
      &state_pos_[3],
      &state_pos_[4]
    );

    if (parsed_values != 5) {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("ArmInterface"),
        throttle_clock(),
        2000,
        "Received malformed arm feedback '%s'. Keeping the last known joint positions.",
        arm_answer_.c_str()
      );
      return return_type::OK;
    }

    for (auto i = 0ul; i < joint_positions_.size(); i++){
      joint_positions_[i] = (float(state_pos_[i]) * 2 * 3.1416) / 4096;
    }

    // The real feedback from Arduino is now being forwarded to ROS 2 correctly!

    return return_type::OK;
  }

  return_type ArmInterface::write(const rclcpp::Time &, const rclcpp::Duration &){
    
    for (auto i = 0ul; i < joint_positions_command_.size(); i++){
      com_pos[i] = joint_positions_command_[i];
    }
    arm_comms_.set_arm_values(com_pos);

    return return_type::OK;
  }

}  // namespace arm3_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arm3_interface::ArmInterface, hardware_interface::SystemInterface)
