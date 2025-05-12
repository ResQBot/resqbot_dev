#include "lotti_control/chain_interface.hpp"
//#include "lotti_control/RS485_comms.hpp"

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

namespace chain_interface{
  CallbackReturn ChainInterface::on_init(const hardware_interface::HardwareInfo &info){
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS){
      return CallbackReturn::ERROR;
    }

    // robot has 2 joints, 4 interfaces
    joint_positions_.assign(2, 0);
    joint_velocities_.assign(2, 0);
    joint_torques_.assign(2, 0);
    joint_temps_.assign(2, 0);
    joint_velocities_command_.assign(2, 0);
    joint_torques_command_.assign(2, 0);

    for (const auto &joint : info_.joints){
      for (const auto &interface : joint.state_interfaces){
        joint_interfaces[interface.name].push_back(joint.name);
      }
    }
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> ChainInterface::export_state_interfaces(){
    std::vector<hardware_interface::StateInterface> state_interfaces;

    int ind = 0;
    for (const auto &joint_name : joint_interfaces["position"]){
      state_interfaces.emplace_back(joint_name, "position", &joint_positions_[ind++]);
    }

    ind = 0;
    for (const auto &joint_name : joint_interfaces["velocity"]){
      state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
    }

    ind = 0;
    for (const auto &joint_name : joint_interfaces["torque"]){
      state_interfaces.emplace_back(joint_name, "torque", &joint_torques_[ind++]);
    }

    ind = 0;
    for (const auto &joint_name : joint_interfaces["temp"]){
      state_interfaces.emplace_back(joint_name, "temp", &joint_temps_[ind++]);
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> ChainInterface::export_command_interfaces(){
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    int ind = 0;
    for (const auto &joint_name : joint_interfaces["velocity"]){
      command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
    }

    ind = 0;
    for (const auto &joint_name : joint_interfaces["torque"]){
      command_interfaces.emplace_back(joint_name, "torque", &joint_torques_command_[ind++]);
    }
    return command_interfaces;
  }

  hardware_interface::CallbackReturn ChainInterface::on_activate(const rclcpp_lifecycle::State &previous_state){
    RCLCPP_INFO(rclcpp::get_logger("ChainInterface"), "Activating ...please wait...");

    //rs485_.conncet(cfg_.baud_rate);

    RCLCPP_INFO(rclcpp::get_logger("ChainInterface"), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ChainInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state){
    RCLCPP_INFO(rclcpp::get_logger("ChainInterface"), "Deactivating ...please wait...");

    //rs485_.disconncet();

    RCLCPP_INFO(rclcpp::get_logger("ChainInterface"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  return_type ChainInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period){
    //rs485_.read_encs()

    for (auto i = 0ul; i < joint_velocities_command_.size(); i++){
      joint_velocities_[i] = joint_velocities_command_[i];
      joint_positions_[i] += joint_velocities_command_[i] * period.seconds();
    }
  
    for (auto i = 0ul; i < joint_torques_command_.size(); i++){
      joint_torques_[i] = joint_torques_command_[i];
    }

    for (auto i = 0ul; i < joint_temps_.size(); i++){
      joint_temps_[i] = 30.0;
    }
  
    return return_type::OK;
  }

  return_type ChainInterface::write(const rclcpp::Time &, const rclcpp::Duration &){


    return return_type::OK;
  }

}  // namespace chain_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  chain_interface::ChainInterface, hardware_interface::SystemInterface)
