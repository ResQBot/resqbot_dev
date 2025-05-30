#include "lotti_control/arm_interface.hpp"
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

namespace arm_interface
{
  CallbackReturn ArmInterface::on_init(const hardware_interface::HardwareInfo & info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }
    // robot has 6 joints, 4 interfaces
    joint_positions_.assign(6, 0);
    joint_velocities_.assign(6, 0);
    joint_torques_.assign(6, 0);
    joint_volts_.assign(6, 0);
    //joint_positions_command_.assign(6, 0);
    joint_velocities_command_.assign(6, 0);

    for (const auto & joint : info_.joints)
    {
      for (const auto & interface : joint.state_interfaces)
      {
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

    ind = 0;
    for (const auto &joint_name : joint_interfaces["velocity"]){
      state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
    }

    ind = 0;
    for (const auto &joint_name : joint_interfaces["torque"]){
      state_interfaces.emplace_back(joint_name, "torque", &joint_torques_[ind++]);
    }

    ind = 0;
    for (const auto &joint_name : joint_interfaces["volt"]){
      state_interfaces.emplace_back(joint_name, "volt", &joint_volts_[ind++]);
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> ArmInterface::export_command_interfaces(){
    std::vector<hardware_interface::CommandInterface> command_interfaces;

/*     int ind = 0;
    for (const auto &joint_name : joint_interfaces["position"]){
      command_interfaces.emplace_back(joint_name, "position", &joint_positions_command_[ind++]);
    } */

    int ind = 0;
    for (const auto &joint_name : joint_interfaces["velocity"]){
      command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn ArmInterface::on_configure(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Configuring ...please wait...");

    //rs485_.conncet(cfg_.baud_rate);

    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Successfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

   hardware_interface::CallbackReturn ArmInterface::on_cleanup(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Cleaning up ...please wait...");

    //rs485_.conncet(cfg_.baud_rate);

    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Successfully cleaned up!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ArmInterface::on_activate(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Activating ...please wait...");

    //rs485_.conncet(cfg_.baud_rate);

    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ArmInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Deactivating ...please wait...");

    //rs485_.disconncet();

    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  return_type ArmInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period){
    for (auto i = 0ul; i < joint_velocities_command_.size(); i++){
      joint_velocities_[i] = joint_velocities_command_[i];
      joint_positions_[i] += joint_velocities_command_[i] * period.seconds();
    }
    return return_type::OK;
  }

  return_type ArmInterface::write(const rclcpp::Time &, const rclcpp::Duration &){
    /* arm_comms_.set_arm_values(
      joint_velocities_command_[0],
      joint_velocities_command_[1],
      joint_velocities_command_[2],
      joint_velocities_command_[3],
      joint_velocities_command_[4],
      joint_velocities_command_[5],
    );  */

    return return_type::OK;
  }

}  // namespace arm_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arm_interface::ArmInterface, hardware_interface::SystemInterface)
