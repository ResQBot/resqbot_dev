#include "lotti_control/arm_interface.hpp"

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
    
    //get the Arduino ID from the ros2_control file
//-    device_ = info_.hardware_parameters["device"];

    // robot has 6 joints, 4 interfaces
    joint_positions_.assign(6, 0);
    joint_velocities_.assign(6, 0);
    joint_torques_.assign(6, 0);
    joint_volts_.assign(6, 0);
    joint_positions_command_.assign(6, 0);
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

    int ind = 0;
    for (const auto &joint_name : joint_interfaces["position"]){
      command_interfaces.emplace_back(joint_name, "position", &joint_positions_command_[ind++]);
    }

    ind = 0;
    for (const auto &joint_name : joint_interfaces["velocity"]){
      command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn ArmInterface::on_configure(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Configuring ...please wait...");

//-    if (arm_comms_.connected()){
//-      arm_comms_.disconnect();
//-    }
//-    arm_comms_.connect(device_); 

    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Successfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

   hardware_interface::CallbackReturn ArmInterface::on_cleanup(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Cleaning up ...please wait...");

//-    if (arm_comms_.connected()){
//-      arm_comms_.disconnect();
//-    } 

    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Successfully cleaned up!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ArmInterface::on_activate(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Activating ...please wait...");

//-    if (!arm_comms_.connected()){
//-      RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"), "Arduino not connected");
//-      return hardware_interface::CallbackReturn::ERROR;
//-    }

    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ArmInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Deactivating ...please wait...");


    RCLCPP_INFO(rclcpp::get_logger("ArmInterface"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  return_type ArmInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period){
//-    if (!arm_comms_.connected()){
//-      return hardware_interface::return_type::ERROR;
//-    }

//-    std::string arm_answer_ = arm_comms_.read_msg();

//-    sscanf(arm_answer_.c_str(), "%lf/%lf/%lf/%lf/%lf/%lf", &state_pos_[0], &state_pos_[1], &state_pos_[2], &state_pos_[3], &state_pos_[4], &state_pos_[5]);

    for (auto i = 0ul; i < joint_velocities_command_.size(); i++){
      joint_velocities_[i] = joint_velocities_command_[i];
    }
    
//-    for (auto i = 0ul; i < joint_positions_.size(); i++){
//-      joint_positions_[i] = (state_pos_[i]/2048)*3,1416;
//-    }

//+
    for (auto i = 0ul; i < joint_positions_.size(); i++){
//+
      joint_positions_[i] = joint_positions_command_[i];
//+
    }


    return return_type::OK;
  }

  return_type ArmInterface::write(const rclcpp::Time &, const rclcpp::Duration &){
//-    arm_comms_.set_arm_values(joint_positions_command_, joint_velocities_command_);  

    return return_type::OK;
  }

}  // namespace arm_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arm_interface::ArmInterface, hardware_interface::SystemInterface)
