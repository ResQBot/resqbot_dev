#include "lotti_control/drive_interface.hpp"

#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"


namespace drive_interface{
  CallbackReturn DriveInterface::on_init(const hardware_interface::HardwareInfo &info){
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS){
      return CallbackReturn::ERROR;
    }

    //get the Arduino ID from the ros2_control file
    device_ = info_.hardware_parameters["device"];
    max_speed_ = std::stof(info_.hardware_parameters["max_speed"]);
    
    // robot has 2 joints, 1 interface
    joint_velocities_command_.assign(2, 0);
    joint_velocities_.assign(2, 0);
    joint_positions_.assign(2, 0);

    for (const auto &joint : info_.joints){
      for (const auto &interface : joint.state_interfaces){
        joint_interfaces[interface.name].push_back(joint.name);
      }
    }
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> DriveInterface::export_state_interfaces(){
    std::vector<hardware_interface::StateInterface> state_interfaces;

    int ind = 0;
    for (const auto &joint_name : joint_interfaces["velocity"]){
      state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
    }

    ind = 0;
    for (const auto &joint_name : joint_interfaces["position"]){
      state_interfaces.emplace_back(joint_name, "position", &joint_positions_[ind++]);
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> DriveInterface::export_command_interfaces(){
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (int ind = 0; ind < 2; ind++){
      command_interfaces.emplace_back(info_.joints[ind].name, "velocity", &joint_velocities_command_[ind]);
    }
    return command_interfaces;
  }

  hardware_interface::CallbackReturn DriveInterface::on_configure(const rclcpp_lifecycle::State &previous_state){
    RCLCPP_INFO(rclcpp::get_logger("DriveInterface"), "Configuring ...please wait...");

//-    serial_ = std::make_unique<SerialPort>(device_);

    RCLCPP_INFO(rclcpp::get_logger("DriveInterface"), "Successfully configured");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DriveInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state){
    RCLCPP_INFO(rclcpp::get_logger("DriveInterface"), "Cleaning up ...please wait...");

    RCLCPP_INFO(rclcpp::get_logger("DriveInterface"), "Successfully cleaned up!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DriveInterface::on_activate(const rclcpp_lifecycle::State &previous_state){
    RCLCPP_INFO(rclcpp::get_logger("DriveInterface"), "Configuring ...please wait...");

    RCLCPP_INFO(rclcpp::get_logger("DriveInterface"), "Successfully activated");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DriveInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state){
    RCLCPP_INFO(rclcpp::get_logger("DriveInterface"), "Deactivating ...please wait...");

    RCLCPP_INFO(rclcpp::get_logger("DriveInterface"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  return_type DriveInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period){    

    joint_velocities_[0] = speed_l;
    joint_velocities_[1] = speed_r;
    
    pos_prev_l = joint_positions_[0]; 
    joint_positions_[0] = pos_prev_l + (speed_l * period.seconds());

    pos_prev_r = joint_positions_[1]; 
    joint_positions_[1] = pos_prev_r + (speed_r * period.seconds());
    
     
    // char str2[100];
    // sprintf(str2, "%f", joint_positions_[0]);
    // RCLCPP_INFO(rclcpp::get_logger("DriveInterface"), str2); 

    return return_type::OK;
  }

  return_type DriveInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration &){

    speed_l = joint_velocities_command_[0] * max_speed_ * gearRatio;  // directly from command interface
    speed_r = joint_velocities_command_[1] * max_speed_ * gearRatio;

    // char str[100];
    // sprintf(str, "%f", speed_l);
    // RCLCPP_INFO(rclcpp::get_logger("DriveInterface"), str); 

    
    // Set commands
    cmd_l_.motorType = MotorType::GO_M8010_6;
    data_l_.motorType = MotorType::GO_M8010_6;
    cmd_l_.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    cmd_l_.id   = 2;
    cmd_l_.kp   = 0.0;
    cmd_l_.kd   = 0.05;
    cmd_l_.q    = 0.0;
    cmd_l_.dq   = speed_l;
    cmd_l_.tau  = 0.0;

    cmd_r_.motorType = MotorType::GO_M8010_6;
    data_r_.motorType = MotorType::GO_M8010_6;
    cmd_r_.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    cmd_r_.id   = 1;
    cmd_r_.kp   = 0.0;
    cmd_r_.kd   = 0.05;
    cmd_r_.q    = 0.0;
    cmd_r_.dq   = speed_r;
    cmd_r_.tau  = 0.0;
 


    // Set other command parameters if necessary
    //cmd_l_.kp = 0.0;
    //cmd_l_.kd = 0.05;
    //cmd_l_.q = 0.0;
    //cmd_l_.tau = 0.0;

    //cmd_r_.kp = 0.0;
    //cmd_r_.kd = 0.05;
    //cmd_r_.q = 0.0;
    //cmd_r_.tau = 0.0;

    // Send commands and receive data over serial port
//-    if (serial_) {
//-      serial_->sendRecv(&cmd_l_, &data_l_);
//-      serial_->sendRecv(&cmd_r_, &data_r_);
//-    }

    return return_type::OK;
  }

}  // namespace drive_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  drive_interface::DriveInterface, hardware_interface::SystemInterface)