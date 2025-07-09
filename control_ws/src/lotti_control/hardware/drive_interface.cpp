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
    max_torque_ = std::stof(info_.hardware_parameters["max_torque"]);
    
    // robot has 2 joints, 1 interface
    joint_velocities_command_.assign(2, 0);
    joint_velocities_.assign(2, 0);
    joint_positions_.assign(2, 0);
    joint_torques_.assign(2, 0);
    joint_temps_.assign(2, 0);

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
    
    /*if (!data_l_.correct){RCLCPP_ERROR(rclcpp::get_logger("DriveInterface"), "LEFT MOTOR DATA ERROR");}
    else {
      joint_positions_[0] = data_l_.q;     //Rads
      joint_velocities_[0] = data_l_.dq / gearRatio;   //Rads/s 
      joint_torques_[0] = data_l_.tau;     //Nm
      joint_temps_[0] = data_l_.temp;      //°C

      if (joint_temps_[0] >= 60) {RCLCPP_ERROR(rclcpp::get_logger("DriveInterface"), "LEFT MOTOR OVER 60 DEGREES");}
      if      (data_l_.merror == 1){RCLCPP_ERROR(rclcpp::get_logger("DriveInterface"), "LEFT MOTOR OVERHEATING");}
      else if (data_l_.merror == 2){RCLCPP_ERROR(rclcpp::get_logger("DriveInterface"), "LEFT MOTOR OVERCURRENT");}
      else if (data_l_.merror == 3){RCLCPP_ERROR(rclcpp::get_logger("DriveInterface"), "LEFT MOTOR OVERVOLTAGE");}
      else if (data_l_.merror == 4){RCLCPP_ERROR(rclcpp::get_logger("DriveInterface"), "LEFT MOTOR ENCODER ERROR");}
    }

    if (!data_r_.correct){RCLCPP_ERROR(rclcpp::get_logger("DriveInterface"), "RIGHT MOTOR DATA ERROR");}
    else {
      joint_positions_[1] = data_r_.q;     //Rads
      joint_velocities_[1] = data_r_.dq / gearRatio;   //Rads/s 
      joint_torques_[1] = data_r_.tau;     //Nm
      joint_temps_[1] = data_r_.temp;      //°C
      
      if (joint_temps_[1] >= 60) {RCLCPP_ERROR(rclcpp::get_logger("DriveInterface"), "RIGHT MOTOR OVER 60 DEGREES");}
      if      (data_r_.merror == 1){RCLCPP_ERROR(rclcpp::get_logger("DriveInterface"), "RIGHT MOTOR OVERHEATING");}
      else if (data_r_.merror == 2){RCLCPP_ERROR(rclcpp::get_logger("DriveInterface"), "RIGHT MOTOR OVERCURRENT");}
      else if (data_r_.merror == 3){RCLCPP_ERROR(rclcpp::get_logger("DriveInterface"), "RIGHT MOTOR OVERVOLTAGE");}
      else if (data_r_.merror == 4){RCLCPP_ERROR(rclcpp::get_logger("DriveInterface"), "RIGHT MOTOR ENCODER ERROR");}
    }
     */
    // char str2[100];
    // sprintf(str2, "%f", joint_positions_[0]);
    // RCLCPP_INFO(rclcpp::get_logger("DriveInterface"), str2); 

    return return_type::OK;
  }

  return_type DriveInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration &){
   
    if (joint_temps_[0] <= 70 && joint_temps_[1] <= 70){
      speed_l = joint_velocities_command_[0] * max_speed_ * gearRatio;
      speed_r = -joint_velocities_command_[1] * max_speed_ * gearRatio;
    }
    else {
      speed_l = 0.0;
      speed_r = 0.0;
      {RCLCPP_ERROR(rclcpp::get_logger("DriveInterface"), "MOTORS OVERHEATING");}
    }

    for (int i = 0; i < 2; i++){
      if (abs(joint_torques_[0]) > max_torque_){
        torque_cmd_[i] = std::clamp(joint_torques_[0], -max_torque_, max_torque_);
      }
      else {
        torque_cmd_[i] = 0.0;
      }
    }

    // char str[100];
    // sprintf(str, "%f", speed_l);
    // RCLCPP_INFO(rclcpp::get_logger("DriveInterface"), str); 

    
    // Set commands
    cmd_l_.motorType = MotorType::GO_M8010_6;
    data_l_.motorType = MotorType::GO_M8010_6;
    cmd_l_.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    cmd_l_.id   = 2;           //motor ID
    cmd_l_.kp   = 0.0;         //positional stiffness  
    cmd_l_.kd   = 0.05;        //velocity stiffness   
    cmd_l_.q    = 0.0;         //position []   
    cmd_l_.dq   = speed_l;     //speed    [Rads/s]
    cmd_l_.tau  = torque_cmd_[0];    //torrque  [Nm]

    cmd_r_.motorType = MotorType::GO_M8010_6;
    data_r_.motorType = MotorType::GO_M8010_6;
    cmd_r_.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    cmd_r_.id   = 1;
    cmd_r_.kp   = 0.0;
    cmd_r_.kd   = 0.05;
    cmd_r_.q    = 0.0;
    cmd_r_.dq   = speed_r;
    cmd_r_.tau  = torque_cmd_[1];
 
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