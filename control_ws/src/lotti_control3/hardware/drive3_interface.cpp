#include "lotti_control3/drive3_interface.hpp"

#include <cstdio>
#include <exception>
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

#include "unitreeMotor/unitreeMotor.h"


namespace {

rclcpp::Clock & throttle_clock(){
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

int get_optional_int_parameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & key,
  const int fallback)
{
  const auto parameter = info.hardware_parameters.find(key);
  if (parameter == info.hardware_parameters.end() || parameter->second.empty()) {
    return fallback;
  }

  return std::stoi(parameter->second);
}

float get_optional_float_parameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & key,
  const float fallback)
{
  const auto parameter = info.hardware_parameters.find(key);
  if (parameter == info.hardware_parameters.end() || parameter->second.empty()) {
    return fallback;
  }

  return std::stof(parameter->second);
}

unsigned short resolve_drive_mode(){
#if LOTTI_HAVE_UNITREE_SDK
  return static_cast<unsigned short>(queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC));
#else
  return 1;
#endif
}

}  // namespace

namespace drive3_interface{
  CallbackReturn DriveInterface::on_init(const hardware_interface::HardwareInfo &info){
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS){
      return CallbackReturn::ERROR;
    }

    // Get the configured serial device and drive tuning values from ros2_control.
    device_ = info_.hardware_parameters["device"];
    max_speed_ = get_optional_int_parameter(info_, "max_speed", 6);
    left_motor_id_ = get_optional_int_parameter(info_, "left_motor_id", 2);
    right_motor_id_ = get_optional_int_parameter(info_, "right_motor_id", 1);
    gearRatio = get_optional_float_parameter(info_, "gear_ratio", 6.33F);
    

    // robot has 2 joints, 1 interface
    joint_velocities_command_.assign(2, 0);
    joint_velocities_.assign(2,0);

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

    drive_transport_ready_ = false;
    drive_feedback_valid_ = false;

#if LOTTI_HAVE_UNITREE_SDK
    try {
      serial_port_ = std::make_unique<SerialPort>(device_);
      drive_transport_ready_ = true;
      gearRatio = queryGearRatio(MotorType::GO_M8010_6);

      RCLCPP_INFO(
        rclcpp::get_logger("DriveInterface"),
        "Drive transport ready on %s with gear ratio %.3f",
        device_.c_str(),
        gearRatio
      );
    } catch (const std::exception & exception) {
      serial_port_.reset();
      RCLCPP_ERROR(
        rclcpp::get_logger("DriveInterface"),
        "Failed to initialize the Unitree drive transport on %s: %s",
        device_.c_str(),
        exception.what()
      );
    } catch (...) {
      serial_port_.reset();
      RCLCPP_ERROR(
        rclcpp::get_logger("DriveInterface"),
        "Failed to initialize the Unitree drive transport on %s due to an unknown error",
        device_.c_str()
      );
    }
#else
    RCLCPP_WARN(
      rclcpp::get_logger("DriveInterface"),
      "Drive transport is disabled because lotti_control3 was built without the optional vendor Unitree SDK."
    );
#endif

    RCLCPP_INFO(rclcpp::get_logger("DriveInterface"), "Successfully configured");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DriveInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state){
    RCLCPP_INFO(rclcpp::get_logger("DriveInterface"), "Cleaning up ...please wait...");

    drive_transport_ready_ = false;
    drive_feedback_valid_ = false;
#if LOTTI_HAVE_UNITREE_SDK
    serial_port_.reset();
#endif

    RCLCPP_INFO(rclcpp::get_logger("DriveInterface"), "Successfully cleaned up!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DriveInterface::on_activate(const rclcpp_lifecycle::State &previous_state){
    RCLCPP_INFO(rclcpp::get_logger("DriveInterface"), "Activating ...please wait...");

    if (!drive_transport_ready_) {
      RCLCPP_WARN(
        rclcpp::get_logger("DriveInterface"),
        "DriveInterface is active without a real motor transport. Leave the drive controller disabled unless the SDK-backed transport is available."
      );
    }

    RCLCPP_INFO(rclcpp::get_logger("DriveInterface"), "Successfully activated");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DriveInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state){
    RCLCPP_INFO(rclcpp::get_logger("DriveInterface"), "Deactivating ...please wait...");

    joint_velocities_command_[0] = 0.0;
    joint_velocities_command_[1] = 0.0;
    drive_feedback_valid_ = false;
    write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0.0));

    RCLCPP_INFO(rclcpp::get_logger("DriveInterface"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  return_type DriveInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period){    
    if (!drive_feedback_valid_) {
      if (drive_transport_ready_) {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("DriveInterface"),
          throttle_clock(),
          2000,
          "Drive transport is up but no valid motor feedback has been received yet."
        );
      }

      return return_type::OK;
    }

    if (data_l_.correct) {
      joint_velocities_[0] = data_l_.dq / gearRatio;
    } else {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("DriveInterface"),
        throttle_clock(),
        2000,
        "Drive transport reported invalid feedback for the left motor."
      );
    }

    if (data_r_.correct) {
      joint_velocities_[1] = data_r_.dq / gearRatio;
    } else {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("DriveInterface"),
        throttle_clock(),
        2000,
        "Drive transport reported invalid feedback for the right motor."
      );
    }
     
    return return_type::OK;
  }

  return_type DriveInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration &){

    speed_l = joint_velocities_command_[0] * max_speed_ * gearRatio;  // directly from command interface
    speed_r = joint_velocities_command_[1] * max_speed_ * gearRatio;
    
    // Set commands
    cmd_l_.motorType = MotorType::GO_M8010_6;
    data_l_.motorType = MotorType::GO_M8010_6;
    cmd_l_.mode = resolve_drive_mode();
    cmd_l_.id   = static_cast<unsigned short>(left_motor_id_);
    cmd_l_.kp   = 0.0;
    cmd_l_.kd   = 0.05;
    cmd_l_.q    = 0.0;
    cmd_l_.dq   = speed_l;
    cmd_l_.tau  = 0.0;

    cmd_r_.motorType = MotorType::GO_M8010_6;
    data_r_.motorType = MotorType::GO_M8010_6;
    cmd_r_.mode = resolve_drive_mode();
    cmd_r_.id   = static_cast<unsigned short>(right_motor_id_);
    cmd_r_.kp   = 0.0;
    cmd_r_.kd   = 0.05;
    cmd_r_.q    = 0.0;
    cmd_r_.dq   = speed_r;
    cmd_r_.tau  = 0.0;

#if LOTTI_HAVE_UNITREE_SDK
    if (drive_transport_ready_ && serial_port_) {
      const bool left_ok = serial_port_->sendRecv(&cmd_l_, &data_l_);
      const bool right_ok = serial_port_->sendRecv(&cmd_r_, &data_r_);
      drive_feedback_valid_ = left_ok && right_ok && data_l_.correct && data_r_.correct;

      if (!drive_feedback_valid_) {
        RCLCPP_ERROR_THROTTLE(
          rclcpp::get_logger("DriveInterface"),
          throttle_clock(),
          2000,
          "Drive command exchange failed or returned invalid feedback."
        );
      }
    } else {
      drive_feedback_valid_ = false;
    }
#else
    drive_feedback_valid_ = false;
#endif

    return return_type::OK;
  }

}  // namespace drive3_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  drive3_interface::DriveInterface, hardware_interface::SystemInterface)
