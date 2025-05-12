#include "lotti_control/flipper_interface.hpp"
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

namespace flipper_interface{
  CallbackReturn FlipperInterface::on_init(const hardware_interface::HardwareInfo &info){
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS){
      return CallbackReturn::ERROR;
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
      RCLCPP_INFO(rclcpp::get_logger("FlipperInterface"), "exporting flipper");
    }


/*      int ind = 0;
      for (const auto &joint_name : joint_interfaces["velocity"]){
      command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
    } */

    return command_interfaces;
  }

  hardware_interface::CallbackReturn FlipperInterface::on_activate(const rclcpp_lifecycle::State &previous_state){
    RCLCPP_INFO(rclcpp::get_logger("FlipperInterface"), "Activating ...please wait...");

    //rs485_.conncet(cfg_.baud_rate);

    RCLCPP_INFO(rclcpp::get_logger("FlipperInterface"), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn FlipperInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state){
    RCLCPP_INFO(rclcpp::get_logger("FlipperInterface"), "Deactivating ...please wait...");

    //rs485_.disconncet();

    RCLCPP_INFO(rclcpp::get_logger("FlipperInterface"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  return_type FlipperInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period){
    //rs485_.read_encs()

    for (auto i = 0ul; i < joint_velocities_command_.size(); i++){
      joint_positions_[i] += joint_velocities_command_[i] * 2*3.1415/12 * period.seconds();
    }
  
    return return_type::OK;
  }

  return_type FlipperInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration &){


    return return_type::OK;
  }

}  // namespace flipper_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  flipper_interface::FlipperInterface, hardware_interface::SystemInterface)
