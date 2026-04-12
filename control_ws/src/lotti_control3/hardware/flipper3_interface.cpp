#include "lotti_control3/flipper3_interface.hpp"
//#include "lotti_control3/flipper_comms.hpp"
//#include "lotti_control3/RS485_comms.hpp"

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


namespace flipper3_interface{
  CallbackReturn FlipperInterface::on_init(const hardware_interface::HardwareInfo &info){
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS){
      return CallbackReturn::ERROR;
    }

    //get the Arduino ID from the ros2_control file
    device_ = info_.hardware_parameters["device"];

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

//-    if (flipper_comms_.connected()){
//-      flipper_comms_.disconnect();
//-    }
//-    flipper_comms_.connect(device_); 
 
    RCLCPP_INFO(rclcpp::get_logger("FlipperInterface"), "Successfully configured");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn FlipperInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state){
    RCLCPP_INFO(rclcpp::get_logger("FlipperInterface"), "Cleaning up ...please wait...");

 //-    if (flipper_comms_.connected()){
 //-      flipper_comms_.disconnect();
 //-    }

    RCLCPP_INFO(rclcpp::get_logger("FlipperInterface"), "Successfully cleaned up!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn FlipperInterface::on_activate(const rclcpp_lifecycle::State &previous_state){
    RCLCPP_INFO(rclcpp::get_logger("FlipperInterface"), "Configuring ...please wait...");

//-    if (!flipper_comms_.connected()){
//-      RCLCPP_ERROR(rclcpp::get_logger("FlipperInterface"), "Arduino not connected");
//-      return hardware_interface::CallbackReturn::ERROR;
//-    }

    RCLCPP_WARN(
      rclcpp::get_logger("FlipperInterface"),
      "FlipperInterface is running in stub mode. Serial transport is not enabled in this branch."
    );

    RCLCPP_INFO(rclcpp::get_logger("FlipperInterface"), "Successfully activated");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn FlipperInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state){
    RCLCPP_INFO(rclcpp::get_logger("FlipperInterface"), "Deactivating ...please wait...");

    RCLCPP_INFO(rclcpp::get_logger("FlipperInterface"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  return_type FlipperInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period){    
//-    if (!flipper_comms_.connected()){
//-      return hardware_interface::return_type::ERROR;
//-    }



    fl_cmd_ = fl_cmd_ + joint_velocities_command_[1] * period.seconds() * 360/12;
    fr_cmd_ = fr_cmd_ + joint_velocities_command_[0] * period.seconds() * 360/12;
    rl_cmd_ = rl_cmd_ + joint_velocities_command_[3] * period.seconds() * 360/12;
    rr_cmd_ = rr_cmd_ + joint_velocities_command_[2] * period.seconds() * 360/12;

    joint_positions_[0] = (fr_cmd_ /360) * (2*3.1416);
    joint_positions_[1] = (fl_cmd_ /360) * (2*3.1416);
    joint_positions_[2] = (rr_cmd_ /360) * (2*3.1416);
    joint_positions_[3] = (rl_cmd_ /360) * (2*3.1416);
     
    return return_type::OK;
  }

  return_type FlipperInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration &){

    //std::cout << std::to_string(joint_velocities_command_[0]) << "\n";

//-     flipper_comms_.set_flipper_values(
//-      joint_velocities_command_[0],
//-      joint_velocities_command_[1],
//-      joint_velocities_command_[2],
//-      joint_velocities_command_[3]
//-    ); 

    return return_type::OK;
  }

}  // namespace flipper3_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  flipper3_interface::FlipperInterface, hardware_interface::SystemInterface)
