#include "lotti_control/chain_controller.hpp"

#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <geometry_msgs/msg/vector3.hpp>


using config_type = controller_interface::interface_configuration_type;

namespace chain_controller{
ChainController::ChainController() : controller_interface::ControllerInterface() {}

  controller_interface::CallbackReturn ChainController::on_init(){
    // should have error handling
    joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
    command_interface_types_ =
      auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
    state_interface_types_ =
      auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration ChainController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

    conf.names.reserve(joint_names_.size() * command_interface_types_.size());
    for (const auto & joint_name : joint_names_){
      for (const auto & interface_type : command_interface_types_){
        conf.names.push_back(joint_name + "/" + interface_type);
      }
    }

    return conf;
  }

  controller_interface::InterfaceConfiguration ChainController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

    conf.names.reserve(joint_names_.size() * state_interface_types_.size());
    for (const auto & joint_name : joint_names_){
      for (const auto & interface_type : state_interface_types_){
        conf.names.push_back(joint_name + "/" + interface_type);
      }
    }

    return conf;
  }

  controller_interface::CallbackReturn ChainController::on_configure(const rclcpp_lifecycle::State &){
    auto chain_callback =
      [this](geometry_msgs::msg::Vector3 chain_msg) -> void {
        x_cmd = chain_msg.x;
        y_cmd = chain_msg.y;
    };

    chain_sub_ =
      get_node()->create_subscription<geometry_msgs::msg::Vector3>(
        "/cmd/chains", 1, chain_callback);

    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn ChainController::on_activate(const rclcpp_lifecycle::State &){
    // clear out vectors in case of restart
    joint_velocity_command_interface_.clear();
    joint_torque_command_interface_.clear();
    joint_position_state_interface_.clear();
    joint_velocity_state_interface_.clear();
    joint_torque_state_interface_.clear();
    joint_temp_state_interface_.clear();

    // assign command interfaces
    for (auto & interface : command_interfaces_){
      command_interface_map_[interface.get_interface_name()]->push_back(interface);
    }

    // assign state interfaces
    for (auto & interface : state_interfaces_){
      state_interface_map_[interface.get_interface_name()]->push_back(interface);
    }

    return CallbackReturn::SUCCESS;
  }


  controller_interface::return_type ChainController::update(
    const rclcpp::Time & time, const rclcpp::Duration & /*period*/){
    
      /* if(velocity_cmd > 0.1){
        right_chain_cmd = sqrt(  (velocity_cmd * max_speed) * (velocity_cmd * max_speed) + 
                                (wheel_sep * wheel_sep) -
                                2 * (velocity_cmd * max_speed) * wheel_sep * cos(angle_cmd));
        
        left_chain_cmd = sqrt(  (velocity_cmd * max_speed)*(velocity_cmd * max_speed) + 
                                (wheel_sep * wheel_sep) -
                                2 * (velocity_cmd * max_speed) * wheel_sep * cos(6.283 - angle_cmd));
      }
 */
      if(y_cmd > 0.1) {
        x_speed = x_cmd * max_speed;
        y_speed = y_cmd * max_speed;

        

      }

      else {
        left_chain_cmd = 0;
        right_chain_cmd = 0;
      }

      joint_velocity_command_interface_[0].get().set_value(left_chain_cmd);
      joint_velocity_command_interface_[1].get().set_value(right_chain_cmd);

    return controller_interface::return_type::OK;
  }

  controller_interface::CallbackReturn ChainController::on_deactivate(const rclcpp_lifecycle::State &){
    release_interfaces();

    return CallbackReturn::SUCCESS;
  }

}  // namespace chain_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  chain_controller::ChainController, controller_interface::ControllerInterface)
